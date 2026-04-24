use anyhow::Result;
use log::{debug, info};
use serialport::{SerialPortInfo, SerialPortType};
use std::fs;

use crate::protocols::{detect_protocol, ProtocolType};

#[derive(Debug, Clone)]
pub struct GimbalDevice {
    pub path: String,
    pub protocol: ProtocolType,
    pub device_info: Option<SerialPortInfo>,
}

pub struct DeviceScanner;

impl DeviceScanner {
    pub fn new() -> Self {
        Self
    }

    /// Find any gimbal device (auto-detect protocol)
    pub async fn find_gimbal_device(&self) -> Option<GimbalDevice> {
        // First try serialport enumeration (works on all platforms)
        if let Some(device) = self.find_via_serialport().await {
            return Some(device);
        }

        // Fallback to system-specific methods
        self.find_via_system_info().await
    }

    /// Find StORM32 device (legacy method for backward compatibility)
    pub fn find_storm32_device(&self) -> Option<String> {
        // First try serialport enumeration (works on all platforms)
        if let Some(path) = self.find_storm32_via_serialport() {
            return Some(path);
        }

        // Fallback to system-specific methods
        self.find_storm32_via_system_info()
    }

    /// Find any suitable device without protocol detection (for when protocol is explicitly specified)
    pub fn find_any_suitable_device(&self) -> Option<String> {
        match serialport::available_ports() {
            Ok(ports) => {
                for port in ports {
                    if self.is_potential_gimbal_device(&port) {
                        info!("Found potential device: {}", port.port_name);
                        return Some(port.port_name);
                    }
                }
            }
            Err(e) => {
                debug!("Error enumerating serial ports: {}", e);
            }
        }
        None
    }

    async fn find_via_serialport(&self) -> Option<GimbalDevice> {
        match serialport::available_ports() {
            Ok(ports) => {
                for port in &ports {
                    if self.is_potential_gimbal_device(port) {
                        info!("Testing potential gimbal device: {}", port.port_name);

                        // Try to detect protocol
                        match detect_protocol(&port.port_name).await {
                            Ok(protocol) => {
                                info!("Detected protocol {} on {}", protocol, port.port_name);
                                return Some(GimbalDevice {
                                    path: port.port_name.clone(),
                                    protocol,
                                    device_info: Some(port.clone()),
                                });
                            }
                            Err(e) => {
                                debug!("Protocol detection failed for {}: {}", port.port_name, e);
                            }
                        }
                    }
                }
            }
            Err(e) => {
                debug!("Error enumerating serial ports: {}", e);
            }
        }
        None
    }

    fn find_storm32_via_serialport(&self) -> Option<String> {
        match serialport::available_ports() {
            Ok(ports) => {
                for port in ports {
                    if self.is_storm32_device(&port) {
                        return Some(port.port_name);
                    }
                }
            }
            Err(e) => {
                debug!("Error enumerating serial ports: {}", e);
            }
        }
        None
    }

    /// Check if a port might be a gimbal device (broader check)
    fn is_potential_gimbal_device(&self, port_info: &SerialPortInfo) -> bool {
        match &port_info.port_type {
            SerialPortType::UsbPort(usb_info) => {
                // Check for common gimbal/drone controller vendors
                // STMicroelectronics (StORM32 and others)
                if usb_info.vid == 1155 {
                    return true;
                }
                // FTDI (many drone controllers use FTDI chips)
                if usb_info.vid == 1027 {
                    return true;
                }
                // Silicon Labs (CP210x series, common in drone hardware)
                if usb_info.vid == 4292 {
                    return true;
                }
                // Prolific (PL2303 series)
                if usb_info.vid == 1659 {
                    return true;
                }
                // Other common vendors that might make gimbal controllers
                false
            }
            SerialPortType::BluetoothPort => false,
            SerialPortType::PciPort => false,
            SerialPortType::Unknown => {
                // For unknown port types, we might still want to try them
                // if they match common naming patterns
                self.matches_gimbal_device_pattern(&port_info.port_name)
            }
        }
    }

    /// Check if device name matches common gimbal device patterns
    fn matches_gimbal_device_pattern(&self, port_name: &str) -> bool {
        let port_lower = port_name.to_lowercase();
        // Common patterns for serial devices that might be gimbals
        port_lower.contains("ttyacm")
            || port_lower.contains("ttyusb")
            || port_lower.contains("usbmodem")
            || port_lower.starts_with("com")
    }

    fn is_storm32_device(&self, port_info: &SerialPortInfo) -> bool {
        match &port_info.port_type {
            SerialPortType::UsbPort(usb_info) => {
                // Check for STM32 Virtual COM Port (STMicroelectronics)
                // VID: 0x0483 (1155), PID: 0x5740 (22336)
                // Also check for common STM32 PIDs
                let storm32_pids = [22336, 22592]; // 0x5740, 0x5840
                usb_info.vid == 1155 && storm32_pids.contains(&usb_info.pid)
            }
            _ => false,
        }
    }

    #[cfg(target_os = "macos")]
    async fn find_via_system_info(&self) -> Option<GimbalDevice> {
        let patterns = ["/dev/tty.usbmodem*", "/dev/tty.SLAB_USBtoUART*"];

        for pattern in &patterns {
            if let Ok(devices) = self.glob_devices(pattern) {
                for device in devices {
                    if self.test_device_exists(&device) {
                        // Try to detect protocol
                        if let Ok(protocol) = detect_protocol(&device).await {
                            return Some(GimbalDevice {
                                path: device,
                                protocol,
                                device_info: None,
                            });
                        }
                    }
                }
            }
        }

        None
    }

    #[cfg(target_os = "macos")]
    fn find_storm32_via_system_info(&self) -> Option<String> {
        // Look for tty devices that match STM32 patterns
        let patterns = ["/dev/tty.usbmodem*"];

        for pattern in &patterns {
            if let Ok(devices) = self.glob_devices(pattern) {
                for device in devices {
                    if self.verify_device_macos(&device) {
                        return Some(device);
                    }
                }
            }
        }

        // Fallback: try any USB modem device
        if let Ok(devices) = self.glob_devices("/dev/tty.usbmodem*") {
            return devices.into_iter().next();
        }

        None
    }

    #[cfg(target_os = "linux")]
    async fn find_via_system_info(&self) -> Option<GimbalDevice> {
        let patterns = ["/dev/ttyACM*", "/dev/ttyUSB*"];

        for pattern in &patterns {
            if let Ok(devices) = self.glob_devices(pattern) {
                for device in devices {
                    if self.test_device_exists(&device) {
                        // Try to detect protocol
                        if let Ok(protocol) = detect_protocol(&device).await {
                            return Some(GimbalDevice {
                                path: device,
                                protocol,
                                device_info: None,
                            });
                        }
                    }
                }
            }
        }

        None
    }

    #[cfg(target_os = "linux")]
    fn find_storm32_via_system_info(&self) -> Option<String> {
        // Check common Linux serial device patterns
        let patterns = ["/dev/ttyACM*", "/dev/ttyUSB*"];

        for pattern in &patterns {
            if let Ok(devices) = self.glob_devices(pattern) {
                for device in devices {
                    if self.verify_device_linux(&device) {
                        return Some(device);
                    }
                }
            }
        }

        // Fallback: return first available device
        for pattern in &patterns {
            if let Ok(devices) = self.glob_devices(pattern) {
                if let Some(device) = devices.into_iter().next() {
                    return Some(device);
                }
            }
        }

        None
    }

    #[cfg(target_os = "windows")]
    async fn find_via_system_info(&self) -> Option<GimbalDevice> {
        // On Windows, try common COM ports
        for i in 1..=20 {
            let port_name = format!("COM{}", i);
            if self.test_device_exists(&port_name) {
                // Try to detect protocol
                if let Ok(protocol) = detect_protocol(&port_name).await {
                    return Some(GimbalDevice {
                        path: port_name,
                        protocol,
                        device_info: None,
                    });
                }
            }
        }
        None
    }

    #[cfg(target_os = "windows")]
    fn find_storm32_via_system_info(&self) -> Option<String> {
        // On Windows, rely primarily on serialport enumeration
        // This is a fallback that checks common COM ports
        for i in 1..=20 {
            let port_name = format!("COM{}", i);
            if self.test_device_exists(&port_name) {
                return Some(port_name);
            }
        }
        None
    }

    #[cfg(not(any(target_os = "macos", target_os = "linux", target_os = "windows")))]
    async fn find_via_system_info(&self) -> Option<GimbalDevice> {
        // Fallback for other platforms
        None
    }

    #[cfg(not(any(target_os = "macos", target_os = "linux", target_os = "windows")))]
    fn find_storm32_via_system_info(&self) -> Option<String> {
        // Fallback for other platforms
        None
    }

    // Helper function to expand glob patterns
    fn glob_devices(&self, pattern: &str) -> Result<Vec<String>, std::io::Error> {
        let mut devices = Vec::new();

        // Simple glob expansion for basic patterns like /dev/tty*
        if let Some(prefix) = pattern.strip_suffix('*') {
            if let Some(dir) = std::path::Path::new(prefix).parent() {
                if let Ok(entries) = fs::read_dir(dir) {
                    for entry in entries.flatten() {
                        let path = entry.path();
                        if let Some(path_str) = path.to_str() {
                            if path_str.starts_with(prefix) {
                                devices.push(path_str.to_string());
                            }
                        }
                    }
                }
            }
        } else {
            // Not a glob, just check if it exists
            if std::path::Path::new(pattern).exists() {
                devices.push(pattern.to_string());
            }
        }

        devices.sort();
        Ok(devices)
    }

    #[cfg(target_os = "macos")]
    fn verify_device_macos(&self, device_path: &str) -> bool {
        // On macOS, we can check USB device info using system_profiler
        // For now, just check if the device is accessible
        self.test_device_exists(device_path)
    }

    #[cfg(target_os = "linux")]
    fn verify_device_linux(&self, device_path: &str) -> bool {
        // On Linux, we can check udev info or sysfs
        // For now, check basic accessibility and try to read USB vendor/product info
        if !self.test_device_exists(device_path) {
            return false;
        }

        // Try to get USB device info from sysfs
        if let Some(usb_path) = self.get_usb_sysfs_path(device_path) {
            if let Ok(vendor) = fs::read_to_string(format!("{}/idVendor", usb_path)) {
                if let Ok(product) = fs::read_to_string(format!("{}/idProduct", usb_path)) {
                    let vendor = vendor.trim();
                    let product = product.trim();
                    // Check for STMicroelectronics VID (0483) and common STM32 PIDs
                    return vendor == "0483" && (product == "5740" || product == "5840");
                }
            }
        }

        // Fallback: assume it might be correct
        true
    }

    #[cfg(target_os = "linux")]
    fn get_usb_sysfs_path(&self, device_path: &str) -> Option<String> {
        // Extract device info from /sys/class/tty/*/device/
        let device_name = std::path::Path::new(device_path).file_name()?.to_str()?;

        let sys_path = format!("/sys/class/tty/{}/device", device_name);
        if std::path::Path::new(&sys_path).exists() {
            // Follow symlinks to find the USB device
            if let Ok(real_path) = fs::read_link(&sys_path) {
                if let Some(real_str) = real_path.to_str() {
                    // Look for USB device info in the path
                    let parts: Vec<&str> = real_str.split('/').collect();
                    for (i, part) in parts.iter().enumerate() {
                        if part.contains(':') && part.contains('.') {
                            // This looks like a USB device (e.g., "1-1.4:1.0")
                            // Go up to the USB device level
                            if i > 0 {
                                let usb_device_path = parts[..i].join("/");
                                let full_path = format!("/sys{}", usb_device_path);
                                if std::path::Path::new(&full_path).exists() {
                                    return Some(full_path);
                                }
                            }
                        }
                    }
                }
            }
        }

        None
    }

    fn test_device_exists(&self, device_path: &str) -> bool {
        std::path::Path::new(device_path).exists()
    }

    pub fn list_all_devices(&self) -> Vec<SerialPortInfo> {
        serialport::available_ports().unwrap_or_default()
    }
}

impl Default for DeviceScanner {
    fn default() -> Self {
        Self::new()
    }
}
