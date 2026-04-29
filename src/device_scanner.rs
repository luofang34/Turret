//! Auto-detect a connected STorM32 gimbal by USB CDC vendor / product id.
//!
//! Two stages, in order:
//! 1. `serialport::available_ports()` — works on every platform.
//! 2. Platform-specific fallback that walks `/dev` (Unix) or probes
//!    `COM1..=COM20` (Windows) when the enumeration above doesn't carry
//!    USB metadata.

use serialport::{SerialPortInfo, SerialPortType};
use std::fs;
use tracing::debug;

/// Scanner that locates a connected STorM32 USB CDC device.
pub struct DeviceScanner;

impl DeviceScanner {
    /// Construct a new scanner. The scanner itself is stateless.
    pub fn new() -> Self {
        Self
    }

    /// Path of the first STorM32 device found, or `None` if none is connected.
    pub fn find_storm32_device(&self) -> Option<String> {
        if let Some(path) = self.find_storm32_via_serialport() {
            return Some(path);
        }
        self.find_storm32_via_system_info()
    }

    /// Every serial port the OS reports — for diagnostic listing only.
    pub fn list_all_devices(&self) -> Vec<SerialPortInfo> {
        serialport::available_ports().unwrap_or_default()
    }

    fn find_storm32_via_serialport(&self) -> Option<String> {
        match serialport::available_ports() {
            Ok(ports) => ports
                .into_iter()
                .find(Self::is_storm32_device)
                .map(|p| p.port_name),
            Err(e) => {
                debug!("Error enumerating serial ports: {}", e);
                None
            }
        }
    }

    fn is_storm32_device(port_info: &SerialPortInfo) -> bool {
        match &port_info.port_type {
            SerialPortType::UsbPort(usb_info) => {
                // STMicroelectronics VID 0x0483 + STM32 Virtual COM Port PIDs
                // 0x5740 / 0x5840.
                let storm32_pids = [0x5740u16, 0x5840u16];
                usb_info.vid == 0x0483 && storm32_pids.contains(&usb_info.pid)
            }
            _ => false,
        }
    }

    #[cfg(target_os = "macos")]
    fn find_storm32_via_system_info(&self) -> Option<String> {
        // No reliable per-device VID/PID readback on macOS without
        // system_profiler — fall back to first matching tty.usbmodem.
        let devices = self.glob_devices("/dev/tty.usbmodem*").ok()?;
        devices.into_iter().find(|d| Self::test_device_exists(d))
    }

    #[cfg(target_os = "linux")]
    fn find_storm32_via_system_info(&self) -> Option<String> {
        for pattern in ["/dev/ttyACM*", "/dev/ttyUSB*"] {
            if let Ok(devices) = self.glob_devices(pattern) {
                for device in &devices {
                    if Self::verify_device_linux(device) {
                        return Some(device.clone());
                    }
                }
            }
        }
        None
    }

    #[cfg(target_os = "windows")]
    fn find_storm32_via_system_info(&self) -> Option<String> {
        (1..=20)
            .map(|i| format!("COM{}", i))
            .find(|name| Self::test_device_exists(name))
    }

    #[cfg(not(any(target_os = "macos", target_os = "linux", target_os = "windows")))]
    fn find_storm32_via_system_info(&self) -> Option<String> {
        None
    }

    fn glob_devices(&self, pattern: &str) -> Result<Vec<String>, std::io::Error> {
        let mut devices = Vec::new();

        if let Some(prefix) = pattern.strip_suffix('*') {
            if let Some(dir) = std::path::Path::new(prefix).parent() {
                if let Ok(entries) = fs::read_dir(dir) {
                    for entry in entries.flatten() {
                        if let Some(s) = entry.path().to_str() {
                            if s.starts_with(prefix) {
                                devices.push(s.to_string());
                            }
                        }
                    }
                }
            }
        } else if std::path::Path::new(pattern).exists() {
            devices.push(pattern.to_string());
        }

        devices.sort();
        Ok(devices)
    }

    fn test_device_exists(device_path: &str) -> bool {
        std::path::Path::new(device_path).exists()
    }

    #[cfg(target_os = "linux")]
    fn verify_device_linux(device_path: &str) -> bool {
        if !Self::test_device_exists(device_path) {
            return false;
        }

        // sysfs check — confirms STMicroelectronics + STM32 VCP PID before
        // committing the user to a wrong device. If sysfs lookup fails the
        // device might still be the right one, so accept on uncertainty.
        if let Some(usb_path) = Self::get_usb_sysfs_path(device_path) {
            if let (Ok(vendor), Ok(product)) = (
                fs::read_to_string(format!("{}/idVendor", usb_path)),
                fs::read_to_string(format!("{}/idProduct", usb_path)),
            ) {
                return vendor.trim() == "0483"
                    && (product.trim() == "5740" || product.trim() == "5840");
            }
        }
        true
    }

    #[cfg(target_os = "linux")]
    fn get_usb_sysfs_path(device_path: &str) -> Option<String> {
        let device_name = std::path::Path::new(device_path).file_name()?.to_str()?;
        let sys_path = format!("/sys/class/tty/{}/device", device_name);
        if !std::path::Path::new(&sys_path).exists() {
            return None;
        }
        let real_path = fs::read_link(&sys_path).ok()?;
        let real_str = real_path.to_str()?;
        let parts: Vec<&str> = real_str.split('/').collect();
        for (i, part) in parts.iter().enumerate() {
            // "1-1.4:1.0" — USB device tag. Walk up one level to land on
            // the device node carrying idVendor / idProduct.
            if part.contains(':') && part.contains('.') && i > 0 {
                let usb_device_path = parts[..i].join("/");
                let full = format!("/sys{}", usb_device_path);
                if std::path::Path::new(&full).exists() {
                    return Some(full);
                }
            }
        }
        None
    }
}

impl Default for DeviceScanner {
    fn default() -> Self {
        Self::new()
    }
}
