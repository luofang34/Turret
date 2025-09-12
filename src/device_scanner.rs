use serialport::{SerialPortInfo, SerialPortType};
use std::process::Command;

pub struct DeviceScanner;

impl DeviceScanner {
    pub fn new() -> Self {
        Self
    }
    
    pub fn find_storm32_device(&self) -> Option<String> {
        // First try serialport enumeration
        if let Some(path) = self.find_via_serialport() {
            return Some(path);
        }
        
        // Fallback to system-specific methods
        self.find_via_system_info()
    }
    
    fn find_via_serialport(&self) -> Option<String> {
        match serialport::available_ports() {
            Ok(ports) => {
                for port in ports {
                    if self.is_storm32_device(&port) {
                        return Some(port.port_name);
                    }
                }
            }
            Err(e) => {
                eprintln!("Error enumerating serial ports: {}", e);
            }
        }
        None
    }
    
    fn is_storm32_device(&self, port_info: &SerialPortInfo) -> bool {
        match &port_info.port_type {
            SerialPortType::UsbPort(usb_info) => {
                // Check for STM32 Virtual COM Port (STMicroelectronics)
                // VID: 0x0483 (1155), PID: 0x5740 (22336)
                usb_info.vid == 1155 && usb_info.pid == 22336
            }
            _ => false,
        }
    }
    
    #[cfg(target_os = "macos")]
    fn find_via_system_info(&self) -> Option<String> {
        // Use system_profiler to get USB device info
        let output = Command::new("system_profiler")
            .args(&["SPUSBDataType", "-xml"])
            .output()
            .ok()?;
        
        let output_str = String::from_utf8_lossy(&output.stdout);
        
        // Look for STM32 Virtual COM Port in system profiler output
        if output_str.contains("STM32 Virtual COM Port") {
            // Look for corresponding tty device
            let ls_output = Command::new("ls")
                .args(&["/dev/tty.usbmodem*"])
                .output()
                .ok()?;
            
            let devices = String::from_utf8_lossy(&ls_output.stdout);
            for device in devices.lines() {
                if !device.is_empty() {
                    return Some(device.to_string());
                }
            }
        }
        
        None
    }
    
    #[cfg(target_os = "linux")]
    fn find_via_system_info(&self) -> Option<String> {
        // Check /dev/ttyACM* and /dev/ttyUSB* devices
        let patterns = ["/dev/ttyACM*", "/dev/ttyUSB*"];
        
        for pattern in &patterns {
            if let Ok(output) = Command::new("ls").arg(pattern).output() {
                let devices = String::from_utf8_lossy(&output.stdout);
                for device in devices.lines() {
                    if !device.is_empty() {
                        // Additional verification could be added here
                        // For now, return the first device found
                        return Some(device.to_string());
                    }
                }
            }
        }
        
        None
    }
    
    #[cfg(not(any(target_os = "macos", target_os = "linux")))]
    fn find_via_system_info(&self) -> Option<String> {
        // Fallback for other platforms
        None
    }
    
    pub fn list_all_devices(&self) -> Vec<SerialPortInfo> {
        serialport::available_ports().unwrap_or_default()
    }
}