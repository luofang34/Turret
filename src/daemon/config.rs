use serde::{Deserialize, Serialize};
use std::path::Path;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DaemonConfig {
    pub device: DeviceConfig,
    pub ipc: IpcConfig,
    pub mavlink: MavlinkConfig,
    pub logging: LoggingConfig,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeviceConfig {
    /// Device path (e.g., "/dev/ttyACM0") or "auto" for auto-detection
    pub path: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IpcConfig {
    /// Unix socket path for IPC
    pub socket_path: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MavlinkConfig {
    /// Enable MAVLink protocol
    pub enabled: bool,
    /// Transport type: "udp" or "serial"
    pub transport: String,
    /// UDP port (if transport is "udp")
    pub udp_port: u16,
    /// Serial device (if transport is "serial")
    #[serde(default)]
    pub serial_device: Option<String>,
    /// MAVLink system ID
    pub sysid: u8,
    /// MAVLink component ID (154 = MAV_COMP_ID_GIMBAL)
    pub compid: u8,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LoggingConfig {
    /// Log level: "trace", "debug", "info", "warn", "error"
    pub level: String,
}

impl Default for DaemonConfig {
    fn default() -> Self {
        Self {
            device: DeviceConfig {
                path: "auto".to_string(),
            },
            ipc: IpcConfig {
                socket_path: "/tmp/turret.sock".to_string(),
            },
            mavlink: MavlinkConfig {
                enabled: true,
                transport: "udp".to_string(),
                udp_port: 14550,
                serial_device: None,
                sysid: 1,
                compid: 154, // MAV_COMP_ID_GIMBAL
            },
            logging: LoggingConfig {
                level: "info".to_string(),
            },
        }
    }
}

impl DaemonConfig {
    /// Load configuration from YAML file
    pub fn load_from_file<P: AsRef<Path>>(path: P) -> anyhow::Result<Self> {
        let contents = std::fs::read_to_string(path)?;
        let config: DaemonConfig = serde_yaml::from_str(&contents)?;
        Ok(config)
    }

    /// Save configuration to YAML file
    pub fn save_to_file<P: AsRef<Path>>(&self, path: P) -> anyhow::Result<()> {
        let contents = serde_yaml::to_string(self)?;
        std::fs::write(path, contents)?;
        Ok(())
    }

    /// Load configuration from file if it exists, otherwise use defaults
    pub fn load_or_default<P: AsRef<Path>>(path: P) -> Self {
        match Self::load_from_file(path) {
            Ok(config) => config,
            Err(e) => {
                log::warn!("Failed to load config file: {}. Using defaults.", e);
                Self::default()
            }
        }
    }
}
