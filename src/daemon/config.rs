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
    /// Gimbal device limits advertised in GIMBAL_MANAGER_INFORMATION and
    /// enforced (by clamping) on every inbound command.
    #[serde(default)]
    pub limits: GimbalLimits,
}

/// Hardware angular range advertised by the gimbal manager (degrees).
///
/// Used to populate `GIMBAL_MANAGER_INFORMATION` and to clamp inbound
/// commands so a buggy GCS can't drive the gimbal past what the
/// hardware can actually achieve.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct GimbalLimits {
    /// Minimum pitch (degrees). Default: -90.
    pub pitch_min_deg: f32,
    /// Maximum pitch (degrees). Default: 90.
    pub pitch_max_deg: f32,
    /// Minimum roll (degrees). Default: -180.
    pub roll_min_deg: f32,
    /// Maximum roll (degrees). Default: 180.
    pub roll_max_deg: f32,
    /// Minimum yaw (degrees). Default: -180.
    pub yaw_min_deg: f32,
    /// Maximum yaw (degrees). Default: 180.
    pub yaw_max_deg: f32,
}

impl Default for GimbalLimits {
    fn default() -> Self {
        Self {
            pitch_min_deg: -90.0,
            pitch_max_deg: 90.0,
            roll_min_deg: -180.0,
            roll_max_deg: 180.0,
            yaw_min_deg: -180.0,
            yaw_max_deg: 180.0,
        }
    }
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
                limits: GimbalLimits::default(),
            },
            logging: LoggingConfig {
                level: "info".to_string(),
            },
        }
    }
}

impl DaemonConfig {
    /// Load configuration from YAML file
    pub fn load_from_file<P: AsRef<Path>>(path: P) -> crate::error::Result<Self> {
        let contents = std::fs::read_to_string(path)?;
        let config: DaemonConfig = serde_yaml::from_str(&contents)?;
        Ok(config)
    }

    /// Save configuration to YAML file
    pub fn save_to_file<P: AsRef<Path>>(&self, path: P) -> crate::error::Result<()> {
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
