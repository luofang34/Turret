//! YAML-backed daemon configuration. Loaded from `turret.yaml` on
//! startup; falls back to [`DaemonConfig::default`] if the file is
//! missing or malformed.

use serde::{Deserialize, Serialize};
use std::path::Path;

/// Top-level daemon config shape — one section per long-lived component.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DaemonConfig {
    /// Serial device selection.
    pub device: DeviceConfig,
    /// Unix-socket IPC server settings.
    pub ipc: IpcConfig,
    /// MAVLink Gimbal Manager settings.
    pub mavlink: MavlinkConfig,
    /// Tracing subscriber settings.
    pub logging: LoggingConfig,
}

/// Where to find the gimbal hardware.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeviceConfig {
    /// Device path (e.g., "/dev/ttyACM0") or `"auto"` to scan USB CDC.
    pub path: String,
}

/// Local IPC server settings.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IpcConfig {
    /// Unix socket path the IPC server binds to (and `unlink(2)`s on shutdown).
    pub socket_path: String,
}

/// MAVLink Gimbal Manager runtime settings.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MavlinkConfig {
    /// Enable the MAVLink Gimbal Manager. When `false`, no UDP socket is bound.
    pub enabled: bool,
    /// Transport type. Currently only `"udp"` is implemented.
    pub transport: String,
    /// UDP port (when `transport == "udp"`).
    pub udp_port: u16,
    /// Serial device for the eventual serial MAVLink transport.
    #[serde(default)]
    pub serial_device: Option<String>,
    /// MAVLink system ID this manager advertises in HEARTBEAT.
    pub sysid: u8,
    /// MAVLink component ID — typically 154 (`MAV_COMP_ID_GIMBAL`).
    pub compid: u8,
    /// Gimbal device limits advertised in `GIMBAL_MANAGER_INFORMATION` and
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

/// Tracing subscriber knob. The CLI's `-v / -vv / -vvv` flag overrides
/// this for command-line usage; the daemon honors only this field.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LoggingConfig {
    /// Log level — `"trace" / "debug" / "info" / "warn" / "error"`.
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
                tracing::warn!("Failed to load config file: {}. Using defaults.", e);
                Self::default()
            }
        }
    }
}
