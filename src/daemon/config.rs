//! YAML-backed daemon configuration. Loaded from `turret.yaml` on
//! startup; falls back to [`DaemonConfig::default`] if the file is
//! missing or malformed.

use serde::{Deserialize, Serialize};
use std::net::{IpAddr, Ipv4Addr};
use std::path::Path;

/// Default `bind_addr` if the YAML omits it: `127.0.0.1` (loopback only).
///
/// Embedded products inherit this default — companion-on-network setups
/// (autopilot on a different host than the gimbal manager, GCS on a
/// laptop talking to a Pi) must opt into a wider bind explicitly with
/// `mavlink.bind_addr: 0.0.0.0` (or a specific interface address) in
/// `turret.yaml`.
///
/// Earlier versions defaulted to `0.0.0.0`. Existing deployments that
/// relied on the broad bind will see the daemon stop responding to
/// off-host MAVLink traffic until the YAML is updated; the change is
/// called out in the CHANGELOG as a breaking config default.
fn default_bind_addr() -> IpAddr {
    IpAddr::V4(Ipv4Addr::LOCALHOST)
}

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
    /// Command arbitration tuning (priority + per-source rate limit).
    #[serde(default)]
    pub arbitrator: ArbitratorConfig,
    /// Gimbal-physical settings (corrector toggles, future mount-offset).
    #[serde(default)]
    pub gimbal: GimbalConfig,
}

/// Gimbal-physical / calibration tuning.
///
/// Currently a single toggle for the continuous yaw drift corrector. As
/// more hardware-side knobs land (mount yaw offset, calibration sweep
/// limits, etc.) they accrete here so `mavlink` and `arbitrator` stay
/// purely about their own concerns.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct GimbalConfig {
    /// Continuous yaw drift corrector ("self-erecting gyro"). When
    /// enabled (the default), the attitude poll loop nudges the
    /// calibration offset toward the most-recent SP once the gimbal
    /// has been settled there for ≥ 2 s. Disable for actively-yawing
    /// flight with pan-mode PAN, where the assumption "the gimbal is
    /// at rest in the world between SETs" no longer holds and the
    /// corrector would slowly walk the calibration off.
    #[serde(default = "default_yaw_corrector_enabled")]
    pub yaw_corrector_enabled: bool,
}

/// Yaw-corrector default = enabled. Pulled out as a free function so the
/// `#[serde(default = "...")]` attribute on the field can reference it
/// (you can't put a literal `true` in `#[serde(default)]` directly).
fn default_yaw_corrector_enabled() -> bool {
    true
}

impl Default for GimbalConfig {
    fn default() -> Self {
        Self {
            yaw_corrector_enabled: default_yaw_corrector_enabled(),
        }
    }
}

/// Command arbitrator tuning.
///
/// Today this carries only the per-source rate limit. Priority itself is
/// hard-coded into [`crate::daemon::models::ControlSource::priority`] —
/// raising sysid-1 (autopilot) above other MAVLink components above IPC
/// above CLI is a policy choice the YAML doesn't override.
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct ArbitratorConfig {
    /// Minimum interval between commands from the *same* `ControlSource`,
    /// in milliseconds. A second command from the same source within this
    /// window is rejected before reaching the gimbal. `0` (the default)
    /// disables the throttle.
    ///
    /// 50 ms (= 20 Hz) is a reasonable starting point for a runaway-GCS
    /// guard without affecting human-driven workflows. Different sources
    /// (autopilot vs IPC vs CLI) are tracked independently.
    #[serde(default)]
    pub min_command_interval_ms: u32,
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

/// Wire transport for the MAVLink link. Only [`Transport::Udp`] is wired
/// up today; serial / TCP transports will land here additively.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Transport {
    /// UDP datagram transport on `<bind_addr>:<udp_port>` — every packet's
    /// source address gets recorded as a peer for subsequent broadcasts.
    #[default]
    Udp,
}

/// MAVLink Gimbal Manager runtime settings.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MavlinkConfig {
    /// Enable the MAVLink Gimbal Manager. When `false`, no UDP socket is bound.
    pub enabled: bool,
    /// Wire transport — see [`Transport`].
    #[serde(default)]
    pub transport: Transport,
    /// Local interface to bind. Defaults to `127.0.0.1` (loopback only)
    /// — safe for embedded products where MAVLink shouldn't be exposed
    /// to the LAN by default. Set to `0.0.0.0` for companion-on-network
    /// deployments (autopilot on a different host, GCS on a laptop), or
    /// to a specific interface address when multi-homed.
    #[serde(default = "default_bind_addr")]
    pub bind_addr: IpAddr,
    /// UDP port (when `transport == Transport::Udp`).
    pub udp_port: u16,
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
                transport: Transport::Udp,
                bind_addr: default_bind_addr(),
                udp_port: 14550,
                sysid: 1,
                compid: 154, // MAV_COMP_ID_GIMBAL
                limits: GimbalLimits::default(),
            },
            logging: LoggingConfig {
                level: "info".to_string(),
            },
            arbitrator: ArbitratorConfig::default(),
            gimbal: GimbalConfig::default(),
        }
    }
}

impl DaemonConfig {
    /// Load configuration from YAML file
    pub fn load_from_file<P: AsRef<Path>>(path: P) -> crate::error::Result<Self> {
        let contents = std::fs::read_to_string(path)?;
        let config: DaemonConfig = serde_yaml_ng::from_str(&contents)?;
        Ok(config)
    }

    /// Save configuration to YAML file
    pub fn save_to_file<P: AsRef<Path>>(&self, path: P) -> crate::error::Result<()> {
        let contents = serde_yaml_ng::to_string(self)?;
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

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;

    /// Minimal YAML used by the corrector-knob tests — contains only the
    /// fields the parser actually requires (no `#[serde(default)]` on
    /// them), so we can exercise the `gimbal` section in isolation.
    fn yaml_without_gimbal_section() -> &'static str {
        r#"
device:
  path: auto
ipc:
  socket_path: /tmp/turret.sock
mavlink:
  enabled: true
  udp_port: 14550
  sysid: 1
  compid: 154
logging:
  level: info
"#
    }

    #[test]
    fn yaw_corrector_enabled_defaults_to_true_when_section_omitted() {
        // Operators upgrading from a pre-knob YAML must keep the
        // existing default-on behavior — no surprise disable.
        let cfg: DaemonConfig = serde_yaml_ng::from_str(yaml_without_gimbal_section()).unwrap();
        assert!(cfg.gimbal.yaw_corrector_enabled);
    }

    #[test]
    fn yaw_corrector_enabled_defaults_to_true_when_section_empty() {
        // A `gimbal: {}` section parses cleanly and inherits the
        // default — covers the case where an operator added the section
        // header for a future field but didn't set the corrector flag.
        let mut yaml = String::from(yaml_without_gimbal_section());
        yaml.push_str("gimbal: {}\n");
        let cfg: DaemonConfig = serde_yaml_ng::from_str(&yaml).unwrap();
        assert!(cfg.gimbal.yaw_corrector_enabled);
    }

    #[test]
    fn yaw_corrector_enabled_can_be_explicitly_disabled() {
        let mut yaml = String::from(yaml_without_gimbal_section());
        yaml.push_str("gimbal:\n  yaw_corrector_enabled: false\n");
        let cfg: DaemonConfig = serde_yaml_ng::from_str(&yaml).unwrap();
        assert!(!cfg.gimbal.yaw_corrector_enabled);
    }

    #[test]
    fn daemon_config_default_enables_corrector() {
        // The non-YAML default path (used by `load_or_default` when the
        // file is missing) must agree with the YAML-default behavior.
        assert!(DaemonConfig::default().gimbal.yaw_corrector_enabled);
    }
}
