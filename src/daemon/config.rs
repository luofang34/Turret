//! YAML-backed daemon configuration. Loaded from `turret.yaml` on
//! startup via [`DaemonConfig::load_for_startup`]: a missing *default*
//! file produces built-in defaults; an explicit `--config` path or any
//! parse / I/O error fails loud rather than silently falling back.

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
///
/// There is no `logging` section: tracing-subscriber is set up before the
/// daemon loads YAML (the CLI's `-v / -vv / -vvv` controls the default
/// floor, `RUST_LOG` tunes per-target filters). A YAML knob would change
/// behavior only after the subscriber has already been installed, which
/// silently does nothing — see commit history if reconsidering.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DaemonConfig {
    /// Serial device selection.
    pub device: DeviceConfig,
    /// Unix-socket IPC server settings.
    pub ipc: IpcConfig,
    /// MAVLink Gimbal Manager settings.
    pub mavlink: MavlinkConfig,
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
            arbitrator: ArbitratorConfig::default(),
            gimbal: GimbalConfig::default(),
        }
    }
}

impl DaemonConfig {
    /// Load configuration from YAML file.
    pub fn load_from_file<P: AsRef<Path>>(path: P) -> crate::error::Result<Self> {
        let contents = std::fs::read_to_string(path)?;
        let config: DaemonConfig = serde_yaml_ng::from_str(&contents)?;
        Ok(config)
    }

    /// Save configuration to YAML file.
    pub fn save_to_file<P: AsRef<Path>>(&self, path: P) -> crate::error::Result<()> {
        let contents = serde_yaml_ng::to_string(self)?;
        std::fs::write(path, contents)?;
        Ok(())
    }

    /// Load configuration for daemon startup.
    ///
    /// Semantics:
    /// - If `explicit_path` is `Some(p)` the path was supplied via
    ///   `--config`. A missing file is an error — the operator asked
    ///   for *this* file, not the default location.
    /// - If `explicit_path` is `None` the daemon falls back to the
    ///   conventional `turret.yaml` in the working directory. A missing
    ///   file silently produces defaults; this is the normal first-run
    ///   path.
    ///
    /// Parse errors (malformed YAML, type mismatch) and other I/O
    /// errors (permission denied, etc.) always propagate. A typo in
    /// `turret.yaml` must never silently degrade to defaults — that
    /// risk-masks `device: auto` and a wide-open IPC socket the
    /// operator didn't ask for.
    pub fn load_for_startup(explicit_path: Option<&str>) -> crate::error::Result<Self> {
        const DEFAULT_PATH: &str = "turret.yaml";
        let (path, explicit) = match explicit_path {
            Some(p) => (p, true),
            None => (DEFAULT_PATH, false),
        };
        match Self::load_from_file(path) {
            Ok(config) => Ok(config),
            Err(crate::error::Error::Io(e))
                if e.kind() == std::io::ErrorKind::NotFound && !explicit =>
            {
                tracing::info!("No config file at {}; using built-in defaults", path);
                Ok(Self::default())
            }
            Err(e) => Err(e),
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
        // The non-YAML default path (used by `load_for_startup` when
        // the default `turret.yaml` is absent) must agree with the
        // YAML-default behavior.
        assert!(DaemonConfig::default().gimbal.yaw_corrector_enabled);
    }

    /// Build a temp directory we can `cd` to without races between
    /// parallel tests poking at `turret.yaml` in the package root.
    fn temp_workdir() -> std::path::PathBuf {
        let pid = std::process::id();
        let n = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_nanos())
            .unwrap_or(0);
        let p = std::env::temp_dir().join(format!("turret-cfg-test-{pid}-{n}"));
        std::fs::create_dir_all(&p).unwrap();
        p
    }

    #[test]
    fn load_for_startup_explicit_missing_file_errors() {
        // `--config /no/such/path.yaml` must fail loudly. The
        // operator asked for *that* file; substituting defaults
        // would be a silent surprise.
        let dir = temp_workdir();
        let missing = dir.join("nope.yaml");
        let r = DaemonConfig::load_for_startup(Some(missing.to_str().unwrap()));
        assert!(r.is_err(), "explicit missing file must error, got Ok");
    }

    #[test]
    fn load_for_startup_default_missing_file_uses_defaults() {
        // No `--config`, no `turret.yaml` in CWD: built-in defaults
        // is the right behavior (first run on a fresh install).
        let dir = temp_workdir();
        let prev = std::env::current_dir().unwrap();
        // `set_current_dir` is process-wide; we restore in a deferred
        // step so a panic doesn't leave the test runner in /tmp.
        std::env::set_current_dir(&dir).unwrap();
        let r = DaemonConfig::load_for_startup(None);
        std::env::set_current_dir(prev).unwrap();
        std::fs::remove_dir_all(&dir).ok();
        let cfg = r.expect("missing default file must produce defaults, not error");
        assert_eq!(cfg.device.path, "auto");
    }

    #[test]
    fn load_for_startup_malformed_yaml_errors() {
        // A typo in `turret.yaml` must NOT silently degrade to
        // defaults — that risk-masks `device: auto` and the default
        // socket the operator didn't pick. Same shape regardless of
        // whether the path was explicit.
        let dir = temp_workdir();
        let path = dir.join("bad.yaml");
        std::fs::write(&path, b"device: { path:\nipc:\n  socket_path: x\n").unwrap();
        let r = DaemonConfig::load_for_startup(Some(path.to_str().unwrap()));
        std::fs::remove_dir_all(&dir).ok();
        assert!(r.is_err(), "malformed YAML must error, got Ok");
    }
}
