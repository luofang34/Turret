//! Domain types shared by the daemon's command path: MAVLink / IPC inbound,
//! arbitrator, state manager.

use serde::{Deserialize, Serialize};
use std::time::SystemTime;

/// Pan-mode bytes that the STorM32 RC `CMD_SETPANMODE` command takes
/// (0..=5). Each name describes the per-axis policy as `pitch | roll | yaw`,
/// where `Pan` follows vehicle yaw and `Hold` locks to earth frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PanMode {
    /// Pan mode disabled — STorM32 returns to its configured default.
    Off = 0,
    /// Pitch held, roll held, yaw pans with vehicle.
    HoldHoldPan = 1,
    /// All three axes hold (earth frame).
    HoldHoldHold = 2,
    /// All three axes pan with vehicle.
    PanPanPan = 3,
    /// Pitch pans, roll holds, yaw holds.
    PanHoldHold = 4,
    /// Pitch pans, roll holds, yaw pans.
    PanHoldPan = 5,
}

impl PanMode {
    /// Decode the wire-format byte (0..=5). Returns `None` for any other
    /// value so callers can reject invalid pan-mode requests at the boundary.
    pub fn from_u8(value: u8) -> Option<Self> {
        match value {
            0 => Some(PanMode::Off),
            1 => Some(PanMode::HoldHoldPan),
            2 => Some(PanMode::HoldHoldHold),
            3 => Some(PanMode::PanPanPan),
            4 => Some(PanMode::PanHoldHold),
            5 => Some(PanMode::PanHoldPan),
            _ => None,
        }
    }

    /// Short uppercase name used in IPC `Status` JSON. Matches the
    /// `CMD_SETPANMODE` constant names from the STorM32 RC spec.
    pub fn to_name(&self) -> &'static str {
        match self {
            PanMode::Off => "OFF",
            PanMode::HoldHoldPan => "HOLDHOLDPAN",
            PanMode::HoldHoldHold => "HOLDHOLDHOLD",
            PanMode::PanPanPan => "PANPANPAN",
            PanMode::PanHoldHold => "PANHOLDHOLD",
            PanMode::PanHoldPan => "PANHOLDPAN",
        }
    }
}

/// Where a gimbal command came from. Used by [`ControlSource::priority`] to
/// decide whether a newly arrived command preempts the in-flight one, and
/// hashed by the per-source rate limiter in `StateManager`.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ControlSource {
    /// IPC client connected to the Unix socket. The `u32` is an
    /// IPC-server-assigned client id, monotonic per daemon run.
    UnixSocket(u32),
    /// MAVLink message originating from sysid `u8`. Sysid 1 is treated as
    /// the autopilot and gets the highest priority.
    Mavlink(u8),
    /// CLI subcommand running on the same host (e.g. `turret set ...`).
    Cli,
}

impl ControlSource {
    /// Priority level used by the arbitrator — a strictly higher number
    /// preempts a strictly lower one. Same-priority sources are ordered by
    /// timestamp ("last writer wins").
    pub fn priority(&self) -> u8 {
        match self {
            ControlSource::Mavlink(1) => 100,
            ControlSource::Mavlink(_) => 50,
            ControlSource::UnixSocket(_) => 20,
            ControlSource::Cli => 10,
        }
    }
}

/// What the command body's three axis values mean. Today only
/// [`CommandMode::Position`] is fully wired through — see the
/// `Arbitrator::process_command` `Rate`/`Manual` branches for the gaps.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum CommandMode {
    /// Absolute angle in degrees per axis.
    Position,
    /// Angular velocity in degrees per second.
    Rate,
    /// Normalized stick-like input on `[-1.0, 1.0]` — handlers scale this
    /// to ±90° before clamping to configured device limits.
    Manual,
}

/// MAVLink primary / secondary control ownership.
///
/// Set via `MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE` (param1..param4) and
/// re-broadcast in every `GIMBAL_MANAGER_STATUS`. A value of `0` means
/// "unset" — when primary is unset, commands from any sysid/compid are
/// accepted.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct PrimaryControl {
    /// Primary controller system ID (0 = unset).
    pub primary_sysid: u8,
    /// Primary controller component ID (0 = unset).
    pub primary_compid: u8,
    /// Secondary controller system ID (0 = unset).
    pub secondary_sysid: u8,
    /// Secondary controller component ID (0 = unset).
    pub secondary_compid: u8,
}

impl PrimaryControl {
    /// True if `(sysid, compid)` is allowed to drive the gimbal.
    ///
    /// If primary is unset (both zero), everyone is allowed. Otherwise only
    /// an exact match is accepted.
    pub fn is_allowed(&self, sysid: u8, compid: u8) -> bool {
        if self.primary_sysid == 0 && self.primary_compid == 0 {
            return true;
        }
        self.primary_sysid == sysid && self.primary_compid == compid
    }
}

/// One inbound command on its way through the arbitrator. `yaw / pitch /
/// roll` are `Option<f32>` so a partial command can leave an axis
/// untouched, which the IPC `pitch`/`roll`/`yaw` subcommands take advantage
/// of.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GimbalCommand {
    /// Where this command came from. Drives priority arbitration.
    pub source: ControlSource,
    /// Whether `yaw / pitch / roll` are angles, rates, or normalized inputs.
    pub mode: CommandMode,
    /// Yaw axis — `None` means "leave alone".
    pub yaw: Option<f32>,
    /// Pitch axis — `None` means "leave alone".
    pub pitch: Option<f32>,
    /// Roll axis — `None` means "leave alone".
    pub roll: Option<f32>,
    /// When this command was constructed. Used for tie-breaking
    /// same-priority sources and for staleness checks.
    pub timestamp: SystemTime,
}

impl GimbalCommand {
    /// Construct a command stamped with the current time.
    pub fn new(
        source: ControlSource,
        mode: CommandMode,
        yaw: Option<f32>,
        pitch: Option<f32>,
        roll: Option<f32>,
    ) -> Self {
        Self {
            source,
            mode,
            yaw,
            pitch,
            roll,
            timestamp: SystemTime::now(),
        }
    }

    /// True if the command is older than 5 seconds — at which point the
    /// arbitrator allows a lower-priority source to take over.
    pub fn is_stale(&self) -> bool {
        if let Ok(elapsed) = self.timestamp.elapsed() {
            elapsed.as_secs() > 5
        } else {
            true
        }
    }
}

/// Current gimbal state.
///
/// `yaw / pitch / roll` reflect the **measured** attitude polled from the
/// hardware. They are written exclusively by the attitude poll loop in
/// `mavlink_manager`; the arbitrator no longer copies commanded angles into
/// these fields. `measured_at` distinguishes "no successful poll yet"
/// (`None`, fields are still at their `Default::default()` zero) from
/// "we measured this at time T" (`Some(t)`).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GimbalState {
    /// Last measured yaw angle, in degrees. Meaningful only if
    /// `measured_at.is_some()`.
    pub yaw: f32,
    /// Last measured pitch angle, in degrees. See [`Self::yaw`].
    pub pitch: f32,
    /// Last measured roll angle, in degrees. See [`Self::yaw`].
    pub roll: f32,
    /// `Some(t)` once we've successfully read attitude from the device at
    /// least once; `None` until then.
    pub measured_at: Option<SystemTime>,
    /// Most recent pan-mode the gimbal was configured into.
    pub pan_mode: PanMode,
    /// Whether `CMD_SETSTANDBY 1` has been sent and not yet undone.
    pub standby: bool,
    /// Decoded `CMD_GETVERSION` string captured at startup, if successful.
    pub firmware_version: Option<String>,
    /// Wallclock of the last write to any field above.
    pub last_update: SystemTime,
}

impl Default for GimbalState {
    fn default() -> Self {
        Self {
            yaw: 0.0,
            pitch: 0.0,
            roll: 0.0,
            measured_at: None,
            pan_mode: PanMode::Off,
            standby: false,
            firmware_version: None,
            last_update: SystemTime::now(),
        }
    }
}

impl GimbalState {
    /// Record a fresh attitude measurement read off the gimbal hardware.
    pub fn update_measured_angles(&mut self, yaw: f32, pitch: f32, roll: f32) {
        self.yaw = yaw;
        self.pitch = pitch;
        self.roll = roll;
        let now = SystemTime::now();
        self.measured_at = Some(now);
        self.last_update = now;
    }

    /// Update the recorded pan mode after a successful `CMD_SETPANMODE`.
    pub fn update_pan_mode(&mut self, mode: PanMode) {
        self.pan_mode = mode;
        self.last_update = SystemTime::now();
    }

    /// Update the recorded standby flag after a successful `CMD_SETSTANDBY`.
    pub fn update_standby(&mut self, standby: bool) {
        self.standby = standby;
        self.last_update = SystemTime::now();
    }
}

/// Wire-format command from an IPC client.
///
/// Serde produces JSON of the form
/// `{"cmd": "set", "pitch": 10.0, "roll": 0.0, "yaw": 0.0}`. Each variant
/// maps to a single corresponding handler in `IpcServer::process_command`.
///
/// ## Naming convention
///
/// Both the discriminator (`cmd`) and field names use `snake_case`,
/// matching every other JSON surface the daemon emits (`firmware_version`,
/// `pan_mode`, `time_boot_ms`, ...). `rename_all = "snake_case"` on the
/// enum gives multi-word variants the right wire form (`PanMode` →
/// `"pan_mode"`) without per-variant overrides; field names are spelled
/// directly.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "cmd", rename_all = "snake_case")]
pub enum IpcCommand {
    /// Move all three axes to the given absolute angles in degrees.
    Set {
        /// Target pitch angle (degrees).
        pitch: f32,
        /// Target roll angle (degrees).
        roll: f32,
        /// Target yaw angle (degrees).
        yaw: f32,
    },
    /// Move all three axes to zero.
    Center,
    /// Read the cached gimbal state — does NOT poll the device.
    Status,
    /// Decode the gimbal's firmware version string.
    Version,
    /// Set the pan-mode byte. See [`PanMode`] for the encoding.
    PanMode {
        /// Pan-mode byte 0..=5; rejected otherwise.
        mode: u8,
    },
    /// Toggle standby (`true` = standby/motors disengaged, `false` = active).
    Standby {
        /// New standby state.
        enabled: bool,
    },
    /// Return a JSON description of all available commands.
    Help,
}

/// Wire-format response sent back over the IPC socket. JSON form is
/// `{"status": "ok", "data": ...}` or `{"status": "error", "message": ...}`.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "status", rename_all = "lowercase")]
pub enum IpcResponse {
    /// Successful command execution. `data` carries any payload the
    /// handler wanted to return (e.g. `Status` returns the gimbal state).
    Ok {
        /// Optional payload returned alongside the OK status.
        #[serde(skip_serializing_if = "Option::is_none")]
        data: Option<serde_json::Value>,
    },
    /// Failed command execution.
    Error {
        /// Human-readable failure reason.
        message: String,
    },
}

impl IpcResponse {
    /// Construct an OK response with no payload.
    pub fn ok() -> Self {
        IpcResponse::Ok { data: None }
    }

    /// Construct an OK response carrying a JSON payload.
    pub fn ok_with_data(data: serde_json::Value) -> Self {
        IpcResponse::Ok { data: Some(data) }
    }

    /// Construct an error response with a human-readable message.
    pub fn error(message: impl Into<String>) -> Self {
        IpcResponse::Error {
            message: message.into(),
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    //! Wire-format lockdowns for the IPC schema. These tests exist to
    //! catch accidental renames — the daemon's JSON surface is a public
    //! contract, and a typo in a `#[serde(rename_all = ...)]` attribute
    //! would silently break every existing client.

    use super::*;

    #[test]
    fn deserialize_set_command() {
        let cmd: IpcCommand =
            serde_json::from_str(r#"{"cmd":"set","pitch":10.0,"roll":-2.5,"yaw":42.0}"#).unwrap();
        match cmd {
            IpcCommand::Set { pitch, roll, yaw } => {
                assert!((pitch - 10.0).abs() < 1e-6);
                assert!((roll + 2.5).abs() < 1e-6);
                assert!((yaw - 42.0).abs() < 1e-6);
            }
            other => panic!("expected Set, got {other:?}"),
        }
    }

    #[test]
    fn deserialize_pan_mode_uses_snake_case_discriminator() {
        // Wire form is `pan_mode`, not `pan-mode` (kebab) or `panmode`
        // (the `rename_all = "lowercase"` would-be default).
        let cmd: IpcCommand =
            serde_json::from_str(r#"{"cmd":"pan_mode","mode":3}"#).unwrap();
        match cmd {
            IpcCommand::PanMode { mode } => assert_eq!(mode, 3),
            other => panic!("expected PanMode, got {other:?}"),
        }
    }

    #[test]
    fn deserialize_standby_uses_enabled_field() {
        // Wire form is `enabled` (boolean state), not `enable` (verb).
        let cmd: IpcCommand =
            serde_json::from_str(r#"{"cmd":"standby","enabled":true}"#).unwrap();
        match cmd {
            IpcCommand::Standby { enabled } => assert!(enabled),
            other => panic!("expected Standby, got {other:?}"),
        }
    }

    #[test]
    fn old_kebab_pan_mode_is_rejected() {
        // The kebab-case form was the previous wire shape; rejecting it
        // outright (rather than silently mis-parsing) helps clients
        // discover the rename quickly.
        let result: Result<IpcCommand, _> =
            serde_json::from_str(r#"{"cmd":"pan-mode","mode":3}"#);
        assert!(
            result.is_err(),
            "kebab-case pan-mode must not parse (got {result:?})"
        );
    }

    #[test]
    fn old_enable_field_is_rejected() {
        let result: Result<IpcCommand, _> =
            serde_json::from_str(r#"{"cmd":"standby","enable":true}"#);
        assert!(
            result.is_err(),
            "old `enable` field must not parse (got {result:?})"
        );
    }

    #[test]
    fn serialize_pan_mode_round_trips_to_snake_case() {
        let cmd = IpcCommand::PanMode { mode: 2 };
        let s = serde_json::to_string(&cmd).unwrap();
        assert!(
            s.contains(r#""cmd":"pan_mode""#),
            "expected snake_case discriminator, got {s}"
        );
        assert!(s.contains(r#""mode":2"#), "got {s}");
    }

    #[test]
    fn serialize_standby_round_trips_to_enabled_field() {
        let cmd = IpcCommand::Standby { enabled: false };
        let s = serde_json::to_string(&cmd).unwrap();
        assert!(s.contains(r#""cmd":"standby""#), "got {s}");
        assert!(
            s.contains(r#""enabled":false"#),
            "expected `enabled`, got {s}"
        );
    }
}
