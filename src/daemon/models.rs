//! Domain types shared by the daemon's command path: MAVLink / IPC inbound,
//! arbitrator, state manager.

use serde::{Deserialize, Serialize};
use std::time::{Duration, Instant, SystemTime};

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
///
/// There is no `Cli` variant: the `turret` binary's subcommands route
/// either through the daemon's IPC socket (becoming `UnixSocket(_)` from
/// the daemon's perspective) or directly to the device when no daemon
/// is reachable (no arbitration involved at all).
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ControlSource {
    /// IPC client connected to the Unix socket. The `u32` is an
    /// IPC-server-assigned client id, monotonic per daemon run.
    UnixSocket(u32),
    /// MAVLink source identity; the number does not confer authority.
    Mavlink(u8),
}

impl ControlSource {
    /// Priority level used by the arbitrator — a strictly higher number
    /// preempts a strictly lower one. Same-priority sources are ordered by
    /// timestamp ("last writer wins").
    pub fn priority(&self) -> u8 {
        1
    }
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

/// One inbound operation with optional axes and an immutable local arrival time.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GimbalCommand {
    /// Operation semantics; angles are never silently interpreted as rates.
    #[serde(default)]
    pub operation: CommandOperation,
    /// Local ingress instant; only this process may create execution deadlines.
    #[serde(skip, default = "Instant::now")]
    pub received_at: Instant,
    /// Where this command came from. Drives priority arbitration.
    pub source: ControlSource,
    /// Pitch axis — `None` means "leave alone".
    pub pitch: Option<f32>,
    /// Roll axis — `None` means "leave alone".
    pub roll: Option<f32>,
    /// Yaw axis — `None` means "leave alone".
    pub yaw: Option<f32>,
    /// When this command was constructed. Used for tie-breaking
    /// same-priority sources and for staleness checks.
    pub timestamp: SystemTime,
}

/// Payload operation admitted by the host authority boundary.
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub enum CommandOperation {
    /// Absolute selected-axis positions, expressed in degrees.
    #[default]
    Position,
    /// Selected-axis angular rates, expressed in degrees per second.
    Rate,
    /// The device's explicitly advertised recenter operation.
    Recenter,
}

impl GimbalCommand {
    /// Construct a selected-axis position command stamped with local arrival time.
    pub fn new(
        source: ControlSource,
        pitch: Option<f32>,
        roll: Option<f32>,
        yaw: Option<f32>,
    ) -> Self {
        Self {
            operation: CommandOperation::Position,
            received_at: Instant::now(),
            source,
            pitch,
            roll,
            yaw,
            timestamp: SystemTime::now(),
        }
    }

    /// Derive the ingress policy from the preserved arrival time without renewing it.
    pub fn timing(&self) -> crate::Result<super::gimbal_handle::HostCommandTiming> {
        Ok(super::gimbal_handle::HostCommandTiming::local_ingress(
            self.payload()?,
            self.received_at,
        ))
    }

    /// Original local deadline, preserved while waiting for queues and device I/O.
    pub fn send_before(&self) -> Instant {
        self.received_at + Duration::from_millis(100)
    }

    /// Whether the local write deadline has elapsed.
    pub fn is_stale(&self) -> bool {
        Instant::now() >= self.send_before()
    }

    /// Convert one admitted operation into finite, typed payload intent.
    pub fn payload(&self) -> crate::Result<turret_core::PayloadCommand> {
        use turret_core::*;
        let frame = ReferenceFrame::Vehicle;
        match self.operation {
            CommandOperation::Position => {
                let angle = |v: Option<f32>| v.map(|v| Angle::radians(v.to_radians())).transpose();
                Ok(PayloadCommand::GimbalPosition(GimbalPosition {
                    angles: Axes {
                        pitch: angle(self.pitch)?,
                        roll: angle(self.roll)?,
                        yaw: angle(self.yaw)?,
                    },
                    frame,
                }))
            }
            CommandOperation::Rate => {
                let rate = |v: Option<f32>| {
                    v.map(|v| AngularRate::radians_per_second(v.to_radians()))
                        .transpose()
                };
                Ok(PayloadCommand::GimbalRate(GimbalRate {
                    rates: Axes {
                        pitch: rate(self.pitch)?,
                        roll: rate(self.roll)?,
                        yaw: rate(self.yaw)?,
                    },
                    frame,
                }))
            }
            CommandOperation::Recenter => Ok(PayloadCommand::Recenter),
        }
    }
}

/// Partial measured state; absent axes are unknown and never inferred from targets.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GimbalState {
    /// Connection epoch to which measurements belong.
    pub connection_epoch: u64,
    /// Last measured yaw angle, in degrees. Meaningful only if
    /// `measured_at.is_some()`.
    pub yaw: Option<f32>,
    /// Last measured pitch angle, in degrees. See [`Self::yaw`].
    pub pitch: Option<f32>,
    /// Last measured roll angle, in degrees. See [`Self::yaw`].
    pub roll: Option<f32>,
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
            connection_epoch: 0,
            yaw: None,
            pitch: None,
            roll: None,
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
        self.yaw = Some(yaw);
        self.pitch = Some(pitch);
        self.roll = Some(roll);
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
    /// Invoke the device's explicitly advertised recenter operation.
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
    /// Auto-calibrate the yaw zero-offset. The daemon temporarily zeroes
    /// the offset, commands `(0, 0, 0)`, waits for the gimbal to settle,
    /// samples the raw IMU yaw, and stores that value as the new offset.
    /// Subsequent operator-frame yaw=0 commands then round-trip to a
    /// status read of yaw≈0 (within IMU noise). Persists the new offset
    /// to `$XDG_STATE_HOME/turret/calibration.toml`. Response data:
    /// `{"yaw_offset_deg": <new>}`.
    CalibrateYaw,
    /// Set the yaw zero-offset to an explicit value (operator-frame
    /// degrees). Persists to disk like `calibrate_yaw`. `0.0` disables
    /// the offset.
    SetYawOffset {
        /// New yaw offset, operator-frame degrees.
        deg: f32,
    },
    /// Sweep the gimbal through a small set of known set-points and
    /// verify the IMU-reported attitude follows within tolerance. Each
    /// step waits ~1.5 s for settling, then samples a fresh
    /// `get_attitude` (operator-frame, post-calibration). Restores the
    /// initial pose at the end. Refuses if standby is engaged. Response
    /// data: `{passed, tolerance_deg, samples[{sp, pv, error_deg}],
    /// max_error_deg{pitch, roll, yaw}}`.
    Selftest,
    /// Return a JSON description of all available commands.
    Help,
}

/// Wire-format response sent back over the IPC socket. JSON form is
/// `{"status": "ok", "data": ...}` or `{"status": "error", "message": ...}`.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "status", rename_all = "lowercase")]
pub enum IpcResponse {
    /// Successful request handling; motion data distinguishes dispatch,
    /// device acknowledgement, and measured completion.
    Ok {
        /// Optional payload returned alongside the OK status.
        #[serde(skip_serializing_if = "Option::is_none")]
        data: Option<serde_json::Value>,
    },
    /// Failed command execution.
    Error {
        /// Human-readable failure reason.
        message: String,
        /// Retained command evidence when execution produced a receipt.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        data: Option<serde_json::Value>,
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
            data: None,
        }
    }

    /// Construct an error response retaining the command's execution evidence.
    pub fn error_with_data(message: impl Into<String>, data: serde_json::Value) -> Self {
        IpcResponse::Error {
            message: message.into(),
            data: Some(data),
        }
    }
}

#[cfg(test)]
mod tests;
