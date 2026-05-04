//! Unix-socket IPC server for local clients. Speaks line-delimited JSON
//! ([`super::models::IpcCommand`] / [`super::models::IpcResponse`]) and
//! routes through the same priority arbitrator as the MAVLink handlers.

#[path = "ipc_server/slot.rs"]
mod slot;

use self::slot::{SlotGuard, SlotPool, MAX_IPC_SLOTS};
use super::arbitrator::{Arbitrator, Outcome};
use super::calibration::Calibration;
use super::config::IpcConfig;
use super::gimbal_handle::GimbalHandle;
use super::models::{CommandMode, ControlSource, GimbalCommand, IpcCommand, IpcResponse, PanMode};
use super::state::StateManager;
use std::sync::Arc;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::{UnixListener, UnixStream};
use tracing::{debug, error, info, warn};

/// Unix socket IPC server for local clients
pub struct IpcServer {
    config: IpcConfig,
    state_manager: StateManager,
    arbitrator: Arc<Arbitrator>,
    gimbal: GimbalHandle,
    slots: SlotPool,
}

impl IpcServer {
    /// Build an IPC server bound to the configured Unix socket path.
    /// Each accepted connection gets its own `Arbitrator` — they all share
    /// the same `StateManager`, so primary-control / priority decisions
    /// stay coherent across clients.
    pub fn new(config: IpcConfig, state_manager: StateManager, gimbal: GimbalHandle) -> Self {
        let arbitrator = Arc::new(Arbitrator::new(state_manager.clone()));
        let slots = SlotPool::new(state_manager.clone());

        Self {
            config,
            state_manager,
            arbitrator,
            gimbal,
            slots,
        }
    }

    /// Start the IPC server
    pub async fn start(self) -> crate::error::Result<()> {
        // Remove existing socket file if present
        let socket_path = &self.config.socket_path;
        if std::path::Path::new(socket_path).exists() {
            std::fs::remove_file(socket_path)?;
        }

        info!("Starting IPC server on {}", socket_path);
        let listener = UnixListener::bind(socket_path)?;

        loop {
            match listener.accept().await {
                Ok((stream, _addr)) => {
                    let Some(slot_guard) = self.slots.acquire() else {
                        warn!(
                            "IPC server at capacity ({} clients); rejecting new connection",
                            MAX_IPC_SLOTS
                        );
                        // Dropping `stream` closes the socket; client sees
                        // an immediate disconnect rather than a half-open
                        // pipe. No bookkeeping to undo since acquire failed.
                        drop(stream);
                        continue;
                    };
                    let client_id = slot_guard.id();

                    info!("New IPC client connected: ID {}", client_id);

                    let state_manager = self.state_manager.clone();
                    let arbitrator = self.arbitrator.clone();
                    let gimbal = self.gimbal.clone();

                    tokio::spawn(async move {
                        if let Err(e) = Self::handle_client(
                            slot_guard,
                            stream,
                            state_manager,
                            arbitrator,
                            gimbal,
                        )
                        .await
                        {
                            error!("Client {} error: {}", client_id, e);
                        }
                        info!("Client {} disconnected", client_id);
                        // `slot_guard` drops here, releasing the slot and
                        // clearing the client's per-source tracking entries.
                    });
                }
                Err(e) => {
                    error!("Failed to accept connection: {}", e);
                }
            }
        }
    }

    /// Handle a single client connection
    async fn handle_client(
        slot_guard: SlotGuard,
        stream: UnixStream,
        state_manager: StateManager,
        arbitrator: Arc<Arbitrator>,
        gimbal: GimbalHandle,
    ) -> crate::error::Result<()> {
        let client_id = slot_guard.id();
        let (reader, mut writer) = stream.into_split();
        let mut reader = BufReader::new(reader);
        let mut line = String::new();

        loop {
            line.clear();

            match reader.read_line(&mut line).await {
                Ok(0) => break, // EOF
                Ok(_) => {
                    let trimmed = line.trim();
                    if trimmed.is_empty() {
                        continue;
                    }

                    debug!("Client {}: received command: {}", client_id, trimmed);

                    let response = Self::process_command(
                        client_id,
                        trimmed,
                        &state_manager,
                        &arbitrator,
                        &gimbal,
                    )
                    .await;

                    let response_json = serde_json::to_string(&response)? + "\n";
                    writer.write_all(response_json.as_bytes()).await?;
                    writer.flush().await?;
                }
                Err(e) => {
                    error!("Client {} read error: {}", client_id, e);
                    break;
                }
            }
        }

        Ok(())
    }

    /// Process a command from a client
    async fn process_command(
        client_id: u32,
        command_str: &str,
        state_manager: &StateManager,
        arbitrator: &Arbitrator,
        gimbal: &GimbalHandle,
    ) -> IpcResponse {
        // Parse JSON command
        let command: IpcCommand = match serde_json::from_str(command_str) {
            Ok(cmd) => cmd,
            Err(e) => return IpcResponse::error(format!("Invalid JSON: {}", e)),
        };

        debug!("Client {}: parsed command: {:?}", client_id, command);

        match command {
            IpcCommand::Set { pitch, roll, yaw } => {
                let gimbal_cmd = GimbalCommand::new(
                    ControlSource::UnixSocket(client_id),
                    CommandMode::Position,
                    Some(yaw),
                    Some(pitch),
                    Some(roll),
                );
                execute_position(arbitrator, gimbal, gimbal_cmd, "set").await
            }

            IpcCommand::Center => {
                let gimbal_cmd = GimbalCommand::new(
                    ControlSource::UnixSocket(client_id),
                    CommandMode::Position,
                    Some(0.0),
                    Some(0.0),
                    Some(0.0),
                );
                execute_position(arbitrator, gimbal, gimbal_cmd, "center").await
            }

            IpcCommand::Status => {
                let state = state_manager.get_state();
                let data = serde_json::json!({
                    "yaw": state.yaw,
                    "pitch": state.pitch,
                    "roll": state.roll,
                    "pan_mode": state.pan_mode.to_name(),
                    "standby": state.standby,
                    "firmware_version": state.firmware_version,
                    "yaw_offset_deg": gimbal.yaw_offset_deg(),
                });
                IpcResponse::ok_with_data(data)
            }

            IpcCommand::Version => match gimbal.get_version().await {
                Ok(version) => {
                    let data = serde_json::json!({ "version": version });
                    IpcResponse::ok_with_data(data)
                }
                Err(e) => IpcResponse::error(format!("Failed to get version: {}", e)),
            },

            IpcCommand::PanMode { mode } => {
                if let Some(pan_mode) = PanMode::from_u8(mode) {
                    match gimbal.set_pan_mode(mode).await {
                        Ok(_) => {
                            state_manager.update_pan_mode(pan_mode);
                            IpcResponse::ok()
                        }
                        Err(e) => IpcResponse::error(format!("Failed to set pan mode: {}", e)),
                    }
                } else {
                    IpcResponse::error(format!("Invalid pan mode: {}", mode))
                }
            }

            IpcCommand::Standby { enabled } => match gimbal.set_standby(enabled).await {
                Ok(_) => {
                    state_manager.update_standby(enabled);
                    IpcResponse::ok()
                }
                Err(e) => IpcResponse::error(format!("Failed to set standby: {}", e)),
            },

            IpcCommand::CalibrateYaw => calibrate_yaw(gimbal).await,

            IpcCommand::SetYawOffset { deg } => set_yaw_offset(gimbal, deg),

            IpcCommand::Selftest => selftest(gimbal, state_manager).await,

            IpcCommand::Help => {
                let help_text = serde_json::json!({
                    "commands": [
                        {"cmd": "set", "params": {"pitch": "f32", "roll": "f32", "yaw": "f32"}},
                        {"cmd": "center"},
                        {"cmd": "status"},
                        {"cmd": "version"},
                        {"cmd": "pan_mode", "params": {"mode": "u8"}},
                        {"cmd": "standby", "params": {"enabled": "bool"}},
                        {"cmd": "calibrate_yaw"},
                        {"cmd": "set_yaw_offset", "params": {"deg": "f32"}},
                        {"cmd": "selftest"},
                        {"cmd": "help"},
                    ]
                });
                IpcResponse::ok_with_data(help_text)
            }
        }
    }
}

/// How long to wait for the gimbal to settle on a commanded angle before
/// sampling its IMU. Sized for a worst-case 30° swing at the empirical
/// ~15°/s observed in-flight on a stock STorM32 — covered in ~2 s with
/// margin to spare, so 2.5 s is the conservative pick. Used for both
/// `calibrate_yaw` (one settle on a small motion) and `selftest` (six
/// settles on motions up to 30°).
const CALIBRATION_SETTLE: std::time::Duration = std::time::Duration::from_millis(2500);

/// Auto-calibrate the yaw zero-offset.
///
/// 1. Snapshot the existing offset so we can restore it on any failure.
/// 2. Zero the offset so the device sees raw operator-frame commands.
/// 3. Command `(0, 0, 0)` and wait for the gimbal to settle.
/// 4. Read the device's raw yaw — that *is* the bias between the
///    command frame and the IMU frame.
/// 5. Store the bias as the new offset and persist it.
///
/// Bypasses the arbitrator: calibration is a one-shot operator action,
/// not a streaming source competing with MAVLink. Skipping arbitration
/// also avoids the case where a higher-priority source is mid-stream
/// and would reject the calibration probe.
async fn calibrate_yaw(gimbal: &GimbalHandle) -> IpcResponse {
    let prior_offset = gimbal.yaw_offset_deg();

    // Step 1: zero the offset so the next command goes through unmodified.
    gimbal.set_yaw_offset_deg(0.0);

    // Step 2: command center. On failure, restore the prior offset and
    // bail — leaving offset=0 would silently change the operator's
    // calibration.
    if let Err(e) = gimbal.set_attitude(0.0, 0.0, 0.0).await {
        gimbal.set_yaw_offset_deg(prior_offset);
        return IpcResponse::error(format!("Calibration failed sending center command: {}", e));
    }

    // Step 3: settle.
    tokio::time::sleep(CALIBRATION_SETTLE).await;

    // Step 4: read raw IMU yaw. `get_attitude_raw` skips the offset
    // application so the value we get back is in device frame —
    // exactly the bias.
    let raw = match gimbal.get_attitude_raw().await {
        Ok(att) => att,
        Err(e) => {
            gimbal.set_yaw_offset_deg(prior_offset);
            return IpcResponse::error(format!("Calibration failed reading attitude: {}", e));
        }
    };

    if !raw.yaw.is_finite() {
        gimbal.set_yaw_offset_deg(prior_offset);
        return IpcResponse::error(format!(
            "Calibration read non-finite yaw: {} — leaving prior offset {:.3}° intact",
            raw.yaw, prior_offset
        ));
    }

    let new_offset = raw.yaw;
    gimbal.set_yaw_offset_deg(new_offset);

    // Step 5: persist. Persistence failure is non-fatal — the offset is
    // applied in memory so the operator sees the corrected behavior
    // immediately; we just warn that they'll need to recalibrate after a
    // daemon restart.
    let mut persistence_warning = None;
    if let Err(e) = (Calibration {
        yaw_offset_deg: new_offset,
    })
    .save()
    {
        warn!(
            "Calibration succeeded in-memory but failed to persist: {} \
             — offset will be lost on daemon restart",
            e
        );
        persistence_warning = Some(format!(
            "calibration applied in memory but persistence failed: {} \
             (will not survive daemon restart)",
            e
        ));
    }

    info!(
        "Yaw calibration: prior_offset={:.3}° → new_offset={:.3}° (raw IMU yaw at SP=0)",
        prior_offset, new_offset
    );

    let mut data = serde_json::json!({
        "yaw_offset_deg": new_offset,
        "prior_yaw_offset_deg": prior_offset,
    });
    if let Some(w) = persistence_warning {
        data["warning"] = serde_json::Value::String(w);
    }
    IpcResponse::ok_with_data(data)
}

/// Set the yaw zero-offset to a caller-supplied value.
fn set_yaw_offset(gimbal: &GimbalHandle, deg: f32) -> IpcResponse {
    if !deg.is_finite() {
        return IpcResponse::error(format!("yaw offset must be finite (got {deg})"));
    }
    gimbal.set_yaw_offset_deg(deg);

    let mut persistence_warning = None;
    if let Err(e) = (Calibration {
        yaw_offset_deg: deg,
    })
    .save()
    {
        warn!(
            "set_yaw_offset succeeded in-memory but failed to persist: {} \
             — offset will be lost on daemon restart",
            e
        );
        persistence_warning = Some(format!(
            "offset applied in memory but persistence failed: {} \
             (will not survive daemon restart)",
            e
        ));
    }

    info!("Yaw offset set explicitly: {:.3}°", deg);

    let mut data = serde_json::json!({ "yaw_offset_deg": deg });
    if let Some(w) = persistence_warning {
        data["warning"] = serde_json::Value::String(w);
    }
    IpcResponse::ok_with_data(data)
}

/// Tolerance for the selftest sweep. Operator-frame, post-calibration —
/// well above expected IMU noise (~0.1° typical) but tight enough that a
/// real pointing error from a bad calibration / loose belt / wrong limit
/// is caught.
const SELFTEST_TOLERANCE_DEG: f32 = 1.0;

/// Sweep the gimbal through known set-points and report SP-vs-PV errors.
///
/// Operator-frame throughout: `set_attitude` writes the SP unchanged and
/// `get_attitude` returns IMU-minus-calibration-offset, so a calibrated
/// gimbal should land within ~0.1° of every test point. Refuses if the
/// daemon believes the gimbal is in standby — without active stabilization,
/// PV reflects gravity rather than commanded angle, and any "failure"
/// would be misleading.
///
/// Bypasses the arbitrator (one-shot operator action, like calibration).
/// Restores the original pose at the end via a single `set_attitude`,
/// best-effort — restoration failures surface as a `warning` field on
/// the response without overriding `passed`.
async fn selftest(gimbal: &GimbalHandle, state_manager: &StateManager) -> IpcResponse {
    if state_manager.is_standby() {
        return IpcResponse::error(
            "selftest: gimbal is in standby (motors disengaged). \
             Toggle standby off before running selftest.",
        );
    }

    // Snapshot the current pose so we can restore the operator's
    // starting position. If we can't even read attitude, bail
    // immediately — running a sweep without a baseline would leave the
    // gimbal in an unknown pose at the end.
    let original = match gimbal.get_attitude().await {
        Ok(att) => att,
        Err(e) => {
            return IpcResponse::error(format!("selftest: failed to read initial pose: {e}"));
        }
    };

    // Modest sweep: pitch ±15°, yaw ±15°, plus center entry/exit so we
    // see "did the gimbal recover from the largest displacement". Roll
    // is skipped because most installations either have a roll-locked
    // setup (PAN modes 1/2/4) or an unbalanced payload that makes large
    // roll commands physically problematic.
    let test_points: &[(f32, f32, f32)] = &[
        (0.0, 0.0, 0.0),
        (15.0, 0.0, 0.0),
        (-15.0, 0.0, 0.0),
        (0.0, 0.0, 15.0),
        (0.0, 0.0, -15.0),
        (0.0, 0.0, 0.0),
    ];

    let mut samples: Vec<serde_json::Value> = Vec::with_capacity(test_points.len());
    let mut max_pitch_err = 0.0_f32;
    let mut max_roll_err = 0.0_f32;
    let mut max_yaw_err = 0.0_f32;
    let mut passed = true;
    let mut aborted_at: Option<String> = None;

    for &(p, r, y) in test_points {
        if let Err(e) = gimbal.set_attitude(p, r, y).await {
            aborted_at = Some(format!("set_attitude({p}, {r}, {y}) failed: {e}"));
            passed = false;
            break;
        }
        tokio::time::sleep(CALIBRATION_SETTLE).await;
        let pv = match gimbal.get_attitude().await {
            Ok(att) => att,
            Err(e) => {
                aborted_at = Some(format!("get_attitude after SP=({p}, {r}, {y}) failed: {e}"));
                passed = false;
                break;
            }
        };
        let ep = (p - pv.pitch).abs();
        let er = (r - pv.roll).abs();
        let ey = (y - pv.yaw).abs();
        max_pitch_err = max_pitch_err.max(ep);
        max_roll_err = max_roll_err.max(er);
        max_yaw_err = max_yaw_err.max(ey);
        if ep > SELFTEST_TOLERANCE_DEG || er > SELFTEST_TOLERANCE_DEG || ey > SELFTEST_TOLERANCE_DEG
        {
            passed = false;
        }
        samples.push(serde_json::json!({
            "sp":        {"pitch": p,        "roll": r,        "yaw": y},
            "pv":        {"pitch": pv.pitch, "roll": pv.roll,  "yaw": pv.yaw},
            "error_deg": {"pitch": ep,       "roll": er,       "yaw": ey},
        }));
    }

    // Best-effort pose restoration. Don't override `passed` on failure —
    // the sweep itself succeeded or didn't independently of whether we
    // could put the gimbal back where the operator had it.
    let restore_warning = match gimbal
        .set_attitude(original.pitch, original.roll, original.yaw)
        .await
    {
        Ok(()) => None,
        Err(e) => {
            warn!("selftest: failed to restore initial pose: {e}");
            Some(format!(
                "selftest sweep finished but the initial pose could not be restored: {e}"
            ))
        }
    };

    info!(
        "Selftest: passed={}, max_err deg pitch={:.3} roll={:.3} yaw={:.3}",
        passed, max_pitch_err, max_roll_err, max_yaw_err
    );

    let mut data = serde_json::json!({
        "passed": passed,
        "tolerance_deg": SELFTEST_TOLERANCE_DEG,
        "samples": samples,
        "max_error_deg": {
            "pitch": max_pitch_err,
            "roll":  max_roll_err,
            "yaw":   max_yaw_err,
        },
    });
    if let Some(w) = aborted_at {
        data["aborted_at"] = serde_json::Value::String(w);
    }
    if let Some(w) = restore_warning {
        data["warning"] = serde_json::Value::String(w);
    }
    IpcResponse::ok_with_data(data)
}

/// Common tail for `Set` / `Center`: arbitrate, then call the gimbal
/// handle if accepted. Failures from either layer collapse to the
/// caller's IPC error response shape.
async fn execute_position(
    arbitrator: &Arbitrator,
    gimbal: &GimbalHandle,
    cmd: GimbalCommand,
    label: &str,
) -> IpcResponse {
    match arbitrator.arbitrate(cmd) {
        Outcome::Execute { pitch, roll, yaw } => {
            match gimbal.set_attitude(pitch, roll, yaw).await {
                Ok(()) => IpcResponse::ok(),
                Err(e) => IpcResponse::error(format!("Failed to {label}: {e}")),
            }
        }
        Outcome::Rejected => IpcResponse::error("Command rejected (lower priority)"),
    }
}
