//! Unix-socket IPC server for local clients. Speaks line-delimited JSON
//! ([`super::models::IpcCommand`] / [`super::models::IpcResponse`]) and
//! routes through the same priority arbitrator as the MAVLink handlers.

mod bench;
mod slot;
use bench::{calibrate_yaw, selftest, set_yaw_offset};

use self::slot::{SlotGuard, SlotPool, MAX_IPC_SLOTS};
use super::arbitrator::Arbitrator;
use super::config::IpcConfig;
use super::gimbal_handle::GimbalHandle;
use super::models::{ControlSource, GimbalCommand, IpcCommand, IpcResponse, PanMode};
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

        let mut clients = tokio::task::JoinSet::new();
        loop {
            let accepted = tokio::select! {
                accepted = listener.accept() => accepted,
                completed = clients.join_next(), if !clients.is_empty() => {
                    if let Some(Err(error)) = completed { error!(%error, "IPC client task failed"); }
                    continue;
                }
            };
            match accepted {
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

                    clients.spawn(async move {
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
                    Some(pitch),
                    Some(roll),
                    Some(yaw),
                );
                execute_position(arbitrator, gimbal, gimbal_cmd, "set").await
            }

            IpcCommand::Center => {
                let mut gimbal_cmd =
                    GimbalCommand::new(ControlSource::UnixSocket(client_id), None, None, None);
                gimbal_cmd.operation = crate::daemon::models::CommandOperation::Recenter;
                execute_position(arbitrator, gimbal, gimbal_cmd, "center").await
            }

            IpcCommand::Status => status_response(state_manager, gimbal).await,

            IpcCommand::Version => match gimbal.firmware_version().await {
                Ok(version) => {
                    let data = serde_json::json!({ "version": version });
                    IpcResponse::ok_with_data(data)
                }
                Err(e) => IpcResponse::error(format!("Failed to get version: {}", e)),
            },

            IpcCommand::PanMode { mode } => set_pan_mode(state_manager, gimbal, mode).await,
            IpcCommand::Standby { enabled } => set_standby(state_manager, gimbal, enabled).await,

            IpcCommand::CalibrateYaw => calibrate_yaw(gimbal).await,

            IpcCommand::SetYawOffset { deg } => set_yaw_offset(gimbal, deg),

            IpcCommand::Selftest => selftest(gimbal, state_manager).await,

            IpcCommand::Help => help_response(),
        }
    }
}

/// Common tail for `Set` / `Center`: arbitrate, then call the gimbal
/// handle if accepted. Failures from either layer collapse to the
/// caller's IPC error response shape.
async fn execute_position(
    arbitrator: &Arbitrator,
    gimbal: &GimbalHandle,
    command: GimbalCommand,
    label: &str,
) -> IpcResponse {
    match arbitrator.execute(command, gimbal).await {
        Ok(progress) => IpcResponse::ok_with_data(serde_json::json!({
            "sent": progress.sent_at.is_some(), "device_acknowledged": progress.acknowledged_at.is_some(),
            "observed_complete": progress.observed_complete_at.is_some(),
        })),
        Err(error) => IpcResponse::error(format!("Failed to {label}: {error}")),
    }
}

async fn status_response(state: &StateManager, gimbal: &GimbalHandle) -> IpcResponse {
    let state = state.state_cached();
    let mut progress = serde_json::Map::new();
    for (name, lane) in [
        ("gimbal", crate::gimbal::PayloadLane::Gimbal),
        ("zoom", crate::gimbal::PayloadLane::Zoom),
    ] {
        let value = match gimbal.last_progress_cached(lane).await {
            Ok(progress) => progress.map(|p| serde_json::json!({
                "epoch": p.identity.epoch, "generation": p.identity.generation, "id": p.identity.id,
                "sent_at_us": p.sent_at.map(|t| t.0), "acknowledged_at_us": p.acknowledged_at.map(|t| t.0),
                "acknowledgement_timed_out": p.acknowledgement_timed_out,
                "observed_complete_at_us": p.observed_complete_at.map(|t| t.0), "ended": p.ended.map(|reason| format!("{reason:?}")),
            })).unwrap_or(serde_json::Value::Null),
            Err(error) => return IpcResponse::error(error.to_string()),
        };
        progress.insert(name.to_owned(), value);
    }
    IpcResponse::ok_with_data(serde_json::json!({
        "yaw": state.yaw, "pitch": state.pitch, "roll": state.roll,
        "measured_at": state.measured_at, "connection_epoch": state.connection_epoch,
        "pan_mode": state.pan_mode.to_name(), "standby": state.standby,
        "firmware_version": state.firmware_version, "yaw_offset_deg": gimbal.yaw_offset_deg(),
        "command_progress": progress,
    }))
}

async fn set_pan_mode(state: &StateManager, gimbal: &GimbalHandle, mode: u8) -> IpcResponse {
    let Some(pan_mode) = PanMode::from_u8(mode) else {
        return IpcResponse::error(format!("Invalid pan mode: {mode}"));
    };
    match gimbal
        .execute_storm32_extension(crate::protocols::storm32_rc::Storm32Action::PanMode(mode))
        .await
    {
        Ok(()) => {
            state.update_pan_mode(pan_mode);
            IpcResponse::ok()
        }
        Err(error) => IpcResponse::error(error.to_string()),
    }
}

async fn set_standby(state: &StateManager, gimbal: &GimbalHandle, enabled: bool) -> IpcResponse {
    match gimbal
        .execute_storm32_extension(crate::protocols::storm32_rc::Storm32Action::Standby(
            enabled,
        ))
        .await
    {
        Ok(()) => {
            state.update_standby(enabled);
            IpcResponse::ok()
        }
        Err(error) => IpcResponse::error(error.to_string()),
    }
}

fn help_response() -> IpcResponse {
    IpcResponse::ok_with_data(serde_json::json!({
        "commands": [
            {"cmd": "set", "params": {"pitch": "f32", "roll": "f32", "yaw": "f32"}},
            {"cmd": "center"}, {"cmd": "status"}, {"cmd": "version"},
            {"cmd": "pan_mode", "params": {"mode": "u8"}},
            {"cmd": "standby", "params": {"enabled": "bool"}},
            {"cmd": "calibrate_yaw"},
            {"cmd": "set_yaw_offset", "params": {"deg": "f32"}},
            {"cmd": "selftest"}, {"cmd": "help"}
        ]
    }))
}
