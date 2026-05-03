//! Unix-socket IPC server for local clients. Speaks line-delimited JSON
//! ([`super::models::IpcCommand`] / [`super::models::IpcResponse`]) and
//! routes through the same priority arbitrator as the MAVLink handlers.

use super::arbitrator::{Arbitrator, Outcome};
use super::config::IpcConfig;
use super::gimbal_handle::GimbalHandle;
use super::models::{CommandMode, ControlSource, GimbalCommand, IpcCommand, IpcResponse, PanMode};
use super::state::StateManager;
use std::sync::Arc;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::{UnixListener, UnixStream};
use tokio::sync::Mutex;
use tracing::{debug, error, info};

/// Unix socket IPC server for local clients
pub struct IpcServer {
    config: IpcConfig,
    state_manager: StateManager,
    arbitrator: Arc<Arbitrator>,
    gimbal: GimbalHandle,
    next_client_id: Arc<Mutex<u32>>,
}

impl IpcServer {
    /// Build an IPC server bound to the configured Unix socket path.
    /// Each accepted connection gets its own `Arbitrator` — they all share
    /// the same `StateManager`, so primary-control / priority decisions
    /// stay coherent across clients.
    pub fn new(config: IpcConfig, state_manager: StateManager, gimbal: GimbalHandle) -> Self {
        let arbitrator = Arc::new(Arbitrator::new(state_manager.clone()));

        Self {
            config,
            state_manager,
            arbitrator,
            gimbal,
            next_client_id: Arc::new(Mutex::new(1)),
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
                    let client_id = {
                        let mut id = self.next_client_id.lock().await;
                        let current = *id;
                        *id += 1;
                        current
                    };

                    info!("New IPC client connected: ID {}", client_id);

                    let state_manager = self.state_manager.clone();
                    let arbitrator = self.arbitrator.clone();
                    let gimbal = self.gimbal.clone();

                    tokio::spawn(async move {
                        if let Err(e) = Self::handle_client(
                            client_id,
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
        client_id: u32,
        stream: UnixStream,
        state_manager: StateManager,
        arbitrator: Arc<Arbitrator>,
        gimbal: GimbalHandle,
    ) -> crate::error::Result<()> {
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

            IpcCommand::Standby { enable } => match gimbal.set_standby(enable).await {
                Ok(_) => {
                    state_manager.update_standby(enable);
                    IpcResponse::ok()
                }
                Err(e) => IpcResponse::error(format!("Failed to set standby: {}", e)),
            },

            IpcCommand::Help => {
                let help_text = serde_json::json!({
                    "commands": [
                        {"cmd": "set", "params": {"pitch": "f32", "roll": "f32", "yaw": "f32"}},
                        {"cmd": "center"},
                        {"cmd": "status"},
                        {"cmd": "version"},
                        {"cmd": "pan-mode", "params": {"mode": "u8"}},
                        {"cmd": "standby", "params": {"enable": "bool"}},
                        {"cmd": "help"},
                    ]
                });
                IpcResponse::ok_with_data(help_text)
            }
        }
    }
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
