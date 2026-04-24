use super::arbitrator::Arbitrator;
use super::config::IpcConfig;
use super::models::{CommandMode, ControlSource, GimbalCommand, IpcCommand, IpcResponse, PanMode};
use super::state::StateManager;
use crate::gimbal::GimbalDevice;
use crate::protocols::storm32_rc::Storm32RC;
use log::{debug, error, info};
use std::sync::Arc;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::{UnixListener, UnixStream};
use tokio::sync::Mutex;

/// Unix socket IPC server for local clients
pub struct IpcServer {
    config: IpcConfig,
    state_manager: StateManager,
    arbitrator: Arc<Arbitrator>,
    gimbal: Arc<Mutex<Box<dyn GimbalDevice>>>,
    next_client_id: Arc<Mutex<u32>>,
}

impl IpcServer {
    pub fn new(
        config: IpcConfig,
        state_manager: StateManager,
        gimbal: Arc<Mutex<Box<dyn GimbalDevice>>>,
    ) -> Self {
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
        gimbal: Arc<Mutex<Box<dyn GimbalDevice>>>,
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
        gimbal: &Arc<Mutex<Box<dyn GimbalDevice>>>,
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

                let mut gimbal_guard = gimbal.lock().await;
                match arbitrator.process_command(gimbal_cmd, &mut gimbal_guard) {
                    Ok(true) => IpcResponse::ok(),
                    Ok(false) => IpcResponse::error("Command rejected (lower priority)"),
                    Err(e) => IpcResponse::error(format!("Failed to set angles: {}", e)),
                }
            }

            IpcCommand::Center => {
                let gimbal_cmd = GimbalCommand::new(
                    ControlSource::UnixSocket(client_id),
                    CommandMode::Position,
                    Some(0.0),
                    Some(0.0),
                    Some(0.0),
                );

                let mut gimbal_guard = gimbal.lock().await;
                match arbitrator.process_command(gimbal_cmd, &mut gimbal_guard) {
                    Ok(true) => IpcResponse::ok(),
                    Ok(false) => IpcResponse::error("Command rejected (lower priority)"),
                    Err(e) => IpcResponse::error(format!("Failed to center: {}", e)),
                }
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

            IpcCommand::Version => {
                let mut gimbal_guard = gimbal.lock().await;
                match gimbal_guard.get_version() {
                    Ok(version) => {
                        let data = serde_json::json!({ "version": version });
                        IpcResponse::ok_with_data(data)
                    }
                    Err(e) => IpcResponse::error(format!("Failed to get version: {}", e)),
                }
            }

            IpcCommand::VersionStr => {
                let mut gimbal_guard = gimbal.lock().await;
                if let Some(storm32) = gimbal_guard
                    .as_any_mut()
                    .and_then(|a| a.downcast_mut::<Storm32RC>())
                {
                    match storm32.get_version_string() {
                        Ok(version) => {
                            let data = serde_json::json!({ "version_string": version });
                            IpcResponse::ok_with_data(data)
                        }
                        Err(e) => {
                            IpcResponse::error(format!("Failed to get version string: {}", e))
                        }
                    }
                } else {
                    IpcResponse::error("Command requires Storm32 device".to_string())
                }
            }

            IpcCommand::Pitch { value } => {
                let mut gimbal_guard = gimbal.lock().await;
                if let Some(storm32) = gimbal_guard
                    .as_any_mut()
                    .and_then(|a| a.downcast_mut::<Storm32RC>())
                {
                    let result = if value == 0 {
                        storm32.recenter_pitch()
                    } else {
                        storm32.set_pitch(value)
                    };
                    match result {
                        Ok(_) => IpcResponse::ok(),
                        Err(e) => IpcResponse::error(format!("Failed to set pitch: {}", e)),
                    }
                } else {
                    IpcResponse::error("Command requires Storm32 device".to_string())
                }
            }

            IpcCommand::Roll { value } => {
                let mut gimbal_guard = gimbal.lock().await;
                if let Some(storm32) = gimbal_guard
                    .as_any_mut()
                    .and_then(|a| a.downcast_mut::<Storm32RC>())
                {
                    let result = if value == 0 {
                        storm32.recenter_roll()
                    } else {
                        storm32.set_roll(value)
                    };
                    match result {
                        Ok(_) => IpcResponse::ok(),
                        Err(e) => IpcResponse::error(format!("Failed to set roll: {}", e)),
                    }
                } else {
                    IpcResponse::error("Command requires Storm32 device".to_string())
                }
            }

            IpcCommand::Yaw { value } => {
                let mut gimbal_guard = gimbal.lock().await;
                if let Some(storm32) = gimbal_guard
                    .as_any_mut()
                    .and_then(|a| a.downcast_mut::<Storm32RC>())
                {
                    let result = if value == 0 {
                        storm32.recenter_yaw()
                    } else {
                        storm32.set_yaw(value)
                    };
                    match result {
                        Ok(_) => IpcResponse::ok(),
                        Err(e) => IpcResponse::error(format!("Failed to set yaw: {}", e)),
                    }
                } else {
                    IpcResponse::error("Command requires Storm32 device".to_string())
                }
            }

            IpcCommand::PanMode { mode } => {
                if let Some(pan_mode) = PanMode::from_u8(mode) {
                    let mut gimbal_guard = gimbal.lock().await;
                    match gimbal_guard.set_pan_mode(mode) {
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

            IpcCommand::Standby { enable } => {
                let mut gimbal_guard = gimbal.lock().await;
                match gimbal_guard.set_standby(enable) {
                    Ok(_) => {
                        state_manager.update_standby(enable);
                        IpcResponse::ok()
                    }
                    Err(e) => IpcResponse::error(format!("Failed to set standby: {}", e)),
                }
            }

            IpcCommand::Help => {
                let help_text = serde_json::json!({
                    "commands": [
                        {"cmd": "set", "params": {"pitch": "f32", "roll": "f32", "yaw": "f32"}},
                        {"cmd": "center"},
                        {"cmd": "status"},
                        {"cmd": "version"},
                        {"cmd": "version-str"},
                        {"cmd": "pitch", "params": {"value": "u16"}},
                        {"cmd": "roll", "params": {"value": "u16"}},
                        {"cmd": "yaw", "params": {"value": "u16"}},
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
