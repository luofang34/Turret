// Rust 2018+ style: module file is src/daemon.rs, submodules in src/daemon/
#[path = "daemon/arbitrator.rs"]
pub mod arbitrator;
#[path = "daemon/config.rs"]
pub mod config;
#[path = "daemon/ipc_server.rs"]
pub mod ipc_server;
#[path = "daemon/mavlink_manager.rs"]
pub mod mavlink_manager;
#[path = "daemon/models.rs"]
pub mod models;
#[path = "daemon/state.rs"]
pub mod state;

use crate::device_scanner::DeviceScanner;
use crate::gimbal;
use config::DaemonConfig;
use ipc_server::IpcServer;
use log::{error, info};
use mavlink_manager::MavlinkManager;
use state::StateManager;
use std::sync::Arc;
use tokio::sync::Mutex;

/// Main daemon entry point
pub async fn run_daemon(config_path: Option<String>) -> anyhow::Result<()> {
    // Load configuration
    let config = if let Some(path) = config_path {
        DaemonConfig::load_or_default(path)
    } else {
        DaemonConfig::load_or_default("turret.yaml")
    };

    info!("Daemon configuration loaded");
    info!("  Device: {}", config.device.path);
    info!("  IPC socket: {}", config.ipc.socket_path);
    info!("  MAVLink enabled: {}", config.mavlink.enabled);

    // Determine device path
    let device_path = if config.device.path == "auto" {
        let scanner = DeviceScanner::new();
        match scanner.find_storm32_device() {
            Some(path) => {
                info!("Auto-detected device: {}", path);
                path
            }
            None => {
                error!("No STorM32 device found. Please specify device path in config.");
                return Err(anyhow::anyhow!("No device found"));
            }
        }
    } else {
        config.device.path.clone()
    };

    // Initialize gimbal device (single instance, shared via Arc<Mutex>)
    info!("Initializing gimbal device: {}", device_path);
    let mut gimbal = gimbal::detect_gimbal(&device_path)?;
    info!("Detected protocol: {}", gimbal.protocol_name());

    // Create shared state manager
    let state_manager = StateManager::new();

    // Get firmware version and update state
    if let Ok(version) = gimbal.get_version() {
        info!("Firmware version: {}", version);
        state_manager.update_firmware_version(version);
    }

    // Wrap gimbal in Arc<Mutex> for sharing between IPC and MAVLink
    let gimbal_shared = Arc::new(Mutex::new(gimbal));

    // Start IPC server
    let ipc_gimbal = gimbal_shared.clone();
    let ipc_state = state_manager.clone();
    let ipc_config = config.ipc.clone();

    tokio::spawn(async move {
        let server = IpcServer::new(ipc_config, ipc_state, ipc_gimbal);
        if let Err(e) = server.start().await {
            error!("IPC server error: {}", e);
        }
    });

    // Start MAVLink manager if enabled
    if config.mavlink.enabled {
        let mavlink_gimbal = gimbal_shared.clone();
        let mavlink_state = state_manager.clone();
        let mavlink_cfg = config.mavlink.clone();

        tokio::spawn(async move {
            let manager = MavlinkManager::new(mavlink_cfg.clone(), mavlink_state, mavlink_gimbal);
            if let Err(e) = manager.start(mavlink_cfg).await {
                error!("MAVLink manager error: {}", e);
            }
        });
    }

    info!("Daemon started successfully");

    // Main loop - just keep alive and handle signals
    tokio::signal::ctrl_c().await?;
    info!("Received shutdown signal, exiting...");

    Ok(())
}