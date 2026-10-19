//! Host daemon joining payload execution, local IPC, and a MAVLink manager.

pub mod arbitrator;
pub mod attitude;
pub mod calibration;
pub mod config;
pub mod gimbal_handle;
pub mod ipc_server;
pub mod mavlink_manager;
pub mod models;
pub mod state;
pub mod yaw_corrector;

use crate::device_scanner::DeviceScanner;
use crate::gimbal::{self, GimbalDevice};
use calibration::Calibration;
use config::DaemonConfig;
use gimbal_handle::GimbalHandle;
use ipc_server::IpcServer;
use mavlink_manager::MavlinkManager;
use state::StateManager;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::Notify;
use tokio::task::JoinHandle;
use tracing::{debug, error, info, warn};

const SHUTDOWN_GRACE: Duration = Duration::from_secs(2);
type Tasks = Vec<(&'static str, JoinHandle<()>)>;

/// Run device ownership and protocol services until shutdown or a critical task failure.
pub async fn run_daemon(config_path: Option<String>) -> crate::Result<()> {
    let (config, state, gimbal) =
        tokio::task::spawn_blocking(move || initialize_blocking(config_path))
            .await
            .map_err(|error| crate::Error::Worker {
                operation: "initialization",
                source: error,
            })??;
    let task_died = Arc::new(Notify::new());
    let mut tasks = Vec::new();
    start_device_tasks(&mut tasks, &task_died, &config, &state, &gimbal);
    let server = IpcServer::new(config.ipc.clone(), state.clone(), gimbal.clone());
    spawn_critical(&mut tasks, "ipc", &task_died, async move {
        if let Err(error) = server.start().await {
            error!(%error, "IPC server failed");
        }
    });
    info!("Daemon services started");
    tokio::select! {
        result = wait_for_shutdown_signal() => { info!(signal = ?result, "daemon shutdown signal"); }
        () = task_died.notified() => { warn!("critical daemon task exited"); }
    }
    match tokio::time::timeout(SHUTDOWN_GRACE, gimbal.revoke()).await {
        Ok(Ok(())) => {}
        Ok(Err(error)) => warn!(%error, "physical stop remains unconfirmed"),
        Err(_) => warn!("payload revoke did not finish within shutdown grace"),
    }
    drain_tasks(tasks).await;
    cleanup_ipc_socket_blocking(&config.ipc.socket_path);
    Ok(())
}

fn initialize_blocking(
    path: Option<String>,
) -> crate::Result<(DaemonConfig, StateManager, GimbalHandle)> {
    let config = DaemonConfig::load_for_startup(path.as_deref())?;
    let path =
        candidate_path_blocking(&config.device.path).ok_or_else(|| crate::Error::NotDetected {
            device: config.device.path.clone(),
            tried: "STorM32 USB CDC".to_string(),
        })?;
    let mut device = gimbal::detect_gimbal_blocking(&path)?;
    let state = StateManager::with_min_command_interval(Duration::from_millis(u64::from(
        config.arbitrator.min_command_interval_ms,
    )));
    if let Ok(version) = device.firmware_version_blocking() {
        state.update_firmware_version(version);
    }
    let calibration = Calibration::load_blocking();
    let handle = GimbalHandle::with_yaw_offset_deg(device, calibration.yaw_offset_deg)?;
    Ok((config, state, handle))
}

fn start_device_tasks(
    tasks: &mut Tasks,
    died: &Arc<Notify>,
    config: &DaemonConfig,
    state: &StateManager,
    gimbal: &GimbalHandle,
) {
    let lost = Arc::new(Notify::new());
    let (tx, seed) = attitude::attitude_channel();
    drop(seed);
    let reconnect = reconnect_task(
        lost.clone(),
        gimbal.clone(),
        state.clone(),
        config.device.path.clone(),
    );
    spawn_critical(tasks, "reconnect", died, reconnect);
    let corrector = config
        .gimbal
        .yaw_corrector_enabled
        .then(yaw_corrector::YawCorrector::new);
    let poll =
        attitude::attitude_poll_loop(gimbal.clone(), state.clone(), lost, corrector, tx.clone());
    spawn_critical(tasks, "attitude-poll", died, poll);
    spawn_critical(
        tasks,
        "payload-deadlines",
        died,
        deadline_loop(gimbal.clone()),
    );
    if config.mavlink.enabled {
        let manager = MavlinkManager::new(state.clone(), gimbal.clone());
        let config = config.mavlink.clone();
        let rx = tx.subscribe();
        spawn_critical(tasks, "mavlink", died, async move {
            if let Err(error) = manager.start(config, rx).await {
                error!(%error, "MAVLink manager failed");
            }
        });
    }
}

async fn deadline_loop(gimbal: GimbalHandle) {
    let mut interval = tokio::time::interval(Duration::from_millis(20));
    interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    loop {
        interval.tick().await;
        if let Err(error) = gimbal.poll_execution().await {
            error!(%error, "payload stopping state is unknown");
        }
    }
}

fn spawn_critical<F>(tasks: &mut Tasks, name: &'static str, died: &Arc<Notify>, future: F)
where
    F: std::future::Future<Output = ()> + Send + 'static,
{
    let died = died.clone();
    tasks.push((
        name,
        tokio::spawn(async move {
            future.await;
            died.notify_one();
        }),
    ));
}

async fn drain_tasks(tasks: Tasks) {
    for (name, handle) in &tasks {
        debug!(name, "aborting daemon task");
        handle.abort();
    }
    for (name, handle) in tasks {
        match tokio::time::timeout(SHUTDOWN_GRACE, handle).await {
            Ok(Ok(())) => debug!(name, "task stopped"),
            Ok(Err(error)) if error.is_cancelled() => debug!(name, "task cancelled"),
            Ok(Err(error)) => warn!(name, %error, "task failed"),
            Err(_) => warn!(name, "task did not exit within shutdown grace"),
        }
    }
}

fn cleanup_ipc_socket_blocking(path: &str) {
    match std::fs::remove_file(path) {
        Ok(()) => {}
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
        Err(error) => warn!(path, %error, "IPC socket cleanup failed"),
    }
}

fn candidate_path_blocking(configured: &str) -> Option<String> {
    if configured == "auto" {
        DeviceScanner::new().find_storm32_device()
    } else if std::path::Path::new(configured).exists() {
        Some(configured.to_string())
    } else {
        None
    }
}

fn reopen_blocking(configured: &str) -> crate::Result<Box<dyn GimbalDevice>> {
    let path = candidate_path_blocking(configured).ok_or_else(|| crate::Error::NotDetected {
        device: configured.to_string(),
        tried: "STorM32 USB CDC".to_string(),
    })?;
    gimbal::detect_gimbal_blocking(&path)
}

async fn reconnect_task(
    lost: Arc<Notify>,
    gimbal: GimbalHandle,
    state: StateManager,
    configured: String,
) {
    loop {
        lost.notified().await;
        if let Err(error) = gimbal.revoke().await {
            warn!(%error, "disconnected device stop unconfirmed");
        }
        let mut backoff = Duration::from_millis(500);
        loop {
            let path = configured.clone();
            match tokio::task::spawn_blocking(move || reopen_blocking(&path)).await {
                Ok(Ok(device)) => match gimbal.replace(device).await {
                    Ok(epoch) => {
                        state.clear_device(epoch);
                        info!(epoch, "device reconnected");
                        break;
                    }
                    Err(error) => warn!(%error, "device replacement failed"),
                },
                Ok(Err(error)) => debug!(%error, "device not ready"),
                Err(error) => warn!(%error, "device reopen task failed"),
            }
            tokio::time::sleep(backoff).await;
            backoff = (backoff * 2).min(Duration::from_secs(10));
        }
    }
}

async fn wait_for_shutdown_signal() -> crate::Result<&'static str> {
    #[cfg(unix)]
    {
        use tokio::signal::unix::{signal, SignalKind};
        let mut terminate = signal(SignalKind::terminate())?;
        tokio::select! {
            result = tokio::signal::ctrl_c() => { result?; Ok("SIGINT") },
            _ = terminate.recv() => Ok("SIGTERM"),
        }
    }
    #[cfg(not(unix))]
    {
        tokio::signal::ctrl_c().await?;
        Ok("Ctrl-C")
    }
}
