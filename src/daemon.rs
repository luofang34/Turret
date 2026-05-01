//! Long-running daemon: IPC server + MAVLink Gimbal Manager + hot-plug
//! recovery glued onto a single gimbal handle.
//!
//! [`run_daemon`] is the entry point; everything else under this module is
//! the building blocks it spawns. Submodules are public so an external
//! crate can wire its own combination — see the `arbitrator` /
//! `mavlink_manager` / `state` types — but the supported way to bring up
//! the full daemon is `run_daemon`.

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
use crate::gimbal::{self, GimbalDevice};
use config::DaemonConfig;
use ipc_server::IpcServer;
use mavlink_manager::MavlinkManager;
use state::StateManager;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::{Mutex, Notify};
use tokio::task::JoinHandle;
use tracing::{debug, error, info, warn};

/// How long to wait for each background task to wind down on shutdown
/// before we give up and let `JoinHandle::abort` finish the job. Long
/// enough for an in-flight serial exchange to time out at the driver's
/// 1 s read timeout, short enough to fit comfortably inside systemd's
/// default 90 s `TimeoutStopSec`.
const SHUTDOWN_GRACE: Duration = Duration::from_secs(2);

/// Main daemon entry point
pub async fn run_daemon(config_path: Option<String>) -> crate::error::Result<()> {
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
                return Err(crate::error::Error::NotDetected {
                    device: "auto".to_string(),
                    tried: "STorM32 USB CDC".to_string(),
                });
            }
        }
    } else {
        config.device.path.clone()
    };

    // Initialize gimbal device (single instance, shared via Arc<Mutex>)
    info!("Initializing gimbal device: {}", device_path);
    let mut gimbal = gimbal::detect_gimbal(&device_path)?;
    info!("Detected protocol: {}", gimbal.protocol_name());

    // Create shared state manager. The arbitrator config knob (per-source
    // rate limit) flows through the constructor; ZERO disables the
    // throttle, matching `StateManager::new`.
    let state_manager = StateManager::with_min_command_interval(Duration::from_millis(u64::from(
        config.arbitrator.min_command_interval_ms,
    )));

    // Get firmware version and update state
    if let Ok(version) = gimbal.get_version() {
        info!("Firmware version: {}", version);
        state_manager.update_firmware_version(version);
    }

    // Wrap gimbal in Arc<Mutex> for sharing between IPC, MAVLink, and the
    // hot-plug recovery task.
    let gimbal_shared = Arc::new(Mutex::new(gimbal));

    // The MAVLink attitude poll loop pings this when it counts too many
    // consecutive read failures; the reconnect task takes over from there.
    let notify_lost = Arc::new(Notify::new());

    // Pinged by `spawn_critical` whenever a tracked task exits — for any
    // reason. The main loop selects on this alongside SIGINT/SIGTERM so a
    // half-running daemon (e.g. MAVLink failed to bind, IPC socket bind
    // failed) doesn't sit there reporting healthy until something else
    // kicks it.
    let task_died = Arc::new(Notify::new());

    // Track every long-lived task so SIGTERM / SIGINT can drain them with a
    // bounded grace window instead of letting the runtime drop kill them
    // mid-syscall.
    let mut tasks: Vec<(&'static str, JoinHandle<()>)> = Vec::new();

    // Hot-plug recovery: scan for a replacement device, open it, swap into
    // the shared gimbal slot. Must run before any user-facing task spawn
    // so an early unplug during start-up still gets recovered.
    {
        let notify = notify_lost.clone();
        let gimbal = gimbal_shared.clone();
        let state = state_manager.clone();
        let configured_path = config.device.path.clone();
        spawn_critical(&mut tasks, "reconnect", &task_died, async move {
            reconnect_task(notify, gimbal, state, configured_path).await;
        });
    }

    // Start IPC server
    let ipc_gimbal = gimbal_shared.clone();
    let ipc_state = state_manager.clone();
    let ipc_config = config.ipc.clone();
    let ipc_socket_path = ipc_config.socket_path.clone();

    spawn_critical(&mut tasks, "ipc", &task_died, async move {
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
        let mavlink_notify_lost = notify_lost.clone();

        spawn_critical(&mut tasks, "mavlink", &task_died, async move {
            let manager = MavlinkManager::new(mavlink_state, mavlink_gimbal, mavlink_notify_lost);
            if let Err(e) = manager.start(mavlink_cfg).await {
                error!("MAVLink manager error: {}", e);
            }
        });
    }

    info!("Daemon started successfully");

    tokio::select! {
        res = wait_for_shutdown_signal() => {
            let name = res?;
            info!("Received {}, beginning bounded shutdown...", name);
        }
        _ = task_died.notified() => {
            warn!("A critical task exited unexpectedly; initiating shutdown...");
        }
    }
    drain_tasks(tasks).await;
    cleanup_ipc_socket(&ipc_socket_path);
    info!("Daemon stopped");

    Ok(())
}

/// Spawn a task whose exit — clean or otherwise — should trigger daemon
/// shutdown. Wraps the inner future so it pings `task_died` after it
/// completes, then stores the resulting `JoinHandle` in `tasks` for
/// later abort + drain.
///
/// Used for `reconnect`, `ipc`, and `mavlink`. If any of these returns
/// (the IPC accept loop hit an unrecoverable error, MAVLink failed to
/// bind, etc.) the main loop wakes up and drains the rest instead of
/// reporting healthy with half the daemon dead.
fn spawn_critical<F>(
    tasks: &mut Vec<(&'static str, JoinHandle<()>)>,
    name: &'static str,
    task_died: &Arc<Notify>,
    fut: F,
) where
    F: std::future::Future<Output = ()> + Send + 'static,
{
    let died = task_died.clone();
    tasks.push((
        name,
        tokio::spawn(async move {
            fut.await;
            died.notify_one();
        }),
    ));
}

/// Abort every tracked task, then wait up to [`SHUTDOWN_GRACE`] for each to
/// actually finish. Tasks that miss the deadline get logged but don't block
/// the rest of shutdown — `JoinHandle::abort` already dropped the future,
/// the worst case is a syscall that hasn't returned yet (e.g. the storm32
/// driver's blocking serial read at the 1 s port timeout).
async fn drain_tasks(tasks: Vec<(&'static str, JoinHandle<()>)>) {
    for (name, handle) in &tasks {
        debug!("Aborting task '{}'", name);
        handle.abort();
    }
    for (name, handle) in tasks {
        match tokio::time::timeout(SHUTDOWN_GRACE, handle).await {
            Ok(Ok(())) => debug!("Task '{}' exited cleanly", name),
            Ok(Err(e)) if e.is_cancelled() => debug!("Task '{}' aborted", name),
            Ok(Err(e)) => warn!("Task '{}' exited with error: {}", name, e),
            Err(_) => warn!(
                "Task '{}' did not exit within {:?}; runtime will drop it",
                name, SHUTDOWN_GRACE
            ),
        }
    }
}

/// Remove the Unix socket file the IPC server bound to. The server itself
/// removes a stale socket on the *next* startup, so leaving the file behind
/// isn't fatal — but a clean shutdown shouldn't litter `/tmp`, and removing
/// it here is also what most `systemd.service` units expect when they kill
/// the daemon.
fn cleanup_ipc_socket(path: &str) {
    match std::fs::remove_file(path) {
        Ok(()) => debug!("Removed IPC socket file {}", path),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
        Err(e) => warn!("Failed to remove IPC socket {}: {}", path, e),
    }
}

/// Hot-plug recovery: wait until the attitude poll loop reports the device
/// is silent, then keep retrying detection until it comes back.
///
/// `configured_path` is whatever the user wrote in the YAML — `"auto"` means
/// rescan via `DeviceScanner`, anything else is a fixed path. We always
/// honor that knob; if the user pinned `/dev/ttyACM0` we don't silently
/// switch to a different device just because something else is plugged in.
///
/// On a successful re-open we swap the new `Box<dyn GimbalDevice>` into the
/// shared `Arc<Mutex<...>>` slot; every other task (IPC, MAVLink, attitude
/// poll) keeps holding the same `Arc` and sees the new gimbal on its next
/// `lock().await`. The lock is held only for the swap itself — scanning and
/// opening happen unlocked so concurrent commands keep failing fast against
/// the dead handle instead of piling up behind us.
async fn reconnect_task(
    notify_lost: Arc<Notify>,
    gimbal: Arc<Mutex<Box<dyn GimbalDevice>>>,
    state_manager: StateManager,
    configured_path: String,
) {
    loop {
        notify_lost.notified().await;
        warn!(
            "Reconnect: attitude loop signaled device lost; beginning recovery (configured path: {})",
            configured_path
        );

        let mut backoff = Duration::from_millis(500);
        const MAX_BACKOFF: Duration = Duration::from_secs(10);
        let mut attempt: u32 = 0;

        let new_gimbal = loop {
            attempt = attempt.wrapping_add(1);

            let path = if configured_path == "auto" {
                DeviceScanner::new().find_storm32_device()
            } else if std::path::Path::new(&configured_path).exists() {
                Some(configured_path.clone())
            } else {
                None
            };

            if let Some(path) = path {
                match gimbal::detect_gimbal(&path) {
                    Ok(g) => {
                        info!(
                            "Reconnect: re-opened gimbal at {} after {} attempts",
                            path, attempt
                        );
                        break g;
                    }
                    Err(e) => debug!("Reconnect attempt {} on {} failed: {}", attempt, path, e),
                }
            } else {
                debug!("Reconnect attempt {}: no candidate device found", attempt);
            }

            tokio::time::sleep(backoff).await;
            backoff = (backoff * 2).min(MAX_BACKOFF);
        };

        let mut new_gimbal = new_gimbal;
        if let Ok(version) = new_gimbal.get_version() {
            info!("Reconnect: firmware version is {}", version);
            state_manager.update_firmware_version(version);
        }

        // Swap last so concurrent ops can't see a half-initialized box.
        *gimbal.lock().await = new_gimbal;
        info!("Reconnect: gimbal swap complete; resuming normal operation");
    }
}

/// Block until SIGINT (`Ctrl-C`) or, on Unix, SIGTERM.
///
/// systemd / docker / k8s all send SIGTERM as the default stop signal;
/// listening only for `ctrl_c` left every container or service-managed
/// deployment relying on the kernel's "delivered SIGKILL after the grace
/// period" fallback, which means in-flight serial I/O can't drain and the
/// IPC socket file is left dangling on disk.
async fn wait_for_shutdown_signal() -> crate::error::Result<&'static str> {
    #[cfg(unix)]
    {
        use tokio::signal::unix::{signal, SignalKind};
        let mut sigterm = signal(SignalKind::terminate())?;
        tokio::select! {
            _ = tokio::signal::ctrl_c() => Ok("SIGINT"),
            _ = sigterm.recv() => Ok("SIGTERM"),
        }
    }
    #[cfg(not(unix))]
    {
        tokio::signal::ctrl_c().await?;
        Ok("Ctrl-C")
    }
}
