//! Periodic broadcasts and the attitude poll loop. Each task is spawned
//! independently from `mavlink_manager.rs::start_udp` and runs forever
//! until the daemon's bounded shutdown aborts its `JoinHandle`.

use super::build::{
    broadcast_to_peers, build_attitude_status_data, build_gimbal_manager_status_data,
};
use crate::daemon::attitude::AttitudeTick;
use crate::daemon::config::MavlinkConfig;
use crate::daemon::mavlink_manager::Peers;
use crate::daemon::state::StateManager;
use mavlink::ardupilotmega::{
    GimbalDeviceErrorFlags, MavAutopilot, MavMessage, MavModeFlag, MavState, MavType,
    HEARTBEAT_DATA,
};
use std::sync::atomic::AtomicU8;
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::net::UdpSocket;
use tokio::sync::broadcast;
use tokio::time;
use tracing::warn;

/// Broadcast `HEARTBEAT` at 1 Hz to every recorded peer.
pub(in crate::daemon::mavlink_manager) async fn heartbeat_loop(
    socket: Arc<UdpSocket>,
    peers: Peers,
    seq: Arc<AtomicU8>,
    config: MavlinkConfig,
) {
    let mut interval = time::interval(Duration::from_secs(1));
    loop {
        interval.tick().await;
        let hb = HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: MavType::MAV_TYPE_GIMBAL,
            autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
            base_mode: MavModeFlag::empty(),
            system_status: MavState::MAV_STATE_ACTIVE,
            mavlink_version: 3,
        };
        broadcast_to_peers(
            &socket,
            &peers,
            &config,
            &seq,
            MavMessage::HEARTBEAT(hb),
            "HEARTBEAT",
        )
        .await;
    }
}

/// Broadcast `GIMBAL_MANAGER_STATUS` at 5 Hz to every recorded peer.
pub(in crate::daemon::mavlink_manager) async fn publish_status_loop(
    socket: Arc<UdpSocket>,
    peers: Peers,
    seq: Arc<AtomicU8>,
    state_manager: StateManager,
    config: MavlinkConfig,
    start_time: Instant,
) {
    let mut interval = time::interval(Duration::from_millis(200));
    loop {
        interval.tick().await;
        let msg = MavMessage::GIMBAL_MANAGER_STATUS(build_gimbal_manager_status_data(
            &state_manager,
            &config,
            start_time,
        ));
        broadcast_to_peers(&socket, &peers, &config, &seq, msg, "GIMBAL_MANAGER_STATUS").await;
    }
}

/// `GIMBAL_DEVICE_ATTITUDE_STATUS` broadcast loop. Subscribes to the
/// daemon's [`AttitudeTick`] broadcast channel and emits one MAVLink
/// frame per tick.
///
/// The hardware poll, state update, failure counter, reconnect
/// signaling, and yaw corrector all live in the daemon-level
/// [`crate::daemon::attitude::attitude_poll_loop`]. This loop is
/// purely the MAVLink-side translator — given a tick, build and send
/// the wire frame.
///
/// `link_recently_failed` from the tick maps to `COMMS_ERROR` so a
/// flaky link reads as "degraded" to the GCS rather than appearing
/// healthy on every other frame.
pub(in crate::daemon::mavlink_manager) async fn attitude_broadcast_loop(
    socket: Arc<UdpSocket>,
    peers: Peers,
    seq: Arc<AtomicU8>,
    config: MavlinkConfig,
    start_time: Instant,
    mut attitude_rx: broadcast::Receiver<AttitudeTick>,
) {
    loop {
        let tick = match attitude_rx.recv().await {
            Ok(t) => t,
            Err(broadcast::error::RecvError::Closed) => {
                // Poll loop has exited — no more ticks ever. Stop
                // cleanly; spawn_critical will pick up the join.
                warn!("Attitude broadcast channel closed; broadcast loop exiting");
                return;
            }
            Err(broadcast::error::RecvError::Lagged(n)) => {
                // Subscriber fell behind the channel buffer. Skip the
                // gap and resume on the next tick — we'd rather drop
                // a few stale frames than block the publisher.
                warn!(
                    "GIMBAL_DEVICE_ATTITUDE_STATUS broadcast lagged by {} ticks",
                    n
                );
                continue;
            }
        };

        let failure_flags = if tick.link_recently_failed {
            GimbalDeviceErrorFlags::GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR
        } else {
            GimbalDeviceErrorFlags::empty()
        };

        let msg = MavMessage::GIMBAL_DEVICE_ATTITUDE_STATUS(build_attitude_status_data(
            &tick.attitude,
            start_time,
            failure_flags,
        ));
        broadcast_to_peers(
            &socket,
            &peers,
            &config,
            &seq,
            msg,
            "GIMBAL_DEVICE_ATTITUDE_STATUS",
        )
        .await;
    }
}
