//! Periodic broadcasts and the attitude poll loop. Each task is spawned
//! independently from `mavlink_manager.rs::start_udp` and runs forever
//! until the daemon's bounded shutdown aborts its `JoinHandle`.

use super::build::{
    broadcast_to_peers, broadcast_to_peers_if_current, build_attitude_status_data,
    build_gimbal_manager_status_data,
};
use crate::daemon::attitude::{AttitudeTick, MAX_MEASUREMENT_AGE};
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
use tracing::{error, warn};

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
/// [`crate::daemon::attitude::attitude_poll_loop`]. This loop
/// validates measurement evidence and translates it into the wire frame.
/// `time_boot_ms` retains monotonic acquisition time relative to manager startup;
/// unknown, future, stale, or pre-start samples cannot become measured broadcasts.
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
    mut attitude_rx: broadcast::Receiver<Arc<AttitudeTick>>,
    state: StateManager,
) {
    let mut dropped = 0_u64;
    let mut last_sample = None;
    loop {
        let Some(tick) = receive_attitude(&mut attitude_rx, &mut dropped).await else {
            return;
        };

        let sample = &tick.observation;
        let epoch = state.state_cached().connection_epoch;
        let Some(sampled_at) = sample.measurement_time(Instant::now(), MAX_MEASUREMENT_AGE) else {
            continue;
        };
        if sample.epoch != epoch
            || sample.frame != turret_core::ReferenceFrame::Vehicle
            || last_sample
                .is_some_and(|(prior_epoch, prior)| prior_epoch == epoch && sampled_at <= prior)
        {
            continue;
        }
        let Some(since_start) = sampled_at.checked_duration_since(start_time) else {
            continue;
        };
        let Ok(attitude) = sample.complete_attitude() else {
            continue;
        };
        last_sample = Some((epoch, sampled_at));
        let failure_flags = if tick.link_recently_failed {
            GimbalDeviceErrorFlags::GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR
        } else {
            GimbalDeviceErrorFlags::empty()
        };

        let msg = MavMessage::GIMBAL_DEVICE_ATTITUDE_STATUS(build_attitude_status_data(
            &attitude,
            since_start.as_millis() as u32,
            failure_flags,
        ));
        broadcast_to_peers_if_current(
            &socket,
            &peers,
            &config,
            &seq,
            msg,
            "GIMBAL_DEVICE_ATTITUDE_STATUS",
            || {
                sample.epoch == state.state_cached().connection_epoch
                    && sample
                        .measurement_time(Instant::now(), MAX_MEASUREMENT_AGE)
                        .is_some()
            },
        )
        .await;
    }
}

async fn receive_attitude(
    receiver: &mut broadcast::Receiver<Arc<AttitudeTick>>,
    dropped: &mut u64,
) -> Option<Arc<AttitudeTick>> {
    loop {
        match receiver.recv().await {
            Ok(tick) => return Some(tick),
            Err(broadcast::error::RecvError::Closed) => {
                warn!("attitude channel closed");
                return None;
            }
            Err(broadcast::error::RecvError::Lagged(count)) => {
                *dropped = dropped.saturating_add(count);
                error!(
                    count,
                    total = *dropped,
                    "attitude observations lost by MAVLink broadcaster"
                );
            }
        }
    }
}

#[cfg(test)]
mod tests;
