//! Periodic broadcasts and the attitude poll loop. Each task is spawned
//! independently from `mavlink_manager.rs::start_udp` and runs forever
//! until the daemon's bounded shutdown aborts its `JoinHandle`.

use super::build::{
    broadcast_to_peers, build_attitude_status_data, build_gimbal_manager_status_data,
};
use crate::daemon::config::MavlinkConfig;
use crate::daemon::gimbal_handle::GimbalHandle;
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
use tokio::sync::Notify;
use tokio::time;
use tracing::{info, warn};

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

/// Attitude poll + `GIMBAL_DEVICE_ATTITUDE_STATUS` broadcast loop.
///
/// Runs at a low fixed cadence (~4 Hz) and is the **only** writer of
/// `state.{yaw,pitch,roll}` — see the comment in
/// `Arbitrator::process_command` for why commanded values are deliberately
/// not mirrored into state. On poll error we skip both the state update and
/// the broadcast so consumers never see a stale-pretending-to-be-fresh
/// value, and we count consecutive failures so a full unplug can be logged
/// once instead of every tick.
///
/// `MissedTickBehavior::Skip` keeps the cadence honest if the gimbal mutex
/// is held longer than the interval (e.g. a slow `set_attitude` queueing
/// behind us) — we don't queue up backlogged ticks.
#[allow(clippy::too_many_arguments)]
pub(in crate::daemon::mavlink_manager) async fn attitude_loop(
    socket: Arc<UdpSocket>,
    peers: Peers,
    seq: Arc<AtomicU8>,
    state_manager: StateManager,
    config: MavlinkConfig,
    gimbal: GimbalHandle,
    start_time: Instant,
    notify_lost: Arc<Notify>,
) {
    /// Number of consecutive poll failures (~250 ms each) before we declare
    /// the device gone and ask the reconnect task to take over. 8 ticks =
    /// ~2 seconds of silence, long enough to ride out a transient serial
    /// hiccup but short enough that an unplug is noticed promptly.
    const FAILURE_THRESHOLD: u32 = 8;
    /// How long after the last failed poll the next *successful* broadcast
    /// still carries `COMMS_ERROR`. Solid devices clear within one tick;
    /// flaky devices keep the flag set so a GCS sees `degraded` instead
    /// of `healthy` on every other frame.
    const COMMS_ERROR_LINGER: Duration = Duration::from_secs(1);

    let mut interval = time::interval(Duration::from_millis(250));
    interval.set_missed_tick_behavior(time::MissedTickBehavior::Skip);
    // We've waited 250ms for the first tick — fire immediately instead.
    interval.tick().await;
    let mut consecutive_failures: u32 = 0;
    let mut last_failure: Option<Instant> = None;

    loop {
        interval.tick().await;

        let attitude = gimbal.get_attitude().await;

        let attitude = match attitude {
            Ok(a) => {
                if consecutive_failures > 0 {
                    info!(
                        "Attitude readback recovered after {} consecutive failures",
                        consecutive_failures
                    );
                }
                consecutive_failures = 0;
                a
            }
            Err(e) => {
                consecutive_failures = consecutive_failures.wrapping_add(1);
                last_failure = Some(Instant::now());
                if consecutive_failures == 1 || consecutive_failures % 20 == 0 {
                    warn!(
                        "Attitude readback failed (#{}): {}",
                        consecutive_failures, e
                    );
                }
                if consecutive_failures == FAILURE_THRESHOLD {
                    warn!(
                        "Gimbal silent for {} consecutive polls; signaling reconnect",
                        FAILURE_THRESHOLD
                    );
                    notify_lost.notify_one();
                }
                continue;
            }
        };

        state_manager.update_measured_angles(attitude.yaw, attitude.pitch, attitude.roll);

        // A *recent* failure (this poll succeeded but one within the last
        // second didn't) means the link is flaky. Surface that to the GCS
        // via COMMS_ERROR rather than letting an in-between healthy frame
        // mask the instability.
        let failure_flags = match last_failure {
            Some(t) if t.elapsed() < COMMS_ERROR_LINGER => {
                GimbalDeviceErrorFlags::GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR
            }
            _ => GimbalDeviceErrorFlags::empty(),
        };

        let msg = MavMessage::GIMBAL_DEVICE_ATTITUDE_STATUS(build_attitude_status_data(
            &attitude,
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
