//! Pure constructors for outbound MAVLink frames plus the single low-level
//! UDP send and the fan-out helper that consumes them.
//!
//! Everything here is either a pure function (unit-testable in isolation)
//! or a thin async wrapper around `socket.send_to`. The loops in
//! [`super::loops`] and the inbound handlers in [`super::inbound`] assemble
//! frames here and forget — there is no module-level state.

use super::super::math::euler_deg_to_quaternion_wxyz;
use crate::daemon::config::MavlinkConfig;
use crate::daemon::mavlink_manager::{Peers, RxCtx};
use crate::daemon::state::StateManager;
use crate::gimbal::Attitude;
use mavlink::ardupilotmega::{
    GimbalDeviceErrorFlags, GimbalDeviceFlags, GimbalManagerFlags, MavCmd, MavMessage, MavResult,
    COMMAND_ACK_DATA, GIMBAL_DEVICE_ATTITUDE_STATUS_DATA, GIMBAL_MANAGER_STATUS_DATA,
};
use std::net::SocketAddr;
use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::Arc;
use std::time::Instant;
use tokio::net::UdpSocket;
use tracing::{debug, error, warn};

/// Standard MAVLink message IDs the manager can serve on demand.
///
/// `GIMBAL_DEVICE_ATTITUDE_STATUS` (285) is intentionally not handled here:
/// the 4 Hz attitude loop already broadcasts it to every recorded peer, so
/// any GCS that requests one will get the next periodic frame within
/// 250 ms. Acknowledging the request without doing anything would be a lie;
/// we'd rather fall through to `MAV_RESULT_DENIED` and let the GCS know to
/// just listen.
pub(super) mod msg_id {
    /// `GIMBAL_MANAGER_INFORMATION` — capability + limit advertisement.
    pub(in crate::daemon::mavlink_manager) const GIMBAL_MANAGER_INFORMATION: u32 = 280;
    /// `GIMBAL_MANAGER_STATUS` — primary/secondary control ownership.
    pub(in crate::daemon::mavlink_manager) const GIMBAL_MANAGER_STATUS: u32 = 281;
}

/// Convert a `COMMAND_LONG.param1` to a message id. NaN / negative / very
/// large values collapse to `None` — the caller will deny the request.
pub(super) fn message_id_from_param(p: f32) -> Option<u32> {
    if !p.is_finite() || p < 0.0 || p > u32::MAX as f32 {
        return None;
    }
    Some(p.round() as u32)
}

/// Build a `GIMBAL_MANAGER_STATUS` frame from the current state. Used both
/// by the 5 Hz publish loop and the on-demand `MAV_CMD_REQUEST_MESSAGE`
/// responder, so periodic and on-demand frames stay byte-identical.
///
/// `flags` advertises `YAW_IN_VEHICLE_FRAME` to mirror what
/// `GIMBAL_DEVICE_ATTITUDE_STATUS` actually publishes. `GimbalManagerFlags::default()`
/// is *not* used here — rust-mavlink generates the default as the first
/// bitflag variant, which happens to be `RETRACT`. Letting that leak out
/// would make every `GIMBAL_MANAGER_STATUS` frame falsely advertise the
/// gimbal as retracted to any GCS / autopilot reading the bus.
pub(super) fn build_gimbal_manager_status_data(
    state_manager: &StateManager,
    config: &MavlinkConfig,
    start_time: Instant,
) -> GIMBAL_MANAGER_STATUS_DATA {
    let pc = state_manager.get_primary_control();
    GIMBAL_MANAGER_STATUS_DATA {
        time_boot_ms: start_time.elapsed().as_millis() as u32,
        flags: GimbalManagerFlags::GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME,
        gimbal_device_id: config.compid,
        primary_control_sysid: pc.primary_sysid,
        primary_control_compid: pc.primary_compid,
        secondary_control_sysid: pc.secondary_sysid,
        secondary_control_compid: pc.secondary_compid,
    }
}

/// Build a `GIMBAL_DEVICE_ATTITUDE_STATUS_DATA` from a measured attitude.
///
/// `target_system / target_component = 0` — the manager broadcasts this to
/// every recorded peer, not to a single addressed system. `flags` is set
/// to `YAW_IN_VEHICLE_FRAME` to be honest about what the manager actually
/// supports today; earth-frame yaw needs vehicle-attitude integration that
/// isn't wired up. `failure_flags` is supplied by the caller — see the
/// attitude loop for the COMMS_ERROR linger window. The persistent
/// "device is dead" failure mode is still expressed by stopping the
/// broadcast entirely, not by setting flags on a stale frame.
pub(super) fn build_attitude_status_data(
    attitude: &Attitude,
    start_time: Instant,
    failure_flags: GimbalDeviceErrorFlags,
) -> GIMBAL_DEVICE_ATTITUDE_STATUS_DATA {
    let q = euler_deg_to_quaternion_wxyz(attitude.pitch, attitude.roll, attitude.yaw);
    GIMBAL_DEVICE_ATTITUDE_STATUS_DATA {
        time_boot_ms: start_time.elapsed().as_millis() as u32,
        target_system: 0,
        target_component: 0,
        flags: GimbalDeviceFlags::GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME,
        q,
        angular_velocity_x: f32::NAN,
        angular_velocity_y: f32::NAN,
        angular_velocity_z: f32::NAN,
        failure_flags,
    }
}

/// Send one fully-built MAVLink frame to a single peer. The single point
/// where outbound frames meet the wire — every other helper in this file
/// goes through this function so the sequence-counter advance is uniform.
pub(super) async fn send_mavlink_message(
    socket: &Arc<UdpSocket>,
    dest_addr: SocketAddr,
    sysid: u8,
    compid: u8,
    seq: &Arc<AtomicU8>,
    msg: MavMessage,
) -> crate::error::Result<()> {
    let header = mavlink::MavHeader {
        system_id: sysid,
        component_id: compid,
        // MAVLink sequence is an 8-bit counter that wraps; fetch_add on
        // AtomicU8 wraps by definition, matching the CLAUDE.md ban on
        // plain `+= 1` on monotonic counters.
        sequence: seq.fetch_add(1, Ordering::Relaxed),
    };

    let mut buf = Vec::new();
    mavlink::write_v2_msg(&mut buf, header, &msg)?;

    socket.send_to(&buf, dest_addr).await?;
    Ok(())
}

/// Broadcast a single frame to every recorded peer. Each broadcast loop
/// (heartbeat, gimbal-manager-status, attitude) shares this fan-out so the
/// "lock peers, snapshot, drop lock, send each, log per-peer failure"
/// pattern lives in one place. `label` is used in the warn message on
/// per-peer send failure.
pub(super) async fn broadcast_to_peers(
    socket: &Arc<UdpSocket>,
    peers: &Peers,
    config: &MavlinkConfig,
    seq: &Arc<AtomicU8>,
    msg: MavMessage,
    label: &'static str,
) {
    let targets: Vec<SocketAddr> = peers.lock().await.iter().copied().collect();
    if targets.is_empty() {
        return;
    }
    for peer in targets {
        if let Err(e) =
            send_mavlink_message(socket, peer, config.sysid, config.compid, seq, msg.clone()).await
        {
            warn!("{} to {} failed: {}", label, peer, e);
        }
    }
}

/// Send `GIMBAL_MANAGER_INFORMATION` to a single requester. Capability
/// flags here describe what the manager actually does today — see the
/// inline comment for the rationale.
pub(super) async fn send_gimbal_manager_information(
    socket: &Arc<UdpSocket>,
    dest_addr: SocketAddr,
    config: &MavlinkConfig,
    seq: &Arc<AtomicU8>,
    start_time: Instant,
) {
    use mavlink::ardupilotmega::GimbalManagerCapFlags as Caps;

    // Capabilities the Storm32-backed manager actually has. We advertise
    // all three axes (Storm32 is a 3-axis gimbal), follow / lock per axis
    // (covered by SetPanMode 0..=5), retract via SetStandby, neutral via
    // recenter_*, and RC-style inputs (the protocol's native command set).
    // We do NOT advertise infinite-yaw, earth-frame yaw, or point-at-location:
    // earth-frame yaw needs vehicle attitude integration we don't have, and
    // point-at-location needs GPS / coordinate-frame plumbing.
    let cap_flags = Caps::GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT
        | Caps::GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL
        | Caps::GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS
        | Caps::GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW
        | Caps::GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK
        | Caps::GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS
        | Caps::GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW
        | Caps::GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK
        | Caps::GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS
        | Caps::GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW
        | Caps::GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK
        | Caps::GIMBAL_MANAGER_CAP_FLAGS_HAS_RC_INPUTS;

    let limits = &config.limits;
    let info = mavlink::ardupilotmega::GIMBAL_MANAGER_INFORMATION_DATA {
        time_boot_ms: start_time.elapsed().as_millis() as u32,
        cap_flags,
        gimbal_device_id: config.compid,
        roll_min: limits.roll_min_deg.to_radians(),
        roll_max: limits.roll_max_deg.to_radians(),
        pitch_min: limits.pitch_min_deg.to_radians(),
        pitch_max: limits.pitch_max_deg.to_radians(),
        yaw_min: limits.yaw_min_deg.to_radians(),
        yaw_max: limits.yaw_max_deg.to_radians(),
    };

    let msg = MavMessage::GIMBAL_MANAGER_INFORMATION(info);

    match send_mavlink_message(socket, dest_addr, config.sysid, config.compid, seq, msg).await {
        Ok(_) => debug!("Sent GIMBAL_MANAGER_INFORMATION"),
        Err(e) => error!("Failed to send GIMBAL_MANAGER_INFORMATION: {}", e),
    }
}

/// Send a `COMMAND_ACK` back to the originator of a `COMMAND_LONG`.
pub(super) async fn send_command_ack(
    dest_addr: SocketAddr,
    ctx: &RxCtx<'_>,
    command: MavCmd,
    result: MavResult,
) {
    let ack = COMMAND_ACK_DATA { command, result };
    let msg = MavMessage::COMMAND_ACK(ack);
    if let Err(e) = send_mavlink_message(
        ctx.socket,
        dest_addr,
        ctx.config.sysid,
        ctx.config.compid,
        ctx.seq,
        msg,
    )
    .await
    {
        error!("Failed to send COMMAND_ACK: {}", e);
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::super::super::math::quaternion_wxyz_to_euler_deg;
    use super::*;

    fn approx_eq(a: f32, b: f32, eps: f32) -> bool {
        (a - b).abs() < eps
    }

    #[test]
    fn build_attitude_status_round_trips_quaternion() {
        let attitude = Attitude {
            pitch: 12.0,
            roll: -7.5,
            yaw: 33.25,
        };
        let data =
            build_attitude_status_data(&attitude, Instant::now(), GimbalDeviceErrorFlags::empty());

        // Decoding the quaternion back must recover the input Euler angles.
        // This catches both the math.rs helper and our use of it.
        let (p, r, y) = quaternion_wxyz_to_euler_deg(data.q);
        assert!(approx_eq(p, attitude.pitch, 1e-3), "pitch: got {p}");
        assert!(approx_eq(r, attitude.roll, 1e-3), "roll: got {r}");
        assert!(approx_eq(y, attitude.yaw, 1e-3), "yaw: got {y}");
    }

    #[test]
    fn build_attitude_status_uses_broadcast_addressing_and_vehicle_frame() {
        let data = build_attitude_status_data(
            &Attitude {
                pitch: 0.0,
                roll: 0.0,
                yaw: 0.0,
            },
            Instant::now(),
            GimbalDeviceErrorFlags::empty(),
        );

        // Manager broadcasts to every peer, not a single addressed system.
        assert_eq!(data.target_system, 0);
        assert_eq!(data.target_component, 0);

        // Honest about what we support: vehicle-frame yaw, no earth-frame
        // tracking, no rate path.
        assert!(data
            .flags
            .contains(GimbalDeviceFlags::GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME));
        assert!(data.angular_velocity_x.is_nan());
        assert!(data.angular_velocity_y.is_nan());
        assert!(data.angular_velocity_z.is_nan());

        // No failures supplied → frame is honestly clean. The persistent
        // "device is dead" failure mode is expressed by stopping the
        // broadcast entirely (see attitude_loop), not by setting flags
        // on a stale frame.
        assert!(data.failure_flags.is_empty());
    }

    #[test]
    fn build_manager_status_does_not_advertise_retract() {
        // Regression: `GimbalManagerFlags::default()` is rust-mavlink's
        // first-bitflag default, which is `GIMBAL_MANAGER_FLAGS_RETRACT`.
        // Letting it leak out makes every periodic / on-demand status
        // frame falsely tell GCSs that the gimbal is retracted.
        let cfg = MavlinkConfig {
            enabled: true,
            transport: Default::default(),
            bind_addr: "0.0.0.0".parse().unwrap(),
            udp_port: 14550,
            sysid: 1,
            compid: 154,
            limits: Default::default(),
        };
        let state = StateManager::new();
        let data = build_gimbal_manager_status_data(&state, &cfg, Instant::now());

        assert!(
            !data
                .flags
                .contains(GimbalManagerFlags::GIMBAL_MANAGER_FLAGS_RETRACT),
            "flags must not advertise RETRACT (got {:?})",
            data.flags
        );
        assert!(
            data.flags
                .contains(GimbalManagerFlags::GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME),
            "flags should mirror GIMBAL_DEVICE_ATTITUDE_STATUS frame (got {:?})",
            data.flags
        );
    }

    #[test]
    fn build_attitude_status_propagates_caller_failure_flags() {
        // The attitude loop sets COMMS_ERROR for ~1 s after any failed
        // poll so a flaky-link recovery doesn't slip through as a fully
        // healthy frame.
        let data = build_attitude_status_data(
            &Attitude {
                pitch: 0.0,
                roll: 0.0,
                yaw: 0.0,
            },
            Instant::now(),
            GimbalDeviceErrorFlags::GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR,
        );
        assert!(data
            .failure_flags
            .contains(GimbalDeviceErrorFlags::GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR));
    }
}
