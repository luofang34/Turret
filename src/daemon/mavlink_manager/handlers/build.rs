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

/// Build manager ownership status in vehicle coordinates. Motor standby does
/// not establish a measured retract pose and therefore never sets RETRACT.
pub(super) fn build_gimbal_manager_status_data(
    state_manager: &StateManager,
    config: &MavlinkConfig,
    start_time: Instant,
) -> GIMBAL_MANAGER_STATUS_DATA {
    let pc = state_manager.primary_control_cached();
    let flags = GimbalManagerFlags::GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME;
    GIMBAL_MANAGER_STATUS_DATA {
        time_boot_ms: start_time.elapsed().as_millis() as u32,
        flags,
        gimbal_device_id: config.compid,
        primary_control_sysid: pc.primary_sysid,
        primary_control_compid: pc.primary_compid,
        secondary_control_sysid: pc.secondary_sysid,
        secondary_control_compid: pc.secondary_compid,
    }
}

/// Build a measured attitude message with its validated acquisition timestamp.
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
    sample_boot_ms: u32,
    failure_flags: GimbalDeviceErrorFlags,
) -> GIMBAL_DEVICE_ATTITUDE_STATUS_DATA {
    let q = euler_deg_to_quaternion_wxyz(attitude.pitch, attitude.roll, attitude.yaw);
    GIMBAL_DEVICE_ATTITUDE_STATUS_DATA {
        time_boot_ms: sample_boot_ms,
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
    send_mavlink_message_if_current(socket, dest_addr, sysid, compid, seq, msg, || true).await
}

async fn send_mavlink_message_if_current(
    socket: &Arc<UdpSocket>,
    dest_addr: SocketAddr,
    sysid: u8,
    compid: u8,
    seq: &Arc<AtomicU8>,
    msg: MavMessage,
    is_current: impl Fn() -> bool,
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

    loop {
        socket.writable().await?;
        if !is_current() {
            return Ok(());
        }
        match socket.try_send_to(&buf, dest_addr) {
            Ok(_) => return Ok(()),
            Err(error) if error.kind() == std::io::ErrorKind::WouldBlock => continue,
            Err(error) => return Err(error.into()),
        }
    }
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
    broadcast_to_peers_if_current(socket, peers, config, seq, msg, label, || true).await;
}

/// Revalidate sampled evidence immediately before each nonblocking UDP write.
pub(super) async fn broadcast_to_peers_if_current(
    socket: &Arc<UdpSocket>,
    peers: &Peers,
    config: &MavlinkConfig,
    seq: &Arc<AtomicU8>,
    msg: MavMessage,
    label: &'static str,
    is_current: impl Fn() -> bool,
) {
    let targets: Vec<SocketAddr> = peers.lock().await.keys().copied().collect();
    if targets.is_empty() {
        return;
    }
    for peer in targets {
        if let Err(e) = send_mavlink_message_if_current(
            socket,
            peer,
            config.sysid,
            config.compid,
            seq,
            msg.clone(),
            &is_current,
        )
        .await
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
    gimbal: &crate::daemon::gimbal_handle::GimbalHandle,
) {
    let capabilities = match gimbal.capabilities().await {
        Ok(capabilities) => capabilities,
        Err(error) => {
            error!(%error, "cannot advertise unknown backend capabilities");
            return;
        }
    };
    let info = build_manager_information(&capabilities, config, start_time);

    let msg = MavMessage::GIMBAL_MANAGER_INFORMATION(info);

    match send_mavlink_message(socket, dest_addr, config.sysid, config.compid, seq, msg).await {
        Ok(_) => debug!("Sent GIMBAL_MANAGER_INFORMATION"),
        Err(e) => error!("Failed to send GIMBAL_MANAGER_INFORMATION: {}", e),
    }
}

fn build_manager_information(
    capabilities: &turret_core::PayloadCapabilities,
    config: &MavlinkConfig,
    start_time: Instant,
) -> mavlink::ardupilotmega::GIMBAL_MANAGER_INFORMATION_DATA {
    use mavlink::ardupilotmega::GimbalManagerCapFlags as Caps;
    use turret_core::Capability;
    let axes = &capabilities.gimbal.axes;
    let position = |axis: &turret_core::AxisCapabilities| match axis.position {
        Capability::Available(mode)
        | Capability::Unavailable {
            capability: mode, ..
        } => Some(mode),
        _ => None,
    };
    let mut cap_flags = Caps::empty();
    for (axis, flag) in [
        (&axes.pitch, Caps::GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS),
        (&axes.roll, Caps::GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS),
        (&axes.yaw, Caps::GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS),
    ] {
        if position(axis).is_some() {
            cap_flags |= flag;
        }
    }
    let bounds = |axis| {
        position(axis)
            .map(|mode| {
                (
                    mode.range.minimum.as_radians(),
                    mode.range.maximum.as_radians(),
                )
            })
            .unwrap_or((f32::NAN, f32::NAN))
    };
    let (pitch_min, pitch_max) = bounds(&axes.pitch);
    let (roll_min, roll_max) = bounds(&axes.roll);
    let (yaw_min, yaw_max) = bounds(&axes.yaw);
    mavlink::ardupilotmega::GIMBAL_MANAGER_INFORMATION_DATA {
        time_boot_ms: start_time.elapsed().as_millis() as u32,
        cap_flags,
        gimbal_device_id: config.compid,
        roll_min,
        roll_max,
        pitch_min,
        pitch_max,
        yaw_min,
        yaw_max,
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
mod tests;
