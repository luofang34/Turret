//! Per-message MAVLink handlers and outbound helpers for
//! [`crate::daemon::mavlink_manager::MavlinkManager`].
//!
//! This module holds everything that happens *after* a UDP datagram has been
//! parsed into a `MavMessage` or during the periodic broadcast loops. The
//! parent module owns the socket / task lifecycle; this one is frame logic.

use super::math::{quaternion_wxyz_to_euler_deg, to_id_byte};
use super::{Peers, RxCtx};
use crate::daemon::config::MavlinkConfig;
use crate::daemon::models::{CommandMode, ControlSource, GimbalCommand, PrimaryControl};
use crate::daemon::state::StateManager;
use log::{debug, error, info, warn};
use mavlink::ardupilotmega::{
    GimbalManagerFlags, MavAutopilot, MavCmd, MavMessage, MavModeFlag, MavResult, MavState,
    MavType, COMMAND_ACK_DATA, COMMAND_LONG_DATA, GIMBAL_MANAGER_STATUS_DATA, HEARTBEAT_DATA,
};
use std::net::SocketAddr;
use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::net::UdpSocket;
use tokio::time;

/// Broadcast `HEARTBEAT` at 1 Hz to every recorded peer.
pub(super) async fn heartbeat_loop(
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
        let msg = MavMessage::HEARTBEAT(hb);

        let targets: Vec<SocketAddr> = peers.lock().await.iter().copied().collect();
        if targets.is_empty() {
            continue;
        }
        for peer in targets {
            if let Err(e) = send_mavlink_message(
                &socket,
                peer,
                config.sysid,
                config.compid,
                &seq,
                msg.clone(),
            )
            .await
            {
                warn!("HEARTBEAT to {} failed: {}", peer, e);
            }
        }
    }
}

/// Broadcast `GIMBAL_MANAGER_STATUS` at 5 Hz to every recorded peer.
pub(super) async fn publish_status_loop(
    socket: Arc<UdpSocket>,
    peers: Peers,
    seq: Arc<AtomicU8>,
    state_manager: StateManager,
    config: MavlinkConfig,
    start_time: Instant,
) {
    let mut interval = time::interval(Duration::from_millis(200)); // 5 Hz
    loop {
        interval.tick().await;

        let pc = state_manager.get_primary_control();
        let status = GIMBAL_MANAGER_STATUS_DATA {
            time_boot_ms: start_time.elapsed().as_millis() as u32,
            flags: GimbalManagerFlags::default(),
            gimbal_device_id: config.compid,
            primary_control_sysid: pc.primary_sysid,
            primary_control_compid: pc.primary_compid,
            secondary_control_sysid: pc.secondary_sysid,
            secondary_control_compid: pc.secondary_compid,
        };
        let msg = MavMessage::GIMBAL_MANAGER_STATUS(status);

        let targets: Vec<SocketAddr> = peers.lock().await.iter().copied().collect();
        if targets.is_empty() {
            continue;
        }
        for peer in targets {
            if let Err(e) = send_mavlink_message(
                &socket,
                peer,
                config.sysid,
                config.compid,
                &seq,
                msg.clone(),
            )
            .await
            {
                warn!("GIMBAL_MANAGER_STATUS to {} failed: {}", peer, e);
            }
        }
    }
}

/// Dispatch a parsed inbound `MavMessage` to a per-type handler.
pub(super) async fn handle_mavlink_message(
    sender_sysid: u8,
    sender_compid: u8,
    src_addr: SocketAddr,
    msg: MavMessage,
    ctx: &RxCtx<'_>,
) {
    match msg {
        MavMessage::GIMBAL_MANAGER_SET_ATTITUDE(m) => {
            on_set_attitude(sender_sysid, sender_compid, m, ctx).await;
        }
        MavMessage::GIMBAL_MANAGER_SET_PITCHYAW(m) => {
            on_set_pitchyaw(sender_sysid, sender_compid, m, ctx).await;
        }
        MavMessage::GIMBAL_MANAGER_SET_MANUAL_CONTROL(m) => {
            on_set_manual(sender_sysid, sender_compid, m, ctx).await;
        }
        MavMessage::GIMBAL_MANAGER_INFORMATION(_) => {
            debug!("Received GIMBAL_MANAGER_INFORMATION request");
            send_gimbal_manager_information(ctx.socket, src_addr, ctx.config, ctx.seq).await;
        }
        MavMessage::COMMAND_LONG(cmd) => {
            on_command_long(sender_sysid, sender_compid, src_addr, cmd, ctx).await;
        }
        _ => debug!("Ignoring MAVLink message type: {:?}", msg),
    }
}

async fn on_set_attitude(
    sender_sysid: u8,
    sender_compid: u8,
    m: mavlink::ardupilotmega::GIMBAL_MANAGER_SET_ATTITUDE_DATA,
    ctx: &RxCtx<'_>,
) {
    debug!("Received GIMBAL_MANAGER_SET_ATTITUDE: {:?}", m);
    if !addressed_to_us(m.target_system, ctx.config.sysid) {
        return;
    }
    if !ctx.state.is_primary_allowed(sender_sysid, sender_compid) {
        warn!(
            "Ignoring GIMBAL_MANAGER_SET_ATTITUDE from non-primary ({},{})",
            sender_sysid, sender_compid
        );
        return;
    }

    // NaN in q means "ignore quaternion, use angular_velocity instead" per
    // spec. Rate control isn't implemented yet, so just drop the command.
    if m.q.iter().any(|c| c.is_nan()) {
        debug!("GIMBAL_MANAGER_SET_ATTITUDE with NaN quaternion — rate path not yet supported");
        return;
    }

    // TODO(item-6): honor frame flags (YAW_IN_VEHICLE_FRAME vs
    // YAW_IN_EARTH_FRAME) and RETRACT / NEUTRAL shortcuts.
    let (pitch_deg, roll_deg, yaw_deg) = quaternion_wxyz_to_euler_deg(m.q);
    let command = GimbalCommand::new(
        ControlSource::Mavlink(sender_sysid),
        CommandMode::Position,
        Some(yaw_deg),
        Some(pitch_deg),
        Some(roll_deg),
    );
    dispatch_gimbal_command(command, ctx, "SET_ATTITUDE").await;
}

async fn on_set_pitchyaw(
    sender_sysid: u8,
    sender_compid: u8,
    m: mavlink::ardupilotmega::GIMBAL_MANAGER_SET_PITCHYAW_DATA,
    ctx: &RxCtx<'_>,
) {
    debug!("Received GIMBAL_MANAGER_SET_PITCHYAW: {:?}", m);
    if !addressed_to_us(m.target_system, ctx.config.sysid) {
        return;
    }
    if !ctx.state.is_primary_allowed(sender_sysid, sender_compid) {
        warn!(
            "Ignoring GIMBAL_MANAGER_SET_PITCHYAW from non-primary ({},{})",
            sender_sysid, sender_compid
        );
        return;
    }

    let pitch_deg = m.pitch.to_degrees();
    let yaw_deg = m.yaw.to_degrees();
    let command = GimbalCommand::new(
        ControlSource::Mavlink(sender_sysid),
        CommandMode::Position,
        Some(yaw_deg),
        Some(pitch_deg),
        Some(0.0),
    );
    dispatch_gimbal_command(command, ctx, "SET_PITCHYAW").await;
}

async fn on_set_manual(
    sender_sysid: u8,
    sender_compid: u8,
    m: mavlink::ardupilotmega::GIMBAL_MANAGER_SET_MANUAL_CONTROL_DATA,
    ctx: &RxCtx<'_>,
) {
    debug!("Received GIMBAL_MANAGER_SET_MANUAL_CONTROL: {:?}", m);
    if !addressed_to_us(m.target_system, ctx.config.sysid) {
        return;
    }
    if !ctx.state.is_primary_allowed(sender_sysid, sender_compid) {
        debug!(
            "Ignoring GIMBAL_MANAGER_SET_MANUAL_CONTROL from non-primary ({},{})",
            sender_sysid, sender_compid
        );
        return;
    }

    // Manual control uses normalized values (-1.0..=1.0). Scale to ±90°.
    let pitch_deg = m.pitch * 90.0;
    let yaw_deg = m.yaw * 90.0;
    let command = GimbalCommand::new(
        ControlSource::Mavlink(sender_sysid),
        CommandMode::Manual,
        Some(yaw_deg),
        Some(pitch_deg),
        Some(0.0),
    );
    dispatch_gimbal_command(command, ctx, "SET_MANUAL_CONTROL").await;
}

async fn on_command_long(
    sender_sysid: u8,
    sender_compid: u8,
    src_addr: SocketAddr,
    cmd: COMMAND_LONG_DATA,
    ctx: &RxCtx<'_>,
) {
    if !addressed_to_us(cmd.target_system, ctx.config.sysid) {
        return;
    }
    match cmd.command {
        MavCmd::MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW => {
            on_cmd_pitchyaw(sender_sysid, sender_compid, src_addr, cmd, ctx).await;
        }
        MavCmd::MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE => {
            on_cmd_configure(src_addr, cmd, ctx).await;
        }
        _ => debug!("Ignoring COMMAND_LONG: {:?}", cmd.command),
    }
}

async fn on_cmd_pitchyaw(
    sender_sysid: u8,
    sender_compid: u8,
    src_addr: SocketAddr,
    cmd: COMMAND_LONG_DATA,
    ctx: &RxCtx<'_>,
) {
    if !ctx.state.is_primary_allowed(sender_sysid, sender_compid) {
        warn!(
            "Rejecting MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW from non-primary ({},{})",
            sender_sysid, sender_compid
        );
        send_command_ack(
            src_addr,
            ctx,
            MavCmd::MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            MavResult::MAV_RESULT_DENIED,
        )
        .await;
        return;
    }

    // param1 = pitch (deg), param2 = yaw (deg). param3/4 are rates and
    // ignored until a rate-control path lands. NaN means "leave axis
    // unchanged" per spec — substitute 0.0 for now.
    let pitch = if cmd.param1.is_nan() { 0.0 } else { cmd.param1 };
    let yaw = if cmd.param2.is_nan() { 0.0 } else { cmd.param2 };
    let command = GimbalCommand::new(
        ControlSource::Mavlink(sender_sysid),
        CommandMode::Position,
        Some(yaw),
        Some(pitch),
        Some(0.0),
    );

    let result = {
        let mut gimbal_guard = ctx.gimbal.lock().await;
        ctx.arbitrator.process_command(command, &mut gimbal_guard)
    };
    let ack = match result {
        Ok(true) => {
            info!(
                "MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW accepted from ({},{})",
                sender_sysid, sender_compid
            );
            MavResult::MAV_RESULT_ACCEPTED
        }
        Ok(false) => {
            debug!("MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW rejected by arbitrator (priority)");
            MavResult::MAV_RESULT_TEMPORARILY_REJECTED
        }
        Err(e) => {
            error!("MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW execution failed: {}", e);
            MavResult::MAV_RESULT_FAILED
        }
    };
    send_command_ack(
        src_addr,
        ctx,
        MavCmd::MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
        ack,
    )
    .await;
}

async fn on_cmd_configure(src_addr: SocketAddr, cmd: COMMAND_LONG_DATA, ctx: &RxCtx<'_>) {
    let pc = PrimaryControl {
        primary_sysid: to_id_byte(cmd.param1),
        primary_compid: to_id_byte(cmd.param2),
        secondary_sysid: to_id_byte(cmd.param3),
        secondary_compid: to_id_byte(cmd.param4),
    };
    ctx.state.set_primary_control(pc);
    info!(
        "MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE → primary=({},{}), secondary=({},{})",
        pc.primary_sysid, pc.primary_compid, pc.secondary_sysid, pc.secondary_compid,
    );
    send_command_ack(
        src_addr,
        ctx,
        MavCmd::MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
        MavResult::MAV_RESULT_ACCEPTED,
    )
    .await;
}

/// Common tail for SET_* handlers: execute through the arbitrator and log.
async fn dispatch_gimbal_command(command: GimbalCommand, ctx: &RxCtx<'_>, label: &str) {
    let mut gimbal_guard = ctx.gimbal.lock().await;
    match ctx.arbitrator.process_command(command, &mut gimbal_guard) {
        Ok(true) => info!("Executed MAVLink {}", label),
        Ok(false) => warn!("MAVLink {} rejected (lower priority)", label),
        Err(e) => error!("Failed to execute MAVLink {}: {}", label, e),
    }
}

fn addressed_to_us(target_system: u8, our_sysid: u8) -> bool {
    target_system == 0 || target_system == our_sysid
}

async fn send_gimbal_manager_information(
    socket: &Arc<UdpSocket>,
    dest_addr: SocketAddr,
    config: &MavlinkConfig,
    seq: &Arc<AtomicU8>,
) {
    let info = mavlink::ardupilotmega::GIMBAL_MANAGER_INFORMATION_DATA {
        time_boot_ms: 0, // TODO(item-6): track uptime
        cap_flags: mavlink::ardupilotmega::GimbalManagerCapFlags::default(),
        gimbal_device_id: config.compid,
        roll_min: -180.0_f32.to_radians(),
        roll_max: 180.0_f32.to_radians(),
        pitch_min: -90.0_f32.to_radians(),
        pitch_max: 90.0_f32.to_radians(),
        yaw_min: -180.0_f32.to_radians(),
        yaw_max: 180.0_f32.to_radians(),
    };

    let msg = MavMessage::GIMBAL_MANAGER_INFORMATION(info);

    match send_mavlink_message(socket, dest_addr, config.sysid, config.compid, seq, msg).await {
        Ok(_) => debug!("Sent GIMBAL_MANAGER_INFORMATION"),
        Err(e) => error!("Failed to send GIMBAL_MANAGER_INFORMATION: {}", e),
    }
}

async fn send_command_ack(
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

async fn send_mavlink_message(
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn addressed_to_us_matches_broadcast_and_our_sysid() {
        assert!(addressed_to_us(0, 1));
        assert!(addressed_to_us(42, 42));
        assert!(!addressed_to_us(42, 1));
    }
}
