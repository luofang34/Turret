//! Per-message MAVLink dispatch: every UDP datagram parsed into a
//! [`MavMessage`] lands in [`handle_mavlink_message`], which fans out to
//! one of the per-type handlers below. All inbound logic — primary-control
//! gating, addressing, frame-flag clamping, ACK shape — lives here so the
//! periodic loops in [`super::loops`] don't see any of it.

use super::super::math::{clamp_axis, quaternion_wxyz_to_euler_deg, to_id_byte};
use super::build::{
    build_gimbal_manager_status_data, message_id_from_param, msg_id, send_command_ack,
    send_gimbal_manager_information, send_mavlink_message,
};
use crate::daemon::config::GimbalLimits;
use crate::daemon::mavlink_manager::RxCtx;
use crate::daemon::models::{CommandMode, ControlSource, GimbalCommand, PrimaryControl};
use mavlink::ardupilotmega::{MavCmd, MavMessage, MavResult, COMMAND_LONG_DATA};
use std::net::SocketAddr;
use tracing::{debug, error, info, warn};

/// Dispatch a parsed inbound `MavMessage` to a per-type handler.
#[tracing::instrument(level = "debug", skip(msg, ctx))]
pub(in crate::daemon::mavlink_manager) async fn handle_mavlink_message(
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
            send_gimbal_manager_information(
                ctx.socket,
                src_addr,
                ctx.config,
                ctx.seq,
                ctx.start_time,
            )
            .await;
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
    if !addressed_to_us(
        m.target_system,
        m.target_component,
        ctx.config.sysid,
        ctx.config.compid,
    ) {
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

    // TODO: honor frame flags (YAW_IN_VEHICLE_FRAME vs YAW_IN_EARTH_FRAME)
    // and RETRACT / NEUTRAL shortcuts. Earth-frame yaw needs vehicle
    // attitude integration, which depends on AUTOPILOT_STATE_FOR_GIMBAL_DEVICE
    // ingestion that isn't wired up yet.
    let (pitch_deg, roll_deg, yaw_deg) = quaternion_wxyz_to_euler_deg(m.q);
    let (pitch, roll, yaw) = clamp_pry(pitch_deg, roll_deg, yaw_deg, &ctx.config.limits);
    let command = GimbalCommand::new(
        ControlSource::Mavlink(sender_sysid),
        CommandMode::Position,
        Some(yaw),
        Some(pitch),
        Some(roll),
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
    if !addressed_to_us(
        m.target_system,
        m.target_component,
        ctx.config.sysid,
        ctx.config.compid,
    ) {
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
    let (pitch, _roll, yaw) = clamp_pry(pitch_deg, 0.0, yaw_deg, &ctx.config.limits);
    let command = GimbalCommand::new(
        ControlSource::Mavlink(sender_sysid),
        CommandMode::Position,
        Some(yaw),
        Some(pitch),
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
    if !addressed_to_us(
        m.target_system,
        m.target_component,
        ctx.config.sysid,
        ctx.config.compid,
    ) {
        return;
    }
    if !ctx.state.is_primary_allowed(sender_sysid, sender_compid) {
        debug!(
            "Ignoring GIMBAL_MANAGER_SET_MANUAL_CONTROL from non-primary ({},{})",
            sender_sysid, sender_compid
        );
        return;
    }

    // Manual control uses normalized values (-1.0..=1.0). Scale to ±90°
    // then clamp to the configured device limits in case those are
    // tighter than ±90°.
    let pitch_deg = m.pitch * 90.0;
    let yaw_deg = m.yaw * 90.0;
    let (pitch, _roll, yaw) = clamp_pry(pitch_deg, 0.0, yaw_deg, &ctx.config.limits);
    let command = GimbalCommand::new(
        ControlSource::Mavlink(sender_sysid),
        CommandMode::Manual,
        Some(yaw),
        Some(pitch),
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
    if !addressed_to_us(
        cmd.target_system,
        cmd.target_component,
        ctx.config.sysid,
        ctx.config.compid,
    ) {
        return;
    }
    match cmd.command {
        MavCmd::MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW => {
            on_cmd_pitchyaw(sender_sysid, sender_compid, src_addr, cmd, ctx).await;
        }
        MavCmd::MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE => {
            on_cmd_configure(src_addr, cmd, ctx).await;
        }
        MavCmd::MAV_CMD_REQUEST_MESSAGE => {
            on_cmd_request_message(src_addr, cmd, ctx).await;
        }
        _ => debug!("Ignoring COMMAND_LONG: {:?}", cmd.command),
    }
}

async fn on_cmd_request_message(src_addr: SocketAddr, cmd: COMMAND_LONG_DATA, ctx: &RxCtx<'_>) {
    let result = match message_id_from_param(cmd.param1) {
        Some(msg_id::GIMBAL_MANAGER_INFORMATION) => {
            send_gimbal_manager_information(
                ctx.socket,
                src_addr,
                ctx.config,
                ctx.seq,
                ctx.start_time,
            )
            .await;
            MavResult::MAV_RESULT_ACCEPTED
        }
        Some(msg_id::GIMBAL_MANAGER_STATUS) => {
            let status = build_gimbal_manager_status_data(ctx.state, ctx.config, ctx.start_time);
            let msg = MavMessage::GIMBAL_MANAGER_STATUS(status);
            match send_mavlink_message(
                ctx.socket,
                src_addr,
                ctx.config.sysid,
                ctx.config.compid,
                ctx.seq,
                msg,
            )
            .await
            {
                Ok(()) => MavResult::MAV_RESULT_ACCEPTED,
                Err(e) => {
                    error!("Failed to send on-demand GIMBAL_MANAGER_STATUS: {}", e);
                    MavResult::MAV_RESULT_FAILED
                }
            }
        }
        Some(other) => {
            debug!("MAV_CMD_REQUEST_MESSAGE for unsupported id {}", other);
            MavResult::MAV_RESULT_DENIED
        }
        None => {
            debug!(
                "MAV_CMD_REQUEST_MESSAGE with non-integer param1 {}",
                cmd.param1
            );
            MavResult::MAV_RESULT_DENIED
        }
    };
    send_command_ack(src_addr, ctx, MavCmd::MAV_CMD_REQUEST_MESSAGE, result).await;
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
    let pitch_in = if cmd.param1.is_nan() { 0.0 } else { cmd.param1 };
    let yaw_in = if cmd.param2.is_nan() { 0.0 } else { cmd.param2 };
    let (pitch, _roll, yaw) = clamp_pry(pitch_in, 0.0, yaw_in, &ctx.config.limits);
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

/// Clamp `(pitch, roll, yaw)` to the configured device limits and log
/// per-axis if a clamp actually occurred. The order of the returned tuple
/// matches the input order, not the order arguments appear in
/// `GimbalCommand::new`.
fn clamp_pry(pitch: f32, roll: f32, yaw: f32, limits: &GimbalLimits) -> (f32, f32, f32) {
    (
        clamp_axis(pitch, limits.pitch_min_deg, limits.pitch_max_deg, "pitch"),
        clamp_axis(roll, limits.roll_min_deg, limits.roll_max_deg, "roll"),
        clamp_axis(yaw, limits.yaw_min_deg, limits.yaw_max_deg, "yaw"),
    )
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

/// True if a frame whose `target_system` / `target_component` we're looking
/// at should be acted on by this manager.
///
/// Both axes obey the MAVLink "0 = broadcast, otherwise must match" rule.
/// Without the component check the manager would happily execute commands
/// addressed to the autopilot (compid 1) on the same vehicle, which is
/// what the GMv2 spec is trying to prevent when multiple components share
/// a sysid.
fn addressed_to_us(target_system: u8, target_component: u8, our_sysid: u8, our_compid: u8) -> bool {
    let sys_ok = target_system == 0 || target_system == our_sysid;
    let comp_ok = target_component == 0 || target_component == our_compid;
    sys_ok && comp_ok
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;

    #[test]
    fn addressed_to_us_accepts_per_axis_broadcast_and_exact_match() {
        // Both axes broadcast.
        assert!(addressed_to_us(0, 0, 1, 154));
        // sysid broadcast, compid exact.
        assert!(addressed_to_us(0, 154, 1, 154));
        // sysid exact, compid broadcast.
        assert!(addressed_to_us(1, 0, 1, 154));
        // Both exact.
        assert!(addressed_to_us(1, 154, 1, 154));
    }

    #[test]
    fn addressed_to_us_rejects_other_components_on_same_sysid() {
        // Targeting the autopilot (compid 1) on the same vehicle must NOT
        // be acted on by the gimbal manager — same-sysid component routing
        // is exactly what the compid axis is for.
        assert!(!addressed_to_us(1, 1, 1, 154));
    }

    #[test]
    fn addressed_to_us_rejects_other_systems() {
        assert!(!addressed_to_us(2, 154, 1, 154));
        assert!(!addressed_to_us(2, 0, 1, 154));
    }
}
