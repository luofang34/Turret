//! `COMMAND_LONG` dispatch + per-command handlers
//! (`MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW`,
//! `MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE`, `MAV_CMD_REQUEST_MESSAGE`).
//!
//! Each handler does its own ACK construction (different from SET_*
//! handlers which are fire-and-forget), so the
//! `arbitrate → set_attitude → ack` flow is inlined per case rather
//! than going through `dispatch_gimbal_command`.

use super::common::{addressed_to_us, clamp_pry};
use super::rate::{any_rate_active, apply_rate_increment};
use crate::daemon::arbitrator::Outcome;
use crate::daemon::mavlink_manager::handlers::build::{
    build_gimbal_manager_status_data, message_id_from_param, msg_id, send_command_ack,
    send_gimbal_manager_information, send_mavlink_message,
};
use crate::daemon::mavlink_manager::math::to_id_byte;
use crate::daemon::mavlink_manager::RxCtx;
use crate::daemon::models::{CommandMode, ControlSource, GimbalCommand, PrimaryControl};
use mavlink::ardupilotmega::{MavCmd, MavMessage, MavResult, COMMAND_LONG_DATA};
use std::net::SocketAddr;
use tracing::{debug, error, info, warn};

pub(super) async fn on_command_long(
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

    // param1 = pitch angle (deg), param2 = yaw angle (deg).
    // param3 = pitch rate (deg/s), param4 = yaw rate (deg/s).
    // Both axes' angles NaN AND at least one rate non-NaN/non-zero =
    // pure rate command. Otherwise treat as position (rate fields
    // ignored when an angle is provided — STorM32 has no concept of
    // "target rate on the way to a target angle").
    if cmd.param1.is_nan() && cmd.param2.is_nan() {
        let rates_deg_per_s = (cmd.param3, 0.0, cmd.param4);
        if any_rate_active(rates_deg_per_s) {
            apply_rate_increment(
                ControlSource::Mavlink(sender_sysid),
                rates_deg_per_s,
                ctx,
                "DO_GIMBAL_MANAGER_PITCHYAW rate",
            )
            .await;
            send_command_ack(
                src_addr,
                ctx,
                MavCmd::MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                MavResult::MAV_RESULT_ACCEPTED,
            )
            .await;
            return;
        }
    }

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

    let ack = match ctx.arbitrator.arbitrate(command) {
        Outcome::Execute { pitch, roll, yaw } => {
            match ctx.gimbal.set_attitude(pitch, roll, yaw).await {
                Ok(()) => {
                    info!(
                        "MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW accepted from ({},{})",
                        sender_sysid, sender_compid
                    );
                    MavResult::MAV_RESULT_ACCEPTED
                }
                Err(e) => {
                    error!("MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW execution failed: {}", e);
                    MavResult::MAV_RESULT_FAILED
                }
            }
        }
        Outcome::Rejected => {
            debug!("MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW rejected by arbitrator (priority)");
            MavResult::MAV_RESULT_TEMPORARILY_REJECTED
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
