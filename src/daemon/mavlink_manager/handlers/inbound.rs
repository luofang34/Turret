//! Per-message MAVLink dispatch: every UDP datagram parsed into a
//! [`MavMessage`] lands in [`handle_mavlink_message`], which fans out to
//! one of the per-type handlers in the submodules below. All inbound
//! logic — primary-control gating, addressing, frame-flag clamping,
//! ACK shape, rate→position integration — lives under this module so
//! the periodic loops in [`super::loops`] don't see any of it.
//!
//! Submodule layout:
//!
//! - [`common`] — shared addressing / clamping / frame-transform /
//!   arbitrate-then-dispatch helpers, plus the addressing tests.
//! - [`set`] — `GIMBAL_MANAGER_SET_ATTITUDE` / `_PITCHYAW` /
//!   `_MANUAL_CONTROL` handlers.
//! - [`cmd_long`] — `COMMAND_LONG` dispatch and per-command handlers
//!   (PITCHYAW, CONFIGURE, REQUEST_MESSAGE).
//! - [`rate`] — rate→position integrator shared by SET_* and
//!   COMMAND_LONG rate paths.

#[path = "inbound/cmd_long.rs"]
mod cmd_long;
#[path = "inbound/common.rs"]
mod common;
#[path = "inbound/rate.rs"]
mod rate;
#[path = "inbound/set.rs"]
mod set;

use super::build::send_gimbal_manager_information;
use crate::daemon::mavlink_manager::math::quaternion_wxyz_to_euler_deg;
use crate::daemon::mavlink_manager::RxCtx;
use mavlink::ardupilotmega::{MavMessage, AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_DATA};
use std::net::SocketAddr;
use tracing::debug;

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
            set::on_set_attitude(sender_sysid, sender_compid, m, ctx).await;
        }
        MavMessage::GIMBAL_MANAGER_SET_PITCHYAW(m) => {
            set::on_set_pitchyaw(sender_sysid, sender_compid, m, ctx).await;
        }
        MavMessage::GIMBAL_MANAGER_SET_MANUAL_CONTROL(m) => {
            set::on_set_manual(sender_sysid, sender_compid, m, ctx).await;
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
            cmd_long::on_command_long(sender_sysid, sender_compid, src_addr, cmd, ctx).await;
        }
        MavMessage::AUTOPILOT_STATE_FOR_GIMBAL_DEVICE(m) => {
            on_autopilot_state(m, ctx);
        }
        _ => debug!("Ignoring MAVLink message type: {:?}", msg),
    }
}

/// Ingest the autopilot's vehicle attitude. Only the yaw axis is stored
/// today — used by the earth-frame yaw transformation in the SET_*
/// handlers. Pitch / roll come "for free" if/when the gimbal-frame
/// outputs need full vehicle attitude.
fn on_autopilot_state(m: AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_DATA, ctx: &RxCtx<'_>) {
    if m.q.iter().any(|c| c.is_nan()) {
        debug!("AUTOPILOT_STATE_FOR_GIMBAL_DEVICE with NaN quaternion — ignoring");
        return;
    }
    let (_pitch, _roll, yaw_deg) = quaternion_wxyz_to_euler_deg(m.q);
    ctx.state.update_vehicle_yaw(yaw_deg);
}
