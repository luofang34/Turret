//! `COMMAND_LONG` dispatch + per-command handlers
//! (`MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW`,
//! `MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE`, `MAV_CMD_REQUEST_MESSAGE`).
//!
//! Each handler does its own ACK construction (different from SET_*
//! handlers which are fire-and-forget), so the
//! `arbitrate → set_attitude → ack` flow is inlined per case rather
//! than going through `dispatch_gimbal_command`.

use super::common::{
    addressed_to_this_device, addressed_to_us, gimbal_device_id_from_param7, selected_position,
};
use super::rate::{any_rate_active, dispatch_rate_command};
use crate::daemon::mavlink_manager::handlers::build::{
    build_gimbal_manager_status_data, message_id_from_param, msg_id, send_command_ack,
    send_gimbal_manager_information, send_mavlink_message,
};
use crate::daemon::mavlink_manager::RxCtx;
use crate::daemon::models::{ControlSource, PrimaryControl};
use mavlink::ardupilotmega::{
    GimbalManagerFlags, MavCmd, MavMessage, MavResult, COMMAND_LONG_DATA,
};
use std::net::SocketAddr;
use tracing::{debug, error, info, warn};

/// Decode `MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW`'s `param5` (an `f32`
/// carrying a `u32` bitmask of `GimbalManagerFlags`) into the typed
/// flag set.
///
/// MAVLink's `COMMAND_LONG` is a generic 7-float container, so flag
/// fields ride as floats and need explicit coercion. NaN / negative /
/// out-of-range inputs collapse to `empty()` (vehicle-frame default
/// per the spec) so a malformed GCS can't accidentally enable
/// earth-frame interpretation it didn't ask for.
fn flags_from_param5(p: f32) -> GimbalManagerFlags {
    if !p.is_finite() || p < 0.0 {
        return GimbalManagerFlags::empty();
    }
    let bits = p.round();
    if bits > u32::MAX as f32 {
        return GimbalManagerFlags::empty();
    }
    GimbalManagerFlags::from_bits_truncate(bits as u32)
}

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
            on_cmd_configure(sender_sysid, sender_compid, src_addr, cmd, ctx).await;
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
                ctx.gimbal,
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
    // param7 carries the gimbal_device_id per the GMv2 spec. Drop
    // silently if it's malformed or addressed to a different gimbal —
    // the ACK obligation only applies when the command is *for us*.
    let device_id = match gimbal_device_id_from_param7(cmd.param7) {
        Some(id) => id,
        None => {
            debug!(
                "Ignoring MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW with malformed param7={}",
                cmd.param7
            );
            return;
        }
    };
    if !addressed_to_this_device(device_id, ctx.config.compid) {
        debug!(
            "Ignoring MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW for gimbal_device_id={} (we are {})",
            device_id, ctx.config.compid
        );
        return;
    }

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

    let ack = execute_pitchyaw(sender_sysid, cmd, ctx).await;

    send_command_ack(
        src_addr,
        ctx,
        MavCmd::MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
        ack,
    )
    .await;
}

async fn execute_pitchyaw(sender_sysid: u8, cmd: COMMAND_LONG_DATA, ctx: &RxCtx<'_>) -> MavResult {
    if cmd.param1.is_nan() && cmd.param2.is_nan() {
        let rates = (cmd.param3, f32::NAN, cmd.param4);
        if any_rate_active(rates) {
            return if dispatch_rate_command(
                ControlSource::Mavlink(sender_sysid),
                rates,
                flags_from_param5(cmd.param5),
                ctx,
                "DO_GIMBAL_MANAGER_PITCHYAW rate",
            )
            .await
            {
                MavResult::MAV_RESULT_ACCEPTED
            } else {
                MavResult::MAV_RESULT_FAILED
            };
        }
    }
    let command = selected_position(
        ControlSource::Mavlink(sender_sysid),
        (!cmd.param1.is_nan()).then_some(cmd.param1),
        None,
        (!cmd.param2.is_nan()).then_some(cmd.param2),
        flags_from_param5(cmd.param5),
        ctx,
    );
    match command {
        Some(command) => match ctx.arbitrator.execute(command, ctx.gimbal).await {
            Ok(_) => MavResult::MAV_RESULT_ACCEPTED,
            Err(error) => {
                error!(%error, "pitch/yaw execution rejected");
                MavResult::MAV_RESULT_FAILED
            }
        },
        None => MavResult::MAV_RESULT_DENIED,
    }
}

/// Per-field special-value semantics from the MAVLink common.xml spec
/// for `MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE`. Each of param1..param4
/// is a `float` carrying a sysid or compid byte, with these special
/// negative / zero values overlaid:
///
/// - `-1`: leave unchanged
/// - `-2`: claim self (substitute the requesting message's sysid/compid)
/// - `-3`: release if currently mine (only clears when the existing
///   value matches the requesting sysid/compid; no-op otherwise)
/// - `0`:  permanently release (anyone may claim afterward)
/// - any other non-negative finite value: take as a literal byte
///
/// Today's `to_id_byte` clamped negatives to `0`, which would make
/// every special value behave like "release" — including `-1` ("leave
/// unchanged"), which would silently overwrite ownership on every
/// configure call from a GCS that didn't realize it had to populate
/// the unchanged fields too. Decoding into a typed enum makes the
/// per-field policy explicit.
#[derive(Debug, Clone, Copy)]
enum ConfigureField {
    /// `-1`: keep the current value.
    NoChange,
    /// `-2`: substitute the requesting sysid/compid.
    ClaimSelf,
    /// `-3`: clear if (and only if) the current value matches the
    /// requesting sysid/compid. No-op otherwise.
    ReleaseIfMine,
    /// `0`: clear unconditionally.
    ReleaseUnconditional,
    /// Any other finite non-negative value, rounded to a byte.
    Literal(u8),
}

impl ConfigureField {
    fn decode(p: f32) -> Self {
        if !p.is_finite() {
            // NaN / inf: spec doesn't mention these. Treat as
            // "leave unchanged" — the safest interpretation that
            // doesn't accidentally release control on a malformed
            // GCS frame.
            return Self::NoChange;
        }
        // Negative special values are integer-coded in the spec.
        // Use `.round()` so a -3.0 sent as float is robustly matched.
        let r = p.round();
        if (r - -1.0).abs() < 0.5 {
            Self::NoChange
        } else if (r - -2.0).abs() < 0.5 {
            Self::ClaimSelf
        } else if (r - -3.0).abs() < 0.5 {
            Self::ReleaseIfMine
        } else if r == 0.0 {
            Self::ReleaseUnconditional
        } else if r > 0.0 && r <= 255.0 {
            Self::Literal(r as u8)
        } else {
            // Invalid values must not become an implicit release request.
            warn!(
                "MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE: out-of-range param {} \
                 (expected -3, -2, -1, 0, or [1, 255]); leaving field unchanged",
                p
            );
            Self::NoChange
        }
    }

    fn apply(self, current: u8, requester: u8) -> u8 {
        match self {
            Self::NoChange => current,
            Self::ClaimSelf => requester,
            Self::ReleaseIfMine => {
                if current == requester {
                    0
                } else {
                    current
                }
            }
            Self::ReleaseUnconditional => 0,
            Self::Literal(b) => b,
        }
    }
}

async fn on_cmd_configure(
    sender_sysid: u8,
    sender_compid: u8,
    src_addr: SocketAddr,
    cmd: COMMAND_LONG_DATA,
    ctx: &RxCtx<'_>,
) {
    // param7 carries gimbal_device_id. Drop silently if it's malformed
    // or addressed to a different gimbal — same shape as PITCHYAW.
    // Otherwise a CONFIGURE meant to claim primary control of a peer
    // gimbal would silently rewrite this manager's ownership too.
    let device_id = match gimbal_device_id_from_param7(cmd.param7) {
        Some(id) => id,
        None => {
            debug!(
                "Ignoring MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE with malformed param7={}",
                cmd.param7
            );
            return;
        }
    };
    if !addressed_to_this_device(device_id, ctx.config.compid) {
        debug!(
            "Ignoring MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE for gimbal_device_id={} (we are {})",
            device_id, ctx.config.compid
        );
        return;
    }

    if !ctx.state.is_primary_allowed(sender_sysid, sender_compid) {
        send_command_ack(
            src_addr,
            ctx,
            MavCmd::MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
            MavResult::MAV_RESULT_DENIED,
        )
        .await;
        return;
    }
    let prev = ctx.state.primary_control_cached();
    let pc = PrimaryControl {
        primary_sysid: ConfigureField::decode(cmd.param1).apply(prev.primary_sysid, sender_sysid),
        primary_compid: ConfigureField::decode(cmd.param2)
            .apply(prev.primary_compid, sender_compid),
        secondary_sysid: ConfigureField::decode(cmd.param3)
            .apply(prev.secondary_sysid, sender_sysid),
        secondary_compid: ConfigureField::decode(cmd.param4)
            .apply(prev.secondary_compid, sender_compid),
    };
    if pc != prev {
        if let Err(error) = ctx.gimbal.revoke().await {
            error!(%error, "control handover could not establish stopping state");
            send_command_ack(
                src_addr,
                ctx,
                MavCmd::MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
                MavResult::MAV_RESULT_FAILED,
            )
            .await;
            return;
        }
        ctx.state.set_primary_control(pc);
    }
    info!(sender_sysid, sender_compid, previous = ?prev, current = ?pc, "manager ownership updated");
    send_command_ack(
        src_addr,
        ctx,
        MavCmd::MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
        MavResult::MAV_RESULT_ACCEPTED,
    )
    .await;
}

#[cfg(test)]
mod tests;
