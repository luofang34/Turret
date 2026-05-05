//! `COMMAND_LONG` dispatch + per-command handlers
//! (`MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW`,
//! `MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE`, `MAV_CMD_REQUEST_MESSAGE`).
//!
//! Each handler does its own ACK construction (different from SET_*
//! handlers which are fire-and-forget), so the
//! `arbitrate → set_attitude → ack` flow is inlined per case rather
//! than going through `dispatch_gimbal_command`.

use super::common::{addressed_to_us, clamp_pry, yaw_to_vehicle_frame};
use super::rate::{any_rate_active, apply_rate_increment};
use crate::daemon::arbitrator::Outcome;
use crate::daemon::mavlink_manager::handlers::build::{
    build_gimbal_manager_status_data, message_id_from_param, msg_id, send_command_ack,
    send_gimbal_manager_information, send_mavlink_message,
};
use crate::daemon::mavlink_manager::RxCtx;
use crate::daemon::models::{CommandMode, ControlSource, GimbalCommand, PrimaryControl};
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
    let flags = flags_from_param5(cmd.param5);
    let yaw_in = yaw_to_vehicle_frame(yaw_in, flags, ctx, "DO_GIMBAL_MANAGER_PITCHYAW");
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
            // Negative-but-not-special, or > 255: log and treat as
            // unchanged. Better than the old behavior of silently
            // collapsing -7.4 to 0 (= release).
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
    let prev = ctx.state.get_primary_control();
    let pc = PrimaryControl {
        primary_sysid: ConfigureField::decode(cmd.param1).apply(prev.primary_sysid, sender_sysid),
        primary_compid: ConfigureField::decode(cmd.param2)
            .apply(prev.primary_compid, sender_compid),
        secondary_sysid: ConfigureField::decode(cmd.param3)
            .apply(prev.secondary_sysid, sender_sysid),
        secondary_compid: ConfigureField::decode(cmd.param4)
            .apply(prev.secondary_compid, sender_compid),
    };
    ctx.state.set_primary_control(pc);
    info!(
        "MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE from ({},{}): \
         primary ({},{}) → ({},{}); secondary ({},{}) → ({},{})",
        sender_sysid,
        sender_compid,
        prev.primary_sysid,
        prev.primary_compid,
        pc.primary_sysid,
        pc.primary_compid,
        prev.secondary_sysid,
        prev.secondary_compid,
        pc.secondary_sysid,
        pc.secondary_compid,
    );
    send_command_ack(
        src_addr,
        ctx,
        MavCmd::MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
        MavResult::MAV_RESULT_ACCEPTED,
    )
    .await;
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    //! Unit tests for `ConfigureField` — the decode + per-field apply
    //! logic that determines how `param1..param4` of CONFIGURE map onto
    //! primary / secondary sysid+compid changes — and `flags_from_param5`,
    //! the bitmask decoder used by the PITCHYAW command. The async
    //! wrappers around them (`on_cmd_configure`, `on_cmd_pitchyaw`) are
    //! harder to test in isolation without a mock RxCtx; the math here
    //! is what actually decides ownership and frame interpretation.

    use super::*;

    #[test]
    fn decode_negative_one_is_no_change() {
        assert!(matches!(
            ConfigureField::decode(-1.0),
            ConfigureField::NoChange
        ));
        // ±0.5 rounding tolerance — float -1.0 might arrive as -0.999.
        assert!(matches!(
            ConfigureField::decode(-0.999),
            ConfigureField::NoChange
        ));
        assert!(matches!(
            ConfigureField::decode(-1.001),
            ConfigureField::NoChange
        ));
    }

    #[test]
    fn decode_negative_two_is_claim_self() {
        assert!(matches!(
            ConfigureField::decode(-2.0),
            ConfigureField::ClaimSelf
        ));
    }

    #[test]
    fn decode_negative_three_is_release_if_mine() {
        assert!(matches!(
            ConfigureField::decode(-3.0),
            ConfigureField::ReleaseIfMine
        ));
    }

    #[test]
    fn decode_zero_is_release_unconditional() {
        assert!(matches!(
            ConfigureField::decode(0.0),
            ConfigureField::ReleaseUnconditional
        ));
    }

    #[test]
    fn decode_positive_byte_is_literal() {
        assert!(matches!(
            ConfigureField::decode(154.0),
            ConfigureField::Literal(154)
        ));
        assert!(matches!(
            ConfigureField::decode(1.0),
            ConfigureField::Literal(1)
        ));
        assert!(matches!(
            ConfigureField::decode(255.0),
            ConfigureField::Literal(255)
        ));
    }

    #[test]
    fn decode_out_of_range_is_no_change() {
        // Negative but not -1/-2/-3 — old code collapsed to 0
        // (silent release); new code keeps the value unchanged and
        // logs a warn.
        assert!(matches!(
            ConfigureField::decode(-7.0),
            ConfigureField::NoChange
        ));
        assert!(matches!(
            ConfigureField::decode(256.0),
            ConfigureField::NoChange
        ));
    }

    #[test]
    fn decode_non_finite_is_no_change() {
        assert!(matches!(
            ConfigureField::decode(f32::NAN),
            ConfigureField::NoChange
        ));
        assert!(matches!(
            ConfigureField::decode(f32::INFINITY),
            ConfigureField::NoChange
        ));
    }

    #[test]
    fn apply_no_change_preserves_current() {
        assert_eq!(ConfigureField::NoChange.apply(42, 99), 42);
    }

    #[test]
    fn apply_claim_self_substitutes_requester() {
        // Even if a value was already set, `-2` always overwrites
        // with the requester.
        assert_eq!(ConfigureField::ClaimSelf.apply(42, 99), 99);
        assert_eq!(ConfigureField::ClaimSelf.apply(0, 7), 7);
    }

    #[test]
    fn apply_release_if_mine_only_clears_on_match() {
        // `-3` clears only when the current owner matches the
        // requester — prevents one client from bumping another off
        // by impersonation.
        assert_eq!(ConfigureField::ReleaseIfMine.apply(99, 99), 0);
        assert_eq!(ConfigureField::ReleaseIfMine.apply(42, 99), 42);
    }

    #[test]
    fn apply_release_unconditional_always_clears() {
        assert_eq!(ConfigureField::ReleaseUnconditional.apply(42, 99), 0);
        assert_eq!(ConfigureField::ReleaseUnconditional.apply(0, 99), 0);
    }

    #[test]
    fn apply_literal_overwrites() {
        assert_eq!(ConfigureField::Literal(154).apply(42, 99), 154);
    }

    #[test]
    fn flags_from_param5_decodes_valid_bits() {
        let f = flags_from_param5(64.0);
        assert!(f.contains(GimbalManagerFlags::GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME));
    }

    #[test]
    fn flags_from_param5_rounds_near_integer_floats() {
        // GCS may emit 64.0 as 63.9999… through f32 round-trip.
        let f = flags_from_param5(63.9999);
        assert!(f.contains(GimbalManagerFlags::GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME));
    }

    #[test]
    fn flags_from_param5_rejects_nan() {
        assert_eq!(flags_from_param5(f32::NAN), GimbalManagerFlags::empty());
    }

    #[test]
    fn flags_from_param5_rejects_negative() {
        assert_eq!(flags_from_param5(-1.0), GimbalManagerFlags::empty());
        assert_eq!(
            flags_from_param5(f32::NEG_INFINITY),
            GimbalManagerFlags::empty()
        );
    }

    #[test]
    fn flags_from_param5_rejects_overflow() {
        assert_eq!(
            flags_from_param5(f32::INFINITY),
            GimbalManagerFlags::empty()
        );
        // 2^33 — beyond u32::MAX.
        assert_eq!(
            flags_from_param5(8_589_934_592.0),
            GimbalManagerFlags::empty()
        );
    }

    #[test]
    fn flags_from_param5_zero_is_empty_default_vehicle_frame() {
        assert_eq!(flags_from_param5(0.0), GimbalManagerFlags::empty());
    }
}
