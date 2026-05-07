//! `GIMBAL_MANAGER_SET_ATTITUDE` / `_SET_PITCHYAW` / `_SET_MANUAL_CONTROL`
//! per-message handlers.
//!
//! Each handler enforces addressing + primary-control gating before
//! decoding the message into a `GimbalCommand` and dispatching through
//! the arbitrator (via [`super::common::dispatch_gimbal_command`]).
//! NaN-position commands route to [`super::rate::apply_rate_increment`]
//! when at least one rate axis is active.

use super::common::{
    addressed_to_us, axis_baseline, clamp_pry, dispatch_gimbal_command, yaw_to_vehicle_frame,
};
use super::rate::{any_rate_active, apply_rate_increment};
use crate::daemon::mavlink_manager::math::quaternion_wxyz_to_euler_deg;
use crate::daemon::mavlink_manager::RxCtx;
use crate::daemon::models::{CommandMode, ControlSource, GimbalCommand};
use tracing::{debug, warn};

pub(super) async fn on_set_attitude(
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
    // spec. Route to the rate→position integrator when the rate fields
    // carry usable data; otherwise drop (NaN-q + NaN-rate is a malformed
    // command that the GCS shouldn't send anyway).
    if m.q.iter().any(|c| c.is_nan()) {
        let rates_deg_per_s = (
            m.angular_velocity_y.to_degrees(),
            m.angular_velocity_x.to_degrees(),
            m.angular_velocity_z.to_degrees(),
        );
        if any_rate_active(rates_deg_per_s) {
            apply_rate_increment(
                ControlSource::Mavlink(sender_sysid),
                rates_deg_per_s,
                ctx,
                "SET_ATTITUDE rate",
            )
            .await;
        } else {
            debug!("GIMBAL_MANAGER_SET_ATTITUDE with NaN quaternion and no rate — dropping");
        }
        return;
    }

    // TODO: honor RETRACT / NEUTRAL shortcuts.
    let (pitch_deg, roll_deg, yaw_deg) = quaternion_wxyz_to_euler_deg(m.q);
    let yaw_deg = yaw_to_vehicle_frame(yaw_deg, m.flags, ctx, "SET_ATTITUDE");
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

pub(super) async fn on_set_pitchyaw(
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

    // Per spec: NaN angle = "leave alone / use rate instead". Route to
    // the rate path when both pitch & yaw angles are NaN AND at least
    // one rate is non-NaN/non-zero.
    if m.pitch.is_nan() && m.yaw.is_nan() {
        let rates_deg_per_s = (
            m.pitch_rate.to_degrees(),
            0.0, // SET_PITCHYAW has no roll axis
            m.yaw_rate.to_degrees(),
        );
        if any_rate_active(rates_deg_per_s) {
            apply_rate_increment(
                ControlSource::Mavlink(sender_sysid),
                rates_deg_per_s,
                ctx,
                "SET_PITCHYAW rate",
            )
            .await;
        } else {
            debug!("GIMBAL_MANAGER_SET_PITCHYAW with NaN angles and no rate — dropping");
        }
        return;
    }

    // NaN on a single axis = "ignore this axis" per the GMv2 spec, not
    // "command 0°". Substitute the per-axis baseline (last commanded →
    // last measured → 0). Roll has no field on this message at all, so
    // it always uses the baseline — keeps an earlier `set roll=15` from
    // being silently zeroed by a yaw-only SET_PITCHYAW.
    let (base_pitch, base_roll, base_yaw) = axis_baseline(ctx.state);
    let pitch_deg = if m.pitch.is_nan() {
        base_pitch
    } else {
        m.pitch.to_degrees()
    };
    let yaw_deg = if m.yaw.is_nan() {
        base_yaw
    } else {
        m.yaw.to_degrees()
    };
    let yaw_deg = yaw_to_vehicle_frame(yaw_deg, m.flags, ctx, "SET_PITCHYAW");
    let (pitch, roll, yaw) = clamp_pry(pitch_deg, base_roll, yaw_deg, &ctx.config.limits);
    let command = GimbalCommand::new(
        ControlSource::Mavlink(sender_sysid),
        CommandMode::Position,
        Some(yaw),
        Some(pitch),
        Some(roll),
    );
    dispatch_gimbal_command(command, ctx, "SET_PITCHYAW").await;
}

/// Full-stick deflection on the rate axes maps to ±[`MANUAL_RATE_FULL_DEG_S`]°/s.
/// Matches the position-axis convention (pitch=±1 → ±90°) so the operator
/// gets symmetric stick feel; still bounded by `clamp_pry` against the
/// configured device limits. `90 deg/s` is fast-but-controllable for
/// joystick slew without saturating the STorM32 follow loop.
pub(super) const MANUAL_RATE_FULL_DEG_S: f32 = 90.0;

pub(super) async fn on_set_manual(
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

    // Per spec: NaN angle = "ignore this axis, use rate instead". When
    // both pitch & yaw angles are NaN AND at least one rate is non-NaN/
    // non-zero, scale the normalized rate fields to deg/s and route
    // through the rate→position integrator. Otherwise treat as a
    // position command on whatever axes have non-NaN angles.
    if m.pitch.is_nan() && m.yaw.is_nan() {
        let pitch_rate = manual_rate_to_deg_per_s(m.pitch_rate);
        let yaw_rate = manual_rate_to_deg_per_s(m.yaw_rate);
        let rates_deg_per_s = (pitch_rate, 0.0, yaw_rate);
        if any_rate_active(rates_deg_per_s) {
            apply_rate_increment(
                ControlSource::Mavlink(sender_sysid),
                rates_deg_per_s,
                ctx,
                "SET_MANUAL_CONTROL rate",
            )
            .await;
        } else {
            debug!("GIMBAL_MANAGER_SET_MANUAL_CONTROL with NaN angles and no rate — dropping");
        }
        return;
    }

    // Manual control uses normalized values (-1.0..=1.0). Scale to ±90°
    // then clamp to the configured device limits in case those are
    // tighter than ±90°. NaN on a single axis collapses to 0 (center)
    // — same convention as SET_PITCHYAW.
    let pitch_deg = if m.pitch.is_nan() {
        0.0
    } else {
        m.pitch * 90.0
    };
    let yaw_deg = if m.yaw.is_nan() { 0.0 } else { m.yaw * 90.0 };
    let yaw_deg = yaw_to_vehicle_frame(yaw_deg, m.flags, ctx, "SET_MANUAL_CONTROL");
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

/// Map a normalized stick-deflection rate (`-1.0..=1.0`, NaN ignored) to
/// degrees-per-second using [`MANUAL_RATE_FULL_DEG_S`] as full-scale.
/// NaN → 0 so a per-axis NaN simply contributes no rate (the
/// `any_rate_active` check decides whether the whole command is dropped).
fn manual_rate_to_deg_per_s(rate_normalized: f32) -> f32 {
    if rate_normalized.is_nan() {
        0.0
    } else {
        rate_normalized * MANUAL_RATE_FULL_DEG_S
    }
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::float_cmp
)]
mod tests {
    use super::*;

    #[test]
    fn manual_rate_full_stick_maps_to_full_scale_deg_per_s() {
        assert_eq!(manual_rate_to_deg_per_s(1.0), MANUAL_RATE_FULL_DEG_S);
        assert_eq!(manual_rate_to_deg_per_s(-1.0), -MANUAL_RATE_FULL_DEG_S);
    }

    #[test]
    fn manual_rate_half_stick_maps_to_half_scale() {
        assert_eq!(manual_rate_to_deg_per_s(0.5), MANUAL_RATE_FULL_DEG_S * 0.5);
        assert_eq!(
            manual_rate_to_deg_per_s(-0.25),
            MANUAL_RATE_FULL_DEG_S * -0.25
        );
    }

    #[test]
    fn manual_rate_zero_maps_to_zero() {
        assert_eq!(manual_rate_to_deg_per_s(0.0), 0.0);
    }

    #[test]
    fn manual_rate_nan_maps_to_zero() {
        // Per-axis NaN must contribute no rate so any_rate_active() can
        // decide whether the whole command is a no-op based on the
        // *other* axis.
        assert_eq!(manual_rate_to_deg_per_s(f32::NAN), 0.0);
    }

    #[test]
    fn manual_rate_passes_through_out_of_range_values() {
        // Spec says -1..=1, but a buggy GCS may exceed that. Don't
        // sanitize here — clamp_pry runs against the configured device
        // limits later, which is the right place to enforce hardware
        // bounds. Here, just preserve the linear scaling so debug logs
        // can show what the GCS actually sent.
        assert_eq!(manual_rate_to_deg_per_s(2.0), MANUAL_RATE_FULL_DEG_S * 2.0);
    }
}
