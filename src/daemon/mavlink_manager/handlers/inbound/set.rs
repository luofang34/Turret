//! `GIMBAL_MANAGER_SET_ATTITUDE` / `_SET_PITCHYAW` / `_SET_MANUAL_CONTROL`
//! per-message handlers.
//!
//! Each handler enforces addressing + primary-control gating before
//! decoding the message into a `GimbalCommand` and dispatching through
//! the arbitrator (via [`super::common::dispatch_gimbal_command`]).
//! NaN-position commands route to [`super::rate::dispatch_rate_command`]
//! when at least one rate axis is active.

use super::common::{
    addressed_to_this_device, addressed_to_us, dispatch_gimbal_command, selected_position,
};
use super::rate::{any_rate_active, dispatch_rate_command};
use crate::daemon::mavlink_manager::math::quaternion_wxyz_to_euler_deg;
use crate::daemon::mavlink_manager::RxCtx;
use crate::daemon::models::ControlSource;
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
    if !addressed_to_this_device(m.gimbal_device_id, ctx.config.compid) {
        debug!(
            "Ignoring GIMBAL_MANAGER_SET_ATTITUDE for gimbal_device_id={} (we are {})",
            m.gimbal_device_id, ctx.config.compid
        );
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
    // spec. Preserve selected rate semantics when the rate fields
    // carry usable data; otherwise drop (NaN-q + NaN-rate is a malformed
    // command that the GCS shouldn't send anyway).
    if m.q.iter().any(|c| c.is_nan()) {
        let rates_deg_per_s = (
            m.angular_velocity_y.to_degrees(),
            m.angular_velocity_x.to_degrees(),
            m.angular_velocity_z.to_degrees(),
        );
        if any_rate_active(rates_deg_per_s) {
            dispatch_rate_command(
                ControlSource::Mavlink(sender_sysid),
                rates_deg_per_s,
                m.flags,
                ctx,
                "SET_ATTITUDE rate",
            )
            .await;
        } else {
            debug!("GIMBAL_MANAGER_SET_ATTITUDE with NaN quaternion and no rate — dropping");
        }
        return;
    }

    let (pitch, roll, yaw) = quaternion_wxyz_to_euler_deg(m.q);
    let Some(command) = selected_position(
        ControlSource::Mavlink(sender_sysid),
        Some(pitch),
        Some(roll),
        Some(yaw),
        m.flags,
        ctx,
    ) else {
        return;
    };
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
    if !addressed_to_this_device(m.gimbal_device_id, ctx.config.compid) {
        debug!(
            "Ignoring GIMBAL_MANAGER_SET_PITCHYAW for gimbal_device_id={} (we are {})",
            m.gimbal_device_id, ctx.config.compid
        );
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
            f32::NAN, // This message does not address roll.
            m.yaw_rate.to_degrees(),
        );
        if any_rate_active(rates_deg_per_s) {
            dispatch_rate_command(
                ControlSource::Mavlink(sender_sysid),
                rates_deg_per_s,
                m.flags,
                ctx,
                "SET_PITCHYAW rate",
            )
            .await;
        } else {
            debug!("GIMBAL_MANAGER_SET_PITCHYAW with NaN angles and no rate — dropping");
        }
        return;
    }

    let Some(command) = selected_position(
        ControlSource::Mavlink(sender_sysid),
        (!m.pitch.is_nan()).then(|| m.pitch.to_degrees()),
        None,
        (!m.yaw.is_nan()).then(|| m.yaw.to_degrees()),
        m.flags,
        ctx,
    ) else {
        return;
    };
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
    if !addressed_to_this_device(m.gimbal_device_id, ctx.config.compid) {
        debug!(
            "Ignoring GIMBAL_MANAGER_SET_MANUAL_CONTROL for gimbal_device_id={} (we are {})",
            m.gimbal_device_id, ctx.config.compid
        );
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
    // through the backend rate command. Otherwise treat as a
    // position command on whatever axes have non-NaN angles.
    if m.pitch.is_nan() && m.yaw.is_nan() {
        let pitch_rate = manual_rate_to_deg_per_s(m.pitch_rate);
        let yaw_rate = manual_rate_to_deg_per_s(m.yaw_rate);
        let rates_deg_per_s = (pitch_rate, f32::NAN, yaw_rate);
        if any_rate_active(rates_deg_per_s) {
            dispatch_rate_command(
                ControlSource::Mavlink(sender_sysid),
                rates_deg_per_s,
                m.flags,
                ctx,
                "SET_MANUAL_CONTROL rate",
            )
            .await;
        } else {
            debug!("GIMBAL_MANAGER_SET_MANUAL_CONTROL with NaN angles and no rate — dropping");
        }
        return;
    }

    let Some(command) = selected_position(
        ControlSource::Mavlink(sender_sysid),
        (!m.pitch.is_nan()).then_some(m.pitch * 90.0),
        None,
        (!m.yaw.is_nan()).then_some(m.yaw * 90.0),
        m.flags,
        ctx,
    ) else {
        return;
    };
    dispatch_gimbal_command(command, ctx, "SET_MANUAL_CONTROL").await;
}

/// Map a normalized stick-deflection rate (`-1.0..=1.0`, NaN ignored) to
/// degrees-per-second using [`MANUAL_RATE_FULL_DEG_S`] as full-scale.
/// NaN remains omitted so a per-axis NaN contributes no demand (the
/// `any_rate_active` check decides whether the whole command is dropped).
fn manual_rate_to_deg_per_s(rate_normalized: f32) -> f32 {
    if rate_normalized.is_nan() {
        f32::NAN
    } else {
        rate_normalized * MANUAL_RATE_FULL_DEG_S
    }
}

#[cfg(test)]
mod tests;
