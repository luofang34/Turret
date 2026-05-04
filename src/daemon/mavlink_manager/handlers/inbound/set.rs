//! `GIMBAL_MANAGER_SET_ATTITUDE` / `_SET_PITCHYAW` / `_SET_MANUAL_CONTROL`
//! per-message handlers.
//!
//! Each handler enforces addressing + primary-control gating before
//! decoding the message into a `GimbalCommand` and dispatching through
//! the arbitrator (via [`super::common::dispatch_gimbal_command`]).
//! NaN-position commands route to [`super::rate::apply_rate_increment`]
//! when at least one rate axis is active.

use super::common::{addressed_to_us, clamp_pry, dispatch_gimbal_command, yaw_to_vehicle_frame};
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

    let pitch_deg = if m.pitch.is_nan() {
        0.0
    } else {
        m.pitch.to_degrees()
    };
    let yaw_deg = if m.yaw.is_nan() {
        0.0
    } else {
        m.yaw.to_degrees()
    };
    let yaw_deg = yaw_to_vehicle_frame(yaw_deg, m.flags, ctx, "SET_PITCHYAW");
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
