//! Rate-command integration helpers.
//!
//! STorM32 only takes position commands, so a streaming rate command
//! from a GCS gets translated into one position write per tick:
//! `target = current + rate * dt`. The first tick from a given source
//! uses the minimum `dt` (10 ms); subsequent ticks use the gap to the
//! source's previous rate tick, clamped to [10 ms, 500 ms]. For the
//! typical 10–50 Hz GCS rate-command stream this gives a serviceable
//! joystick experience without a closed-loop rate controller.

use super::common::{clamp_pry, dispatch_gimbal_command};
use crate::daemon::mavlink_manager::math::{clamp_rate_dt, compute_rate_target_deg};
use crate::daemon::mavlink_manager::RxCtx;
use crate::daemon::models::{CommandMode, ControlSource, GimbalCommand};
use std::time::Instant;

/// True if any of the three axis rates is finite and non-zero. Used to
/// decide whether a NaN-position command should be routed to the
/// rate→position integrator or dropped.
pub(super) fn any_rate_active(rates_deg_per_s: (f32, f32, f32)) -> bool {
    let (p, r, y) = rates_deg_per_s;
    [p, r, y].iter().any(|v| v.is_finite() && *v != 0.0)
}

/// Convert a rate command into a position update and dispatch through
/// the regular position pipeline.
pub(super) async fn apply_rate_increment(
    source: ControlSource,
    rates_deg_per_s: (f32, f32, f32),
    ctx: &RxCtx<'_>,
    label: &'static str,
) {
    // Primary-control gating already happened in the caller — every
    // SET_* / COMMAND_LONG handler does `is_primary_allowed` before
    // branching into the rate path.

    let now = Instant::now();
    let dt = match ctx.state.last_rate_tick(&source) {
        Some(prev) => clamp_rate_dt(now.duration_since(prev)),
        None => clamp_rate_dt(std::time::Duration::ZERO), // = MIN
    };
    let dt_secs = dt.as_secs_f32();

    let (current_pitch, current_roll, current_yaw) = current_position(ctx);
    let (rate_pitch, rate_roll, rate_yaw) = rates_deg_per_s;

    let target_pitch = compute_rate_target_deg(current_pitch, rate_pitch, dt_secs);
    let target_roll = compute_rate_target_deg(current_roll, rate_roll, dt_secs);
    let target_yaw = compute_rate_target_deg(current_yaw, rate_yaw, dt_secs);

    let (pitch, roll, yaw) = clamp_pry(target_pitch, target_roll, target_yaw, &ctx.config.limits);

    let command = GimbalCommand::new(
        source.clone(),
        CommandMode::Position,
        Some(yaw),
        Some(pitch),
        Some(roll),
    );
    dispatch_gimbal_command(command, ctx, label).await;
    ctx.state.record_rate_tick(source, now);
}

/// Best-known current commanded position for the gimbal, as a degrees
/// triple `(pitch, roll, yaw)`.
///
/// Order of preference:
///   1. `state.last_command` if present (the in-flight commanded
///      position — what the gimbal *should* be at).
///   2. `state.measured` if a poll has succeeded (what the gimbal
///      actually is at).
///   3. `(0.0, 0.0, 0.0)` as a fall-back so the rate integrator can
///      still produce a sensible first tick before any state exists.
fn current_position(ctx: &RxCtx<'_>) -> (f32, f32, f32) {
    if let Some(last) = ctx.state.get_last_command() {
        return (
            last.pitch.unwrap_or(0.0),
            last.roll.unwrap_or(0.0),
            last.yaw.unwrap_or(0.0),
        );
    }
    let snapshot = ctx.state.get_state();
    if snapshot.measured_at.is_some() {
        return (snapshot.pitch, snapshot.roll, snapshot.yaw);
    }
    (0.0, 0.0, 0.0)
}
