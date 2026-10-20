//! Shared helpers used by every inbound handler: addressing predicate,
//! per-axis limit clamp, earth→vehicle yaw transform, and the common
//! arbitrate-then-dispatch tail for SET_* handlers.
//!
//! All `pub(super)` so the per-message handler files (`set.rs`,
//! `cmd_long.rs`) and the selected-rate adapter (`rate.rs`) can call into
//! them without each duplicating the logic.

use crate::daemon::command_receipt::dispatch_succeeded;
use crate::daemon::mavlink_manager::math::{clamp_axis, earth_to_vehicle_yaw_deg};
use crate::daemon::mavlink_manager::RxCtx;
use crate::daemon::models::GimbalCommand;
use mavlink::ardupilotmega::GimbalManagerFlags;
use std::time::Duration;
use tracing::{error, info, warn};

/// How long an autopilot vehicle-attitude reading stays usable for
/// earth-frame yaw transformation. Expired context rejects that transform.
pub(super) const VEHICLE_ATTITUDE_MAX_AGE: Duration = Duration::from_secs(1);

/// True if a frame whose `target_system` / `target_component` we're
/// looking at should be acted on by this manager.
///
/// Both axes obey the MAVLink "0 = broadcast, otherwise must match"
/// rule. Without the component check the manager would happily execute
/// commands addressed to the autopilot (compid 1) on the same vehicle,
/// which is what the GMv2 spec is trying to prevent when multiple
/// components share a sysid.
pub(super) fn addressed_to_us(
    target_system: u8,
    target_component: u8,
    our_sysid: u8,
    our_compid: u8,
) -> bool {
    let sys_ok = target_system == 0 || target_system == our_sysid;
    let comp_ok = target_component == 0 || target_component == our_compid;
    sys_ok && comp_ok
}

/// True if a command carrying `gimbal_device_id` (or COMMAND_LONG
/// `param7` decoded to a byte) should be acted on by *this* manager's
/// single gimbal.
///
/// MAVLink GMv2 lets one manager front several gimbal devices; messages
/// carry the device's component-id so the manager can route within
/// itself. Turret runs one gimbal per manager today, advertised at
/// `config.compid`, so the per-message device-id must be either:
///
/// - `0` — "all gimbal device components" per spec (broadcast).
/// - `config.compid` — addressed specifically to us.
///
/// Any other value targets a different gimbal — non-MAVLink slots
/// `1..=6` are explicitly someone else's, and any other component id
/// is a different gimbal manager. We drop those silently rather than
/// stealing motion intended for a peer.
pub(super) fn addressed_to_this_device(gimbal_device_id: u8, our_compid: u8) -> bool {
    gimbal_device_id == 0 || gimbal_device_id == our_compid
}

/// Decode a COMMAND_LONG `param7` (an `f32` carrying the
/// `gimbal_device_id` per the spec for `MAV_CMD_DO_GIMBAL_MANAGER_*`)
/// into the byte form. Anything not representable as a clean `u8`
/// (NaN, infinity, negative, out of range, non-integer) returns `None`
/// — callers treat that as "not for us" so a malformed GCS can't
/// accidentally hit this gimbal.
pub(super) fn gimbal_device_id_from_param7(p: f32) -> Option<u8> {
    if !p.is_finite() || p < 0.0 || p > u8::MAX as f32 {
        return None;
    }
    let rounded = p.round();
    if (rounded - p).abs() > 0.5 {
        // Shouldn't trigger after the round, but guards against future
        // refactors. Belt + suspenders.
        return None;
    }
    Some(rounded as u8)
}

/// Translate a commanded yaw from whatever frame the GCS asked for into
/// the vehicle frame the STorM32 needs.
///
/// The MAVLink GMv2 spec lets the GCS pick frame via flags:
///   - `YAW_IN_VEHICLE_FRAME` (default) — yaw is relative to the
///     vehicle's nose. Pass through unchanged.
///   - `YAW_IN_EARTH_FRAME` — yaw is absolute earth heading. Subtract
///     the current vehicle heading from `AUTOPILOT_STATE_FOR_GIMBAL_DEVICE`
///     and pass the difference.
///
/// Missing or stale vehicle heading makes earth-frame input unavailable.
pub(super) fn yaw_to_vehicle_frame(
    raw_yaw_deg: f32,
    flags: GimbalManagerFlags,
    ctx: &RxCtx<'_>,
    label: &str,
) -> Option<f32> {
    let vehicle = flags.contains(GimbalManagerFlags::GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME);
    let earth = flags.contains(GimbalManagerFlags::GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME);
    if vehicle && earth {
        return None;
    }
    if vehicle || (!earth && !flags.contains(GimbalManagerFlags::GIMBAL_MANAGER_FLAGS_YAW_LOCK)) {
        return Some(raw_yaw_deg);
    }
    match ctx.state.vehicle_yaw_deg(VEHICLE_ATTITUDE_MAX_AGE) {
        Some(vehicle_yaw) => Some(earth_to_vehicle_yaw_deg(raw_yaw_deg, vehicle_yaw)),
        None => {
            warn!(
                "{}: YAW_IN_EARTH_FRAME requested but no fresh AUTOPILOT_STATE — \
                 rejecting unavailable coordinate transform",
                label
            );
            None
        }
    }
}

/// Common tail for SET_* handlers: arbitrate, then call the gimbal
/// handle if the arbitrator accepted.
pub(super) async fn dispatch_gimbal_command(
    command: GimbalCommand,
    ctx: &RxCtx<'_>,
    label: &str,
) -> bool {
    match ctx.arbitrator.execute(command, ctx.gimbal).await {
        Ok(progress) => {
            let dispatched = dispatch_succeeded(&progress);
            info!(?progress, dispatched, "MAVLink {} dispatch evidence", label);
            dispatched
        }
        Err(error) => {
            error!(%error, "MAVLink {} dispatch failed", label);
            false
        }
    }
}

/// This manager cannot transform horizon pitch/roll or execute retract/neutral flags.
pub(super) fn position_flags_supported(flags: GimbalManagerFlags) -> bool {
    !flags.intersects(
        GimbalManagerFlags::GIMBAL_MANAGER_FLAGS_ROLL_LOCK
            | GimbalManagerFlags::GIMBAL_MANAGER_FLAGS_PITCH_LOCK
            | GimbalManagerFlags::GIMBAL_MANAGER_FLAGS_RETRACT
            | GimbalManagerFlags::GIMBAL_MANAGER_FLAGS_NEUTRAL,
    )
}

/// Angular-rate transforms require vehicle angular velocity, which is not supplied here.
pub(super) fn rate_flags_supported(flags: GimbalManagerFlags) -> bool {
    position_flags_supported(flags)
        && !flags.intersects(
            GimbalManagerFlags::GIMBAL_MANAGER_FLAGS_YAW_LOCK
                | GimbalManagerFlags::GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME,
        )
}

/// Preserve omitted axes and reject earth-frame input without fresh vehicle state.
pub(super) fn selected_position(
    source: crate::daemon::models::ControlSource,
    pitch: Option<f32>,
    roll: Option<f32>,
    yaw: Option<f32>,
    flags: GimbalManagerFlags,
    ctx: &RxCtx<'_>,
) -> Option<GimbalCommand> {
    if !position_flags_supported(flags) {
        return None;
    }
    let yaw = match yaw {
        Some(yaw) => Some(yaw_to_vehicle_frame(yaw, flags, ctx, "position")?),
        None => None,
    };
    let limits = &ctx.config.limits;
    Some(GimbalCommand::new(
        source,
        pitch.map(|v| clamp_axis(v, limits.pitch_min_deg, limits.pitch_max_deg, "pitch")),
        roll.map(|v| clamp_axis(v, limits.roll_min_deg, limits.roll_max_deg, "roll")),
        yaw.map(|v| clamp_axis(v, limits.yaw_min_deg, limits.yaw_max_deg, "yaw")),
    ))
}

#[cfg(test)]
mod tests;
