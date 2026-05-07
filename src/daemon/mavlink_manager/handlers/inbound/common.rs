//! Shared helpers used by every inbound handler: addressing predicate,
//! per-axis limit clamp, earth→vehicle yaw transform, and the common
//! arbitrate-then-dispatch tail for SET_* handlers.
//!
//! All `pub(super)` so the per-message handler files (`set.rs`,
//! `cmd_long.rs`) and the rate integrator (`rate.rs`) can call into
//! them without each duplicating the logic.

use crate::daemon::arbitrator::Outcome;
use crate::daemon::config::GimbalLimits;
use crate::daemon::mavlink_manager::math::{clamp_axis, earth_to_vehicle_yaw_deg};
use crate::daemon::mavlink_manager::RxCtx;
use crate::daemon::models::GimbalCommand;
use mavlink::ardupilotmega::GimbalManagerFlags;
use std::time::Duration;
use tracing::{error, info, warn};

/// How long an autopilot vehicle-attitude reading stays usable for
/// earth-frame yaw transformation. After this, earth-frame commands
/// fall back to vehicle frame with a warn.
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

/// Clamp `(pitch, roll, yaw)` to the configured device limits and log
/// per-axis if a clamp actually occurred. The order of the returned
/// tuple matches the input order, not the order arguments appear in
/// `GimbalCommand::new`.
pub(super) fn clamp_pry(pitch: f32, roll: f32, yaw: f32, limits: &GimbalLimits) -> (f32, f32, f32) {
    (
        clamp_axis(pitch, limits.pitch_min_deg, limits.pitch_max_deg, "pitch"),
        clamp_axis(roll, limits.roll_min_deg, limits.roll_max_deg, "roll"),
        clamp_axis(yaw, limits.yaw_min_deg, limits.yaw_max_deg, "yaw"),
    )
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
/// If `YAW_IN_EARTH_FRAME` is requested but vehicle attitude is stale
/// (no autopilot connected, or > [`VEHICLE_ATTITUDE_MAX_AGE`] since
/// the last frame), fall back to vehicle frame with a one-line warn.
/// Better honest degradation than silently rejecting the command.
pub(super) fn yaw_to_vehicle_frame(
    raw_yaw_deg: f32,
    flags: GimbalManagerFlags,
    ctx: &RxCtx<'_>,
    label: &str,
) -> f32 {
    if !flags.contains(GimbalManagerFlags::GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME) {
        return raw_yaw_deg;
    }
    match ctx.state.vehicle_yaw_deg(VEHICLE_ATTITUDE_MAX_AGE) {
        Some(vehicle_yaw) => earth_to_vehicle_yaw_deg(raw_yaw_deg, vehicle_yaw),
        None => {
            warn!(
                "{}: YAW_IN_EARTH_FRAME requested but no fresh AUTOPILOT_STATE — \
                 falling back to vehicle-frame interpretation",
                label
            );
            raw_yaw_deg
        }
    }
}

/// Common tail for SET_* handlers: arbitrate, then call the gimbal
/// handle if the arbitrator accepted.
pub(super) async fn dispatch_gimbal_command(command: GimbalCommand, ctx: &RxCtx<'_>, label: &str) {
    match ctx.arbitrator.arbitrate(command) {
        Outcome::Execute { pitch, roll, yaw } => {
            match ctx.gimbal.set_attitude(pitch, roll, yaw).await {
                Ok(()) => info!("Executed MAVLink {}", label),
                Err(e) => error!("Failed to execute MAVLink {}: {}", label, e),
            }
        }
        Outcome::Rejected => warn!("MAVLink {} rejected (lower priority)", label),
    }
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
