//! Continuous yaw drift corrector — the "self-erecting gyro" of the
//! daemon.
//!
//! ## Why
//!
//! The STorM32's IMU has accelerometer-corrected pitch and roll (gravity
//! is always there) but pure gyro-integrated yaw with no absolute
//! reference. Yaw bias drifts at roughly 0.5–2°/min on consumer MEMS
//! parts, so the `Calibration` offset captured at boot stales over a
//! session. The user-visible symptom is "I commanded yaw=10°, status
//! reads 9.2°" — and the gap grows the longer the daemon runs.
//!
//! ## How
//!
//! Once the gimbal has settled at a known SP, the operator-frame
//! `PV - SP` gap *is* the accumulated drift. Treating that gap as the
//! error signal of a slow integral controller, we nudge the
//! calibration offset on each attitude poll until `PV ≈ SP` again. The
//! filter has natural negative feedback: increasing the offset shifts
//! `PV = raw - offset` toward zero, which shrinks the next error term,
//! which shrinks the next correction. Geometric convergence with rate
//! `(1 - α)` per call.
//!
//! ## What it does NOT do
//!
//! - It does **not** replace one-shot `calibrate_yaw`. That command
//!   sets the boot-time offset; this corrector takes that as the
//!   starting point and refines it over time.
//! - It does **not** correct mid-motion. A `set` followed by a status
//!   poll while the gimbal is still slewing produces no correction —
//!   we only act when at least `SETTLE_AFTER` has elapsed since the
//!   most recent `set`.
//! - It does **not** move fast. `MAX_STEP_DEG` per call (4 Hz attitude
//!   poll) caps the slew rate at ~0.2°/s of correction. An IMU spike
//!   can't yank the offset across ranges that would surprise the
//!   operator.
//! - It does **not** assume a static vehicle. Most deployments are
//!   static mounts or parked vehicles with autopilot — in both cases
//!   the gimbal is at rest in the world between SETs, and the
//!   `PV - SP` error is genuine drift. In actively-yawing flight with
//!   pan-mode PAN, the IMU also moves with the vehicle and the
//!   "settled" assumption breaks. For that case, set
//!   `gimbal.yaw_corrector_enabled: false` in YAML.
//!
//! ## Future hardware paths (not in scope for this code)
//!
//! - **Encoder feedback** on the gimbal motor shafts (a STorM32
//!   firmware option) would give absolute axis positions independent
//!   of the IMU. From the daemon's perspective the encoder data would
//!   arrive through the same `IMU1ANGLES` channel and be drift-free
//!   for free; no daemon-side change required.
//! - **Non-orthogonal axis design** (each axis off-vertical so gravity
//!   provides a yaw-axis reference) is a hardware choice the firmware
//!   handles internally; transparent at the host level.
//! - **Cross-IMU comparison** (gimbal IMU vs autopilot IMU). When pan
//!   mode is HOLD or the vehicle is parked, the gimbal IMU yaw should
//!   equal `vehicle_imu_yaw + commanded_sp + mount_offset`. We
//!   already ingest the autopilot's yaw via
//!   `AUTOPILOT_STATE_FOR_GIMBAL_DEVICE`; a future revision could use
//!   it as a stronger reference (especially for actively-yawing
//!   flight). The current PV-vs-SP filter is a strict subset of that
//!   approach — both reduce to the same correction when the vehicle
//!   is stationary.

use crate::daemon::mavlink_manager::math::wrap_180_deg;
use std::time::Duration;

/// How long after the most recent `set` we wait before sampling the
/// `PV - SP` error. STorM32 motor speed is ~10°/s; a worst-case 30°
/// swing settles in ~2 s, so 2 s is the conservative pick. Past this
/// window the gimbal is at rest at *whatever* the settled value is —
/// and that's exactly the read we want, even if it's offset from the
/// SP (that offset is the drift we're correcting).
const SETTLE_AFTER: Duration = Duration::from_secs(2);

/// Integral-controller gain per call. With the attitude loop running
/// at 4 Hz, `α = 0.005` gives a time constant of ~50 s — slow enough
/// that an IMU noise spike doesn't visibly nudge the gimbal, fast
/// enough that real drift over minutes is caught and held.
const ALPHA: f32 = 0.005;

/// Maximum |offset change| per call. Caps slew if the error signal
/// jumps (boot-time offset way off, mid-motion sample slipped past
/// the gate, IMU outlier). With 4 Hz polling, 0.05° / call ≈ 0.2°/s
/// of correction — well below motor speed, so the gimbal isn't
/// visibly fighting the corrector even when a big drift is being
/// worked off.
const MAX_STEP_DEG: f32 = 0.05;

/// Errors larger than this (in absolute degrees) are rejected as
/// "not drift." Real drift over a session is rarely > 5°; a single
/// large reading usually indicates the gimbal is actually still
/// slewing (gate slipped), the operator just commanded something
/// outside the gimbal's range and the device clamped silently, or
/// the IMU has failed. In any of those cases we'd rather sit still
/// than slew toward a bad signal.
const MAX_PLAUSIBLE_ERROR_DEG: f32 = 30.0;

/// Per-call diagnostic summary. Returned for logging / observability;
/// the caller adds `correction_deg` to the existing offset.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CorrectorStep {
    /// Operator-frame `PV - SP` error, wrapped into `[-180, 180)`.
    /// This is what the corrector is trying to drive to zero. Sustained
    /// large values (over many calls) mean the corrector can't keep up
    /// — either drift rate exceeds the slew cap, or another bug.
    pub error_deg: f32,
    /// The actual correction we apply. Caller adds this to the
    /// existing offset (`offset_new = offset_old + correction_deg`).
    /// Same sign as `error_deg`: `PV > SP` ⇒ positive error ⇒ positive
    /// correction ⇒ larger offset ⇒ next `PV = raw - offset` is
    /// smaller, closing the loop.
    pub correction_deg: f32,
}

/// Stateless drift estimator. The calling code holds the actual
/// offset (in `GimbalHandle`) — this module just answers
/// "given these readings, by how much should the offset move now?"
///
/// `mount_yaw_deg` is plumbed through for future use (when we move to
/// an autopilot-cross-IMU reference for moving vehicles); the current
/// PV-vs-SP filter doesn't need it.
#[derive(Debug, Clone, Copy)]
pub struct YawCorrector {
    /// Mounting offset: gimbal-zero (body frame) relative to the vehicle
    /// nose, in degrees. Default 0 ("gimbal forward = nose"). Set in the
    /// `gimbal.mount_yaw_deg` YAML field. Unused by the current
    /// PV-vs-SP filter — kept for forward-compatibility with a future
    /// autopilot-cross-IMU mode.
    pub mount_yaw_deg: f32,
}

impl YawCorrector {
    /// Construct with the given mounting offset.
    pub const fn new(mount_yaw_deg: f32) -> Self {
        Self { mount_yaw_deg }
    }

    /// Compute one correction step.
    ///
    /// Returns `None` when no correction should be applied — no SP has
    /// been recorded yet, the gimbal hasn't settled long enough since
    /// the most recent `set`, or the error signal is implausibly
    /// large. `Some(step)` otherwise; the caller adds
    /// `step.correction_deg` to the current offset.
    ///
    /// All inputs in degrees / `Duration`:
    /// - `pv_op_yaw` — gimbal yaw the operator currently sees
    ///   (i.e. `raw_imu - current_offset`).
    /// - `last_sp_body` — most recent yaw SP (operator frame).
    /// - `time_since_last_set` — wall-clock since the SP was issued.
    pub fn step(
        &self,
        pv_op_yaw: f32,
        last_sp_body: Option<f32>,
        time_since_last_set: Option<Duration>,
    ) -> Option<CorrectorStep> {
        let sp = last_sp_body?;

        let time_since = time_since_last_set?;
        if time_since < SETTLE_AFTER {
            return None;
        }

        // Error signal in operator frame. Wrap so SP and PV that
        // straddle ±180° take the short way around.
        let error = wrap_180_deg(pv_op_yaw - sp);

        if !error.is_finite() || error.abs() > MAX_PLAUSIBLE_ERROR_DEG {
            return None;
        }

        let raw_correction = ALPHA * error;
        let correction = raw_correction.clamp(-MAX_STEP_DEG, MAX_STEP_DEG);

        Some(CorrectorStep {
            error_deg: error,
            correction_deg: correction,
        })
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    //! Pure-math tests for the corrector. No hardware, no async — the
    //! corrector is deliberately stateless so all the failure modes
    //! reduce to "given these inputs, did we return the right step?"

    use super::*;

    fn approx_eq(a: f32, b: f32, eps: f32) -> bool {
        (a - b).abs() < eps
    }

    /// Convenience: a settled `Duration` past `SETTLE_AFTER`.
    const SETTLED: Duration = Duration::from_secs(3);

    #[test]
    fn returns_none_without_recorded_sp() {
        let c = YawCorrector::new(0.0);
        assert!(c.step(0.0, None, Some(SETTLED)).is_none());
    }

    #[test]
    fn returns_none_when_still_slewing() {
        // Most recent `set` was 0.5 s ago — well within motor settling
        // time. Don't sample drift while the gimbal is en route.
        let c = YawCorrector::new(0.0);
        assert!(c
            .step(5.0, Some(0.0), Some(Duration::from_millis(500)))
            .is_none());
    }

    #[test]
    fn returns_none_with_no_set_yet() {
        // Daemon just started — `time_since_last_set` is None.
        let c = YawCorrector::new(0.0);
        assert!(c.step(0.0, Some(0.0), None).is_none());
    }

    #[test]
    fn perfect_calibration_yields_zero_correction() {
        // PV exactly at SP → zero error → zero correction.
        let c = YawCorrector::new(0.0);
        let step = c.step(10.0, Some(10.0), Some(SETTLED)).unwrap();
        assert!(
            approx_eq(step.error_deg, 0.0, 1e-6),
            "error {}",
            step.error_deg
        );
        assert!(approx_eq(step.correction_deg, 0.0, 1e-6));
    }

    #[test]
    fn positive_error_yields_positive_correction() {
        // PV reads 12 but SP was 10 → error = +2 → correction = +0.01
        // (under MAX_STEP). Positive correction means offset grows,
        // which makes PV = raw - offset come back down toward SP.
        let c = YawCorrector::new(0.0);
        let step = c.step(12.0, Some(10.0), Some(SETTLED)).unwrap();
        assert!(
            approx_eq(step.error_deg, 2.0, 1e-4),
            "error {}",
            step.error_deg
        );
        assert!(
            approx_eq(step.correction_deg, 0.01, 1e-4),
            "correction {}",
            step.correction_deg
        );
    }

    #[test]
    fn negative_error_yields_negative_correction() {
        // PV reads 5 but SP was 10 → error = -5 → correction = -0.025.
        let c = YawCorrector::new(0.0);
        let step = c.step(5.0, Some(10.0), Some(SETTLED)).unwrap();
        assert!(approx_eq(step.error_deg, -5.0, 1e-4));
        assert!(approx_eq(step.correction_deg, -0.025, 1e-4));
    }

    #[test]
    fn correction_is_capped_at_max_step() {
        // Big error (20° — credible right at boot if calibration was
        // never run on a brand-new install). Correction must clamp to
        // MAX_STEP_DEG so we don't slew the offset at user-visible
        // speed.
        let c = YawCorrector::new(0.0);
        let step = c.step(30.0, Some(10.0), Some(SETTLED)).unwrap();
        assert!(approx_eq(step.error_deg, 20.0, 1e-4));
        assert!(
            approx_eq(step.correction_deg, MAX_STEP_DEG, 1e-6),
            "correction {} should clamp to {MAX_STEP_DEG}",
            step.correction_deg
        );
    }

    #[test]
    fn error_wraps_correctly_across_180_boundary() {
        // PV at -179°, SP at +179° — naive subtraction gives -358°,
        // wrapped should be +2° (the short way). Correction +0.01.
        let c = YawCorrector::new(0.0);
        let step = c.step(-179.0, Some(179.0), Some(SETTLED)).unwrap();
        assert!(
            approx_eq(step.error_deg, 2.0, 1e-3),
            "error {} should wrap to ~+2",
            step.error_deg
        );
        assert!(approx_eq(step.correction_deg, 0.01, 1e-4));
    }

    #[test]
    fn rejects_implausibly_large_error() {
        // Error > MAX_PLAUSIBLE_ERROR_DEG ⇒ likely something other
        // than drift (mid-motion, clamped command, IMU failure). Sit
        // still rather than slew toward a bad signal.
        let c = YawCorrector::new(0.0);
        let step = c.step(50.0, Some(0.0), Some(SETTLED));
        assert!(step.is_none(), "expected None, got {step:?}");
    }

    #[test]
    fn rejects_nan_inputs() {
        let c = YawCorrector::new(0.0);
        assert!(c.step(f32::NAN, Some(0.0), Some(SETTLED)).is_none());
        assert!(c.step(0.0, Some(f32::NAN), Some(SETTLED)).is_none());
    }

    #[test]
    fn many_steps_converge_to_correct_offset() {
        // The realistic scenario: there's a true drift D in the IMU
        // bias relative to where calibration captured it. The
        // gimbal is settled at SP, so raw IMU reads sp + D and PV
        // (operator frame) reads sp + D - offset. The corrector
        // should drive offset toward D so PV converges to SP.
        let c = YawCorrector::new(0.0);
        let true_drift_deg: f32 = 5.0;
        let sp: f32 = 0.0;
        let mut offset: f32 = 0.0;
        for _ in 0..3000 {
            // raw stays at sp + true_drift forever (the gimbal is at
            // rest, the IMU has the bias). PV depends on the offset
            // we've accumulated.
            let raw = sp + true_drift_deg;
            let pv = raw - offset;
            let Some(step) = c.step(pv, Some(sp), Some(SETTLED)) else {
                panic!("corrector should fire when settled and SP is set");
            };
            offset += step.correction_deg;
        }
        // After convergence, offset should approximate true_drift.
        assert!(
            approx_eq(offset, true_drift_deg, 0.05),
            "offset {} did not converge to {} within 0.05°",
            offset,
            true_drift_deg
        );
    }
}
