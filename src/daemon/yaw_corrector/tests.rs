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
    let c = YawCorrector::new();
    assert!(c.step(0.0, None, Some(SETTLED)).is_none());
}

#[test]
fn returns_none_when_still_slewing() {
    // Most recent `set` was 0.5 s ago — well within motor settling
    // time. Don't sample drift while the gimbal is en route.
    let c = YawCorrector::new();
    assert!(c
        .step(5.0, Some(0.0), Some(Duration::from_millis(500)))
        .is_none());
}

#[test]
fn returns_none_with_no_set_yet() {
    // Daemon just started — `time_since_last_set` is None.
    let c = YawCorrector::new();
    assert!(c.step(0.0, Some(0.0), None).is_none());
}

#[test]
fn perfect_calibration_yields_zero_correction() {
    // PV exactly at SP → zero error → zero correction.
    let c = YawCorrector::new();
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
    let c = YawCorrector::new();
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
    let c = YawCorrector::new();
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
    let c = YawCorrector::new();
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
    let c = YawCorrector::new();
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
    let c = YawCorrector::new();
    let step = c.step(50.0, Some(0.0), Some(SETTLED));
    assert!(step.is_none(), "expected None, got {step:?}");
}

#[test]
fn rejects_nan_inputs() {
    let c = YawCorrector::new();
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
    let c = YawCorrector::new();
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
