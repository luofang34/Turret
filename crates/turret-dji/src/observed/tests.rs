use super::*;
use turret_core::AngularRate;

fn planner() -> ObservedPitchPlanner {
    ObservedPitchPlanner::new(
        Mini3ControlProfile::new(
            -900,
            AngularRate::radians_per_second(5_f32.to_radians()).unwrap(),
        )
        .unwrap(),
    )
}

fn at(millis: u64) -> Duration {
    Duration::from_millis(millis)
}

#[test]
fn a_pending_plan_keeps_its_identity_but_only_a_full_write_suppresses_it() {
    let mut planner = planner();
    let rate = -5_f32.to_radians();
    let plan = planner.plan(0, rate, false, at(0)).unwrap().unwrap();
    assert_eq!(plan.target().tenths(), -50);
    assert_eq!(planner.plan(0, rate, false, at(40)).unwrap(), Some(plan));
    assert!(planner.written(plan));
    assert_eq!(planner.plan(0, rate, false, at(80)).unwrap(), None);
    let next = planner.plan(-5, rate, false, at(120)).unwrap().unwrap();
    assert_eq!(next.target().tenths(), -55);
    assert!(!planner.written(plan));
    assert_eq!(planner.plan(-5, rate, false, at(160)).unwrap(), Some(next));
}

#[test]
fn recenter_follows_feedback_to_zero_and_does_not_repeat_the_final_target() {
    let mut planner = planner();
    for step in 0..=180 {
        let pitch = -900 + step * 5;
        let plan = planner
            .plan(pitch, 0.0, step == 0, at(step as u64 * 100))
            .unwrap();
        if pitch < -50 {
            let plan = plan.unwrap();
            assert_eq!(plan.target().tenths(), pitch + 50);
            assert!(planner.written(plan));
        } else if pitch == -50 {
            let plan = plan.unwrap();
            assert_eq!(plan.target().tenths(), 0);
            assert!(planner.written(plan));
        } else {
            assert_eq!(plan, None);
        }
    }
    assert_eq!(planner.plan(0, 0.0, false, at(18_100)).unwrap(), None);
}

#[test]
fn manual_input_cancels_recenter_and_neutral_requests_one_measured_hold() {
    let mut planner = planner();
    let recenter = planner.plan(-300, 0.0, true, at(0)).unwrap().unwrap();
    let manual = planner
        .plan(-300, -5_f32.to_radians(), false, at(40))
        .unwrap()
        .unwrap();
    assert_eq!(manual.target().tenths(), -350);
    assert!(!planner.written(recenter));
    assert!(planner.written(manual));
    let hold = planner.plan(-302, 0.0, false, at(80)).unwrap().unwrap();
    assert_eq!(hold.target().tenths(), -302);
    assert_eq!(planner.plan(-304, 0.0, false, at(120)).unwrap(), Some(hold));
    assert!(planner.written(hold));
    assert_eq!(planner.plan(-304, 0.0, false, at(160)).unwrap(), None);
}

#[test]
fn gaps_backwards_time_and_explicit_cancel_do_not_resume_recenter() {
    for interruption in [at(101), at(0)] {
        let mut planner = planner();
        let old = planner.plan(-300, 0.0, true, at(1)).unwrap().unwrap();
        assert!(planner.written(old));
        let later = if interruption == at(101) {
            at(102)
        } else {
            interruption
        };
        assert_eq!(planner.plan(-295, 0.0, false, later).unwrap(), None);
    }
    let mut planner = planner();
    let old = planner.plan(-300, 0.0, true, at(0)).unwrap().unwrap();
    planner.cancel();
    assert!(!planner.written(old));
    assert_eq!(planner.plan(-295, 0.0, false, at(40)).unwrap(), None);
}

#[test]
fn an_old_zero_receipt_cannot_suppress_a_new_recenter_to_the_same_target() {
    let mut planner = planner();
    let old = planner.plan(-40, 0.0, true, at(0)).unwrap().unwrap();
    planner.cancel();
    let new = planner.plan(-40, 0.0, true, at(40)).unwrap().unwrap();
    assert_eq!(old.target(), new.target());
    assert_ne!(old, new);
    assert!(!planner.written(old));
    assert_eq!(planner.plan(-40, 0.0, false, at(80)).unwrap(), Some(new));
    assert!(planner.written(new));
    assert_eq!(planner.plan(-30, 0.0, false, at(120)).unwrap(), None);
    let explicit = planner.plan(-30, 0.0, true, at(160)).unwrap().unwrap();
    assert_eq!(explicit.target().tenths(), 0);
    assert_ne!(explicit, new);
}

#[test]
fn invalid_inputs_create_no_plan_and_geometry_stays_bounded() {
    let mut planner = planner();
    for pitch in [-1351, 801] {
        assert!(planner.plan(pitch, 0.0, true, at(0)).is_err());
    }
    for rate in [f32::NAN, f32::INFINITY, 6_f32.to_radians()] {
        assert!(planner.plan(-300, rate, false, at(0)).is_err());
    }
    for pitch in -900..=0 {
        for rate in [-5_f32.to_radians(), 5_f32.to_radians()] {
            planner.cancel();
            let plan = planner.plan(pitch, rate, false, at(0)).unwrap().unwrap();
            assert!((-900..=0).contains(&plan.target().tenths()));
        }
    }
}

#[test]
fn rejected_input_or_observation_cancels_recenter_and_invalidates_its_receipt() {
    for (pitch, rate) in [
        (-300, f32::NAN),
        (-300, 6_f32.to_radians()),
        (-1351, 0.0),
        (801, 0.0),
    ] {
        let mut planner = planner();
        let old = planner.plan(-300, 0.0, true, at(0)).unwrap().unwrap();
        assert!(planner.plan(pitch, rate, false, at(40)).is_err());
        assert!(!planner.written(old));
        assert_eq!(planner.plan(-295, 0.0, false, at(80)).unwrap(), None);
    }
}

#[test]
fn positive_feedback_is_not_a_zero_arrival_or_a_new_receipt() {
    let mut planner = planner();
    let plan = planner.plan(1, 0.0, true, at(0)).unwrap().unwrap();
    assert_eq!(plan.target().tenths(), 0);
    assert_eq!(planner.motion, Motion::Recenter);
    assert!(planner.written(plan));
    assert_eq!(planner.plan(1, 0.0, false, at(40)).unwrap(), None);
    assert_eq!(planner.motion, Motion::Recenter);
    assert_eq!(planner.plan(0, 0.0, false, at(80)).unwrap(), None);
    assert_eq!(planner.motion, Motion::Idle);
}

#[test]
fn neutral_after_manual_uses_one_bounded_hold_without_changing_the_feedback() {
    let mut planner = planner();
    let manual = planner
        .plan(1, -5_f32.to_radians(), false, at(0))
        .unwrap()
        .unwrap();
    assert_eq!(manual.target().tenths(), -49);
    assert!(planner.written(manual));
    let hold = planner.plan(1, 0.0, false, at(40)).unwrap().unwrap();
    assert_eq!(hold.target().tenths(), 0);
    assert_eq!(planner.plan(2, 0.0, false, at(80)).unwrap(), Some(hold));
    assert!(planner.written(hold));
    assert_eq!(planner.plan(1, 0.0, false, at(120)).unwrap(), None);
}

#[test]
fn unreachable_legal_target_cancels_the_episode_and_old_receipts() {
    for (outside, recenter) in [(800, false), (-1350, false), (800, true), (-1350, true)] {
        let mut planner = planner();
        let old = planner
            .plan(-300, -5_f32.to_radians(), false, at(0))
            .unwrap()
            .unwrap();
        assert!(planner.plan(outside, 0.0, recenter, at(40)).is_err());
        assert!(!planner.written(old));
        assert_eq!(planner.plan(-300, 0.0, false, at(80)).unwrap(), None);
    }
}

#[test]
fn a_retained_hold_is_rechecked_against_each_fresh_observation() {
    for observed in [-51, 800, -1350] {
        let mut planner = planner();
        planner.plan(1, -5_f32.to_radians(), false, at(0)).unwrap();
        let hold = planner.plan(1, 0.0, false, at(40)).unwrap().unwrap();
        assert_eq!(hold.target().tenths(), 0);
        assert!(planner.plan(observed, 0.0, false, at(80)).is_err());
        assert!(!planner.written(hold));
        assert_eq!(planner.plan(-10, 0.0, false, at(120)).unwrap(), None);
    }
}

#[test]
fn manual_input_never_reverses_or_enlarges_to_reenter_the_command_envelope() {
    for (observed, rate) in [
        (1, 5_f32.to_radians()),
        (-901, -5_f32.to_radians()),
        (20, -0.01_f32.to_radians()),
    ] {
        let mut planner = planner();
        let old = planner.plan(-300, 0.0, true, at(0)).unwrap().unwrap();
        assert!(planner.plan(observed, rate, false, at(40)).is_err());
        assert!(!planner.written(old));
        assert_eq!(planner.plan(-10, 0.0, false, at(80)).unwrap(), None);
    }
    let mut planner = planner();
    let plan = planner
        .plan(-901, 5_f32.to_radians(), false, at(0))
        .unwrap()
        .unwrap();
    assert_eq!(plan.target().tenths(), -851);
}
