use super::{MAX_PITCH_RATE, PitchPlanner};
use crate::GimbalPitchTarget;
use core::time::Duration;

#[test]
fn observed_pitch_targets_keep_the_verified_envelope_and_recenter_axis() {
    use super::observed_pitch_target;
    for pose in -50..=0 {
        for rate in [-MAX_PITCH_RATE, 0.0, MAX_PITCH_RATE] {
            assert!(
                (-50..=0).contains(&observed_pitch_target(pose, rate, false).unwrap().tenths())
            );
            assert_eq!(observed_pitch_target(pose, rate, true).unwrap().tenths(), 0);
        }
    }
    assert_eq!(
        observed_pitch_target(-25, 0.0, false).unwrap().tenths(),
        -25
    );
    for pose in [-51, 1] {
        assert!(observed_pitch_target(pose, 0.0, false).is_err());
    }
    for rate in [f32::NAN, f32::INFINITY, MAX_PITCH_RATE * 2.0] {
        assert!(observed_pitch_target(-25, rate, false).is_err());
    }
}

#[test]
fn input_generates_a_target_but_only_a_write_commits_it() {
    let mut planner = PitchPlanner::from_confirmed_horizontal();
    assert_eq!(
        planner.target(-MAX_PITCH_RATE, Duration::ZERO).unwrap(),
        None
    );
    let next = planner
        .target(-MAX_PITCH_RATE, Duration::from_millis(100))
        .unwrap();
    assert_eq!(next, Some(-5));
    assert_eq!(planner.commanded(), 0);
    planner.written(
        GimbalPitchTarget::from_tenths(-5).unwrap(),
        Duration::from_millis(100),
    );
    assert_eq!(planner.commanded(), -5);
}

#[test]
fn a_gap_or_backwards_clock_never_accumulates_a_large_movement() {
    let mut planner = PitchPlanner::from_confirmed_horizontal();
    planner.target(-MAX_PITCH_RATE, Duration::ZERO).unwrap();
    assert_eq!(
        planner
            .target(-MAX_PITCH_RATE, Duration::from_secs(10))
            .unwrap(),
        None
    );
    assert_eq!(
        planner.target(-MAX_PITCH_RATE, Duration::ZERO).unwrap(),
        None
    );
    assert_eq!(planner.commanded(), 0);
}

#[test]
fn a_manual_rate_cancels_recenter_and_revocation_discards_it() {
    let mut planner = PitchPlanner::from_confirmed_horizontal();
    planner.written(
        GimbalPitchTarget::from_tenths(-300).unwrap(),
        Duration::ZERO,
    );
    assert_eq!(
        planner.demand(0.0, true, Duration::ZERO).unwrap(),
        MAX_PITCH_RATE
    );
    assert_eq!(
        planner.demand(0.0, false, Duration::ZERO).unwrap(),
        MAX_PITCH_RATE
    );
    assert_eq!(
        planner
            .demand(-MAX_PITCH_RATE, false, Duration::ZERO)
            .unwrap(),
        -MAX_PITCH_RATE
    );
    assert_eq!(planner.demand(0.0, false, Duration::ZERO).unwrap(), 0.0);
    planner.demand(0.0, true, Duration::ZERO).unwrap();
    planner.cancel(Duration::from_secs(1));
    assert_eq!(planner.demand(0.0, false, Duration::ZERO).unwrap(), 0.0);
}

#[test]
fn invalid_rates_cannot_mutate_an_active_planner() {
    let mut planner = PitchPlanner::from_confirmed_horizontal();
    for rate in [f32::NAN, f32::INFINITY, MAX_PITCH_RATE * 2.0] {
        assert!(planner.target(rate, Duration::ZERO).is_err());
        assert!(planner.demand(rate, false, Duration::ZERO).is_err());
    }
    assert_eq!(planner.commanded(), 0);
}

#[test]
fn a_recenter_requires_a_new_request_after_an_input_gap() {
    let mut planner = PitchPlanner::from_confirmed_horizontal();
    planner.written(
        GimbalPitchTarget::from_tenths(-300).unwrap(),
        Duration::ZERO,
    );
    planner.discard_elapsed(Duration::ZERO);
    assert_eq!(
        planner.demand(0.0, true, Duration::ZERO).unwrap(),
        MAX_PITCH_RATE
    );
    assert_eq!(
        planner
            .demand(0.0, false, Duration::from_millis(101))
            .unwrap(),
        0.0
    );
    assert_eq!(
        planner
            .demand(0.0, false, Duration::from_millis(120))
            .unwrap(),
        0.0
    );
    assert_eq!(
        planner
            .demand(0.0, true, Duration::from_millis(140))
            .unwrap(),
        MAX_PITCH_RATE
    );
}
