use super::PitchPlanner;
use crate::Mini3ControlProfile;
use core::time::Duration;
use turret_core::AngularRate;

const TEST_RATE: f32 = core::f32::consts::PI / 36.0;

fn profile(minimum: i16) -> Mini3ControlProfile {
    Mini3ControlProfile::new(minimum, AngularRate::radians_per_second(TEST_RATE).unwrap()).unwrap()
}

#[test]
fn observed_pitch_targets_keep_the_verified_envelope_and_recenter_axis() {
    let profile = profile(-50);
    for pose in -50..=0 {
        for rate in [-TEST_RATE, 0.0, TEST_RATE] {
            assert!(
                (-50..=0).contains(
                    &profile
                        .observed_pitch_target(pose, rate, false)
                        .unwrap()
                        .tenths()
                )
            );
            assert_eq!(
                profile
                    .observed_pitch_target(pose, rate, true)
                    .unwrap()
                    .tenths(),
                0
            );
        }
    }
    assert_eq!(
        profile
            .observed_pitch_target(-25, 0.0, false)
            .unwrap()
            .tenths(),
        -25
    );
    for pose in [-51, 1] {
        assert!(profile.observed_pitch_target(pose, 0.0, false).is_err());
    }
    for rate in [f32::NAN, f32::INFINITY, TEST_RATE * 2.0] {
        assert!(profile.observed_pitch_target(-25, rate, false).is_err());
    }
}

#[test]
fn input_generates_a_target_but_only_a_write_commits_it() {
    let profile = profile(-900);
    let mut planner = PitchPlanner::from_confirmed_horizontal(profile);
    assert_eq!(planner.target(-TEST_RATE, Duration::ZERO).unwrap(), None);
    let next = planner
        .target(-TEST_RATE, Duration::from_millis(100))
        .unwrap();
    assert_eq!(next, Some(profile.pitch_target(-5).unwrap()));
    assert_eq!(planner.commanded(), 0);
    planner
        .written(
            profile.pitch_target(-5).unwrap(),
            Duration::from_millis(100),
        )
        .unwrap();
    assert_eq!(planner.commanded(), -5);
}

#[test]
fn a_gap_or_backwards_clock_never_accumulates_a_large_movement() {
    let mut planner = PitchPlanner::from_confirmed_horizontal(profile(-900));
    planner.target(-TEST_RATE, Duration::ZERO).unwrap();
    assert_eq!(
        planner.target(-TEST_RATE, Duration::from_secs(10)).unwrap(),
        None
    );
    assert_eq!(planner.target(-TEST_RATE, Duration::ZERO).unwrap(), None);
    assert_eq!(planner.commanded(), 0);
}

#[test]
fn a_manual_rate_cancels_recenter_and_revocation_discards_it() {
    let profile = profile(-900);
    let mut planner = PitchPlanner::from_confirmed_horizontal(profile);
    planner
        .written(profile.pitch_target(-300).unwrap(), Duration::ZERO)
        .unwrap();
    assert_eq!(
        planner.demand(0.0, true, Duration::ZERO).unwrap(),
        TEST_RATE
    );
    assert_eq!(
        planner.demand(0.0, false, Duration::ZERO).unwrap(),
        TEST_RATE
    );
    assert_eq!(
        planner.demand(-TEST_RATE, false, Duration::ZERO).unwrap(),
        -TEST_RATE
    );
    assert_eq!(planner.demand(0.0, false, Duration::ZERO).unwrap(), 0.0);
    planner.demand(0.0, true, Duration::ZERO).unwrap();
    planner.cancel(Duration::from_secs(1));
    assert_eq!(planner.demand(0.0, false, Duration::ZERO).unwrap(), 0.0);
}

#[test]
fn invalid_rates_cannot_mutate_an_active_planner() {
    let mut planner = PitchPlanner::from_confirmed_horizontal(profile(-900));
    for rate in [f32::NAN, f32::INFINITY, TEST_RATE * 2.0] {
        assert!(planner.target(rate, Duration::ZERO).is_err());
        assert!(planner.demand(rate, false, Duration::ZERO).is_err());
    }
    assert_eq!(planner.commanded(), 0);
}

#[test]
fn a_recenter_requires_a_new_request_after_an_input_gap() {
    let profile = profile(-900);
    let mut planner = PitchPlanner::from_confirmed_horizontal(profile);
    planner
        .written(profile.pitch_target(-300).unwrap(), Duration::ZERO)
        .unwrap();
    planner.discard_elapsed(Duration::ZERO);
    assert_eq!(
        planner.demand(0.0, true, Duration::ZERO).unwrap(),
        TEST_RATE
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
        TEST_RATE
    );
}

#[test]
fn a_planner_uses_its_configured_rate_for_targets_and_recenter() {
    let rate = AngularRate::radians_per_second(2_f32.to_radians()).unwrap();
    let profile = Mini3ControlProfile::new(-50, rate).unwrap();
    let mut planner = PitchPlanner::from_confirmed_horizontal(profile);
    assert!(planner.demand(-TEST_RATE, false, Duration::ZERO).is_err());
    assert!(planner.target(-TEST_RATE, Duration::ZERO).is_err());
    planner
        .target(-rate.as_radians_per_second(), Duration::ZERO)
        .unwrap();
    let target = planner
        .target(-rate.as_radians_per_second(), Duration::from_millis(100))
        .unwrap()
        .unwrap();
    assert_eq!(target.tenths(), -2);
    planner.written(target, Duration::from_millis(100)).unwrap();
    assert_eq!(
        planner
            .demand(0.0, true, Duration::from_millis(100))
            .unwrap(),
        rate.as_radians_per_second()
    );
}
