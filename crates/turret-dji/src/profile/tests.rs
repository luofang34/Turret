use super::*;
use turret_core::{Command, CommandIdentity, CommandTiming, Executor, PayloadCommand, Tick};

#[test]
fn the_actual_pitch_contract_rejects_yaw_and_unverified_ranges() {
    let capabilities =
        Mini3ControlProfile::new(-50, AngularRate::radians_per_second(0.05).unwrap())
            .unwrap()
            .capabilities()
            .unwrap();
    let mut executor = Executor::new(capabilities, 0, 0);
    let command = |pitch, yaw| Command {
        identity: CommandIdentity {
            epoch: 0,
            generation: 0,
            id: 1,
            sequence: 1,
        },
        capability_revision: capabilities.revision,
        payload: PayloadCommand::GimbalPosition(GimbalPosition {
            angles: Axes {
                pitch,
                roll: None,
                yaw,
            },
            frame: ReferenceFrame::Mount,
        }),
        timing: CommandTiming {
            send_before: Tick(100),
            valid_until: None,
            ack_before: None,
            completion_before: None,
        },
    };
    assert!(
        executor
            .submit(command(None, Some(Angle::ZERO)), Tick(0))
            .is_err()
    );
    assert!(
        executor
            .submit(command(Some(Angle::radians(-1.0).unwrap()), None), Tick(0))
            .is_err()
    );
    executor
        .submit(command(Some(Angle::ZERO), None), Tick(0))
        .unwrap();
    assert!(executor.prepare_send(Tick(1)).unwrap().is_some());
    assert_eq!(
        capabilities.zoom.available(),
        Err(turret_core::Error::NotImplemented)
    );
    assert_eq!(capabilities.gimbal.stop, StopGuarantee::Unknown);
}

#[test]
fn the_version_query_is_valid_read_only_duml() {
    let mut parser = crate::DumlParser::new();
    let mut observed = None;
    parser.push(&rc_version_request(), |frame| observed = Some(frame));
    let frame = observed.unwrap();
    assert_eq!((frame.sender().0, frame.receiver().0), (10, 6));
    assert_eq!((frame.command_set(), frame.command_id()), (0, 1));
    assert!(frame.payload().is_empty());
}

#[test]
fn profiles_require_supported_geometry_and_positive_policy_rates() {
    let rate = AngularRate::radians_per_second(5_f32.to_radians()).unwrap();
    for minimum in [i16::MIN, -901, 1, i16::MAX] {
        assert!(Mini3ControlProfile::new(minimum, rate).is_err());
    }
    for rate in [0.0, -0.1] {
        assert!(
            Mini3ControlProfile::new(-50, AngularRate::radians_per_second(rate).unwrap()).is_err()
        );
    }
    for degrees_per_second in [2_f32, 5.0, 10.0] {
        let rate = AngularRate::radians_per_second(degrees_per_second.to_radians()).unwrap();
        let profile = Mini3ControlProfile::new(-50, rate).unwrap();
        let capabilities = profile.capabilities().unwrap();
        assert_eq!(
            capabilities
                .gimbal
                .axes
                .pitch
                .rate
                .available()
                .unwrap()
                .maximum_rate,
            rate
        );
        assert_eq!(
            capabilities
                .gimbal
                .axes
                .pitch
                .position
                .available()
                .unwrap()
                .maximum_rate,
            None
        );
    }
}

#[test]
fn planners_and_encoded_targets_use_the_advertised_envelope() {
    use crate::{DumlParser, PitchPlanner};
    use core::time::Duration;

    for minimum in [-900, -50, -37, -1, 0] {
        let rate = AngularRate::radians_per_second(5_f32.to_radians()).unwrap();
        let profile = Mini3ControlProfile::new(minimum, rate).unwrap();
        let capabilities = profile.capabilities().unwrap();
        let position = capabilities.gimbal.axes.pitch.position.available().unwrap();
        let mut planner = PitchPlanner::from_confirmed_horizontal(profile);
        planner
            .target(-rate.as_radians_per_second(), Duration::ZERO)
            .unwrap();
        for step in 1..=2000 {
            let now = Duration::from_millis(step * 100);
            let demand = if step <= 1000 {
                -rate.as_radians_per_second()
            } else {
                rate.as_radians_per_second()
            };
            let target = planner.target(demand, now).unwrap();
            if let Some(target) = target {
                assert!((minimum..=0).contains(&target.tenths()));
                position
                    .range
                    .validate(
                        Angle::radians(f32::from(target.tenths()) * core::f32::consts::PI / 1800.0)
                            .unwrap(),
                    )
                    .unwrap();
                let mut encoded = None;
                DumlParser::new().push(
                    &profile.encode_pitch(target, step as u16).unwrap(),
                    |frame| {
                        encoded = Some(frame);
                    },
                );
                let encoded = encoded.unwrap();
                assert_eq!(
                    i16::from_le_bytes([encoded.payload()[4], encoded.payload()[5]]),
                    target.tenths()
                );
                planner.written(target, now).unwrap();
            }
            if step == 1000 {
                assert_eq!(planner.commanded(), minimum);
            }
        }
        assert_eq!(planner.commanded(), 0);
    }
}

#[test]
fn final_encoding_and_write_commit_reject_a_target_from_a_wider_profile() {
    use crate::PitchPlanner;
    use core::time::Duration;

    let rate = AngularRate::radians_per_second(5_f32.to_radians()).unwrap();
    let wide = Mini3ControlProfile::new(-900, rate).unwrap();
    let narrow = Mini3ControlProfile::new(-50, rate).unwrap();
    let outside = wide.pitch_target(-51).unwrap();
    assert!(narrow.encode_pitch(outside, 1).is_err());
    let mut planner = PitchPlanner::from_confirmed_horizontal(narrow);
    assert!(planner.written(outside, Duration::ZERO).is_err());
    assert_eq!(planner.commanded(), 0);
    let boundary = wide.pitch_target(-50).unwrap();
    assert!(narrow.encode_pitch(boundary, 2).is_ok());
    planner.written(boundary, Duration::ZERO).unwrap();
    assert_eq!(planner.commanded(), -50);
}

#[test]
fn observed_targets_and_recenter_share_the_selected_range_and_rate() {
    for minimum in [-900, -50, -37] {
        let rate = AngularRate::radians_per_second(2_f32.to_radians()).unwrap();
        let profile = Mini3ControlProfile::new(minimum, rate).unwrap();
        for observed in minimum..=0 {
            for demand in [
                -rate.as_radians_per_second(),
                0.0,
                rate.as_radians_per_second(),
            ] {
                let target = profile
                    .observed_pitch_target(observed, demand, false)
                    .unwrap();
                assert!((minimum..=0).contains(&target.tenths()));
                assert!(profile.encode_pitch(target, 1).is_ok());
                assert_eq!(
                    profile
                        .observed_pitch_target(observed, demand, true)
                        .unwrap()
                        .tenths(),
                    (observed + 20).min(0)
                );
            }
        }
        assert_eq!(
            profile.observed_hold_target(minimum - 1).unwrap().tenths(),
            minimum
        );
        assert!(
            profile
                .observed_pitch_target(minimum, rate.as_radians_per_second() * 2.0, false)
                .is_err()
        );
        assert_eq!(
            profile
                .observed_pitch_target(0, -rate.as_radians_per_second(), false)
                .unwrap()
                .tenths(),
            -20
        );
    }
}

#[test]
fn observed_recenter_is_one_bounded_step_and_not_an_immediate_zero_target() {
    let profile = Mini3ControlProfile::new(
        -900,
        AngularRate::radians_per_second(2_f32.to_radians()).unwrap(),
    )
    .unwrap();
    let target = profile.observed_pitch_target(-900, 0.0, true).unwrap();
    assert_eq!(target.tenths(), -880);
    let mut encoded = None;
    crate::DumlParser::new().push(&profile.encode_pitch(target, 1).unwrap(), |frame| {
        encoded = Some(frame);
    });
    let encoded = encoded.unwrap();
    assert_eq!(
        i16::from_le_bytes([encoded.payload()[4], encoded.payload()[5]]),
        -880
    );
    assert_eq!(encoded.payload()[7], 10);
}

#[test]
fn feedback_envelope_does_not_expand_the_command_envelope() {
    let profile = Mini3ControlProfile::new(
        -900,
        AngularRate::radians_per_second(30_f32.to_radians()).unwrap(),
    )
    .unwrap();
    for observed in [-1350, -901, 1, 800] {
        assert!(profile.validate_observed_pitch(observed).is_ok());
        assert!(profile.pitch_target(observed).is_err());
    }
    for observed in [i16::MIN, -1351, 801, i16::MAX] {
        assert!(profile.validate_observed_pitch(observed).is_err());
        assert!(profile.observed_hold_target(observed).is_err());
    }
    assert_eq!(profile.observed_hold_target(1).unwrap().tenths(), 0);
    assert_eq!(profile.observed_hold_target(-901).unwrap().tenths(), -900);
    assert_eq!(
        profile
            .observed_pitch_target(1, -30_f32.to_radians(), false)
            .unwrap()
            .tenths(),
        -299
    );
    assert_eq!(
        profile
            .observed_pitch_target(1, 0.0, true)
            .unwrap()
            .tenths(),
        0
    );
    for observed in [-1350, 800] {
        assert!(profile.observed_hold_target(observed).is_err());
        assert!(profile.observed_pitch_target(observed, 0.0, true).is_err());
    }
}

#[test]
fn every_legal_observed_step_preserves_the_policy_budget_and_wire_range() {
    for degrees in [0.01_f32, 2.03, 5.0, 30.0] {
        let rate = AngularRate::radians_per_second(degrees.to_radians()).unwrap();
        let profile = Mini3ControlProfile::new(-900, rate).unwrap();
        for observed in -1350..=800 {
            for demand in [
                -rate.as_radians_per_second(),
                0.0,
                rate.as_radians_per_second(),
            ] {
                for recenter in [false, true] {
                    if let Ok(target) = profile.observed_pitch_target(observed, demand, recenter) {
                        assert!((-900..=0).contains(&target.tenths()));
                        let step = (i32::from(target.tenths()) - i32::from(observed)).abs();
                        assert!(f64::from(step) <= f64::from(degrees) * 10.0 + 0.00001);
                        assert!(profile.encode_pitch(target, 1).is_ok());
                    }
                }
            }
        }
    }
}

#[test]
fn a_queued_target_is_valid_only_within_the_current_observed_step() {
    let rate = AngularRate::radians_per_second(30_f32.to_radians()).unwrap();
    let profile = Mini3ControlProfile::new(-900, rate).unwrap();
    let target = profile.observed_hold_target(1).unwrap();
    for observed in [-300, 0, 1, 300] {
        assert!(profile.validate_observed_target(observed, target).is_ok());
    }
    for observed in [-301, 301, 800, -1350, i16::MAX] {
        assert!(profile.validate_observed_target(observed, target).is_err());
    }
    let narrow = Mini3ControlProfile::new(-50, rate).unwrap();
    assert!(
        narrow
            .validate_observed_target(-51, profile.pitch_target(-51).unwrap())
            .is_err()
    );
}

#[test]
fn three_axis_stabilization_does_not_advertise_three_axis_operator_control() {
    let caps = Mini3ControlProfile::new(
        -900,
        AngularRate::radians_per_second(30_f32.to_radians()).unwrap(),
    )
    .unwrap()
    .capabilities()
    .unwrap();
    for axis in [
        caps.gimbal.axes.pitch,
        caps.gimbal.axes.roll,
        caps.gimbal.axes.yaw,
    ] {
        assert_eq!(axis.mechanics.movement, HardwareSupport::Supported);
        assert_eq!(axis.mechanics.stabilization, HardwareSupport::Supported);
        assert_eq!(axis.feedback.angle, Capability::Unknown);
    }
    assert!(caps.gimbal.axes.pitch.rate.available().is_ok());
    assert_eq!(caps.gimbal.axes.yaw.position, Capability::Unsupported);
    assert_eq!(caps.gimbal.axes.yaw.rate, Capability::Unsupported);
    assert_eq!(caps.gimbal.axes.roll.rate, Capability::Unsupported);
    assert_eq!(
        caps.gimbal.axes.roll.position,
        Capability::Unimplemented { capability: None }
    );
    assert_eq!(caps.zoom, Capability::Unimplemented { capability: None });
}
