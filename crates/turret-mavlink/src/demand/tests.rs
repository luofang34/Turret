use super::*;

#[test]
fn selected_pitch_rate_omits_other_axes_and_preserves_reference() {
    let command = PayloadCommand::GimbalRate(GimbalRate {
        rates: Axes {
            pitch: Some(AngularRate::radians_per_second(-0.4).unwrap()),
            roll: None,
            yaw: None,
        },
        frame: ReferenceFrame::HorizonVehicleYaw,
    });
    let Demand::AttitudeRate {
        flags,
        pitch_rps,
        roll_rps,
        yaw_rps,
        gimbal_device_id,
    } = encode_payload(command, 3, 0).unwrap()
    else {
        panic!("expected manager rate packet");
    };
    assert_eq!(flags, 12);
    assert_eq!(pitch_rps, -0.4);
    assert!(roll_rps.is_nan());
    assert!(yaw_rps.is_nan());
    assert_eq!(gimbal_device_id, 3);
}

#[test]
fn ordinary_vehicle_frame_cannot_silently_become_horizon_pitch() {
    let command = PayloadCommand::GimbalRate(GimbalRate {
        rates: Axes {
            pitch: Some(AngularRate::ZERO),
            roll: None,
            yaw: None,
        },
        frame: ReferenceFrame::Vehicle,
    });
    assert_eq!(encode_payload(command, 0, 0), Err(Error::UnsupportedFrame));
}

#[test]
fn recenter_claim_and_camera_keep_distinct_protocol_targets() {
    let Demand::CommandLong { command, params } = claim_command(42, 17, 3) else {
        panic!("expected claim command");
    };
    assert_eq!(command, 1001);
    assert_eq!(params, [42.0, 17.0, -1.0, -1.0, 0.0, 0.0, 3.0]);
    let Demand::CommandLong { command, params } =
        encode_payload(PayloadCommand::Recenter, 3, 100).unwrap()
    else {
        panic!("expected recenter command");
    };
    assert_eq!(command, 1000);
    assert_eq!(params[0..2], [0.0, 0.0]);
    assert!(params[2].is_nan());
    assert!(params[3].is_nan());
    assert_eq!(params[4], 12.0);
    assert_eq!(params[6], 3.0);
}

#[test]
fn zoom_coordinates_map_only_to_their_native_protocol_units() {
    let examples = [
        (ZoomUnit::TravelFraction, 0.3, 2.0, 30.0),
        (ZoomUnit::FocalLengthMm, 50.0, 3.0, 50.0),
        (
            ZoomUnit::HorizontalFovRadians,
            core::f32::consts::FRAC_PI_2,
            4.0,
            90.0,
        ),
    ];
    for (unit, value, expected_type, expected_value) in examples {
        let payload = PayloadCommand::Zoom(ZoomCommand::Absolute(
            ZoomAbsolute::new(unit, value).unwrap(),
        ));
        let Demand::CommandLong { command, params } = encode_payload(payload, 0, 100).unwrap()
        else {
            panic!("expected camera command");
        };
        assert_eq!(command, 531);
        assert_eq!(params[0], expected_type);
        assert!((params[1] - expected_value).abs() < 0.0001);
        assert_eq!(params[2], 100.0);
    }
    let ratio = PayloadCommand::Zoom(ZoomCommand::Absolute(
        ZoomAbsolute::new(ZoomUnit::Ratio, 2.0).unwrap(),
    ));
    assert_eq!(encode_payload(ratio, 0, 0), Err(Error::Unsupported));
}

#[test]
fn camera_continuous_is_normalized_and_discrete_actions_do_not_invent_cancel() {
    let payload = PayloadCommand::Zoom(ZoomCommand::Continuous(
        NormalizedZoomRate::new(-0.5).unwrap(),
    ));
    let Demand::CommandLong { params, .. } = encode_payload(payload, 0, 2).unwrap() else {
        panic!("expected camera command");
    };
    assert_eq!(params[0..3], [1.0, -0.5, 2.0]);
    assert_eq!(
        stop_command(payload).unwrap(),
        PayloadCommand::Zoom(ZoomCommand::Continuous(
            NormalizedZoomRate::new(0.0).unwrap()
        ))
    );
    assert_eq!(
        stop_command(PayloadCommand::Zoom(ZoomCommand::Step(1))),
        Err(Error::Unsupported)
    );
    assert_eq!(
        encode_payload(PayloadCommand::Zoom(ZoomCommand::Step(2)), 0, 0),
        Err(Error::OutOfRange)
    );
}

#[test]
fn direct_position_encoding_rejects_angles_outside_protocol_domain() {
    let position = PayloadCommand::GimbalPosition(GimbalPosition {
        angles: Axes {
            pitch: Some(Angle::radians(4.0).unwrap()),
            roll: None,
            yaw: None,
        },
        frame: ReferenceFrame::HorizonVehicleYaw,
    });
    assert_eq!(encode_payload(position, 1, 0), Err(Error::OutOfRange));
}

#[test]
fn release_cannot_unconditionally_clear_another_primary_owner() {
    let Demand::CommandLong { command, params } = release_command(3) else {
        panic!("expected release");
    };
    assert_eq!(command, 1001);
    assert_eq!(params, [-3.0, -3.0, -1.0, -1.0, 0.0, 0.0, 3.0]);
}
