use super::*;

fn information(flags: u32) -> ManagerInformation {
    ManagerInformation {
        cap_flags: flags,
        roll_min: -0.5,
        roll_max: 0.5,
        pitch_min: -1.5,
        pitch_max: 0.5,
        yaw_min: -2.0,
        yaw_max: 2.0,
    }
}

fn capabilities(info: Option<ManagerInformation>) -> Result<PayloadCapabilities, Error> {
    let maximum = AngularRate::radians_per_second(0.8).unwrap();
    manager_capabilities(7, info, maximum, maximum)
}

fn submit(
    caps: PayloadCapabilities,
    payload: PayloadCommand,
) -> Result<Option<CommandProgress>, Error> {
    Executor::new(caps, 4, 6).submit(
        Command {
            identity: CommandIdentity {
                epoch: 4,
                generation: 6,
                id: 1,
                sequence: 1,
            },
            capability_revision: 7,
            payload,
            timing: CommandTiming {
                send_before: Tick(10),
                valid_until: payload.is_continuous().then_some(Tick(100)),
                ack_before: None,
                completion_before: None,
            },
        },
        Tick(0),
    )
}

fn pitch_rate(frame: ReferenceFrame) -> PayloadCommand {
    PayloadCommand::GimbalRate(GimbalRate {
        rates: Axes {
            pitch: Some(AngularRate::ZERO),
            roll: None,
            yaw: None,
        },
        frame,
    })
}

#[test]
fn missing_manager_information_cannot_grant_control() {
    let caps = capabilities(None).unwrap();
    assert_eq!(caps.revision, 7);
    for axis in [
        caps.gimbal.axes.pitch,
        caps.gimbal.axes.roll,
        caps.gimbal.axes.yaw,
    ] {
        assert_eq!(axis.position, Capability::Unknown);
        assert_eq!(axis.rate, Capability::Unknown);
        assert_eq!(axis.feedback.angle, Capability::Unknown);
    }
    assert_eq!(caps.gimbal.recenter, Capability::Unknown);
    assert_eq!(caps.gimbal.stop, StopGuarantee::Unknown);
    assert_eq!(
        submit(caps, pitch_rate(ReferenceFrame::HorizonVehicleYaw)),
        Err(Error::UnknownCapability)
    );
}

#[test]
fn known_zero_axis_flags_establish_unsupported_operations() {
    let caps = capabilities(Some(information(0))).unwrap();
    assert_eq!(caps.gimbal.axes.pitch, AxisCapabilities::UNSUPPORTED);
    assert_eq!(caps.gimbal.axes.yaw, AxisCapabilities::UNSUPPORTED);
    assert_eq!(caps.gimbal.recenter, Capability::Unsupported);
    assert_eq!(
        submit(caps, pitch_rate(ReferenceFrame::Vehicle)),
        Err(Error::Unsupported)
    );
}

#[test]
fn pitch_only_discovery_enforces_axis_frame_and_position_bounds() {
    let caps = capabilities(Some(information(32 | 128))).unwrap();
    assert_eq!(caps.gimbal.axes.yaw, AxisCapabilities::UNSUPPORTED);
    assert_eq!(caps.gimbal.axes.roll, AxisCapabilities::UNSUPPORTED);
    assert!(submit(caps, pitch_rate(ReferenceFrame::HorizonVehicleYaw)).is_ok());
    assert_eq!(
        submit(caps, pitch_rate(ReferenceFrame::Vehicle)),
        Err(Error::UnsupportedFrame)
    );
    let position = PayloadCommand::GimbalPosition(GimbalPosition {
        angles: Axes {
            pitch: Some(Angle::radians(0.6).unwrap()),
            roll: None,
            yaw: None,
        },
        frame: ReferenceFrame::HorizonVehicleYaw,
    });
    assert_eq!(submit(caps, position), Err(Error::OutOfRange));
    assert_eq!(caps.gimbal.axes.pitch.feedback.angle, Capability::Unknown);
}

#[test]
fn follow_and_lock_bits_define_each_executable_axis_frame() {
    let cases = [
        (32 | 64, true, false, false),
        (32 | 128, false, true, true),
        (32 | 64 | 128, true, true, true),
    ];
    for (flags, vehicle, horizon, earth) in cases {
        let caps = capabilities(Some(information(flags))).unwrap();
        let frames = caps.gimbal.axes.pitch.rate.available().unwrap().frames;
        assert_eq!(
            (frames.vehicle, frames.horizon_vehicle_yaw, frames.earth_ned),
            (vehicle, horizon, earth)
        );
        assert!(!frames.mount);
    }
    for (flags, vehicle, horizon, earth) in [
        (256 | 512, true, true, false),
        (256 | 1024, false, false, true),
        (256 | 512 | 1024, true, true, true),
    ] {
        let caps = capabilities(Some(information(flags))).unwrap();
        let frames = caps.gimbal.axes.yaw.rate.available().unwrap().frames;
        assert_eq!(
            (frames.vehicle, frames.horizon_vehicle_yaw, frames.earth_ned),
            (vehicle, horizon, earth)
        );
    }
}

#[test]
fn two_axes_and_neutral_do_not_invent_feedback_or_a_measured_neutral_pose() {
    let caps = capabilities(Some(information(2 | 32 | 128 | 256 | 512))).unwrap();
    assert!(caps.gimbal.axes.pitch.rate.available().is_ok());
    assert!(caps.gimbal.axes.yaw.rate.available().is_ok());
    let neutral = caps.gimbal.recenter.available().unwrap();
    assert_eq!(neutral.implementation, Implementation::Native);
    assert_eq!(neutral.target, None);
    assert_eq!(caps.gimbal.axes.yaw.feedback.rate, Capability::Unknown);
    assert_eq!(caps.gimbal.stop, StopGuarantee::BestEffort);
}

#[test]
fn unimplemented_roll_and_unrecognized_flags_never_grant_execution() {
    let caps = capabilities(Some(information(4 | 8 | 16))).unwrap();
    assert!(matches!(
        caps.gimbal.axes.roll.position,
        Capability::Unavailable {
            reason: UnavailableReason::Policy,
            ..
        }
    ));
    assert_eq!(caps.gimbal.axes.roll.rate, Capability::Unknown);
    let caps = capabilities(Some(information(256 | 2048 | 4096 | 0x8000_0000))).unwrap();
    assert_eq!(
        caps.gimbal.axes.yaw.rate.available(),
        Err(Error::Unavailable)
    );
    assert_eq!(caps.gimbal.axes.pitch, AxisCapabilities::UNSUPPORTED);
    assert_eq!(caps.gimbal.recenter, Capability::Unsupported);
}

#[test]
fn unknown_limits_preserve_rate_without_inventing_geometry() {
    let mut info = information(32 | 128 | 256 | 512 | 2048);
    info.pitch_min = f32::NAN;
    info.yaw_max = f32::NAN;
    let caps = capabilities(Some(info)).unwrap();
    assert_eq!(caps.gimbal.axes.pitch.position, Capability::Unknown);
    assert_eq!(caps.gimbal.axes.yaw.position, Capability::Unknown);
    assert!(caps.gimbal.axes.pitch.rate.available().is_ok());
    assert!(caps.gimbal.axes.yaw.rate.available().is_ok());
}

#[test]
fn invalid_limits_return_typed_errors_without_silent_clamping() {
    for (minimum, maximum) in [(1.0, -1.0), (f32::INFINITY, 1.0), (-1.0, f32::NEG_INFINITY)] {
        let mut info = information(32 | 128);
        info.pitch_min = minimum;
        info.pitch_max = maximum;
        assert_eq!(capabilities(Some(info)), Err(Error::InvalidLimits));
    }
    assert_eq!(
        manager_capabilities(7, None, AngularRate::ZERO, AngularRate::ZERO),
        Err(Error::InvalidLimits)
    );
}

#[test]
fn wider_mechanical_range_is_intersected_with_the_position_encoder_domain() {
    let mut info = information(256 | 512);
    info.yaw_min = -10.0;
    info.yaw_max = 10.0;
    let caps = capabilities(Some(info)).unwrap();
    let range = caps.gimbal.axes.yaw.position.available().unwrap().range;
    assert_eq!(range.minimum.as_radians(), -core::f32::consts::PI);
    assert_eq!(range.maximum.as_radians(), core::f32::consts::PI);
    info.yaw_min = 4.0;
    let caps = capabilities(Some(info)).unwrap();
    assert_eq!(
        caps.gimbal.axes.yaw.position.available(),
        Err(Error::Unavailable)
    );
    assert!(caps.gimbal.axes.yaw.rate.available().is_ok());
}
