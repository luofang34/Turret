use super::*;

#[test]
fn manager_profile_never_invents_device_geometry_or_feedback() {
    let caps = manager_capabilities(
        4,
        AngularRate::radians_per_second(0.8).unwrap(),
        AngularRate::radians_per_second(0.8).unwrap(),
    )
    .unwrap();
    assert_eq!(caps.gimbal.axes.pitch.position, Capability::Unknown);
    assert_eq!(caps.gimbal.axes.pitch.feedback.angle, Capability::Unknown);
    assert_eq!(caps.gimbal.axes.roll.rate, Capability::Unsupported);
    let rate = caps.gimbal.axes.yaw.rate.available().unwrap();
    assert!(rate.frames.contains(ReferenceFrame::HorizonVehicleYaw));
    assert!(!rate.frames.contains(ReferenceFrame::Vehicle));
    assert_eq!(caps.gimbal.recenter.available().unwrap().target, None);
    assert_eq!(caps.gimbal.stop, StopGuarantee::BestEffort);
}

#[test]
fn basic_zoom_does_not_establish_physical_units_or_continuous_support() {
    let caps = camera_capabilities(1, true).unwrap();
    let zoom = caps.zoom.available().unwrap();
    assert!(zoom.absolute.travel_fraction.available().is_ok());
    assert_eq!(zoom.absolute.ratio, Capability::Unknown);
    assert_eq!(zoom.absolute.focal_length_mm, Capability::Unknown);
    assert_eq!(zoom.continuous, Capability::Unknown);
    assert_eq!(zoom.feedback.travel_fraction, Capability::Unknown);
    assert_eq!(
        camera_capabilities(1, false).unwrap().zoom,
        Capability::Unknown
    );
}

#[test]
fn core_executor_rejects_unverified_zoom_operation_before_encoding() {
    let mut executor = Executor::new(camera_capabilities(1, true).unwrap(), 4, 6);
    let identity = CommandIdentity {
        epoch: 4,
        generation: 6,
        id: 1,
        sequence: 1,
    };
    let command = Command {
        identity,
        capability_revision: 1,
        payload: PayloadCommand::Zoom(ZoomCommand::Absolute(
            ZoomAbsolute::new(ZoomUnit::Ratio, 2.0).unwrap(),
        )),
        timing: CommandTiming {
            send_before: Tick(10),
            valid_until: None,
            ack_before: None,
            completion_before: None,
        },
    };
    assert_eq!(
        executor.submit(command, Tick(0)),
        Err(Error::UnknownCapability)
    );
    assert_eq!(executor.prepare_send(Tick(1)).unwrap(), None);
}
