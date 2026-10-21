use super::*;

#[test]
fn operation_support_implementation_and_availability_are_distinct() {
    let mut caps = capabilities().unwrap();
    caps.gimbal.axes.pitch.position = Capability::Unknown;
    let mut executor = Executor::new(caps, 5, 7);
    assert_eq!(
        executor.submit(command(position()), Tick(0)),
        Err(Error::UnknownCapability)
    );
    caps.gimbal.axes.pitch.position = Capability::Unsupported;
    let mut executor = Executor::new(caps, 5, 7);
    assert_eq!(
        executor.submit(command(position()), Tick(0)),
        Err(Error::Unsupported)
    );
    let known = capabilities()
        .unwrap()
        .gimbal
        .axes
        .pitch
        .position
        .available()
        .copied()
        .unwrap();
    caps.gimbal.axes.pitch.position = Capability::Unavailable {
        capability: known,
        reason: UnavailableReason::Transport,
    };
    let mut executor = Executor::new(caps, 5, 7);
    assert_eq!(
        executor.submit(command(position()), Tick(0)),
        Err(Error::Unavailable)
    );
}

#[test]
fn unimplemented_operations_never_enter_the_write_slot() {
    let mut caps = capabilities().unwrap();
    let contract = caps
        .gimbal
        .axes
        .pitch
        .position
        .available()
        .copied()
        .unwrap();
    caps.gimbal.axes.pitch.mechanics = AxisMechanics {
        movement: HardwareSupport::Supported,
        stabilization: HardwareSupport::Supported,
    };
    for known in [None, Some(contract)] {
        caps.gimbal.axes.pitch.position = Capability::Unimplemented { capability: known };
        let mut executor = Executor::new(caps, 5, 7);
        assert_eq!(
            executor.submit(command(position()), Tick(0)),
            Err(Error::NotImplemented)
        );
        assert_eq!(executor.prepare_send(Tick(1)), Ok(None));
    }
}

#[test]
fn physical_axis_presence_does_not_grant_unsupported_motion() {
    let mut caps = capabilities().unwrap();
    caps.gimbal.axes.pitch.mechanics = AxisMechanics {
        movement: HardwareSupport::Supported,
        stabilization: HardwareSupport::Supported,
    };
    caps.gimbal.axes.pitch.position = Capability::Unsupported;
    let mut executor = Executor::new(caps, 5, 7);
    assert_eq!(
        executor.submit(command(position()), Tick(0)),
        Err(Error::Unsupported)
    );
    assert_eq!(executor.prepare_send(Tick(1)), Ok(None));
}
