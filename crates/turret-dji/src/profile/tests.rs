use super::*;
use turret_core::{Command, CommandIdentity, CommandTiming, Executor, PayloadCommand, Tick};

#[test]
fn the_actual_pitch_contract_rejects_yaw_and_unverified_ranges() {
    let capabilities = pitch_capabilities(-50).unwrap();
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
    assert!(matches!(capabilities.zoom, Capability::Unknown));
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
