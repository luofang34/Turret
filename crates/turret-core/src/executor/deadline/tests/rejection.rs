use super::*;

#[test]
fn rejected_command_keeps_evidence_through_expiry_and_revocation() {
    let mut executor = Executor::new(capabilities().unwrap(), 5, 7);
    let mut command = command(rate());
    command.timing.ack_before = Some(Tick(200));
    command.timing.valid_until = Some(Tick(300));
    let sent = sent(&mut executor, command).unwrap();
    assert_eq!(sent.current.progress.rejected_at, None);
    let rejected = executor
        .acknowledge(command.identity, false, Tick(150))
        .unwrap();
    assert_eq!(rejected.progress.rejected_at, Some(Tick(150)));
    assert_eq!(rejected.effect, Effect::StopContinuous);
    let expired = executor.poll(Tick(300)).unwrap().unwrap();
    assert_eq!(expired.progress.ended, Some(EndReason::ValidityExpired));
    assert_eq!(expired.progress.rejected_at, Some(Tick(150)));
    assert_eq!(expired.effect, Effect::StopContinuous);
    let revoked = executor.revoke().active.unwrap();
    assert_eq!(revoked.progress.ended, Some(EndReason::Revoked));
    assert_eq!(revoked.progress.rejected_at, Some(Tick(150)));
    assert_eq!(revoked.effect, Effect::StopContinuous);
    assert_eq!(
        executor.progress(command.identity).unwrap().rejected_at,
        Some(Tick(150))
    );
}

#[test]
fn timely_negative_ack_is_a_response_and_does_not_time_out() {
    let mut executor = Executor::new(capabilities().unwrap(), 5, 7);
    let mut command = command(position());
    command.timing.ack_before = Some(Tick(200));
    let _sent = sent(&mut executor, command).unwrap();
    let rejected = executor
        .acknowledge(command.identity, false, Tick(150))
        .unwrap();
    assert_eq!(rejected.effect, Effect::CancelAction);
    assert_eq!(executor.poll(Tick(200)).unwrap(), None);
    let progress = executor.progress(command.identity).unwrap();
    assert!(!progress.acknowledgement_timed_out);
    assert_eq!(progress.acknowledged_at, None);
    assert_eq!(progress.observed_complete_at, None);
    assert_eq!(progress.ended, Some(EndReason::DeviceRejected));
}

#[test]
fn late_negative_ack_preserves_timeout_with_or_without_an_intermediate_poll() {
    for poll_first in [false, true] {
        let mut executor = Executor::new(capabilities().unwrap(), 5, 7);
        let mut command = command(position());
        command.timing.ack_before = Some(Tick(200));
        let _sent = sent(&mut executor, command).unwrap();
        if poll_first {
            let timed_out = executor.poll(Tick(200)).unwrap().unwrap();
            assert!(timed_out.progress.acknowledgement_timed_out);
            assert_eq!(timed_out.effect, Effect::None);
        }
        let rejected = executor
            .acknowledge(command.identity, false, Tick(200))
            .unwrap();
        assert!(rejected.progress.acknowledgement_timed_out);
        assert_eq!(rejected.progress.rejected_at, Some(Tick(200)));
        assert_eq!(rejected.progress.ended, Some(EndReason::DeviceRejected));
        assert_eq!(rejected.effect, Effect::CancelAction);
        assert_eq!(executor.poll(Tick(201)).unwrap(), None);
    }
}

#[test]
fn positive_ack_after_rejection_cannot_convert_it_to_success() {
    let mut executor = Executor::new(capabilities().unwrap(), 5, 7);
    let mut command = command(position());
    command.timing.ack_before = Some(Tick(200));
    command.timing.valid_until = Some(Tick(300));
    let _sent = sent(&mut executor, command).unwrap();
    let rejected = executor
        .acknowledge(command.identity, false, Tick(150))
        .unwrap();
    assert_eq!(
        executor.acknowledge(command.identity, true, Tick(160)),
        Err(Error::Ended)
    );
    assert_eq!(executor.progress(command.identity), Some(rejected.progress));
    let expired = executor.poll(Tick(300)).unwrap().unwrap();
    let duplicate = executor
        .acknowledge(command.identity, false, Tick(301))
        .unwrap();
    assert_eq!(duplicate.progress, expired.progress);
    assert_eq!(duplicate.effect, Effect::None);
    assert_eq!(
        executor.acknowledge(command.identity, true, Tick(302)),
        Err(Error::Ended)
    );
    assert_eq!(executor.poll(Tick(303)).unwrap(), None);
}

#[test]
fn negative_ack_after_a_timely_positive_ack_preserves_both_without_timeout() {
    let mut executor = Executor::new(capabilities().unwrap(), 5, 7);
    let mut command = command(position());
    command.timing.ack_before = Some(Tick(200));
    let _sent = sent(&mut executor, command).unwrap();
    let _acknowledged = executor
        .acknowledge(command.identity, true, Tick(150))
        .unwrap();
    let duplicate = executor
        .acknowledge(command.identity, true, Tick(210))
        .unwrap();
    assert_eq!(duplicate.progress.acknowledged_at, Some(Tick(150)));
    assert!(!duplicate.progress.acknowledgement_timed_out);
    let rejected = executor
        .acknowledge(command.identity, false, Tick(220))
        .unwrap();
    assert_eq!(rejected.progress.acknowledged_at, Some(Tick(150)));
    assert_eq!(rejected.progress.rejected_at, Some(Tick(220)));
    assert_eq!(rejected.progress.ended, Some(EndReason::DeviceRejected));
    assert!(!rejected.progress.acknowledgement_timed_out);
    assert_eq!(rejected.effect, Effect::CancelAction);
}
