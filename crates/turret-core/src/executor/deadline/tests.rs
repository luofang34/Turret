use crate::executor::tests::{capabilities, command, observation, position, rate, sent, tolerance};
use crate::*;

mod rejection;

#[test]
fn completion_timeout_does_not_disable_execution_validity() {
    let mut executor = Executor::new(capabilities().unwrap(), 5, 7);
    let mut command = command(position());
    command.timing.completion_before = Some(Tick(200));
    command.timing.valid_until = Some(Tick(300));
    assert_eq!(
        sent(&mut executor, command).unwrap().current.effect,
        Effect::None
    );
    assert_eq!(
        executor.poll(Tick(200)).unwrap().unwrap().progress.ended,
        Some(EndReason::CompletionTimeout)
    );
    let expired = executor.poll(Tick(300)).unwrap().unwrap();
    assert_eq!(expired.effect, Effect::CancelAction);
    assert_eq!(expired.progress.ended, Some(EndReason::ValidityExpired));
    assert_eq!(executor.poll(Tick(301)).unwrap(), None);
}

#[test]
fn rejected_continuous_command_requests_stop_and_keeps_expiry() {
    let mut executor = Executor::new(capabilities().unwrap(), 5, 7);
    let mut command = command(rate());
    command.timing.valid_until = Some(Tick(300));
    assert_eq!(
        sent(&mut executor, command).unwrap().current.effect,
        Effect::None
    );
    let rejected = executor
        .acknowledge(command.identity, false, Tick(40))
        .unwrap();
    assert_eq!(rejected.effect, Effect::StopContinuous);
    assert_eq!(
        executor.poll(Tick(300)).unwrap().unwrap().effect,
        Effect::StopContinuous
    );
}

#[test]
fn completion_requires_matching_samples_through_settle_interval() {
    let mut executor = Executor::new(capabilities().unwrap(), 5, 7);
    let command = command(position());
    assert_eq!(
        sent(&mut executor, command).unwrap().current.effect,
        Effect::None
    );
    let mut tolerance = tolerance();
    tolerance.settle_time_us = 40;
    tolerance.maximum_sample_gap_us = 30;
    let mut sample = observation();
    for at in [40, 60] {
        sample.sampled_at = Some(Tick(at));
        sample.received_at = Tick(at);
        assert!(!executor
            .observe_completion(command.identity, &sample, Tick(at), 20, tolerance)
            .unwrap());
    }
    sample.sampled_at = Some(Tick(80));
    sample.received_at = Tick(80);
    assert!(executor
        .observe_completion(command.identity, &sample, Tick(80), 20, tolerance)
        .unwrap());
}

#[test]
fn stale_or_wrong_frame_sample_resets_settling() {
    let mut executor = Executor::new(capabilities().unwrap(), 5, 7);
    let command = command(position());
    assert_eq!(
        sent(&mut executor, command).unwrap().current.effect,
        Effect::None
    );
    let mut tolerance = tolerance();
    tolerance.settle_time_us = 40;
    tolerance.maximum_sample_gap_us = 100;
    let mut sample = observation();
    assert!(!executor
        .observe_completion(command.identity, &sample, Tick(45), 20, tolerance)
        .unwrap());
    assert_eq!(
        executor.observe_completion(command.identity, &sample, Tick(70), 20, tolerance),
        Err(Error::InvalidObservation)
    );
    sample.sampled_at = Some(Tick(80));
    sample.received_at = Tick(80);
    assert!(!executor
        .observe_completion(command.identity, &sample, Tick(80), 20, tolerance)
        .unwrap());
    sample.sampled_at = Some(Tick(100));
    sample.received_at = Tick(100);
    sample.frame = Some(ReferenceFrame::EarthNed);
    assert_eq!(
        executor.observe_completion(command.identity, &sample, Tick(100), 20, tolerance),
        Err(Error::InvalidObservation)
    );
    sample.sampled_at = Some(Tick(120));
    sample.received_at = Tick(120);
    sample.frame = Some(ReferenceFrame::Mount);
    assert!(!executor
        .observe_completion(command.identity, &sample, Tick(120), 20, tolerance)
        .unwrap());
    sample.sampled_at = Some(Tick(160));
    sample.received_at = Tick(160);
    assert!(executor
        .observe_completion(command.identity, &sample, Tick(160), 20, tolerance)
        .unwrap());
}

#[test]
fn observation_gap_restarts_settling() {
    let mut executor = Executor::new(capabilities().unwrap(), 5, 7);
    let command = command(position());
    assert_eq!(
        sent(&mut executor, command).unwrap().current.effect,
        Effect::None
    );
    let mut tolerance = tolerance();
    tolerance.settle_time_us = 40;
    tolerance.maximum_sample_gap_us = 30;
    let mut sample = observation();
    assert!(!executor
        .observe_completion(command.identity, &sample, Tick(45), 20, tolerance)
        .unwrap());
    sample.sampled_at = Some(Tick(100));
    sample.received_at = Tick(100);
    assert!(!executor
        .observe_completion(command.identity, &sample, Tick(100), 20, tolerance)
        .unwrap());
    sample.sampled_at = Some(Tick(120));
    sample.received_at = Tick(120);
    assert!(!executor
        .observe_completion(command.identity, &sample, Tick(120), 20, tolerance)
        .unwrap());
    sample.sampled_at = Some(Tick(140));
    sample.received_at = Tick(140);
    assert!(executor
        .observe_completion(command.identity, &sample, Tick(140), 20, tolerance)
        .unwrap());
}

#[test]
fn capability_change_fences_old_generation_without_claiming_a_reconnect() {
    let mut executor = Executor::new(capabilities().unwrap(), 5, 7);
    let command = command(position());
    assert_eq!(
        sent(&mut executor, command).unwrap().current.effect,
        Effect::None
    );
    let mut caps = capabilities().unwrap();
    caps.revision = 5;
    let invalidation = executor.update_capabilities(caps);
    assert_eq!(invalidation.active.unwrap().effect, Effect::CancelAction);
    assert_eq!(executor.epoch(), 5);
    assert_eq!(executor.generation(), 8);
    assert!(executor
        .progress(command.identity)
        .unwrap()
        .sent_at
        .is_some());
    assert_eq!(
        executor.submit(command, Tick(40)),
        Err(Error::StaleIdentity)
    );
}

#[test]
fn native_recenter_with_unknown_target_cannot_claim_zero_completion() {
    let mut caps = capabilities().unwrap();
    caps.gimbal.recenter = Capability::Available(RecenterCapability {
        implementation: Implementation::Native,
        target: None,
    });
    let mut executor = Executor::new(caps, 5, 7);
    let command = command(PayloadCommand::Recenter);
    assert_eq!(
        sent(&mut executor, command).unwrap().current.effect,
        Effect::None
    );
    assert_eq!(
        executor.observe_completion(command.identity, &observation(), Tick(45), 100, tolerance()),
        Err(Error::NoCompletionTarget)
    );
    assert_eq!(
        executor
            .progress(command.identity)
            .unwrap()
            .observed_complete_at,
        None
    );
}

#[test]
fn completed_observation_preserves_late_ack_and_later_execution_expiry() {
    let mut executor = Executor::new(capabilities().unwrap(), 5, 7);
    let mut command = command(position());
    command.timing.send_before = Tick(40);
    command.timing.ack_before = Some(Tick(80));
    command.timing.valid_until = Some(Tick(100));
    assert_eq!(
        sent(&mut executor, command).unwrap().current.effect,
        Effect::None
    );
    assert!(executor
        .observe_completion(command.identity, &observation(), Tick(45), 100, tolerance())
        .unwrap());
    assert!(
        executor
            .poll(Tick(80))
            .unwrap()
            .unwrap()
            .progress
            .acknowledgement_timed_out
    );
    assert_eq!(
        executor
            .acknowledge(command.identity, true, Tick(90))
            .unwrap()
            .progress
            .observed_complete_at,
        Some(Tick(45))
    );
    let expired = executor.poll(Tick(100)).unwrap().unwrap();
    assert_eq!(expired.progress.observed_complete_at, Some(Tick(45)));
    assert_eq!(expired.progress.acknowledged_at, Some(Tick(90)));
}
