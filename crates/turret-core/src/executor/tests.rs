use super::*;
use crate::*;

#[cfg(test)]
pub(super) fn capabilities() -> Result<PayloadCapabilities, Error> {
    let frames = Frames {
        mount: true,
        vehicle: true,
        earth_ned: false,
        horizon_vehicle_yaw: false,
    };
    let pitch = AxisCapabilities {
        position: Capability::Available(PositionCapability {
            implementation: Implementation::Native,
            range: AngleRange {
                minimum: Angle::radians(-1.5)?,
                maximum: Angle::radians(0.5)?,
            },
            frames,
            maximum_rate: None,
        }),
        rate: Capability::Available(RateCapability {
            implementation: Implementation::Synthesized,
            maximum_rate: AngularRate::radians_per_second(1.0)?,
            frames,
        }),
        feedback: FeedbackCapabilities {
            angle: Capability::Available(()),
            rate: Capability::Unsupported,
        },
    };
    let target = GimbalPosition {
        angles: Axes {
            pitch: Some(Angle::ZERO),
            roll: None,
            yaw: None,
        },
        frame: ReferenceFrame::Mount,
    };
    Ok(PayloadCapabilities {
        revision: 4,
        gimbal: GimbalCapabilities {
            axes: Axes {
                pitch,
                roll: AxisCapabilities::UNSUPPORTED,
                yaw: AxisCapabilities::UNSUPPORTED,
            },
            recenter: Capability::Available(RecenterCapability {
                implementation: Implementation::Synthesized,
                target: Some(target),
            }),
            stop: StopGuarantee::BestEffort,
        },
        zoom: Capability::Unsupported,
    })
}

#[cfg(test)]
pub(super) fn position() -> PayloadCommand {
    PayloadCommand::GimbalPosition(GimbalPosition {
        angles: Axes {
            pitch: Some(Angle::ZERO),
            roll: None,
            yaw: None,
        },
        frame: ReferenceFrame::Mount,
    })
}

#[cfg(test)]
pub(super) fn rate() -> PayloadCommand {
    PayloadCommand::GimbalRate(GimbalRate {
        rates: Axes {
            pitch: Some(AngularRate::ZERO),
            roll: None,
            yaw: None,
        },
        frame: ReferenceFrame::Mount,
    })
}

#[cfg(test)]
pub(super) fn command(payload: PayloadCommand) -> Command {
    Command {
        identity: CommandIdentity {
            epoch: 5,
            generation: 7,
            id: 11,
            sequence: 1,
        },
        capability_revision: 4,
        payload,
        timing: CommandTiming {
            send_before: Tick(100),
            valid_until: None,
            ack_before: None,
            completion_before: None,
        },
    }
}

#[cfg(test)]
pub(super) fn sent(executor: &mut Executor, command: Command) -> Result<SendOutcome, Error> {
    executor.submit(command, Tick(10))?;
    executor.prepare_send(Tick(20))?;
    executor.record_sent(command.identity, Tick(30))
}

#[cfg(test)]
pub(super) fn observation() -> Observation<PayloadState> {
    Observation {
        value: PayloadState::Gimbal(Axes {
            pitch: Some(Angle::ZERO),
            roll: None,
            yaw: None,
        }),
        source: ObservationSource::DeviceMeasured,
        frame: Some(ReferenceFrame::Mount),
        sampled_at: Some(Tick(40)),
        received_at: Tick(45),
        epoch: 5,
    }
}

#[cfg(test)]
pub(super) fn tolerance() -> CompletionTolerance {
    CompletionTolerance {
        angle: Angle::ZERO,
        zoom: None,
        settle_time_us: 0,
        maximum_sample_gap_us: 100,
    }
}

#[test]
fn expired_submission_never_enters_pending_slot() {
    let mut executor = Executor::new(capabilities().unwrap(), 5, 7);
    assert_eq!(
        executor.submit(command(position()), Tick(100)),
        Err(Error::Expired)
    );
    assert_eq!(executor.prepare_send(Tick(101)), Ok(None));
}

#[test]
fn queued_command_is_rechecked_at_actual_write_boundary() {
    let mut executor = Executor::new(capabilities().unwrap(), 5, 7);
    let command = command(position());
    executor.submit(command, Tick(0)).unwrap();
    assert!(executor.prepare_send(Tick(10)).unwrap().is_some());
    assert_eq!(executor.prepare_send(Tick(100)), Err(Error::Expired));
    let progress = executor.progress(command.identity).unwrap();
    assert_eq!(progress.ended, Some(EndReason::ExpiredBeforeSend));
    assert_eq!(progress.sent_at, None);
    assert_eq!(
        executor.record_sent(command.identity, Tick(101)),
        Err(Error::NotFound)
    );
}

#[test]
fn sent_and_acknowledged_do_not_mean_observed_complete() {
    let mut executor = Executor::new(capabilities().unwrap(), 5, 7);
    let command = command(position());
    let result = sent(&mut executor, command).unwrap();
    assert_eq!(result.current.effect, Effect::None);
    assert_eq!(result.current.progress.sent_at, Some(Tick(30)));
    assert_eq!(result.current.progress.observed_complete_at, None);
    let acknowledged = executor
        .acknowledge(command.identity, true, Tick(40))
        .unwrap();
    assert_eq!(acknowledged.progress.acknowledged_at, Some(Tick(40)));
    assert_eq!(acknowledged.progress.ended, None);
    assert!(executor
        .observe_completion(command.identity, &observation(), Tick(50), 20, tolerance())
        .unwrap());
    assert_eq!(
        executor.progress(command.identity).unwrap().ended,
        Some(EndReason::ObservedComplete)
    );
}

#[test]
fn estimated_or_precommand_or_stale_samples_cannot_complete() {
    let mut executor = Executor::new(capabilities().unwrap(), 5, 7);
    let command = command(position());
    assert_eq!(
        sent(&mut executor, command).unwrap().current.effect,
        Effect::None
    );
    let mut sample = observation();
    sample.source = ObservationSource::Estimated;
    assert_eq!(
        executor.observe_completion(command.identity, &sample, Tick(50), 30, tolerance()),
        Err(Error::InvalidObservation)
    );
    sample.source = ObservationSource::DeviceMeasured;
    sample.sampled_at = Some(Tick(20));
    assert_eq!(
        executor.observe_completion(command.identity, &sample, Tick(50), 30, tolerance()),
        Err(Error::InvalidObservation)
    );
    sample.sampled_at = Some(Tick(40));
    assert_eq!(
        executor.observe_completion(command.identity, &sample, Tick(100), 20, tolerance()),
        Err(Error::InvalidObservation)
    );
    sample.sampled_at = None;
    assert_eq!(
        executor.observe_completion(command.identity, &sample, Tick(100), 20, tolerance()),
        Err(Error::InvalidObservation)
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
fn partial_measurement_does_not_fill_unknown_axes_with_zero() {
    let mut executor = Executor::new(capabilities().unwrap(), 5, 7);
    let command = command(position());
    assert_eq!(
        sent(&mut executor, command).unwrap().current.effect,
        Effect::None
    );
    let mut sample = observation();
    sample.value = PayloadState::Gimbal(Axes {
        pitch: None,
        roll: Some(Angle::ZERO),
        yaw: None,
    });
    assert_eq!(
        executor.observe_completion(command.identity, &sample, Tick(50), 20, tolerance()),
        Err(Error::InvalidObservation)
    );
}

#[test]
fn continuous_expiry_requests_stop_at_exclusive_deadline() {
    let mut executor = Executor::new(capabilities().unwrap(), 5, 7);
    let mut command = command(rate());
    command.timing.valid_until = Some(Tick(200));
    assert_eq!(
        sent(&mut executor, command).unwrap().current.effect,
        Effect::None
    );
    assert_eq!(executor.poll(Tick(199)).unwrap(), None);
    let event = executor.poll(Tick(200)).unwrap().unwrap();
    assert_eq!(event.effect, Effect::StopContinuous);
    assert_eq!(event.progress.ended, Some(EndReason::ValidityExpired));
    assert_eq!(executor.poll(Tick(201)).unwrap(), None);
}

#[test]
fn later_pending_command_does_not_hide_active_stream_expiry() {
    let mut executor = Executor::new(capabilities().unwrap(), 5, 7);
    let mut active = command(rate());
    active.timing.valid_until = Some(Tick(200));
    assert_eq!(
        sent(&mut executor, active).unwrap().current.effect,
        Effect::None
    );
    let mut pending = command(position());
    pending.identity.sequence = 2;
    pending.identity.id = 12;
    pending.timing.send_before = Tick(300);
    executor.submit(pending, Tick(50)).unwrap();
    let event = executor.poll(Tick(200)).unwrap().unwrap();
    assert_eq!(event.progress.identity, active.identity);
    assert_eq!(event.effect, Effect::StopContinuous);
    assert_eq!(executor.prepare_send(Tick(210)).unwrap(), Some(pending));
}

#[test]
fn sent_position_survives_its_send_deadline() {
    let mut executor = Executor::new(capabilities().unwrap(), 5, 7);
    let command = command(position());
    assert_eq!(
        sent(&mut executor, command).unwrap().current.effect,
        Effect::None
    );
    assert_eq!(executor.poll(Tick(500)).unwrap(), None);
    assert_eq!(executor.progress(command.identity).unwrap().ended, None);
}

#[test]
fn ack_timeout_and_completion_timeout_have_different_evidence() {
    let mut executor = Executor::new(capabilities().unwrap(), 5, 7);
    let mut command = command(position());
    command.timing.ack_before = Some(Tick(200));
    command.timing.completion_before = Some(Tick(300));
    assert_eq!(
        sent(&mut executor, command).unwrap().current.effect,
        Effect::None
    );
    let ack = executor.poll(Tick(200)).unwrap().unwrap();
    assert!(ack.progress.acknowledgement_timed_out);
    assert_eq!(ack.progress.ended, None);
    assert_eq!(ack.effect, Effect::None);
    let completion = executor.poll(Tick(300)).unwrap().unwrap();
    assert_eq!(
        completion.progress.ended,
        Some(EndReason::CompletionTimeout)
    );
    assert_eq!(completion.effect, Effect::None);
    assert_eq!(
        executor.revoke().active.unwrap().effect,
        Effect::CancelAction
    );
}

#[test]
fn revoke_clears_pending_and_fences_late_ack_and_input() {
    let mut executor = Executor::new(capabilities().unwrap(), 5, 7);
    let active = command(position());
    assert_eq!(
        sent(&mut executor, active).unwrap().current.effect,
        Effect::None
    );
    let mut pending = active;
    pending.identity.sequence = 2;
    pending.identity.id = 12;
    executor.submit(pending, Tick(40)).unwrap();
    let invalidated = executor.revoke();
    assert_eq!(invalidated.pending.unwrap().identity, pending.identity);
    assert_eq!(invalidated.active.unwrap().effect, Effect::CancelAction);
    assert_eq!(executor.prepare_send(Tick(50)), Ok(None));
    assert_eq!(
        executor.acknowledge(active.identity, true, Tick(60)),
        Err(Error::StaleIdentity)
    );
    assert_eq!(
        executor.submit(pending, Tick(70)),
        Err(Error::StaleIdentity)
    );
}

#[test]
fn reconnect_fences_old_connection_and_wraps_counters() {
    let caps = capabilities().unwrap();
    let mut executor = Executor::new(caps, u64::MAX, u64::MAX);
    let invalidation = executor.reconnect(caps);
    assert_eq!(invalidation.active, None);
    assert_eq!(executor.epoch(), 0);
    assert_eq!(executor.generation(), 0);
    let mut old = command(position());
    old.identity.epoch = u64::MAX;
    old.identity.generation = u64::MAX;
    assert_eq!(executor.submit(old, Tick(0)), Err(Error::StaleIdentity));
}

#[test]
fn sequence_wrap_is_accepted_but_replay_is_not() {
    let mut executor = Executor::new(capabilities().unwrap(), 5, 7);
    let mut command = command(position());
    command.identity.sequence = u64::MAX;
    executor.submit(command, Tick(0)).unwrap();
    command.identity.sequence = 0;
    assert!(executor.submit(command, Tick(1)).unwrap().is_some());
    assert_eq!(executor.submit(command, Tick(2)), Err(Error::StaleSequence));
    command.identity.sequence = u64::MAX;
    assert_eq!(executor.submit(command, Tick(3)), Err(Error::StaleSequence));
}

#[test]
fn a_late_full_write_is_recorded_and_requests_stop() {
    let mut executor = Executor::new(capabilities().unwrap(), 5, 7);
    let mut command = command(rate());
    command.timing.valid_until = Some(Tick(150));
    executor.submit(command, Tick(0)).unwrap();
    executor.prepare_send(Tick(20)).unwrap();
    let written = executor.record_sent(command.identity, Tick(200)).unwrap();
    assert_eq!(written.current.progress.sent_at, Some(Tick(200)));
    assert_eq!(written.current.effect, Effect::StopContinuous);
    assert_eq!(
        written.current.progress.ended,
        Some(EndReason::ValidityExpired)
    );
}

#[test]
fn failed_write_does_not_claim_sent_and_invalidates_generation() {
    let mut executor = Executor::new(capabilities().unwrap(), 5, 7);
    let mut command = command(rate());
    command.timing.valid_until = Some(Tick(200));
    executor.submit(command, Tick(0)).unwrap();
    executor.prepare_send(Tick(20)).unwrap();
    let failure = executor
        .record_write_failure(command.identity, Tick(30))
        .unwrap();
    assert_eq!(failure.attempted.progress.sent_at, None);
    assert_eq!(failure.attempted.effect, Effect::StopContinuous);
    assert_eq!(executor.generation(), 8);
    assert_eq!(
        executor.record_sent(command.identity, Tick(40)),
        Err(Error::StaleIdentity)
    );
}

#[test]
fn unknown_unsupported_and_unavailable_are_distinct() {
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
fn unsupported_axis_cannot_be_silently_converted_to_vehicle_motion() {
    let mut executor = Executor::new(capabilities().unwrap(), 5, 7);
    let yaw = PayloadCommand::GimbalPosition(GimbalPosition {
        angles: Axes {
            pitch: None,
            roll: None,
            yaw: Some(Angle::ZERO),
        },
        frame: ReferenceFrame::Vehicle,
    });
    assert_eq!(
        executor.submit(command(yaw), Tick(0)),
        Err(Error::Unsupported)
    );
    assert_eq!(executor.prepare_send(Tick(10)), Ok(None));
}

#[test]
fn continuous_command_requires_validity_and_rejects_completion_deadline() {
    let mut executor = Executor::new(capabilities().unwrap(), 5, 7);
    let mut command = command(rate());
    assert_eq!(
        executor.submit(command, Tick(0)),
        Err(Error::MissingValidity)
    );
    command.timing.valid_until = Some(Tick(200));
    command.timing.completion_before = Some(Tick(150));
    assert_eq!(executor.submit(command, Tick(0)), Err(Error::InvalidTiming));
}

#[test]
fn wrong_revision_or_backwards_clock_rejects_execution() {
    let mut executor = Executor::new(capabilities().unwrap(), 5, 7);
    let mut command = command(position());
    command.capability_revision = 3;
    assert_eq!(
        executor.submit(command, Tick(10)),
        Err(Error::CapabilityRevision)
    );
    command.capability_revision = 4;
    assert_eq!(
        executor.submit(command, Tick(9)),
        Err(Error::ClockRegression)
    );
}
