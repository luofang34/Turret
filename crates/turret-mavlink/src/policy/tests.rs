use super::*;
use crate::{encode_payload, manager_capabilities, stop_command, Demand, ManagerInformation};

fn policy() -> ManagerPolicy {
    let mut policy = ManagerPolicy::new(
        AngularRate::radians_per_second(0.8).unwrap(),
        AngularRate::radians_per_second(0.8).unwrap(),
    )
    .unwrap();
    policy.request_claim(42, 17).unwrap();
    policy.observe_status(42, 17, Tick(0)).unwrap();
    policy
}

fn information() -> ManagerInformation {
    ManagerInformation {
        cap_flags: 2 | 32 | 128 | 256 | 512,
        roll_min: f32::NAN,
        roll_max: f32::NAN,
        pitch_min: -1.5,
        pitch_max: 0.5,
        yaw_min: -core::f32::consts::PI,
        yaw_max: core::f32::consts::PI,
    }
}

fn capabilities() -> PayloadCapabilities {
    let maximum = AngularRate::radians_per_second(0.8).unwrap();
    manager_capabilities(1, Some(information()), maximum, maximum).unwrap()
}

fn command(payload: PayloadCommand, sequence: u64, deadline: u64) -> Command {
    Command {
        identity: CommandIdentity {
            epoch: 4,
            generation: 6,
            id: sequence,
            sequence,
        },
        capability_revision: 1,
        payload,
        timing: CommandTiming {
            send_before: Tick(deadline),
            valid_until: payload.is_continuous().then_some(Tick(deadline + 200)),
            ack_before: None,
            completion_before: None,
        },
    }
}

#[test]
fn primary_control_requires_status_not_a_sent_claim() {
    let mut policy = policy();
    policy.revoke();
    policy.request_claim(42, 17).unwrap();
    assert!(policy.status_needed(Tick(0)).unwrap());
    assert!(!policy.claim_due(Tick(0)).unwrap());
    assert!(!policy.has_authority(Tick(0)).unwrap());
    assert_eq!(
        policy.rate_intent(1.0, 0.0, &capabilities(), Tick(0)),
        Err(Error::Unavailable)
    );
    policy.observe_status(0, 0, Tick(10)).unwrap();
    assert!(policy.claim_due(Tick(20)).unwrap());
    policy.record_claim_sent(Tick(30)).unwrap();
    assert!(!policy.has_authority(Tick(40)).unwrap());
    policy.observe_status(0, 0, Tick(1_000_000)).unwrap();
    assert!(!policy.claim_due(Tick(1_000_001)).unwrap());
    policy.request_claim(42, 17).unwrap();
    assert!(!policy.claim_due(Tick(1_000_001)).unwrap());
    policy.observe_status(42, 17, Tick(1_000_002)).unwrap();
    assert!(policy.has_authority(Tick(1_000_003)).unwrap());
    policy.observe_status(9, 3, Tick(1_000_004)).unwrap();
    assert!(!policy.has_authority(Tick(1_000_005)).unwrap());
    assert!(!policy.claim_due(Tick(1_000_005)).unwrap());
    policy.revoke();
    assert!(!policy.has_authority(Tick(1_000_006)).unwrap());
}

#[test]
fn recenter_suppression_changes_only_on_sent_motion() {
    let mut policy = policy();
    let recenter = policy.recenter_intent(Tick(1)).unwrap();
    assert!(policy
        .rate_intent(0.0, 0.0, &capabilities(), Tick(1))
        .unwrap()
        .is_some());
    policy.record_sent(recenter, Tick(10)).unwrap();
    assert!(policy
        .rate_intent(0.0, 0.0, &capabilities(), Tick(20))
        .unwrap()
        .is_none());
    let demand = policy
        .rate_intent(-1.0, 1.0, &capabilities(), Tick(30))
        .unwrap()
        .unwrap();
    assert!(policy
        .rate_intent(0.0, 0.0, &capabilities(), Tick(40))
        .unwrap()
        .is_none());
    policy.record_sent(demand, Tick(50)).unwrap();
    assert!(policy
        .rate_intent(0.0, 0.0, &capabilities(), Tick(60))
        .unwrap()
        .is_some());
    assert!(!policy.stale_stop_due(Tick(300_049)).unwrap());
    assert!(policy.stale_stop_due(Tick(300_050)).unwrap());
    policy
        .record_sent(stop_command(demand).unwrap(), Tick(300_060))
        .unwrap();
    assert!(!policy.stale_stop_due(Tick(900_000)).unwrap());
}

#[test]
fn final_writer_rejects_expired_queue_and_dispatches_validity_stop() {
    let mut policy = policy();
    let caps = manager_capabilities(
        1,
        Some(information()),
        policy.pitch_maximum,
        policy.yaw_maximum,
    )
    .unwrap();
    let mut executor = Executor::new(caps, 4, 6);
    let payload = policy
        .rate_intent(-1.0, 1.0, &capabilities(), Tick(0))
        .unwrap()
        .unwrap();
    executor.submit(command(payload, 1, 10), Tick(0)).unwrap();
    assert_eq!(executor.prepare_send(Tick(10)), Err(Error::Expired));
    let next = command(payload, 2, 100);
    executor.submit(next, Tick(20)).unwrap();
    let ready = executor.prepare_send(Tick(30)).unwrap().unwrap();
    assert!(matches!(
        encode_payload(ready.payload, 0, 0).unwrap(),
        Demand::AttitudeRate { .. }
    ));
    let sent = executor.record_sent(next.identity, Tick(40)).unwrap();
    assert_eq!(sent.current.effect, Effect::None);
    policy.record_sent(payload, Tick(40)).unwrap();
    assert_eq!(sent.current.progress.acknowledged_at, None);
    assert_eq!(sent.current.progress.observed_complete_at, None);
    let expiry = executor.poll(Tick(300)).unwrap().unwrap();
    assert_eq!(expiry.effect, Effect::StopContinuous);
    let Demand::AttitudeRate {
        pitch_rps, yaw_rps, ..
    } = encode_payload(stop_command(payload).unwrap(), 0, 0).unwrap()
    else {
        panic!("expected rate stop");
    };
    assert_eq!((pitch_rps, yaw_rps), (0.0, 0.0));
}

#[test]
fn revoked_or_reconnected_writes_cannot_reenter_manager_lane() {
    let policy = policy();
    let caps = manager_capabilities(
        1,
        Some(information()),
        policy.pitch_maximum,
        policy.yaw_maximum,
    )
    .unwrap();
    let mut executor = Executor::new(caps, 4, 6);
    let payload = policy
        .rate_intent(0.0, 1.0, &capabilities(), Tick(0))
        .unwrap()
        .unwrap();
    let pending = command(payload, 1, 100);
    executor.submit(pending, Tick(0)).unwrap();
    let invalidated = executor.revoke();
    assert!(invalidated.pending.is_some());
    assert_eq!(executor.prepare_send(Tick(1)).unwrap(), None);
    assert_eq!(executor.submit(pending, Tick(2)), Err(Error::StaleIdentity));
    let invalidated = executor.reconnect(caps);
    assert_eq!(invalidated.active, None);
    assert_eq!(executor.submit(pending, Tick(3)), Err(Error::StaleIdentity));
}

#[test]
fn idle_never_ends_recenter_from_elapsed_time_alone() {
    let mut policy = policy();
    policy
        .record_sent(PayloadCommand::Recenter, Tick(10))
        .unwrap();
    policy.observe_status(42, 17, Tick(10_000_000)).unwrap();
    assert_eq!(
        policy.rate_intent(0.0, 0.0, &capabilities(), Tick(10_000_001)),
        Ok(None)
    );
    assert_eq!(
        policy.recenter_intent(Tick(11_000_000)),
        Err(Error::Unavailable)
    );
}

#[test]
fn single_axis_policy_omits_zero_unsupported_axis_but_rejects_nonzero_input() {
    let policy = policy();
    let mut info = information();
    info.cap_flags = 32 | 128;
    let caps =
        manager_capabilities(1, Some(info), policy.pitch_maximum, policy.yaw_maximum).unwrap();
    let payload = policy
        .rate_intent(-1.0, 0.0, &caps, Tick(1))
        .unwrap()
        .unwrap();
    let PayloadCommand::GimbalRate(rate) = payload else {
        panic!("expected rate");
    };
    assert_eq!(rate.rates.pitch.unwrap().as_radians_per_second(), -0.8);
    assert_eq!(rate.rates.yaw, None);
    assert_eq!(
        policy.rate_intent(0.0, 1.0, &caps, Tick(1)),
        Err(Error::Unsupported)
    );
    let stopped = stop_command(payload).unwrap();
    let Demand::AttitudeRate {
        pitch_rps, yaw_rps, ..
    } = encode_payload(stopped, 3, 0).unwrap()
    else {
        panic!("expected rate packet");
    };
    assert_eq!(pitch_rps, 0.0);
    assert!(yaw_rps.is_nan());
}

#[test]
fn unknown_or_wrong_frame_support_cannot_become_stick_control() {
    let policy = policy();
    let caps = manager_capabilities(1, None, policy.pitch_maximum, policy.yaw_maximum).unwrap();
    assert_eq!(policy.rate_intent(0.0, 0.0, &caps, Tick(1)), Ok(None));
    assert_eq!(
        policy.rate_intent(1.0, 0.0, &caps, Tick(1)),
        Err(Error::UnknownCapability)
    );
    let mut info = information();
    info.cap_flags = 32 | 64;
    let caps =
        manager_capabilities(1, Some(info), policy.pitch_maximum, policy.yaw_maximum).unwrap();
    assert_eq!(policy.rate_intent(0.0, 0.0, &caps, Tick(1)), Ok(None));
    assert_eq!(
        policy.rate_intent(1.0, 0.0, &caps, Tick(1)),
        Err(Error::UnsupportedFrame)
    );
}

#[test]
fn stick_scaling_cannot_exceed_current_capability_limit_or_accept_invalid_input() {
    let policy = policy();
    let maximum = AngularRate::radians_per_second(0.2).unwrap();
    let caps = manager_capabilities(1, Some(information()), maximum, maximum).unwrap();
    assert_eq!(
        policy.rate_intent(1.0, 0.0, &caps, Tick(1)),
        Err(Error::OutOfRange)
    );
    for input in [f32::NAN, f32::INFINITY, 1.1] {
        assert_eq!(
            policy.rate_intent(input, 0.0, &caps, Tick(1)),
            Err(Error::InvalidValue)
        );
    }
}
