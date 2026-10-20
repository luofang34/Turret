use super::*;
use turret_core::{CommandIdentity, Tick};

fn sent_progress() -> CommandProgress {
    CommandProgress {
        identity: CommandIdentity {
            epoch: 11,
            generation: 12,
            id: 13,
            sequence: 14,
        },
        accepted_at: Tick(15),
        prepared_at: Some(Tick(16)),
        sent_at: Some(Tick(17)),
        acknowledged_at: None,
        rejected_at: None,
        acknowledgement_timed_out: false,
        observed_complete_at: None,
        ended: None,
    }
}

#[test]
fn dispatched_without_ack_does_not_imply_acceptance_or_completion() {
    let progress = sent_progress();
    assert!(dispatch_succeeded(&progress));
    assert_eq!(
        to_json(&progress),
        json!({
            "epoch": 11, "generation": 12, "id": 13, "sequence": 14,
            "accepted_at_us": 15, "prepared_at_us": 16, "sent_at_us": 17,
            "acknowledged_at_us": null, "rejected_at_us": null,
            "acknowledgement_timed_out": false, "observed_complete_at_us": null,
            "ended": null, "sent": true, "device_acknowledged": false,
            "observed_complete": false,
        })
    );
}

#[test]
fn each_failure_remains_unsuccessful_even_with_a_completed_write() {
    for reason in [
        EndReason::ExpiredBeforeSend,
        EndReason::ValidityExpired,
        EndReason::CompletionTimeout,
        EndReason::Revoked,
        EndReason::Reconnected,
        EndReason::Superseded,
        EndReason::DeviceRejected,
        EndReason::TransportFailure,
    ] {
        let mut progress = sent_progress();
        progress.ended = Some(reason);
        assert!(!dispatch_succeeded(&progress), "{reason:?}");
    }
    let mut progress = sent_progress();
    progress.sent_at = None;
    assert!(!dispatch_succeeded(&progress));
    let mut progress = sent_progress();
    progress.acknowledgement_timed_out = true;
    assert!(!dispatch_succeeded(&progress));
}

#[test]
fn retained_rejection_is_not_hidden_by_a_later_end_reason() {
    let mut progress = sent_progress();
    progress.rejected_at = Some(Tick(18));
    progress.ended = Some(EndReason::Revoked);
    assert!(!dispatch_succeeded(&progress));
    let data = to_json(&progress);
    assert_eq!(data["rejected_at_us"], 18);
    assert_eq!(data["ended"], "Revoked");
    progress.ended = None;
    assert!(!dispatch_succeeded(&progress));
}

#[test]
fn physical_completion_preserves_independent_ack_evidence() {
    let mut progress = sent_progress();
    progress.observed_complete_at = Some(Tick(19));
    progress.ended = Some(EndReason::ObservedComplete);
    assert!(dispatch_succeeded(&progress));
    let data = to_json(&progress);
    assert_eq!(data["observed_complete_at_us"], 19);
    assert_eq!(data["device_acknowledged"], false);
    assert_eq!(data["observed_complete"], true);
    assert_eq!(data["ended"], "ObservedComplete");
    progress.acknowledged_at = Some(Tick(20));
    assert_eq!(to_json(&progress)["acknowledged_at_us"], 20);
}

#[test]
fn terminal_wire_tokens_are_explicit_and_stable() {
    for (reason, token) in [
        (EndReason::ExpiredBeforeSend, "ExpiredBeforeSend"),
        (EndReason::ValidityExpired, "ValidityExpired"),
        (EndReason::CompletionTimeout, "CompletionTimeout"),
        (EndReason::Revoked, "Revoked"),
        (EndReason::Reconnected, "Reconnected"),
        (EndReason::Superseded, "Superseded"),
        (EndReason::DeviceRejected, "DeviceRejected"),
        (EndReason::ObservedComplete, "ObservedComplete"),
        (EndReason::TransportFailure, "TransportFailure"),
    ] {
        assert_eq!(ended_token(reason), token);
    }
}
