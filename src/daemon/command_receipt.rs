//! Host dispatch classification and stable IPC command evidence.

use serde_json::{json, Value};
use turret_core::{CommandProgress, EndReason};

/// A completed host dispatch with no known failure, not device acceptance or completion.
pub(crate) fn dispatch_succeeded(progress: &CommandProgress) -> bool {
    progress.sent_at.is_some()
        && progress.rejected_at.is_none()
        && !progress.acknowledgement_timed_out
        && matches!(progress.ended, None | Some(EndReason::ObservedComplete))
}

/// Preserve the same identity and lifecycle evidence in execution and status responses.
pub(crate) fn to_json(progress: &CommandProgress) -> Value {
    json!({
        "epoch": progress.identity.epoch,
        "generation": progress.identity.generation,
        "id": progress.identity.id,
        "sequence": progress.identity.sequence,
        "accepted_at_us": progress.accepted_at.0,
        "prepared_at_us": progress.prepared_at.map(|time| time.0),
        "sent_at_us": progress.sent_at.map(|time| time.0),
        "acknowledged_at_us": progress.acknowledged_at.map(|time| time.0),
        "rejected_at_us": progress.rejected_at.map(|time| time.0),
        "acknowledgement_timed_out": progress.acknowledgement_timed_out,
        "observed_complete_at_us": progress.observed_complete_at.map(|time| time.0),
        "ended": progress.ended.map(ended_token),
        "sent": progress.sent_at.is_some(),
        "device_acknowledged": progress.acknowledged_at.is_some(),
        "observed_complete": progress.observed_complete_at.is_some(),
    })
}

fn ended_token(reason: EndReason) -> &'static str {
    match reason {
        EndReason::ExpiredBeforeSend => "ExpiredBeforeSend",
        EndReason::ValidityExpired => "ValidityExpired",
        EndReason::CompletionTimeout => "CompletionTimeout",
        EndReason::Revoked => "Revoked",
        EndReason::Reconnected => "Reconnected",
        EndReason::Superseded => "Superseded",
        EndReason::DeviceRejected => "DeviceRejected",
        EndReason::ObservedComplete => "ObservedComplete",
        EndReason::TransportFailure => "TransportFailure",
    }
}

#[cfg(test)]
mod tests;
