//! Deadline events retain actuator expiry independently of result waiting.

use super::Record;
use crate::{Effect, EndReason, LifecycleEvent, Tick};

pub(super) fn active_deadline(record: &mut Record, now: Tick) -> Option<LifecycleEvent> {
    let timing = record.command.timing;
    if timing.valid_until.is_some_and(|deadline| now >= deadline)
        && record.progress.ended != Some(EndReason::ValidityExpired)
    {
        return Some(record.finish(EndReason::ValidityExpired));
    }
    if record.progress.acknowledged_at.is_none()
        && !record.progress.acknowledgement_timed_out
        && timing.ack_before.is_some_and(|deadline| now >= deadline)
    {
        record.progress.acknowledgement_timed_out = true;
        return Some(LifecycleEvent {
            progress: record.progress,
            effect: Effect::None,
        });
    }
    if record.progress.ended.is_some() {
        return None;
    }
    if timing
        .completion_before
        .is_some_and(|deadline| now >= deadline)
    {
        record.progress.ended = Some(EndReason::CompletionTimeout);
        return Some(LifecycleEvent {
            progress: record.progress,
            effect: Effect::None,
        });
    }
    None
}

#[cfg(test)]
mod tests;
