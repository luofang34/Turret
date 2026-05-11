//! Last-accepted-command bookkeeping + per-source rate-limit
//! timestamps.
//!
//! Two locks: one over the single `last_command` slot (the in-flight
//! command for primary-control / observability), one over the per-source
//! map used by the rate-limit floor. Splitting the lock means the
//! rate-limit check (a hot path on busy GCSs) doesn't block reads of
//! the in-flight command.

use super::source_map::evict_for_insert;
use crate::daemon::models::{ControlSource, GimbalCommand};
use parking_lot::RwLock;
use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, Instant};
use tracing::warn;

/// Owns the last-accepted command and the per-source rate-limit
/// timestamps that gate new commands from the same source.
#[derive(Clone)]
pub struct CommandStore {
    /// The single in-flight command — what the gimbal is currently
    /// "supposed" to do.
    last_command: Arc<RwLock<Option<GimbalCommand>>>,
    /// When each `ControlSource` last had a command accepted. Used by
    /// the rate-limit check; empty map + `min_interval == ZERO` means
    /// "no throttle". Bounded by the shared TTL + hard-cap eviction
    /// in `super::source_map`.
    last_command_at: Arc<RwLock<HashMap<ControlSource, Instant>>>,
    /// Minimum delta between two accepted commands from the same source.
    /// `Duration::ZERO` disables the check.
    min_interval: Duration,
}

impl CommandStore {
    /// Empty store, rate-limit disabled (interval `ZERO`).
    pub fn new() -> Self {
        Self::with_min_interval(Duration::ZERO)
    }

    /// Empty store, per-source rate-limit floor set to `interval`. Two
    /// commands from the same source within this window will see the
    /// second one rejected. `ZERO` disables the check.
    pub fn with_min_interval(interval: Duration) -> Self {
        Self {
            last_command: Arc::new(RwLock::new(None)),
            last_command_at: Arc::new(RwLock::new(HashMap::new())),
            min_interval: interval,
        }
    }

    /// Get the in-flight command, if any.
    pub fn last(&self) -> Option<GimbalCommand> {
        self.last_command.read().clone()
    }

    /// Run a command through arbitration: per-source rate limit first,
    /// then the priority check. Returns `true` if accepted (the command
    /// becomes the new `last_command` and the source's rate-limit
    /// timestamp advances).
    ///
    /// The rate-limit check runs on its own lock so the (more contended)
    /// `last_command` lock isn't held across the time arithmetic.
    pub fn try_accept(&self, command: GimbalCommand) -> bool {
        if !self.check_and_record_rate(&command.source) {
            return false;
        }

        let mut last_cmd = self.last_command.write();

        // Accept iff:
        //   1. No previous command, OR
        //   2. Previous command is stale (older than 5 s — see
        //      `GimbalCommand::is_stale`), OR
        //   3. New command has strictly higher priority, OR
        //   4. Same priority and strictly newer timestamp ("last
        //      writer wins" within a priority class).
        let should_accept = match &*last_cmd {
            None => true,
            Some(prev) => {
                if prev.is_stale() {
                    true
                } else {
                    let new_priority = command.source.priority();
                    let prev_priority = prev.source.priority();
                    new_priority > prev_priority
                        || (new_priority == prev_priority && command.timestamp > prev.timestamp)
                }
            }
        };

        if should_accept {
            *last_cmd = Some(command);
            true
        } else {
            false
        }
    }

    /// Per-source rate-limit check. Returns `true` if the command is
    /// allowed to proceed; on `false` the command is rejected before
    /// the priority arbitration sees it.
    ///
    /// On `true` the source's timestamp advances to `now`. The check
    /// is a no-op when `min_interval == ZERO`. Different sources are
    /// tracked independently — a noisy autopilot doesn't throttle an
    /// IPC client.
    ///
    /// Map size is bounded by the TTL + hard-cap eviction in
    /// `super::source_map`.
    fn check_and_record_rate(&self, source: &ControlSource) -> bool {
        if self.min_interval.is_zero() {
            return true;
        }
        let now = Instant::now();
        let mut map = self.last_command_at.write();
        evict_for_insert(&mut map, now);
        if let Some(prev) = map.get(source) {
            if now.duration_since(*prev) < self.min_interval {
                warn!(
                    "Rate-limited command from {:?}: {} ms since last (min {} ms)",
                    source,
                    now.duration_since(*prev).as_millis(),
                    self.min_interval.as_millis(),
                );
                return false;
            }
        }
        map.insert(source.clone(), now);
        true
    }

    /// Drop the rate-limit timestamp for `source`. Called when an IPC
    /// slot is recycled so a new client doesn't inherit the previous
    /// tenant's rate-limit history.
    pub fn clear(&self, source: &ControlSource) {
        self.last_command_at.write().remove(source);
    }

    /// Test-only: peek at the rate-limit map. Exposed so the existing
    /// state-level tests can assert eviction behavior end-to-end.
    #[cfg(test)]
    pub(crate) fn last_command_at_handle(&self) -> &Arc<RwLock<HashMap<ControlSource, Instant>>> {
        &self.last_command_at
    }
}

impl Default for CommandStore {
    fn default() -> Self {
        Self::new()
    }
}
