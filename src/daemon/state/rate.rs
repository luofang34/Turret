//! Per-source rate-tick timestamps.
//!
//! Distinct from [`super::commands::CommandStore`]'s rate-limit
//! timestamps: this tracks when each source last emitted a *rate*
//! command (so the rate→position integrator can compute `dt` between
//! ticks), independent of position-command bookkeeping. A position
//! command from the same source must not reset the rate timer.

use super::source_map::evict_for_insert;
use crate::daemon::models::ControlSource;
use parking_lot::RwLock;
use std::collections::HashMap;
use std::sync::Arc;
use std::time::Instant;

/// Owns the per-source rate-tick timestamp map.
#[derive(Clone, Default)]
pub struct RateTracker {
    inner: Arc<RwLock<HashMap<ControlSource, Instant>>>,
}

impl RateTracker {
    /// Empty tracker.
    pub fn new() -> Self {
        Self::default()
    }

    /// Most recent rate-command timestamp for `source`, if any.
    pub fn last_tick(&self, source: &ControlSource) -> Option<Instant> {
        self.inner.read().get(source).copied()
    }

    /// Mark `source` as having issued a rate command at `now`. The
    /// rate integrator calls this after computing the new target so
    /// the next rate tick from the same source sees the right `dt`.
    ///
    /// Bounded by the shared TTL + hard-cap eviction in
    /// `super::source_map`.
    pub fn record_tick(&self, source: ControlSource, now: Instant) {
        let mut map = self.inner.write();
        evict_for_insert(&mut map, now);
        map.insert(source, now);
    }

    /// Drop the rate-tick timestamp for `source`. Called when an IPC
    /// slot is recycled.
    pub fn clear(&self, source: &ControlSource) {
        self.inner.write().remove(source);
    }

    /// Test-only: peek at the map.
    #[cfg(test)]
    pub(crate) fn map_handle(&self) -> &Arc<RwLock<HashMap<ControlSource, Instant>>> {
        &self.inner
    }
}
