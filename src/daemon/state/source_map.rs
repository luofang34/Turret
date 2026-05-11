//! Bounded per-source timestamp map.
//!
//! The arbitration store and the rate-tick tracker both keep a
//! `HashMap<ControlSource, Instant>` of "when did this source last do
//! X?" Both need the same eviction discipline: TTL-based normal-path
//! cleanup, plus a hard cap so a burst of distinct sources can't grow
//! the map unboundedly.
//!
//! This module is the shared implementation. Each store wraps it
//! behind its own semantic API ([`super::commands::CommandStore`] for
//! rate-limit timestamps, [`super::rate::RateTracker`] for rate-tick
//! `dt` bookkeeping).

use crate::daemon::models::ControlSource;
use std::collections::HashMap;
use std::time::{Duration, Instant};

/// How long a per-source `Instant` stays in the map before lazy
/// eviction drops it on the next write. After this much silence the
/// source is treated as fresh: the rate-limit window resets and the
/// rate integrator's `dt` falls back to the minimum.
pub(super) const SOURCE_TRACKING_TTL: Duration = Duration::from_secs(300);

/// Hard upper bound on per-source tracking map size. The TTL alone
/// only protects against long-term accumulation; under a burst that
/// churns through IPC client IDs faster than the TTL can clear them
/// (a buggy or malicious client looping connect/disconnect, an
/// automated test harness, etc.) the map could still spike. When at
/// this many entries an insert evicts the single oldest one.
///
/// 256 matches MAVLink's sysid space (`u8`), so even if every
/// possible MAVLink sysid is alive we never evict a "real" MAVLink
/// source — only the rest of the map (IPC slots) churns. At
/// `~32 B/entry`, the bound is ~8 KB per map, ~16 KB across both —
/// negligible on any platform we ship to.
///
/// Eviction of an active source is functionally minor:
///   - rate-limit timestamps: the source's next command is treated as
///     "first time", skipping one rate-limit hit. Not security-critical
///     (gimbal-limit clamping happens earlier in the pipeline;
///     rate-limit is a fairness knob, not a safety gate).
///   - rate-tick timestamps: the next rate tick falls back to the
///     10 ms `MIN_DT`, under-shooting one position increment. Operator
///     feels a one-frame jitter; the next tick recovers.
pub(super) const MAX_TRACKED_SOURCES: usize = 256;

/// Bound a per-source map's size before an insert. Two passes:
///
/// 1. TTL sweep — drop every entry older than [`SOURCE_TRACKING_TTL`].
///    Cheap O(n) `retain`. Handles the steady-state case where the
///    working set is tiny but stale entries from disconnected clients
///    would otherwise linger.
/// 2. Hard-cap pass — if the map is *still* at capacity (every entry
///    is fresh — burst workload), drop the single oldest entry to
///    make room for the new one. O(n) find-min, only fires under the
///    burst.
///
/// Together these guarantee post-insert `len ≤ MAX_TRACKED_SOURCES`
/// regardless of workload.
pub(super) fn evict_for_insert(map: &mut HashMap<ControlSource, Instant>, now: Instant) {
    map.retain(|_, t| now.duration_since(*t) <= SOURCE_TRACKING_TTL);
    if map.len() >= MAX_TRACKED_SOURCES {
        if let Some(oldest_key) = map.iter().min_by_key(|(_, t)| *t).map(|(k, _)| k.clone()) {
            map.remove(&oldest_key);
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;

    #[test]
    fn evict_for_insert_drops_entries_past_ttl_keeps_fresh() {
        // The eviction helper runs against a synthetic map so the test
        // doesn't have to actually wait 5 minutes. Stamp two entries:
        // one ten minutes ago (well past the TTL) and one a moment ago.
        let mut map: HashMap<ControlSource, Instant> = HashMap::new();
        let now = Instant::now();
        let stale = now - Duration::from_secs(600);
        let fresh = now - Duration::from_millis(100);
        map.insert(ControlSource::Mavlink(1), stale);
        map.insert(ControlSource::UnixSocket(99), fresh);
        assert_eq!(map.len(), 2);

        evict_for_insert(&mut map, now);
        assert_eq!(map.len(), 1);
        assert!(map.contains_key(&ControlSource::UnixSocket(99)));
        assert!(!map.contains_key(&ControlSource::Mavlink(1)));
    }

    #[test]
    fn evict_for_insert_enforces_hard_cap_under_burst() {
        // Synthesise a burst that the TTL cannot help with: every entry
        // is fresh (within the 5-min window), but the count exceeds the
        // hard cap. After the helper runs the map must be exactly at
        // `MAX_TRACKED_SOURCES - 1` so the caller's insert leaves it at
        // MAX. The single evicted entry is the oldest one we put in.
        let mut map: HashMap<ControlSource, Instant> = HashMap::new();
        let now = Instant::now();

        // Fill to exactly the cap with timestamps that order
        // monotonically (entry `i` is `i` ms before `now`, so larger `i`
        // means older).
        for i in 0..MAX_TRACKED_SOURCES as u32 {
            map.insert(
                ControlSource::UnixSocket(i),
                now - Duration::from_millis(u64::from(i)),
            );
        }
        assert_eq!(map.len(), MAX_TRACKED_SOURCES);

        evict_for_insert(&mut map, now);

        // Hard cap kicked in: one entry dropped, room for the imminent insert.
        assert_eq!(map.len(), MAX_TRACKED_SOURCES - 1);
        // The evicted entry is the oldest (highest `i`).
        let evicted = ControlSource::UnixSocket(MAX_TRACKED_SOURCES as u32 - 1);
        assert!(!map.contains_key(&evicted));
    }
}
