//! Bounded slot allocator for IPC client IDs.
//!
//! Bounds the `ControlSource::UnixSocket(u32)` keyspace at the source.
//! Each accepted connection claims one of `MAX_IPC_SLOTS` indices via
//! [`SlotPool::acquire`]; the returned [`SlotGuard`] releases the slot
//! on drop (i.e. when the per-client task exits). When all slots are
//! occupied, `acquire` returns `None` and the listener rejects the
//! connection — the daemon prefers a clean rejection over an
//! unbounded backlog.
//!
//! On release the guard also clears the slot's entries from
//! [`StateManager`]'s per-source tracking maps, so a new client
//! claiming the same slot index isn't penalized by the previous
//! tenant's rate-limit timestamp or rate-tick history. This is the
//! property the per-source maps' lazy eviction can't guarantee on
//! its own — the maps may still hold a stale entry under their TTL,
//! but the slot guard's cleanup makes slot reuse semantically clean.
//!
//! 64 slots is well above any sane same-time-active IPC client count
//! (a daemon typically sees one CLI invocation at a time, occasionally
//! a status monitor) and well below the 256-source map cap, so even
//! a fully-saturated IPC pool plus all 256 MAVLink sysids fits the
//! daemon's bounded source set.

use crate::daemon::models::ControlSource;
use crate::daemon::state::StateManager;
use parking_lot::Mutex;
use std::sync::Arc;

/// Maximum simultaneous IPC connections the daemon will service.
/// New connections beyond this are rejected (the listener closes the
/// stream immediately and logs a warn). `u32` slot indices stay in
/// `0..MAX_IPC_SLOTS`, so the `ControlSource::UnixSocket` keyspace
/// is permanently bounded by this.
pub const MAX_IPC_SLOTS: usize = 64;

/// Pool of `MAX_IPC_SLOTS` IPC client slots. Cheap to clone (`Arc`).
pub struct SlotPool {
    inner: Arc<Inner>,
}

struct Inner {
    in_use: Mutex<[bool; MAX_IPC_SLOTS]>,
    state: StateManager,
}

impl SlotPool {
    /// Build an empty pool that will clear released slots' entries
    /// from `state`'s per-source tracking maps on drop of each guard.
    pub fn new(state: StateManager) -> Self {
        Self {
            inner: Arc::new(Inner {
                in_use: Mutex::new([false; MAX_IPC_SLOTS]),
                state,
            }),
        }
    }

    /// Try to claim a free slot. Returns `None` if the pool is full —
    /// the caller should reject the incoming connection.
    pub fn acquire(&self) -> Option<SlotGuard> {
        let mut slots = self.inner.in_use.lock();
        let idx = slots.iter().position(|busy| !busy)?;
        slots[idx] = true;
        // SAFETY: idx came from a slice with `MAX_IPC_SLOTS <= u32::MAX`,
        // so the cast is lossless. (`MAX_IPC_SLOTS` is a small `usize` const.)
        let slot = u32::try_from(idx).ok()?;
        Some(SlotGuard {
            inner: self.inner.clone(),
            slot,
        })
    }

    /// Currently occupied slot count. Test/debug only — the daemon
    /// doesn't read this in the hot path.
    #[cfg(test)]
    pub fn in_use_count(&self) -> usize {
        self.inner.in_use.lock().iter().filter(|b| **b).count()
    }
}

impl Clone for SlotPool {
    fn clone(&self) -> Self {
        Self {
            inner: self.inner.clone(),
        }
    }
}

/// RAII handle for an IPC client slot. The slot is released and the
/// associated `ControlSource::UnixSocket(slot)` entries cleared from
/// per-source tracking maps when the guard is dropped.
pub struct SlotGuard {
    inner: Arc<Inner>,
    slot: u32,
}

impl SlotGuard {
    /// The slot index, used as the `ControlSource::UnixSocket(_)` payload.
    pub fn id(&self) -> u32 {
        self.slot
    }
}

impl Drop for SlotGuard {
    fn drop(&mut self) {
        self.inner
            .state
            .clear_source(&ControlSource::UnixSocket(self.slot));
        let mut slots = self.inner.in_use.lock();
        slots[self.slot as usize] = false;
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;
    use crate::daemon::models::{CommandMode, GimbalCommand};
    use std::time::Duration;

    fn fresh_pool() -> SlotPool {
        SlotPool::new(StateManager::new())
    }

    #[test]
    fn acquire_assigns_distinct_slots_until_full() {
        let pool = fresh_pool();
        let mut guards: Vec<SlotGuard> = Vec::with_capacity(MAX_IPC_SLOTS);
        for _ in 0..MAX_IPC_SLOTS {
            guards.push(pool.acquire().expect("slot must be available"));
        }
        assert_eq!(pool.in_use_count(), MAX_IPC_SLOTS);
        // All assigned IDs are distinct.
        let mut ids: Vec<u32> = guards.iter().map(|g| g.id()).collect();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), MAX_IPC_SLOTS);
    }

    #[test]
    fn acquire_returns_none_when_pool_full() {
        let pool = fresh_pool();
        let _holders: Vec<SlotGuard> = (0..MAX_IPC_SLOTS)
            .map(|_| pool.acquire().expect("fits"))
            .collect();
        assert!(pool.acquire().is_none());
    }

    #[test]
    fn drop_releases_slot_for_reuse() {
        let pool = fresh_pool();
        let g1 = pool.acquire().expect("first");
        let id1 = g1.id();
        drop(g1);
        assert_eq!(pool.in_use_count(), 0);
        // Slot index recycles immediately (lowest free index).
        let g2 = pool.acquire().expect("after drop");
        assert_eq!(g2.id(), id1);
    }

    #[test]
    fn drop_clears_state_entries_for_recycled_slot() {
        // A previous tenant's rate-limit timestamp must not penalize a
        // new client claiming the same slot. Set a tight rate-limit
        // window, simulate "client A used slot N, hit rate-limit map,
        // disconnected; client B reuses slot N immediately".
        let state = StateManager::with_min_command_interval(Duration::from_secs(60));
        let pool = SlotPool::new(state.clone());

        let g_a = pool.acquire().expect("first");
        let slot_id = g_a.id();
        // Client A issues a command — populates last_command_at[slot].
        let cmd_a = GimbalCommand::new(
            ControlSource::UnixSocket(slot_id),
            CommandMode::Position,
            Some(0.0),
            Some(0.0),
            Some(0.0),
        );
        assert!(state.set_command(cmd_a));
        // Disconnect — guard drop must clear the rate-limit entry.
        drop(g_a);

        // Client B claims the same slot and tries a command. Without
        // the slot guard's cleanup the 60-s rate-limit window would
        // reject it (the prior timestamp is still well within range).
        let g_b = pool.acquire().expect("recycle");
        assert_eq!(g_b.id(), slot_id);
        let cmd_b = GimbalCommand::new(
            ControlSource::UnixSocket(slot_id),
            CommandMode::Position,
            Some(1.0),
            Some(1.0),
            Some(1.0),
        );
        assert!(
            state.set_command(cmd_b),
            "fresh client on recycled slot was rate-limited by prior tenant"
        );
    }
}
