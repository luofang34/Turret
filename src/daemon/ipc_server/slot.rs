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
mod tests;
