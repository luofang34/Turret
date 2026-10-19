use super::*;
use crate::daemon::models::GimbalCommand;
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
        Some(1.0),
        Some(1.0),
        Some(1.0),
    );
    assert!(
        state.set_command(cmd_b),
        "fresh client on recycled slot was rate-limited by prior tenant"
    );
}
