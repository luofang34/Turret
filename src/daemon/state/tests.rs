#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use super::*;
use crate::daemon::models::{CommandMode, ControlSource, GimbalCommand, PrimaryControl};

#[test]
fn primary_control_default_allows_all() {
    let m = StateManager::new();
    assert!(m.is_primary_allowed(1, 1));
    assert!(m.is_primary_allowed(42, 200));
}

#[test]
fn primary_control_exact_match_only() {
    let m = StateManager::new();
    m.set_primary_control(PrimaryControl {
        primary_sysid: 7,
        primary_compid: 190,
        secondary_sysid: 0,
        secondary_compid: 0,
    });
    assert!(m.is_primary_allowed(7, 190));
    assert!(!m.is_primary_allowed(7, 191));
    assert!(!m.is_primary_allowed(8, 190));
}

#[test]
fn test_state_manager_init() {
    let manager = StateManager::new();
    let state = manager.get_state();

    assert_eq!(state.yaw, 0.0);
    assert_eq!(state.pitch, 0.0);
    assert_eq!(state.roll, 0.0);
}

#[test]
fn test_state_update() {
    let manager = StateManager::new();
    assert!(manager.get_state().measured_at.is_none());

    manager.update_measured_angles(10.0, 20.0, 30.0);

    let state = manager.get_state();
    assert_eq!(state.yaw, 10.0);
    assert_eq!(state.pitch, 20.0);
    assert_eq!(state.roll, 30.0);
    assert!(state.measured_at.is_some());
}

#[test]
fn vehicle_yaw_returns_none_until_set() {
    let m = StateManager::new();
    assert!(m.vehicle_yaw_deg(Duration::from_secs(1)).is_none());
}

#[test]
fn vehicle_yaw_returns_value_within_window() {
    let m = StateManager::new();
    m.update_vehicle_yaw(42.5);
    let yaw = m.vehicle_yaw_deg(Duration::from_secs(1));
    assert!(yaw.is_some());
    assert!((yaw.unwrap() - 42.5).abs() < 1e-4);
}

#[test]
fn vehicle_yaw_returns_none_when_stale() {
    let m = StateManager::new();
    m.update_vehicle_yaw(42.5);
    // 0 ns max-age means "everything is stale" — proxy for the
    // real "last AUTOPILOT_STATE was > 1 s ago" case without
    // sleeping the test.
    assert!(m.vehicle_yaw_deg(Duration::ZERO).is_none());
}

#[test]
fn evict_stale_drops_entries_past_ttl_keeps_fresh() {
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

    evict_stale(&mut map, now);
    assert_eq!(map.len(), 1);
    assert!(map.contains_key(&ControlSource::UnixSocket(99)));
    assert!(!map.contains_key(&ControlSource::Mavlink(1)));
}

#[test]
fn rate_limit_evicts_stale_source_entries() {
    // End-to-end: a very tight rate-limit window so the test runs
    // fast, plus a synthetic stale entry inserted directly into
    // the map. The next set_command call should sweep the stale
    // entry out before checking the live source.
    let manager = StateManager::with_min_command_interval(std::time::Duration::from_millis(20));

    // Pre-populate with a stale entry from a fictitious old IPC
    // client (the ID won't recur — IPC IDs are monotonic).
    {
        let mut map = manager.last_command_at.write();
        map.insert(
            ControlSource::UnixSocket(1234),
            Instant::now() - Duration::from_secs(600),
        );
        assert_eq!(map.len(), 1);
    }

    // A real command from a live source triggers eviction.
    let cmd = GimbalCommand::new(
        ControlSource::Mavlink(2),
        CommandMode::Position,
        Some(0.0),
        Some(0.0),
        Some(0.0),
    );
    assert!(manager.set_command(cmd));

    // Stale entry is gone; only the freshly-recorded live source
    // remains.
    let map = manager.last_command_at.read();
    assert_eq!(map.len(), 1);
    assert!(map.contains_key(&ControlSource::Mavlink(2)));
}

#[test]
fn rate_tick_evicts_stale_source_entries() {
    let manager = StateManager::new();

    // Pre-populate with a stale rate-tick entry.
    {
        let mut map = manager.last_rate_at.write();
        map.insert(
            ControlSource::UnixSocket(7777),
            Instant::now() - Duration::from_secs(600),
        );
    }

    manager.record_rate_tick(ControlSource::Mavlink(3), Instant::now());

    let map = manager.last_rate_at.read();
    assert_eq!(map.len(), 1);
    assert!(map.contains_key(&ControlSource::Mavlink(3)));
}

#[test]
fn rate_limit_disabled_by_default() {
    // `StateManager::new` constructs with ZERO interval — the
    // rate-limit branch returns `true` immediately. The 1 ms gap
    // between commands guarantees each `GimbalCommand`'s
    // `SystemTime::now()` is strictly later than the previous one,
    // so the existing same-priority-same-source priority check
    // (which requires `command.timestamp > prev.timestamp`) doesn't
    // contaminate this test.
    let manager = StateManager::new();
    let cmd = || {
        GimbalCommand::new(
            ControlSource::Cli,
            CommandMode::Position,
            Some(0.0),
            Some(0.0),
            Some(0.0),
        )
    };
    assert!(manager.set_command(cmd()));
    std::thread::sleep(std::time::Duration::from_millis(1));
    assert!(manager.set_command(cmd()));
}

#[test]
fn rate_limit_blocks_back_to_back_same_source() {
    let manager = StateManager::with_min_command_interval(std::time::Duration::from_millis(50));
    let cmd = || {
        GimbalCommand::new(
            ControlSource::Mavlink(1),
            CommandMode::Position,
            Some(0.0),
            Some(0.0),
            Some(0.0),
        )
    };
    // First accepted; second within 50 ms rejected.
    assert!(manager.set_command(cmd()));
    assert!(!manager.set_command(cmd()));
}

#[test]
fn rate_limit_isolates_per_source() {
    // A noisy GCS must not throttle a different GCS. We use two
    // distinct same-priority MAVLink sources (`Mavlink(2)` and
    // `Mavlink(3)`, both priority 50) so the priority arbitration
    // doesn't reject the second command for unrelated reasons —
    // this test isolates the rate-limit dimension specifically.
    let manager = StateManager::with_min_command_interval(std::time::Duration::from_millis(500));
    let cmd_from = |sysid: u8| {
        GimbalCommand::new(
            ControlSource::Mavlink(sysid),
            CommandMode::Position,
            Some(0.0),
            Some(0.0),
            Some(0.0),
        )
    };
    assert!(manager.set_command(cmd_from(2)));
    // Same-source second command is rate-limited …
    assert!(!manager.set_command(cmd_from(2)));
    // … but a different source at the same priority goes through.
    assert!(manager.set_command(cmd_from(3)));
}

#[test]
fn rate_limit_clears_after_interval() {
    let interval = std::time::Duration::from_millis(20);
    let manager = StateManager::with_min_command_interval(interval);
    let cmd = || {
        GimbalCommand::new(
            ControlSource::Cli,
            CommandMode::Position,
            Some(0.0),
            Some(0.0),
            Some(0.0),
        )
    };
    assert!(manager.set_command(cmd()));
    // Sleep a hair past the window and the same source goes through
    // again. Cheap real-time dependency: 20 ms test interval, 60 ms
    // sleep gives 3× margin against scheduler jitter on slow CI.
    std::thread::sleep(std::time::Duration::from_millis(60));
    assert!(manager.set_command(cmd()));
}

#[test]
fn test_command_priority() {
    let manager = StateManager::new();

    // Low priority command
    let cmd1 = GimbalCommand::new(
        ControlSource::Cli,
        CommandMode::Position,
        Some(0.0),
        Some(0.0),
        Some(0.0),
    );
    assert!(manager.set_command(cmd1.clone()));

    // High priority command should override
    let cmd2 = GimbalCommand::new(
        ControlSource::Mavlink(1),
        CommandMode::Position,
        Some(10.0),
        Some(10.0),
        Some(10.0),
    );
    assert!(manager.set_command(cmd2.clone()));

    // Low priority command should be rejected
    let cmd3 = GimbalCommand::new(
        ControlSource::UnixSocket(1),
        CommandMode::Position,
        Some(20.0),
        Some(20.0),
        Some(20.0),
    );
    assert!(!manager.set_command(cmd3));

    // Verify last command is cmd2
    let last = manager.get_last_command().unwrap();
    assert_eq!(last.yaw, Some(10.0));
}
