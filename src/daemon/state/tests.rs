#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use super::source_map::MAX_TRACKED_SOURCES;
use super::*;
use crate::daemon::models::{ControlSource, GimbalCommand, PrimaryControl};
use std::time::Duration;

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
fn rate_tick_burst_stays_bounded_at_cap() {
    // End-to-end: hammer `record_rate_tick` with `MAX_TRACKED_SOURCES * 4`
    // distinct synthetic IPC clients in tight succession. The TTL can't
    // help (all writes are within microseconds), so the only thing
    // keeping the map bounded is the hard cap. After the burst the
    // map must hold ≤ MAX entries, period.
    let manager = StateManager::new();
    let now = std::time::Instant::now();
    for i in 0..(MAX_TRACKED_SOURCES as u32) * 4 {
        manager.record_rate_tick(
            ControlSource::UnixSocket(i),
            now + Duration::from_nanos(u64::from(i)),
        );
    }
    let map = manager.rate_tracker.map_handle().read();
    assert!(
        map.len() <= MAX_TRACKED_SOURCES,
        "map.len() = {} exceeded cap {}",
        map.len(),
        MAX_TRACKED_SOURCES
    );
}

#[test]
fn rate_limit_evicts_stale_source_entries() {
    // End-to-end: a very tight rate-limit window so the test runs
    // fast, plus a synthetic stale entry inserted directly into
    // the map. The next set_command call should sweep the stale
    // entry out before checking the live source.
    let manager = StateManager::with_min_command_interval(Duration::from_millis(20));

    // Pre-populate with a stale entry from a fictitious old IPC
    // client (the ID won't recur — IPC IDs are monotonic).
    {
        let mut map = manager.commands.last_command_at_handle().write();
        map.insert(
            ControlSource::UnixSocket(1234),
            std::time::Instant::now() - Duration::from_secs(600),
        );
        assert_eq!(map.len(), 1);
    }

    // A real command from a live source triggers eviction.
    let cmd = GimbalCommand::new(ControlSource::Mavlink(2), Some(0.0), Some(0.0), Some(0.0));
    assert!(manager.set_command(cmd));

    // Stale entry is gone; only the freshly-recorded live source
    // remains.
    let map = manager.commands.last_command_at_handle().read();
    assert_eq!(map.len(), 1);
    assert!(map.contains_key(&ControlSource::Mavlink(2)));
}

#[test]
fn rate_tick_evicts_stale_source_entries() {
    let manager = StateManager::new();

    // Pre-populate with a stale rate-tick entry.
    {
        let mut map = manager.rate_tracker.map_handle().write();
        map.insert(
            ControlSource::UnixSocket(7777),
            std::time::Instant::now() - Duration::from_secs(600),
        );
    }

    manager.record_rate_tick(ControlSource::Mavlink(3), std::time::Instant::now());

    let map = manager.rate_tracker.map_handle().read();
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
            ControlSource::UnixSocket(0),
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
    let cmd = || GimbalCommand::new(ControlSource::Mavlink(1), Some(0.0), Some(0.0), Some(0.0));
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
            ControlSource::UnixSocket(0),
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

    // Low priority command (IPC client).
    let cmd1 = GimbalCommand::new(
        ControlSource::UnixSocket(0),
        Some(0.0),
        Some(0.0),
        Some(0.0),
    );
    assert!(manager.set_command(cmd1.clone()));

    // High priority command (autopilot, sysid 1) should override.
    let cmd2 = GimbalCommand::new(
        ControlSource::Mavlink(1),
        Some(10.0),
        Some(10.0),
        Some(10.0),
    );
    assert!(manager.set_command(cmd2.clone()));

    // Another IPC client at the same lower priority is rejected.
    let cmd3 = GimbalCommand::new(
        ControlSource::UnixSocket(1),
        Some(20.0),
        Some(20.0),
        Some(20.0),
    );
    assert!(!manager.set_command(cmd3));

    // Verify last command is cmd2
    let last = manager.get_last_command().unwrap();
    assert_eq!(last.yaw, Some(10.0));
}

#[test]
fn axis_baseline_zero_when_cold() {
    // No last command, no successful measurement → all zeros.
    let s = StateManager::new();
    assert_eq!(s.axis_baseline(), (0.0, 0.0, 0.0));
}

#[test]
fn axis_baseline_falls_through_to_measured_when_no_command() {
    // Measurement landed but no command has been issued — every axis
    // falls through to the measured PV. This is the "first partial
    // command after boot" case: don't snap unmoved axes to zero, hold
    // them where the IMU says they are.
    let s = StateManager::new();
    s.update_measured_angles(20.0, 10.0, 30.0); // (yaw, pitch, roll)
    assert_eq!(s.axis_baseline(), (10.0, 30.0, 20.0));
}

#[test]
fn axis_baseline_prefers_last_command_per_axis() {
    // last_command has only pitch + yaw; roll is None. Even with a
    // measurement on file, roll falls through to measured.roll and
    // pitch/yaw use the commanded values, not the measured PV — IMU
    // noise and yaw drift correction don't perturb an axis-baseline
    // read.
    let s = StateManager::new();
    s.update_measured_angles(20.5, 10.5, 30.5); // PV slightly drifted
    let cmd = GimbalCommand::new(
        ControlSource::UnixSocket(0),
        Some(10.0), // pitch
        None,       // roll: not commanded
        Some(20.0), // yaw
    );
    assert!(s.set_command(cmd));
    assert_eq!(s.axis_baseline(), (10.0, 30.5, 20.0));
}

#[test]
fn axis_baseline_command_only_no_measured() {
    // Command issued before any successful poll — the measured branch
    // is gated by `measured_at.is_some()` so axes the command didn't
    // touch fall to 0, not to the Default::default() zeros that look
    // identical at runtime but mean "no measurement yet."
    let s = StateManager::new();
    let cmd = GimbalCommand::new(
        ControlSource::UnixSocket(0),
        Some(10.0), // pitch
        None,       // roll: not commanded
        None,       // yaw: not commanded
    );
    assert!(s.set_command(cmd));
    assert_eq!(s.axis_baseline(), (10.0, 0.0, 0.0));
}
