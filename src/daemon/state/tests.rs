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

fn sample_vehicle_state(yaw_deg: f32) -> VehicleState {
    VehicleState {
        pitch_deg: 0.0,
        roll_deg: 0.0,
        yaw_deg,
        yaw_rate_deg_s: None,
        ned_velocity_m_s: (0.0, 0.0, 0.0),
    }
}

#[test]
fn vehicle_yaw_returns_none_until_set() {
    let m = StateManager::new();
    assert!(m.vehicle_yaw_deg(Duration::from_secs(1)).is_none());
    assert!(m.vehicle_state(Duration::from_secs(1)).is_none());
}

#[test]
fn vehicle_yaw_returns_value_within_window() {
    let m = StateManager::new();
    m.update_vehicle_state(sample_vehicle_state(42.5));
    let yaw = m.vehicle_yaw_deg(Duration::from_secs(1));
    assert!(yaw.is_some());
    assert!((yaw.unwrap() - 42.5).abs() < 1e-4);
}

#[test]
fn vehicle_yaw_returns_none_when_stale() {
    let m = StateManager::new();
    m.update_vehicle_state(sample_vehicle_state(42.5));
    // 0 ns max-age means "everything is stale" — proxy for the
    // real "last AUTOPILOT_STATE was > 1 s ago" case without
    // sleeping the test.
    assert!(m.vehicle_yaw_deg(Duration::ZERO).is_none());
}

#[test]
fn vehicle_state_round_trips_full_struct() {
    // Confirm every field decoded from AUTOPILOT_STATE_FOR_GIMBAL_DEVICE
    // makes it through update / read unchanged. Catches future
    // refactors that might silently drop a field while keeping the
    // yaw-only convenience accessor working.
    let m = StateManager::new();
    let s = VehicleState {
        pitch_deg: -3.5,
        roll_deg: 1.25,
        yaw_deg: 175.0,
        yaw_rate_deg_s: Some(2.5),
        ned_velocity_m_s: (10.0, -1.0, 0.5),
    };
    m.update_vehicle_state(s);
    let got = m.vehicle_state(Duration::from_secs(1)).unwrap();
    assert!((got.pitch_deg - s.pitch_deg).abs() < 1e-4);
    assert!((got.roll_deg - s.roll_deg).abs() < 1e-4);
    assert!((got.yaw_deg - s.yaw_deg).abs() < 1e-4);
    assert!(got.yaw_rate_deg_s.is_some());
    assert!((got.yaw_rate_deg_s.unwrap() - 2.5).abs() < 1e-4);
    assert_eq!(got.ned_velocity_m_s.0, 10.0);
    assert_eq!(got.ned_velocity_m_s.1, -1.0);
    assert_eq!(got.ned_velocity_m_s.2, 0.5);
}

#[test]
fn vehicle_state_preserves_nan_velocity_per_axis() {
    // Spec says NAN per axis is allowed for velocity ("unknown"). Don't
    // collapse it to zero — consumers branch on per-axis validity and
    // a fake zero would be a positive-but-wrong claim.
    let m = StateManager::new();
    m.update_vehicle_state(VehicleState {
        pitch_deg: 0.0,
        roll_deg: 0.0,
        yaw_deg: 0.0,
        yaw_rate_deg_s: None,
        ned_velocity_m_s: (1.0, f32::NAN, 0.0),
    });
    let got = m.vehicle_state(Duration::from_secs(1)).unwrap();
    assert_eq!(got.ned_velocity_m_s.0, 1.0);
    assert!(got.ned_velocity_m_s.1.is_nan());
    assert_eq!(got.ned_velocity_m_s.2, 0.0);
}

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

#[test]
fn rate_tick_burst_stays_bounded_at_cap() {
    // End-to-end: hammer `record_rate_tick` with `MAX_TRACKED_SOURCES * 4`
    // distinct synthetic IPC clients in tight succession. The TTL can't
    // help (all writes are within microseconds), so the only thing
    // keeping the map bounded is the hard cap. After the burst the
    // map must hold ≤ MAX entries, period.
    let manager = StateManager::new();
    let now = Instant::now();
    for i in 0..(MAX_TRACKED_SOURCES as u32) * 4 {
        manager.record_rate_tick(
            ControlSource::UnixSocket(i),
            now + Duration::from_nanos(u64::from(i)),
        );
    }
    let map = manager.last_rate_at.read();
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
        ControlSource::Cli,
        CommandMode::Position,
        Some(20.0), // yaw
        Some(10.0), // pitch
        None,       // roll: not commanded
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
        ControlSource::Cli,
        CommandMode::Position,
        None,       // yaw: not commanded
        Some(10.0), // pitch
        None,       // roll: not commanded
    );
    assert!(s.set_command(cmd));
    assert_eq!(s.axis_baseline(), (10.0, 0.0, 0.0));
}
