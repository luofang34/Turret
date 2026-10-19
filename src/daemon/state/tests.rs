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
    let state = manager.state_cached();

    assert_eq!(state.yaw, None);
    assert_eq!(state.pitch, None);
    assert_eq!(state.roll, None);
}

#[test]
fn test_state_update() {
    let manager = StateManager::new();
    assert!(manager.state_cached().measured_at.is_none());

    manager.update_measured_angles(10.0, 20.0, 30.0);

    let state = manager.state_cached();
    assert_eq!(state.yaw, Some(10.0));
    assert_eq!(state.pitch, Some(20.0));
    assert_eq!(state.roll, Some(30.0));
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
    assert!(m.vehicle_yaw_deg(Duration::ZERO).is_none());
}

#[test]
fn rate_limit_evicts_stale_source_entries() {
    let manager = StateManager::with_min_command_interval(Duration::from_millis(20));

    {
        let mut map = manager.commands.last_command_at_handle().write();
        map.insert(
            ControlSource::UnixSocket(1234),
            std::time::Instant::now() - Duration::from_secs(600),
        );
        assert_eq!(map.len(), 1);
    }

    let cmd = GimbalCommand::new(ControlSource::Mavlink(2), Some(0.0), Some(0.0), Some(0.0));
    assert!(manager.set_command(cmd));

    let map = manager.commands.last_command_at_handle().read();
    assert_eq!(map.len(), 1);
    assert!(map.contains_key(&ControlSource::Mavlink(2)));
}

#[test]
fn rate_limit_disabled_by_default() {
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
    assert!(manager.set_command(cmd()));
}

#[test]
fn rate_limit_blocks_back_to_back_same_source() {
    let manager = StateManager::with_min_command_interval(std::time::Duration::from_millis(50));
    let cmd = || GimbalCommand::new(ControlSource::Mavlink(1), Some(0.0), Some(0.0), Some(0.0));
    assert!(manager.set_command(cmd()));
    assert!(!manager.set_command(cmd()));
}

#[test]
fn rate_limit_isolates_per_source() {
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
    assert!(!manager.set_command(cmd_from(2)));
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
    let mut later = cmd();
    later.received_at += interval;
    assert!(manager.set_command(later));
}

#[test]
fn source_number_never_grants_priority() {
    let manager = StateManager::new();

    let cmd1 = GimbalCommand::new(
        ControlSource::UnixSocket(0),
        Some(0.0),
        Some(0.0),
        Some(0.0),
    );
    assert!(manager.set_command(cmd1.clone()));

    let cmd2 = GimbalCommand::new(
        ControlSource::Mavlink(1),
        Some(10.0),
        Some(10.0),
        Some(10.0),
    );
    assert!(manager.set_command(cmd2.clone()));

    let cmd3 = GimbalCommand::new(
        ControlSource::UnixSocket(1),
        Some(20.0),
        Some(20.0),
        Some(20.0),
    );
    assert!(manager.set_command(cmd3));

    let last = manager.commands.last_cached().unwrap();
    assert_eq!(last.yaw, Some(20.0));
}

#[test]
fn axis_baseline_zero_when_cold() {
    let s = StateManager::new();
    assert_eq!(s.axis_baseline_cached(), (None, None, None));
}

#[test]
fn axis_baseline_falls_through_to_measured_when_no_command() {
    let s = StateManager::new();
    s.update_measured_angles(20.0, 10.0, 30.0); // (yaw, pitch, roll)
    assert_eq!(
        s.axis_baseline_cached(),
        (Some(10.0), Some(30.0), Some(20.0))
    );
}

#[test]
fn axis_baseline_prefers_last_command_per_axis() {
    let s = StateManager::new();
    s.update_measured_angles(20.5, 10.5, 30.5); // PV slightly drifted
    let cmd = GimbalCommand::new(
        ControlSource::UnixSocket(0),
        Some(10.0), // pitch
        None,       // roll: not commanded
        Some(20.0), // yaw
    );
    s.commands.record_sent(cmd);
    assert_eq!(
        s.axis_baseline_cached(),
        (Some(10.0), Some(30.5), Some(20.0))
    );
}

#[test]
fn axis_baseline_command_only_no_measured() {
    let s = StateManager::new();
    let cmd = GimbalCommand::new(
        ControlSource::UnixSocket(0),
        Some(10.0), // pitch
        None,       // roll: not commanded
        None,       // yaw: not commanded
    );
    s.commands.record_sent(cmd);
    assert_eq!(s.axis_baseline_cached(), (Some(10.0), None, None));
}
