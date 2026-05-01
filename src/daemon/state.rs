//! Shared in-memory daemon state: measured attitude, last-accepted
//! command, primary/secondary control ownership. All access goes through
//! `parking_lot::RwLock` so a panicking writer can't poison readers.

use super::models::{ControlSource, GimbalCommand, GimbalState, PrimaryControl};
use parking_lot::RwLock;
use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, Instant};
use tracing::warn;

/// Thread-safe gimbal state manager.
///
/// Uses `parking_lot::RwLock` (not `std::sync::RwLock`) so a panicking
/// writer can never poison the lock and bring the daemon down with it.
#[derive(Clone)]
pub struct StateManager {
    state: Arc<RwLock<GimbalState>>,
    last_command: Arc<RwLock<Option<GimbalCommand>>>,
    primary_control: Arc<RwLock<PrimaryControl>>,
    /// When each `ControlSource` last had a command accepted. Used by
    /// the per-source rate limiter inside [`Self::set_command`]. Empty
    /// map + `min_command_interval == ZERO` means "no throttle".
    last_command_at: Arc<RwLock<HashMap<ControlSource, Instant>>>,
    /// Minimum delta between two accepted commands from the same source.
    /// `Duration::ZERO` disables the check.
    min_command_interval: Duration,
}

impl StateManager {
    /// Construct a fresh state manager with default zero values and no
    /// per-source rate limit. Cheap to `clone()` — the inner state is
    /// `Arc<RwLock<...>>` so all clones share the same underlying buffers.
    pub fn new() -> Self {
        Self::with_min_command_interval(Duration::ZERO)
    }

    /// Construct a state manager with a per-source rate-limit floor.
    /// Two commands from the same `ControlSource` within `interval` will
    /// see the second one rejected by [`Self::set_command`]. `ZERO`
    /// disables the check (matching [`Self::new`]).
    pub fn with_min_command_interval(interval: Duration) -> Self {
        Self {
            state: Arc::new(RwLock::new(GimbalState::default())),
            last_command: Arc::new(RwLock::new(None)),
            primary_control: Arc::new(RwLock::new(PrimaryControl::default())),
            last_command_at: Arc::new(RwLock::new(HashMap::new())),
            min_command_interval: interval,
        }
    }

    /// Get current gimbal state (blocking read)
    pub fn get_state(&self) -> GimbalState {
        self.state.read().clone()
    }

    /// Record a fresh attitude measurement from the device.
    ///
    /// Called by the attitude poll loop in `mavlink_manager`. The arbitrator
    /// must not call this — it should only ever record `last_command`.
    pub fn update_measured_angles(&self, yaw: f32, pitch: f32, roll: f32) {
        let mut state = self.state.write();
        state.update_measured_angles(yaw, pitch, roll);
    }

    /// Update pan mode
    pub fn update_pan_mode(&self, mode: super::models::PanMode) {
        let mut state = self.state.write();
        state.update_pan_mode(mode);
    }

    /// Update standby state
    pub fn update_standby(&self, standby: bool) {
        let mut state = self.state.write();
        state.update_standby(standby);
    }

    /// Update firmware version
    pub fn update_firmware_version(&self, version: String) {
        let mut state = self.state.write();
        state.firmware_version = Some(version);
    }

    /// Get last command
    pub fn get_last_command(&self) -> Option<GimbalCommand> {
        self.last_command.read().clone()
    }

    /// Run a command through arbitration: per-source rate limit first,
    /// then the priority check. Returns `true` if accepted (in which case
    /// the command becomes the new `last_command` and the source's
    /// `last_command_at` advances).
    ///
    /// The rate-limit check runs on its own lock so the (more contended)
    /// `last_command` lock isn't held across the time arithmetic.
    pub fn set_command(&self, command: GimbalCommand) -> bool {
        if !self.check_and_record_rate(&command.source) {
            return false;
        }

        let mut last_cmd = self.last_command.write();

        // Check if new command has higher priority than current
        let should_accept = match &*last_cmd {
            None => true, // No previous command, accept
            Some(prev) => {
                // Accept if:
                // 1. New command has higher priority, OR
                // 2. Same priority and newer timestamp, OR
                // 3. Previous command is stale
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
    /// allowed to proceed; on `false` the command is rejected before the
    /// priority arbitration sees it.
    ///
    /// On `true` the source's `last_command_at` advances to `now`. The
    /// check is no-op when `min_command_interval == ZERO`. Different
    /// sources are tracked independently — a noisy autopilot doesn't
    /// throttle an IPC client.
    ///
    /// Map growth: bounded by the number of distinct `ControlSource`s
    /// the daemon has ever seen. `Mavlink(u8)` caps at 256, `Cli` is one,
    /// `UnixSocket(u32)` grows over time. For long-running daemons that
    /// serve thousands of IPC clients this should grow eviction; for
    /// now it's an accepted slow leak.
    fn check_and_record_rate(&self, source: &ControlSource) -> bool {
        if self.min_command_interval.is_zero() {
            return true;
        }
        let now = Instant::now();
        let mut map = self.last_command_at.write();
        if let Some(prev) = map.get(source) {
            if now.duration_since(*prev) < self.min_command_interval {
                warn!(
                    "Rate-limited command from {:?}: {} ms since last (min {} ms)",
                    source,
                    now.duration_since(*prev).as_millis(),
                    self.min_command_interval.as_millis(),
                );
                return false;
            }
        }
        map.insert(source.clone(), now);
        true
    }

    /// Current primary/secondary control ownership.
    pub fn get_primary_control(&self) -> PrimaryControl {
        *self.primary_control.read()
    }

    /// Set primary + secondary control. All-zero means "unset / open".
    pub fn set_primary_control(&self, pc: PrimaryControl) {
        *self.primary_control.write() = pc;
    }

    /// True if `(sysid, compid)` is authorized to drive the gimbal under the
    /// current primary-control configuration.
    pub fn is_primary_allowed(&self, sysid: u8, compid: u8) -> bool {
        self.get_primary_control().is_allowed(sysid, compid)
    }
}

impl Default for StateManager {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
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
        let manager =
            StateManager::with_min_command_interval(std::time::Duration::from_millis(500));
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
}
