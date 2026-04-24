use super::models::{GimbalCommand, GimbalState, PrimaryControl};
use parking_lot::RwLock;
use std::sync::Arc;

/// Thread-safe gimbal state manager.
///
/// Uses `parking_lot::RwLock` (not `std::sync::RwLock`) so a panicking
/// writer can never poison the lock and bring the daemon down with it.
#[derive(Clone)]
pub struct StateManager {
    state: Arc<RwLock<GimbalState>>,
    last_command: Arc<RwLock<Option<GimbalCommand>>>,
    primary_control: Arc<RwLock<PrimaryControl>>,
}

impl StateManager {
    pub fn new() -> Self {
        Self {
            state: Arc::new(RwLock::new(GimbalState::default())),
            last_command: Arc::new(RwLock::new(None)),
            primary_control: Arc::new(RwLock::new(PrimaryControl::default())),
        }
    }

    /// Get current gimbal state (blocking read)
    pub fn get_state(&self) -> GimbalState {
        self.state.read().clone()
    }

    /// Update gimbal angles
    pub fn update_angles(&self, yaw: f32, pitch: f32, roll: f32) {
        let mut state = self.state.write();
        state.update_angles(yaw, pitch, roll);
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

    /// Set last command (with arbitration check)
    /// Returns true if command was accepted (higher or equal priority)
    pub fn set_command(&self, command: GimbalCommand) -> bool {
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

    /// Clear stale commands
    pub fn clear_stale_commands(&self) {
        let mut last_cmd = self.last_command.write();
        if let Some(ref cmd) = *last_cmd {
            if cmd.is_stale() {
                *last_cmd = None;
            }
        }
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
        manager.update_angles(10.0, 20.0, 30.0);

        let state = manager.get_state();
        assert_eq!(state.yaw, 10.0);
        assert_eq!(state.pitch, 20.0);
        assert_eq!(state.roll, 30.0);
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
