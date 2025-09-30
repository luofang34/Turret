use super::models::{CommandMode, GimbalCommand};
use super::state::StateManager;
use crate::gimbal::GimbalDevice;
use log::{debug, info, warn};

/// Command arbitrator that executes commands on the gimbal hardware
pub struct Arbitrator {
    state_manager: StateManager,
}

impl Arbitrator {
    pub fn new(state_manager: StateManager) -> Self {
        Self { state_manager }
    }

    /// Process and execute a gimbal command
    /// Returns true if command was accepted and executed
    pub fn process_command(
        &self,
        command: GimbalCommand,
        gimbal: &mut Box<dyn GimbalDevice>,
    ) -> anyhow::Result<bool> {
        // Try to set command (checks priority)
        if !self.state_manager.set_command(command.clone()) {
            debug!(
                "Command rejected: lower priority than current command (source: {:?})",
                command.source
            );
            return Ok(false);
        }

        info!(
            "Accepted command from {:?} with priority {}",
            command.source,
            command.source.priority()
        );

        // Execute command based on mode
        match command.mode {
            CommandMode::Position => {
                // Absolute angle positioning
                let yaw = command.yaw.unwrap_or(0.0);
                let pitch = command.pitch.unwrap_or(0.0);
                let roll = command.roll.unwrap_or(0.0);

                debug!("Executing position command: yaw={:.1}, pitch={:.1}, roll={:.1}", yaw, pitch, roll);
                gimbal.set_attitude(pitch, roll, yaw)?;

                // Update state
                self.state_manager.update_angles(yaw, pitch, roll);
            }
            CommandMode::Rate => {
                // Angular velocity control (not yet implemented)
                warn!("Rate control mode not yet implemented");
                return Ok(false);
            }
            CommandMode::Manual => {
                // Direct stick-like inputs (not yet implemented)
                warn!("Manual control mode not yet implemented");
                return Ok(false);
            }
        }

        Ok(true)
    }

    /// Clear stale commands periodically
    pub fn cleanup_stale_commands(&self) {
        self.state_manager.clear_stale_commands();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gimbal::Attitude;

    struct MockGimbal {
        last_attitude: Option<(f32, f32, f32)>,
    }

    impl MockGimbal {
        fn new() -> Self {
            Self {
                last_attitude: None,
            }
        }
    }

    impl GimbalDevice for MockGimbal {
        fn protocol_name(&self) -> &'static str {
            "Mock"
        }

        fn set_attitude(&mut self, pitch: f32, roll: f32, yaw: f32) -> anyhow::Result<()> {
            self.last_attitude = Some((pitch, roll, yaw));
            Ok(())
        }

        fn get_attitude(&mut self) -> anyhow::Result<Attitude> {
            Ok(Attitude {
                pitch: 0.0,
                roll: 0.0,
                yaw: 0.0,
            })
        }
    }

    #[test]
    fn test_command_execution() {
        let state_manager = StateManager::new();
        let arbitrator = Arbitrator::new(state_manager.clone());
        let mut gimbal: Box<dyn GimbalDevice> = Box::new(MockGimbal::new());

        let cmd = GimbalCommand::new(
            ControlSource::Cli,
            CommandMode::Position,
            Some(10.0),
            Some(20.0),
            Some(30.0),
        );

        let result = arbitrator.process_command(cmd, &mut gimbal);
        assert!(result.is_ok());
        assert!(result.unwrap());

        // Verify state was updated
        let state = state_manager.get_state();
        assert_eq!(state.yaw, 10.0);
        assert_eq!(state.pitch, 20.0);
        assert_eq!(state.roll, 30.0);
    }

    #[test]
    fn test_priority_enforcement() {
        let state_manager = StateManager::new();
        let arbitrator = Arbitrator::new(state_manager.clone());
        let mut gimbal: Box<dyn GimbalDevice> = Box::new(MockGimbal::new());

        // High priority command
        let cmd1 = GimbalCommand::new(
            ControlSource::Mavlink(1),
            CommandMode::Position,
            Some(10.0),
            Some(10.0),
            Some(10.0),
        );
        assert!(arbitrator.process_command(cmd1, &mut gimbal).unwrap());

        // Low priority command should be rejected
        let cmd2 = GimbalCommand::new(
            ControlSource::Cli,
            CommandMode::Position,
            Some(20.0),
            Some(20.0),
            Some(20.0),
        );
        assert!(!arbitrator.process_command(cmd2, &mut gimbal).unwrap());

        // State should still be from cmd1
        let state = state_manager.get_state();
        assert_eq!(state.yaw, 10.0);
    }
}