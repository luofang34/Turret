//! Priority-aware command arbitrator: serializes IPC / MAVLink / CLI
//! commands onto the single gimbal handle and rejects lower-priority
//! commands when a higher-priority one is still in flight.
//!
//! The arbitrator no longer touches the gimbal hardware itself — that's
//! [`super::gimbal_handle::GimbalHandle`]'s job, which runs the actual
//! I/O via `spawn_blocking`. Callers receive an [`Outcome`] from
//! [`Arbitrator::arbitrate`] telling them whether to do nothing
//! (`Rejected`) or to invoke `gimbal.set_attitude(...)` with the
//! decoded `(pitch, roll, yaw)`.

use super::models::{CommandMode, GimbalCommand};
use super::state::StateManager;
use tracing::{debug, info, warn};

/// What the caller should do after asking the arbitrator about a
/// command. Decoupled from the gimbal handle so the arbitrator stays
/// pure (no I/O) and unit-testable without a mock device.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Outcome {
    /// Caller should call `gimbal.set_attitude(pitch, roll, yaw)`.
    /// Order matches the `GimbalDevice::set_attitude` parameter order
    /// (NOT the `GimbalCommand::new` order, which puts yaw first).
    Execute {
        /// Pitch angle in degrees.
        pitch: f32,
        /// Roll angle in degrees.
        roll: f32,
        /// Yaw angle in degrees.
        yaw: f32,
    },
    /// Command rejected by priority arbitration or the per-source
    /// rate-limit floor — caller should not touch the gimbal.
    Rejected,
}

/// Command arbitrator. Decides whether an inbound command should
/// reach the gimbal and, if so, with what `(pitch, roll, yaw)`.
pub struct Arbitrator {
    state_manager: StateManager,
}

impl Arbitrator {
    /// Build an arbitrator that records accepted commands in the given
    /// shared `StateManager`.
    pub fn new(state_manager: StateManager) -> Self {
        Self { state_manager }
    }

    /// Run a command through priority + rate-limit arbitration. Returns
    /// `Outcome::Execute { ... }` on accept, `Outcome::Rejected`
    /// otherwise. The caller is responsible for the actual gimbal I/O —
    /// see [`super::gimbal_handle::GimbalHandle::set_attitude`].
    #[tracing::instrument(
        level = "debug",
        skip(self),
        fields(source = ?command.source, mode = ?command.mode)
    )]
    pub fn arbitrate(&self, command: GimbalCommand) -> Outcome {
        // Try to set command (checks priority + rate-limit).
        if !self.state_manager.set_command(command.clone()) {
            debug!(
                "Command rejected by state arbitration (source: {:?})",
                command.source
            );
            return Outcome::Rejected;
        }

        info!(
            "Accepted command from {:?} with priority {}",
            command.source,
            command.source.priority()
        );

        match command.mode {
            CommandMode::Position => Outcome::Execute {
                pitch: command.pitch.unwrap_or(0.0),
                roll: command.roll.unwrap_or(0.0),
                yaw: command.yaw.unwrap_or(0.0),
            },
            CommandMode::Rate => {
                // Rate path is converted to position at the inbound layer
                // before reaching the arbitrator (see `inbound::apply_rate_increment`).
                // A bare `CommandMode::Rate` here is a routing bug or an
                // IPC client constructing one directly — neither of which
                // we can sensibly execute.
                warn!("Rate-mode command reached arbitrator unmapped — rejecting");
                Outcome::Rejected
            }
            CommandMode::Manual => {
                warn!("Manual control mode not yet implemented — rejecting");
                Outcome::Rejected
            }
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;
    use crate::daemon::models::{CommandMode, ControlSource, GimbalCommand};

    #[test]
    fn position_command_decodes_to_execute() {
        let state_manager = StateManager::new();
        let arbitrator = Arbitrator::new(state_manager.clone());

        let cmd = GimbalCommand::new(
            ControlSource::Cli,
            CommandMode::Position,
            Some(10.0), // yaw
            Some(20.0), // pitch
            Some(30.0), // roll
        );

        match arbitrator.arbitrate(cmd) {
            Outcome::Execute { pitch, roll, yaw } => {
                assert_eq!(pitch, 20.0);
                assert_eq!(roll, 30.0);
                assert_eq!(yaw, 10.0);
            }
            other => panic!("expected Execute, got {other:?}"),
        }

        // Arbitrator records the command in `last_command` for
        // primary-control and observability …
        let last = state_manager.get_last_command().unwrap();
        assert_eq!(last.yaw, Some(10.0));
        assert_eq!(last.pitch, Some(20.0));
        assert_eq!(last.roll, Some(30.0));

        // … but does NOT touch state.{yaw,pitch,roll} — those are owned by
        // the attitude poll loop and stay at their default zero until a
        // poll succeeds.
        let state = state_manager.get_state();
        assert_eq!(state.yaw, 0.0);
        assert_eq!(state.pitch, 0.0);
        assert_eq!(state.roll, 0.0);
        assert!(state.measured_at.is_none());
    }

    #[test]
    fn lower_priority_command_is_rejected() {
        let state_manager = StateManager::new();
        let arbitrator = Arbitrator::new(state_manager.clone());

        let cmd1 = GimbalCommand::new(
            ControlSource::Mavlink(1),
            CommandMode::Position,
            Some(10.0),
            Some(10.0),
            Some(10.0),
        );
        assert!(matches!(
            arbitrator.arbitrate(cmd1),
            Outcome::Execute { .. }
        ));

        let cmd2 = GimbalCommand::new(
            ControlSource::Cli,
            CommandMode::Position,
            Some(20.0),
            Some(20.0),
            Some(20.0),
        );
        assert_eq!(arbitrator.arbitrate(cmd2), Outcome::Rejected);

        // The previously-accepted command stays as last_command.
        let last = state_manager.get_last_command().unwrap();
        assert_eq!(last.yaw, Some(10.0));
    }

    #[test]
    fn rate_mode_at_arbitrator_is_rejected() {
        // Rate-path commands are translated to Position at the inbound
        // layer; a bare CommandMode::Rate reaching here is a routing bug.
        let state_manager = StateManager::new();
        let arbitrator = Arbitrator::new(state_manager);

        let cmd = GimbalCommand::new(
            ControlSource::Cli,
            CommandMode::Rate,
            Some(0.0),
            Some(0.0),
            Some(0.0),
        );
        assert_eq!(arbitrator.arbitrate(cmd), Outcome::Rejected);
    }

    #[test]
    fn manual_mode_is_rejected() {
        let state_manager = StateManager::new();
        let arbitrator = Arbitrator::new(state_manager);

        let cmd = GimbalCommand::new(
            ControlSource::Cli,
            CommandMode::Manual,
            Some(0.0),
            Some(0.0),
            Some(0.0),
        );
        assert_eq!(arbitrator.arbitrate(cmd), Outcome::Rejected);
    }
}
