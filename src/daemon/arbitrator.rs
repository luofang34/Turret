//! Priority-aware command arbitrator: serializes IPC / MAVLink commands
//! onto the single gimbal handle and rejects lower-priority commands
//! when a higher-priority one is still in flight.
//!
//! The arbitrator no longer touches the gimbal hardware itself — that's
//! [`super::gimbal_handle::GimbalHandle`]'s job, which runs the actual
//! I/O via `spawn_blocking`. Callers receive an [`Outcome`] from
//! [`Arbitrator::arbitrate`] telling them whether to do nothing
//! (`Rejected`) or to invoke `gimbal.set_attitude(...)` with the
//! decoded `(pitch, roll, yaw)`.

use super::models::GimbalCommand;
use super::state::StateManager;
use tracing::{debug, info};

/// What the caller should do after asking the arbitrator about a
/// command. Decoupled from the gimbal handle so the arbitrator stays
/// pure (no I/O) and unit-testable without a mock device.
///
/// `Execute` field order matches [`GimbalCommand`]'s field order and
/// [`crate::gimbal::GimbalDevice::set_attitude`] — same `(pitch, roll,
/// yaw)` shape all the way through.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Outcome {
    /// Caller should call `gimbal.set_attitude(pitch, roll, yaw)`.
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
        fields(source = ?command.source)
    )]
    pub fn arbitrate(&self, command: GimbalCommand) -> Outcome {
        // Capture the per-axis baseline *before* set_command overwrites
        // last_command with the (potentially partial) new one. A `None`
        // axis on the incoming command means "leave this axis where I
        // last set it" — resolved against the *previous* last_command,
        // not the post-update view that would see our own None.
        // See `StateManager::axis_baseline` for the resolution order
        // (last_command → measured → 0).
        let (base_pitch, base_roll, base_yaw) = self.state_manager.axis_baseline();

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

        // None on an axis = "leave alone" per the GimbalCommand
        // contract. Resolve against the captured baseline rather than
        // collapsing to 0.0 — a partial command (e.g. IPC `pitch 10`
        // or a yaw-only MAVLink SET_PITCHYAW) must not silently snap
        // unmentioned axes to center.
        Outcome::Execute {
            pitch: command.pitch.unwrap_or(base_pitch),
            roll: command.roll.unwrap_or(base_roll),
            yaw: command.yaw.unwrap_or(base_yaw),
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;
    use crate::daemon::models::{ControlSource, GimbalCommand};

    #[test]
    fn position_command_decodes_to_execute() {
        let state_manager = StateManager::new();
        let arbitrator = Arbitrator::new(state_manager.clone());

        let cmd = GimbalCommand::new(
            ControlSource::UnixSocket(0),
            Some(20.0), // pitch
            Some(30.0), // roll
            Some(10.0), // yaw
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
        assert_eq!(last.pitch, Some(20.0));
        assert_eq!(last.roll, Some(30.0));
        assert_eq!(last.yaw, Some(10.0));

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
            Some(10.0),
            Some(10.0),
            Some(10.0),
        );
        assert!(matches!(
            arbitrator.arbitrate(cmd1),
            Outcome::Execute { .. }
        ));

        let cmd2 = GimbalCommand::new(
            ControlSource::UnixSocket(0),
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
    fn none_axis_resolves_to_baseline_not_zero() {
        // A partial command (only some axes commanded) must keep the
        // omitted axes at their last commanded value, not snap them to
        // 0°. Regression for the silent `unwrap_or(0.0)` that used to
        // sit in the Position branch.
        let state_manager = StateManager::new();
        let arbitrator = Arbitrator::new(state_manager.clone());

        // Prime last_command with a full triple at higher priority than
        // the partial command below — otherwise the partial would lose
        // arbitration and never reach the Position branch.
        let prime = GimbalCommand::new(
            ControlSource::Mavlink(1),
            Some(10.0), // pitch
            Some(15.0), // roll
            Some(20.0), // yaw
        );
        assert!(matches!(
            arbitrator.arbitrate(prime),
            Outcome::Execute { .. }
        ));

        // Same source, a yaw-only follow-up. None on pitch/roll must
        // resolve to the previously commanded 10 / 15, not 0.
        let yaw_only = GimbalCommand::new(
            ControlSource::Mavlink(1),
            None,       // pitch — leave alone
            None,       // roll — leave alone
            Some(30.0), // yaw — changed
        );
        match arbitrator.arbitrate(yaw_only) {
            Outcome::Execute { pitch, roll, yaw } => {
                assert_eq!(yaw, 30.0);
                assert_eq!(pitch, 10.0, "pitch must preserve baseline, not snap to 0");
                assert_eq!(roll, 15.0, "roll must preserve baseline, not snap to 0");
            }
            other => panic!("expected Execute, got {other:?}"),
        }
    }

    #[test]
    fn none_axis_at_cold_boot_falls_back_to_zero() {
        // No prior command and no measurement yet — None still has to
        // resolve to *something* for the gimbal call. The final-fallback
        // 0.0 from `axis_baseline` is the right answer here (cold boot,
        // first-ever command, nothing to preserve). Documents the
        // boundary so a future refactor doesn't accidentally start
        // panicking on unresolved None.
        let state_manager = StateManager::new();
        let arbitrator = Arbitrator::new(state_manager);

        let cmd = GimbalCommand::new(
            ControlSource::UnixSocket(0),
            None,
            None,
            Some(15.0), // yaw — only this axis commanded
        );
        match arbitrator.arbitrate(cmd) {
            Outcome::Execute { pitch, roll, yaw } => {
                assert_eq!(yaw, 15.0);
                assert_eq!(pitch, 0.0);
                assert_eq!(roll, 0.0);
            }
            other => panic!("expected Execute, got {other:?}"),
        }
    }
}
