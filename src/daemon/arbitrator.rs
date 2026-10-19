//! Host authority admission, separate from device execution evidence.

use super::gimbal_handle::GimbalHandle;
use super::models::GimbalCommand;
use super::state::StateManager;
use crate::error::{Error, Result};
use turret_core::CommandProgress;

/// Host policy adapter; protocol source identifiers do not grant privilege.
pub struct Arbitrator {
    state_manager: StateManager,
}

impl Arbitrator {
    /// Share the authority and command-evidence stores.
    pub fn new(state_manager: StateManager) -> Self {
        Self { state_manager }
    }

    /// Admit, execute within the ingress deadline, then publish sent evidence.
    pub async fn execute(
        &self,
        command: GimbalCommand,
        gimbal: &GimbalHandle,
    ) -> Result<CommandProgress> {
        let payload = command.payload()?;
        if command.is_stale() || !self.state_manager.set_command(command.clone()) {
            return Err(Error::InvalidInput(
                "command rejected by authority, rate limit, or freshness".to_string(),
            ));
        }
        let progress = gimbal.execute(payload, command.timing()?).await?;
        if progress.sent_at.is_some() {
            self.state_manager.commands.record_sent(command);
        }
        Ok(progress)
    }
}

#[cfg(test)]
mod tests;
