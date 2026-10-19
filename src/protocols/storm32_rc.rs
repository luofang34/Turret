//! STorM32 RC Commands protocol driver.
//!
//! Internal layout (all submodules are private to this module):
//! - `error` — driver error type ([`Storm32Error`]) and local [`Result`] alias.
//! - `frame` — wire format: [`RCCommand`], [`RCMessage`], CRC, payload builders.
//! - `driver` — [`Storm32Gimbal`] driver (serial I/O, validation, command wrappers).
//!
//! This file hosts the bridge that implements [`crate::gimbal::GimbalDevice`]
//! for [`Storm32Gimbal`] so callers of the library trait never need to name
//! the submodules directly.

mod commands;
mod driver;
mod error;
mod frame;

pub use driver::{GimbalAngles, GimbalStatus, Storm32Gimbal, Storm32RC};
pub use error::{Result, Storm32Error};
pub use frame::{RCCommand, RCMessage};

mod capability;
mod payload;
pub use capability::storm32_capabilities;

/// Explicit protocol-specific operations outside the portable payload command set.
#[derive(Debug, Clone, Copy)]
pub enum Storm32Action {
    /// STorM32 RC pan-mode byte, validated against the device command domain.
    PanMode(u8),
    /// STorM32 stabilization standby state.
    Standby(bool),
}

impl Storm32Gimbal {
    /// Execute a protocol-specific action under the same actual-write deadline.
    pub fn execute_extension_blocking(
        &mut self,
        action: Storm32Action,
        send_before: std::time::Instant,
    ) -> crate::Result<()> {
        self.write_deadline = Some(send_before);
        let result = match action {
            Storm32Action::PanMode(mode) => self.set_pan_mode_blocking(mode),
            Storm32Action::Standby(enabled) => self.set_standby_blocking(enabled),
        };
        self.write_deadline = None;
        Ok(result?)
    }
}
