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

#[path = "storm32_rc/driver.rs"]
mod driver;
#[path = "storm32_rc/error.rs"]
mod error;
#[path = "storm32_rc/frame.rs"]
mod frame;

pub use driver::{GimbalAngles, GimbalStatus, Storm32Gimbal, Storm32RC};
pub use error::{Result, Storm32Error};
pub use frame::{RCCommand, RCMessage};

impl crate::gimbal::GimbalDevice for Storm32Gimbal {
    fn protocol_name(&self) -> &'static str {
        "STorM32 RC Commands"
    }

    fn set_attitude(&mut self, pitch: f32, roll: f32, yaw: f32) -> crate::error::Result<()> {
        self.set_angles(pitch, roll, yaw)?;
        Ok(())
    }

    fn get_attitude(&mut self) -> crate::error::Result<crate::gimbal::Attitude> {
        self.update_status()?;
        let status = self.get_status();
        Ok(crate::gimbal::Attitude {
            pitch: status.pv.pitch,
            roll: status.pv.roll,
            yaw: status.pv.yaw,
        })
    }

    fn set_pan_mode(&mut self, mode: u8) -> crate::error::Result<()> {
        Storm32Gimbal::set_pan_mode(self, mode)?;
        Ok(())
    }

    fn set_standby(&mut self, enabled: bool) -> crate::error::Result<()> {
        Storm32Gimbal::set_standby(self, enabled)?;
        Ok(())
    }

    fn get_version(&mut self) -> crate::error::Result<String> {
        Ok(Storm32Gimbal::get_version(self)?)
    }

    fn as_any(&self) -> Option<&dyn std::any::Any> {
        Some(self)
    }

    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        Some(self)
    }
}
