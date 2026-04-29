//! Wire-protocol drivers that implement [`crate::gimbal::GimbalDevice`].
//!
//! Each backend lives under its own submodule. The trait boundary in
//! [`crate::gimbal`] is what the rest of the crate consumes; backends in
//! this module only need to be `pub` so that downstream crates can name
//! them when they want protocol-specific extensions via
//! [`crate::gimbal::GimbalDevice::as_any_mut`].

#[path = "protocols/storm32_rc.rs"]
pub mod storm32_rc;
