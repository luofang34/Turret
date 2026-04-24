//! Turret — gimbal control and MAVLink Gimbal Manager.
//!
//! This crate is the library half of the `turret` package. The binary of the
//! same name (`src/main.rs`) is a thin CLI consumer of the API below.
//!
//! # Quick start
//!
//! ```no_run
//! # fn main() -> turret::Result<()> {
//! let mut gimbal = turret::detect_gimbal("/dev/ttyUSB0")?;
//! gimbal.set_attitude(10.0, 0.0, -15.0)?;
//! # Ok(()) }
//! ```
//!
//! # Public surface
//!
//! - [`GimbalDevice`] — trait implemented by every supported gimbal backend.
//! - [`detect_gimbal`] — probe a serial device for a supported protocol and
//!   return a boxed [`GimbalDevice`].
//! - [`Attitude`] — Euler angles returned by the device.
//! - [`Error`] / [`Result`] — typed errors at the library boundary.
//!
//! Lower-level modules ([`protocols`], [`daemon`], [`device_scanner`]) are
//! exposed for advanced integration but do not yet guarantee a stable API
//! surface. In particular, `daemon` currently returns `anyhow::Result`
//! internally; that will be migrated in a follow-up.

#![forbid(unsafe_code)]

pub mod daemon;
pub mod device_scanner;
pub mod error;
pub mod gimbal;
pub mod protocols;

pub use error::{Error, Result};
pub use gimbal::{detect_gimbal, Attitude, GimbalDevice};
