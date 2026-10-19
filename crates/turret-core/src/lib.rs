//! Allocation-free payload semantics and device-side command execution state.
#![no_std]

pub mod capability;
pub mod command;
pub mod error;
pub mod executor;
pub mod observation;
pub mod units;
pub mod zoom;

pub use capability::*;
pub use command::*;
pub use error::Error;
pub use executor::{Executor, SendOutcome, WriteFailure};
pub use observation::*;
pub use units::*;
pub use zoom::*;
