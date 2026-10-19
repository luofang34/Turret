//! Host payload drivers and services built on the allocation-free Turret contract.

#[cfg(feature = "daemon")]
pub mod daemon;
pub mod device_scanner;
pub mod error;
pub mod gimbal;
pub mod protocols;

pub use error::{Error, Result};
pub use gimbal::{
    detect_gimbal_blocking, Attitude, DeviceObservation, DeviceReceipt, GimbalDevice,
};
pub use turret_core;
