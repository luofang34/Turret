//! Library-boundary error type.
//!
//! Public API functions return `Result<T, Error>`. Internal modules may still
//! use `anyhow` (e.g. the daemon wiring); conversions at the boundary funnel
//! into `Error` variants here.

use thiserror::Error as ThisError;

use crate::protocols::storm32_rc::Storm32Error;

/// Errors returned by the public library API.
#[derive(Debug, ThisError)]
#[non_exhaustive]
pub enum Error {
    /// Failure from the STorM32 RC driver.
    #[error("storm32: {0}")]
    Storm32(#[from] Storm32Error),

    /// Serial port open / configuration failure.
    #[error("serial port: {0}")]
    Serial(#[from] serialport::Error),

    /// Underlying I/O failure.
    #[error("i/o: {0}")]
    Io(#[from] std::io::Error),

    /// No supported gimbal protocol was detected on the given device path.
    #[error("no supported gimbal protocol detected on {device}: tried {tried}")]
    NotDetected {
        /// Device path that was probed.
        device: String,
        /// Comma-separated list of protocols that were attempted.
        tried: String,
    },

    /// The requested operation is not supported by the active protocol.
    #[error("operation not supported by {protocol}: {op}")]
    Unsupported {
        /// Human-readable protocol name reported by the device.
        protocol: &'static str,
        /// Name of the operation that was attempted.
        op: &'static str,
    },

    /// Caller passed an argument that failed validation.
    #[error("invalid input: {0}")]
    InvalidInput(String),
}

/// Convenience alias for `Result<T, Error>`.
pub type Result<T> = std::result::Result<T, Error>;
