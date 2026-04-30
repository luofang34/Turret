//! Error type for the STorM32 RC driver.

use thiserror::Error as ThisError;

/// Failure modes produced by the STorM32 RC driver.
#[derive(Debug, ThisError)]
pub enum Storm32Error {
    /// `serialport` failed to open or configure the port (permissions,
    /// nonexistent device, conflicting settings).
    #[error("Serial port error: {0}")]
    SerialError(#[from] serialport::Error),

    /// Underlying read / write on the open port failed (timeout, EIO,
    /// device disappeared mid-exchange).
    #[error("I/O error: {0}")]
    IoError(#[from] std::io::Error),

    /// Wire-format violation: framing bytes out of place, payload too
    /// short, unexpected response to a command.
    #[error("Invalid response: {0}")]
    ProtocolError(String),

    /// Inbound frame's CRC didn't match the value computed locally.
    /// Carries the bad CRC so logs can correlate against test fixtures.
    #[error("CRC check failed: expected valid CRC, got 0x{0:04X}")]
    CrcError(u16),

    /// Caller passed a value the driver rejected before opening the port
    /// (out-of-range angle, bad pan-mode byte, etc.).
    #[error("Invalid input: {0}")]
    ValidationError(String),

    /// Device-path-level failure: missing file, not a character device,
    /// path-traversal pattern, etc.
    #[error("Device error: {0}")]
    DeviceError(String),
}

/// Convenience alias for driver-local results.
pub type Result<T> = std::result::Result<T, Storm32Error>;
