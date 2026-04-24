//! Error type for the STorM32 RC driver.

use thiserror::Error as ThisError;

/// Failure modes produced by the STorM32 RC driver.
#[derive(Debug, ThisError)]
pub enum Storm32Error {
    #[error("Serial port error: {0}")]
    SerialError(#[from] serialport::Error),

    #[error("I/O error: {0}")]
    IoError(#[from] std::io::Error),

    #[error("Invalid response: {0}")]
    ProtocolError(String),

    #[error("CRC check failed: expected valid CRC, got 0x{0:04X}")]
    CrcError(u16),

    #[error("Invalid input: {0}")]
    ValidationError(String),

    #[error("Device error: {0}")]
    DeviceError(String),
}

/// Convenience alias for driver-local results.
pub type Result<T> = std::result::Result<T, Storm32Error>;
