//! Gimbal device abstraction and runtime auto-detection.

use crate::error::{Error, Result};

/// Gimbal attitude (Euler angles in degrees).
#[derive(Debug, Clone, Copy)]
pub struct Attitude {
    /// Pitch angle in degrees.
    pub pitch: f32,
    /// Roll angle in degrees.
    pub roll: f32,
    /// Yaw angle in degrees.
    pub yaw: f32,
}

/// Common interface for all gimbal devices.
///
/// Implementations are synchronous (serial I/O happens on the calling thread).
/// Callers that need async use should wrap in `spawn_blocking` or similar.
pub trait GimbalDevice: Send {
    /// Human-readable protocol name used by this backend.
    fn protocol_name(&self) -> &'static str;

    /// Command the gimbal to the given Euler attitude (degrees).
    fn set_attitude(&mut self, pitch: f32, roll: f32, yaw: f32) -> Result<()>;

    /// Query current attitude from the device.
    fn get_attitude(&mut self) -> Result<Attitude>;

    /// Command the gimbal to its center (0, 0, 0).
    fn center(&mut self) -> Result<()> {
        self.set_attitude(0.0, 0.0, 0.0)
    }

    /// Set pan mode (optional — default returns [`Error::Unsupported`]).
    fn set_pan_mode(&mut self, _mode: u8) -> Result<()> {
        Err(Error::Unsupported {
            protocol: self.protocol_name(),
            op: "set_pan_mode",
        })
    }

    /// Set standby mode (optional — default returns [`Error::Unsupported`]).
    fn set_standby(&mut self, _enabled: bool) -> Result<()> {
        Err(Error::Unsupported {
            protocol: self.protocol_name(),
            op: "set_standby",
        })
    }

    /// Get firmware version (optional — default returns [`Error::Unsupported`]).
    fn get_version(&mut self) -> Result<String> {
        Err(Error::Unsupported {
            protocol: self.protocol_name(),
            op: "get_version",
        })
    }

    /// Downcast hook for protocol-specific access. Implementors that want to
    /// expose extension APIs return `Some(self)`; the default is `None`.
    fn as_any(&self) -> Option<&dyn std::any::Any> {
        None
    }

    /// Mutable variant of [`GimbalDevice::as_any`].
    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        None
    }
}

/// Detect and initialize a gimbal on `device_path`, trying each supported
/// protocol in turn.
pub fn detect_gimbal(device_path: &str) -> Result<Box<dyn GimbalDevice>> {
    use crate::protocols::storm32_rc::Storm32RC;

    if let Ok(gimbal) = Storm32RC::try_detect(device_path) {
        return Ok(Box::new(gimbal));
    }

    Err(Error::NotDetected {
        device: device_path.to_string(),
        tried: "STorM32 RC".to_string(),
    })
}
