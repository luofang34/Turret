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
///
/// On failure, distinguishes "device path is held by another process"
/// (typically a `turret --daemon` running locally — returns
/// [`Error::DeviceBusy`]) from "no supported protocol responded"
/// (returns [`Error::NotDetected`]). The two need different fixes from
/// the operator, so collapsing them was the visible UX bug that drove
/// this distinction.
pub fn detect_gimbal(device_path: &str) -> Result<Box<dyn GimbalDevice>> {
    use crate::protocols::storm32_rc::Storm32RC;

    match Storm32RC::try_detect(device_path) {
        Ok(gimbal) => Ok(Box::new(gimbal)),
        Err(e) => {
            if is_resource_busy(&e) {
                Err(Error::DeviceBusy {
                    device: device_path.to_string(),
                })
            } else {
                Err(Error::NotDetected {
                    device: device_path.to_string(),
                    tried: "STorM32 RC".to_string(),
                })
            }
        }
    }
}

/// Walk the error chain looking for an `io::Error` with kind
/// [`std::io::ErrorKind::ResourceBusy`] — the EBUSY that
/// `serialport-rs` raises when the device is held exclusively by
/// another process. Used by [`detect_gimbal`] to surface
/// [`Error::DeviceBusy`] instead of the generic
/// [`Error::NotDetected`] in that specific case.
fn is_resource_busy(err: &Error) -> bool {
    // `dyn std::error::Error` source chain. The actual error is a few
    // layers down: Error::Storm32(Storm32Error::SerialError(serialport::Error{ kind, .. }))
    // → wraps io::Error in some versions of serialport. Walk until we
    // find an io::Error and check its kind.
    let mut current: Option<&(dyn std::error::Error + 'static)> = Some(err);
    while let Some(e) = current {
        if let Some(io_err) = e.downcast_ref::<std::io::Error>() {
            if io_err.kind() == std::io::ErrorKind::ResourceBusy {
                return true;
            }
        }
        // serialport::Error has its own Kind enum that signals
        // "InvalidInput" / "NoDevice" / "Unknown"; the EBUSY case
        // surfaces as Unknown but with the wrapped io::Error
        // distinguishable. Conservative belt-and-braces: also peek at
        // the Display string for the common "Device or resource busy"
        // text — `serialport-rs` includes it on Linux even when the
        // ErrorKind has been remapped.
        let display = e.to_string();
        if display.contains("Device or resource busy") || display.contains("Resource busy") {
            return true;
        }
        current = e.source();
    }
    false
}
