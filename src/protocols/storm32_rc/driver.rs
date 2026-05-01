//! STorM32 RC driver: connection lifecycle + the [`Storm32Gimbal::send_rc_command`]
//! kernel that every command wrapper in [`super::commands`] funnels through.
//!
//! Anything user-facing — `set_angles`, `get_version`, `update_status`,
//! and friends — lives next door in `commands.rs`. Both files share access
//! to the `port` / `current_status` fields and the `send_rc_command`
//! method via `pub(super)` visibility, but those internals stay private
//! to the `storm32_rc` module.

use super::error::{Result, Storm32Error};
use super::frame::{RCMessage, RC_START_SIGN_OUT};
use serialport::SerialPort;
use std::io::{Read, Write};
use std::path::Path;
use std::time::Duration;
use tracing::{debug, trace};

/// Gimbal attitude reported by the STorM32 driver (Euler angles in degrees).
#[derive(Debug, Clone, Copy)]
pub struct GimbalAngles {
    /// Pitch axis, in degrees.
    pub pitch: f32,
    /// Roll axis, in degrees.
    pub roll: f32,
    /// Yaw axis, in degrees.
    pub yaw: f32,
}

/// Gimbal process-variable / set-point pair.
#[derive(Debug, Clone, Copy)]
pub struct GimbalStatus {
    /// Current measured angles (process variable).
    pub pv: GimbalAngles,
    /// Commanded target angles (set point).
    pub sp: GimbalAngles,
}

/// STorM32 gimbal driver.
pub struct Storm32Gimbal {
    pub(super) port: Box<dyn SerialPort>,
    pub(super) current_status: GimbalStatus,
}

impl Storm32Gimbal {
    // Reject path traversal, missing paths, and non-char-devices on Unix.
    pub(super) fn validate_device_path(device_path: &str) -> Result<()> {
        if device_path.contains("..") {
            return Err(Storm32Error::ValidationError(
                "Invalid device path: path traversal detected".to_string(),
            ));
        }

        let path = Path::new(device_path);
        if !path.exists() {
            return Err(Storm32Error::DeviceError(format!(
                "Device path does not exist: {}",
                device_path
            )));
        }

        #[cfg(unix)]
        {
            use std::os::unix::fs::FileTypeExt;
            let metadata = std::fs::metadata(path)?;
            let file_type = metadata.file_type();

            if !file_type.is_char_device() {
                return Err(Storm32Error::DeviceError(format!(
                    "Path is not a character device: {}",
                    device_path
                )));
            }
        }

        Ok(())
    }

    /// Open the given path at 115200 8N1 with a 1 s read timeout. The
    /// constructor only opens the port; protocol detection (a `CMD_GETVERSION`
    /// probe) is deferred to [`Storm32RC::try_detect`].
    pub fn new(device_path: &str) -> Result<Self> {
        Self::validate_device_path(device_path)?;

        let port = serialport::new(device_path, 115200)
            .timeout(Duration::from_millis(1000))
            .open()?;

        Ok(Self {
            port,
            current_status: GimbalStatus {
                pv: GimbalAngles {
                    pitch: 0.0,
                    roll: 0.0,
                    yaw: 0.0,
                },
                sp: GimbalAngles {
                    pitch: 0.0,
                    roll: 0.0,
                    yaw: 0.0,
                },
            },
        })
    }

    /// Snapshot of the most recent (process variable, set point) pair the
    /// driver has computed. Reading does NOT poll the device — see
    /// [`Self::update_status`] to refresh.
    pub fn get_status(&self) -> GimbalStatus {
        self.current_status
    }

    /// Send a framed RC command and read the framed response. Every typed
    /// command wrapper in `commands.rs` funnels through here so the
    /// "drop stale input → write → wait → read with garbage-byte recovery"
    /// pattern lives in exactly one place.
    #[tracing::instrument(level = "trace", skip(self, message), fields(cmd = ?message.command))]
    pub(super) fn send_rc_command(&mut self, message: &RCMessage) -> Result<Vec<u8>> {
        // Drop any stale input bytes so a previous incomplete response can't
        // poison this exchange.
        self.port.clear(serialport::ClearBuffer::Input)?;

        let data = message.serialize();
        trace!("Sending RC command: {:02X?}", data);
        self.port.write_all(&data)?;
        self.port.flush()?;

        // Short delay while the gimbal processes the command.
        std::thread::sleep(Duration::from_millis(5));

        let mut buffer = vec![0u8; 256];
        let mut total_bytes = 0;
        let max_attempts = 10;
        let mut attempts = 0;

        while attempts < max_attempts && total_bytes < 4 {
            match self.port.read(&mut buffer[total_bytes..]) {
                Ok(bytes_read) => {
                    if bytes_read > 0 {
                        total_bytes += bytes_read;

                        // First byte must be RC_START_SIGN_OUT (0xFB) or the
                        // 0x4A single-byte ACK — anything else is garbage
                        // from a previous exchange and we restart.
                        if total_bytes > 0 && buffer[0] != RC_START_SIGN_OUT && buffer[0] != 0x4A {
                            debug!(
                                "Received garbage data (wrong start byte 0x{:02X}), flushing buffer",
                                buffer[0]
                            );
                            self.port.clear(serialport::ClearBuffer::Input)?;
                            total_bytes = 0;
                            attempts += 1;
                            std::thread::sleep(Duration::from_millis(10));
                            continue;
                        }

                        if total_bytes >= 4 || (total_bytes == 1 && buffer[0] == 0x4A) {
                            break;
                        }
                    } else if total_bytes > 0 {
                        break;
                    }
                    attempts += 1;
                    std::thread::sleep(Duration::from_millis(10));
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {
                    if total_bytes > 0 {
                        break;
                    }
                    attempts += 1;
                    continue;
                }
                Err(e) => return Err(Storm32Error::IoError(e)),
            }
        }

        if total_bytes == 0 {
            return Err(Storm32Error::ProtocolError(
                "No response received".to_string(),
            ));
        }

        trace!("Received RC response: {:02X?}", &buffer[..total_bytes]);
        RCMessage::parse_response(&buffer[..total_bytes])
    }

    // Typical gimbal range is ±180°. Reject NaN / infinities and anything outside the range.
    pub(super) fn validate_angle(angle: f32, name: &str) -> Result<()> {
        if !angle.is_finite() {
            return Err(Storm32Error::ValidationError(format!(
                "{} must be a finite number",
                name
            )));
        }
        if !(-180.0..=180.0).contains(&angle) {
            return Err(Storm32Error::ValidationError(format!(
                "{} must be between -180° and +180° (got {:.1}°)",
                name, angle
            )));
        }
        Ok(())
    }

    pub(super) fn validate_rc_value(value: u16, name: &str) -> Result<()> {
        if value != 0 && !(700..=2300).contains(&value) {
            return Err(Storm32Error::ValidationError(format!(
                "{} must be 0 (recenter) or 700-2300 (got {})",
                name, value
            )));
        }
        Ok(())
    }

    pub(super) fn validate_pan_mode(mode: u8) -> Result<()> {
        if mode > 5 {
            return Err(Storm32Error::ValidationError(format!(
                "Pan mode must be 0-5 (got {})",
                mode
            )));
        }
        Ok(())
    }
}

/// Type alias for callers that prefer "Storm32RC" over "Storm32Gimbal".
pub type Storm32RC = Storm32Gimbal;

impl Storm32RC {
    /// Try to detect and initialize a STorM32 RC device on the given port.
    pub fn try_detect(device_path: &str) -> crate::error::Result<Self> {
        Self::validate_device_path(device_path)?;
        let mut gimbal = Self::new(device_path)?;

        // Version probe confirms this is really a STorM32.
        match gimbal.get_version() {
            Ok(version) => {
                tracing::info!("Detected STorM32 RC device: {}", version);
                Ok(gimbal)
            }
            Err(e) => {
                tracing::debug!("STorM32 RC detection failed: {}", e);
                Err(e.into())
            }
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;

    #[test]
    fn test_angle_validation() {
        assert!(Storm32Gimbal::validate_angle(0.0, "Test").is_ok());
        assert!(Storm32Gimbal::validate_angle(180.0, "Test").is_ok());
        assert!(Storm32Gimbal::validate_angle(-180.0, "Test").is_ok());
        assert!(Storm32Gimbal::validate_angle(45.5, "Test").is_ok());

        assert!(Storm32Gimbal::validate_angle(181.0, "Test").is_err());
        assert!(Storm32Gimbal::validate_angle(-181.0, "Test").is_err());
        assert!(Storm32Gimbal::validate_angle(f32::NAN, "Test").is_err());
        assert!(Storm32Gimbal::validate_angle(f32::INFINITY, "Test").is_err());
    }

    #[test]
    fn test_rc_value_validation() {
        assert!(Storm32Gimbal::validate_rc_value(0, "Test").is_ok());
        assert!(Storm32Gimbal::validate_rc_value(700, "Test").is_ok());
        assert!(Storm32Gimbal::validate_rc_value(1500, "Test").is_ok());
        assert!(Storm32Gimbal::validate_rc_value(2300, "Test").is_ok());

        assert!(Storm32Gimbal::validate_rc_value(699, "Test").is_err());
        assert!(Storm32Gimbal::validate_rc_value(2301, "Test").is_err());
        assert!(Storm32Gimbal::validate_rc_value(500, "Test").is_err());
    }

    #[test]
    fn test_pan_mode_validation() {
        for mode in 0..=5 {
            assert!(Storm32Gimbal::validate_pan_mode(mode).is_ok());
        }

        assert!(Storm32Gimbal::validate_pan_mode(6).is_err());
        assert!(Storm32Gimbal::validate_pan_mode(255).is_err());
    }

    #[test]
    fn test_device_path_validation() {
        assert!(Storm32Gimbal::validate_device_path("../etc/passwd").is_err());
        assert!(Storm32Gimbal::validate_device_path("/dev/../etc/passwd").is_err());
        assert!(Storm32Gimbal::validate_device_path("/dev/nonexistent123456").is_err());
    }
}
