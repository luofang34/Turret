//! STorM32 RC driver: connection lifecycle + the [`Storm32Gimbal::send_rc_command_blocking`]
//! kernel that every command wrapper in [`super::commands`] funnels through.
//!
//! Anything user-facing — `set_angles_blocking`, `get_version_blocking`, `update_status_blocking`,
//! and friends — lives next door in `commands.rs`. Both files share access
//! to the `port` / `current_status` fields and the `send_rc_command_blocking`
//! method via `pub(super)` visibility, but those internals stay private
//! to the `storm32_rc` module.

use super::error::{Result, Storm32Error};
use super::frame::{RCMessage, RC_START_SIGN_OUT};
use serialport::SerialPort;
use std::io::{Read, Write};
use std::path::Path;
use std::time::{Duration, Instant};
use tracing::{trace, warn};

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

/// Retained measurement and locally transmitted target, independently optional.
#[derive(Debug, Clone, Copy)]
pub struct GimbalStatus {
    /// Current measured angles (process variable).
    pub pv: Option<GimbalAngles>,
    /// Last fully transmitted target; not an independently read device setpoint.
    pub sp: Option<GimbalAngles>,
}

/// STorM32 gimbal driver.
pub struct Storm32Gimbal {
    pub(super) port: Box<dyn SerialPort>,
    pub(super) current_status: GimbalStatus,
    pub(super) write_deadline: Option<Instant>,
    pub(super) last_rate_at: Option<Instant>,
    pub(super) last_acknowledged: bool,
    pub(super) last_motion_sent_at: Option<Instant>,
    pub(super) last_acknowledged_at: Option<Instant>,
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
    /// probe) is deferred to [`Storm32RC::try_detect_blocking`].
    pub fn open_blocking(device_path: &str) -> Result<Self> {
        Self::validate_device_path(device_path)?;

        let port = serialport::new(device_path, 115200)
            .timeout(Duration::from_millis(1000))
            .open()?;

        Ok(Self {
            port,
            current_status: GimbalStatus { pv: None, sp: None },
            write_deadline: None,
            last_rate_at: None,
            last_acknowledged: false,
            last_motion_sent_at: None,
            last_acknowledged_at: None,
        })
    }

    /// Snapshot of the most recent (process variable, set point) pair the
    /// driver has computed. Reading does NOT poll the device — see
    /// [`Self::update_status_blocking`] to refresh.
    pub fn get_status_cached(&self) -> GimbalStatus {
        self.current_status
    }

    /// Write one command before its deadline, then inspect the independent response.
    pub(super) fn send_rc_command_blocking(&mut self, message: &RCMessage) -> Result<Vec<u8>> {
        self.last_acknowledged = false;
        self.last_acknowledged_at = None;
        self.port.clear(serialport::ClearBuffer::Input)?;
        let bytes = message.serialize();
        if self
            .write_deadline
            .is_some_and(|deadline| Instant::now() >= deadline)
        {
            return Err(Storm32Error::ValidationError(
                "command expired before serial write".to_string(),
            ));
        }
        self.port.write_all(&bytes)?;
        self.port.flush()?;
        let motion = matches!(
            message.command,
            super::frame::RCCommand::SetAngle
                | super::frame::RCCommand::SetPitchRollYaw
                | super::frame::RCCommand::SetPitch
                | super::frame::RCCommand::SetRoll
                | super::frame::RCCommand::SetYaw
        );
        if motion {
            self.last_motion_sent_at = Some(Instant::now());
        }
        match self.read_response_blocking(message.command as u8) {
            Ok(bytes) => Ok(bytes),
            Err(error) if motion => {
                warn!(%error, "motion was written; device acknowledgement remains unknown");
                Ok(Vec::new())
            }
            Err(error) => Err(error),
        }
    }

    fn read_response_blocking(&mut self, expected_command: u8) -> Result<Vec<u8>> {
        const ACK_BYTE: u8 = 0x4A;
        let deadline = Instant::now() + Duration::from_millis(200);
        let mut buffer = [0_u8; 256];
        let mut count = 0;
        let mut restarts = 0_u32;
        while Instant::now() < deadline {
            self.port.set_timeout(
                deadline
                    .saturating_duration_since(Instant::now())
                    .min(Duration::from_millis(10)),
            )?;
            match self.port.read(&mut buffer[count..]) {
                Ok(0) => continue,
                Ok(read) => count += read,
                Err(error) if error.kind() == std::io::ErrorKind::TimedOut => continue,
                Err(error) => return Err(error.into()),
            }
            if buffer[0] == ACK_BYTE {
                self.last_acknowledged = true;
                self.last_acknowledged_at = Some(Instant::now());
                return Ok(Vec::new());
            }
            if buffer[0] != RC_START_SIGN_OUT {
                restarts = restarts.wrapping_add(1);
                if restarts > 3 {
                    return Err(Storm32Error::ProtocolError(
                        "serial response has no recognized frame boundary".to_string(),
                    ));
                }
                self.port.clear(serialport::ClearBuffer::Input)?;
                count = 0;
                continue;
            }
            if count < 2 {
                continue;
            }
            let expected = 5 + usize::from(buffer[1]);
            if expected > buffer.len() {
                return Err(Storm32Error::ProtocolError(format!(
                    "response length {expected} exceeds buffer {}",
                    buffer.len()
                )));
            }
            if count >= expected {
                if buffer[2] != expected_command {
                    return Err(Storm32Error::ProtocolError(format!(
                        "response command {} does not match request {expected_command}",
                        buffer[2]
                    )));
                }
                trace!("Received RC response: {:02X?}", &buffer[..expected]);
                return RCMessage::parse_response(&buffer[..expected]);
            }
        }
        Err(Storm32Error::ProtocolError(format!(
            "response deadline expired with {count} bytes"
        )))
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
    pub fn try_detect_blocking(device_path: &str) -> crate::error::Result<Self> {
        Self::validate_device_path(device_path)?;
        let mut gimbal = Self::open_blocking(device_path)?;

        // Version probe confirms this is really a STorM32.
        match gimbal.get_version_blocking() {
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
mod tests;
