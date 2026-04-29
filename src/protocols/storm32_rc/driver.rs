//! STorM32 RC driver: serial I/O, validation, command wrappers, detection.

use super::error::{Result, Storm32Error};
use super::frame::{RCCommand, RCMessage, RC_START_SIGN_OUT};
use serialport::SerialPort;
use std::io::{Read, Write};
use std::path::Path;
use std::time::Duration;
use tracing::{debug, trace, warn};

/// Gimbal attitude reported by the STorM32 driver (Euler angles in degrees).
#[derive(Debug, Clone, Copy)]
pub struct GimbalAngles {
    pub pitch: f32,
    pub roll: f32,
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
    port: Box<dyn SerialPort>,
    current_status: GimbalStatus,
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

    pub fn get_status(&self) -> GimbalStatus {
        self.current_status
    }

    // Send an RC command and read the framed response.
    #[tracing::instrument(level = "trace", skip(self, message), fields(cmd = ?message.command))]
    fn send_rc_command(&mut self, message: &RCMessage) -> Result<Vec<u8>> {
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

    /// Set gimbal angles using `CMD_SETANGLE` (float precision).
    pub fn set_angles(&mut self, pitch: f32, roll: f32, yaw: f32) -> Result<()> {
        Self::validate_angle(pitch, "Pitch")?;
        Self::validate_angle(roll, "Roll")?;
        Self::validate_angle(yaw, "Yaw")?;

        let payload = RCMessage::create_set_angle_payload(pitch, roll, yaw);
        let message = RCMessage::new(RCCommand::SetAngle, payload);

        self.send_rc_command(&message)?;
        self.current_status.sp = GimbalAngles { pitch, roll, yaw };
        Ok(())
    }

    /// Set gimbal angles using `CMD_SETPITCHROLLYAW` (RC values 700-2300).
    pub fn set_angles_rc(&mut self, pitch: u16, roll: u16, yaw: u16) -> Result<()> {
        let payload = RCMessage::create_set_pitch_roll_yaw_payload(pitch, roll, yaw);
        let message = RCMessage::new(RCCommand::SetPitchRollYaw, payload);
        self.send_rc_command(&message)?;
        Ok(())
    }

    pub fn set_pitch(&mut self, value: u16) -> Result<()> {
        Self::validate_rc_value(value, "Pitch")?;
        let payload = RCMessage::create_single_axis_payload(value);
        let message = RCMessage::new(RCCommand::SetPitch, payload);
        self.send_rc_command(&message)?;
        Ok(())
    }

    pub fn set_roll(&mut self, value: u16) -> Result<()> {
        Self::validate_rc_value(value, "Roll")?;
        let payload = RCMessage::create_single_axis_payload(value);
        let message = RCMessage::new(RCCommand::SetRoll, payload);
        self.send_rc_command(&message)?;
        Ok(())
    }

    pub fn set_yaw(&mut self, value: u16) -> Result<()> {
        Self::validate_rc_value(value, "Yaw")?;
        let payload = RCMessage::create_single_axis_payload(value);
        let message = RCMessage::new(RCCommand::SetYaw, payload);
        self.send_rc_command(&message)?;
        Ok(())
    }

    pub fn recenter_pitch(&mut self) -> Result<()> {
        self.set_pitch(0)
    }

    pub fn recenter_roll(&mut self) -> Result<()> {
        self.set_roll(0)
    }

    pub fn recenter_yaw(&mut self) -> Result<()> {
        self.set_yaw(0)
    }

    /// Refresh `current_status.pv` using `CMD_GETDATAFIELDS`.
    pub fn update_status(&mut self) -> Result<()> {
        // Request IMU1 angles (0x0020 = LIVEDATA_IMU1ANGLES).
        let bitmask = 0x0020u16;
        let payload = RCMessage::create_get_data_fields_payload(bitmask);
        let message = RCMessage::new(RCCommand::GetDataFields, payload);

        match self.send_rc_command(&message) {
            Ok(response) => {
                if response.is_empty() {
                    debug!("GetDataFields command returned empty response (possibly unsupported)");
                    return Ok(());
                }
                self.parse_data_fields_response(&response)?;
                Ok(())
            }
            Err(e) => {
                // Losing current angles isn't fatal — log and move on.
                warn!("Could not update gimbal status: {}", e);
                Ok(())
            }
        }
    }

    fn parse_data_fields_response(&mut self, response: &[u8]) -> Result<()> {
        if response.len() < 7 {
            return Err(Storm32Error::ProtocolError(format!(
                "Data fields response too short: {} bytes",
                response.len()
            )));
        }

        // Response: [command_byte, bitmask_lo, bitmask_hi, data...]
        let _bitmask = u16::from_le_bytes([response[1], response[2]]);

        // For IMU1ANGLES (0x0020) we expect three i16 in 0.01° units. Only
        // pitch+roll are present in the 4 data bytes; yaw defaults to 0.
        if response.len() >= 7 {
            let pitch_raw = i16::from_le_bytes([response[3], response[4]]);
            let roll_raw = i16::from_le_bytes([response[5], response[6]]);
            let yaw_raw = 0i16;

            self.current_status.pv = GimbalAngles {
                pitch: pitch_raw as f32 / 100.0,
                roll: roll_raw as f32 / 100.0,
                yaw: yaw_raw as f32 / 100.0,
            };
        }

        Ok(())
    }

    pub fn set_pan_mode(&mut self, mode: u8) -> Result<()> {
        Self::validate_pan_mode(mode)?;
        let payload = RCMessage::create_set_pan_mode_payload(mode);
        let message = RCMessage::new(RCCommand::SetPanMode, payload);
        self.send_rc_command(&message)?;
        Ok(())
    }

    pub fn set_standby(&mut self, enabled: bool) -> Result<()> {
        let state = if enabled { 1u8 } else { 0u8 };
        let payload = RCMessage::create_set_standby_payload(state);
        let message = RCMessage::new(RCCommand::SetStandby, payload);
        self.send_rc_command(&message)?;
        Ok(())
    }

    /// Decoded firmware version string via `CMD_GETVERSION`.
    pub fn get_version(&mut self) -> Result<String> {
        let message = RCMessage::new(RCCommand::GetVersion, Vec::new());

        let response = self.send_rc_command(&message)?;
        if response.len() < 6 {
            return Err(Storm32Error::ProtocolError(
                "Version response too short".to_string(),
            ));
        }

        let firmware_version = u16::from_le_bytes([response[0], response[1]]);
        let layout_version = u16::from_le_bytes([response[2], response[3]]);
        let capabilities = u16::from_le_bytes([response[4], response[5]]);

        Ok(format!(
            "Firmware: {}.{}, Layout: {}.{}, Capabilities: 0x{:04X}",
            firmware_version >> 8,
            firmware_version & 0xFF,
            layout_version >> 8,
            layout_version & 0xFF,
            capabilities
        ))
    }

    /// Human-readable version / name / board strings via `CMD_GETVERSIONSTR`.
    pub fn get_version_string(&mut self) -> Result<String> {
        let message = RCMessage::new(RCCommand::GetVersionStr, Vec::new());

        let response = self.send_rc_command(&message)?;
        if response.len() < 48 {
            return Err(Storm32Error::ProtocolError(
                "Version string response too short".to_string(),
            ));
        }

        // Three 16-byte null-padded ASCII fields.
        let version_str = String::from_utf8_lossy(&response[0..16])
            .trim_end_matches('\0')
            .to_string();
        let name_str = String::from_utf8_lossy(&response[16..32])
            .trim_end_matches('\0')
            .to_string();
        let board_str = String::from_utf8_lossy(&response[32..48])
            .trim_end_matches('\0')
            .to_string();

        Ok(format!(
            "Version: '{}', Name: '{}', Board: '{}'",
            version_str, name_str, board_str
        ))
    }

    /// Test-only passthrough — send an arbitrary command and return the raw payload.
    #[allow(dead_code)]
    pub fn debug_send_raw(&mut self, command: RCCommand, payload: Vec<u8>) -> Result<Vec<u8>> {
        let message = RCMessage::new(command, payload);
        self.send_rc_command(&message)
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
