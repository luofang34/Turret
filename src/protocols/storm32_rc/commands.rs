//! Typed command wrappers around [`Storm32Gimbal::send_rc_command_blocking`].
//!
//! Each method here builds an [`RCMessage`], hands it to the I/O kernel
//! in [`super::driver`], and decodes the response into a typed result.
//! The kernel itself, the validation helpers, and the connection
//! lifecycle live next door — this file is purely "STorM32 RC command
//! → struct field" plumbing.

use super::driver::{GimbalAngles, Storm32Gimbal};
use super::error::{Result, Storm32Error};
use super::frame::{RCCommand, RCMessage};

impl Storm32Gimbal {
    /// Set gimbal angles using `CMD_SETANGLE` (float precision).
    pub fn set_angles_blocking(&mut self, pitch: f32, roll: f32, yaw: f32) -> Result<()> {
        Self::validate_angle(pitch, "Pitch")?;
        Self::validate_angle(roll, "Roll")?;
        Self::validate_angle(yaw, "Yaw")?;

        let payload = RCMessage::create_set_angle_payload(pitch, roll, yaw);
        let message = RCMessage::new(RCCommand::SetAngle, payload);

        self.send_rc_command_blocking(&message)?;
        self.current_status.sp = Some(GimbalAngles { pitch, roll, yaw });
        Ok(())
    }

    /// Set gimbal angles using `CMD_SETPITCHROLLYAW` (RC values 700-2300).
    pub fn set_angles_rc_blocking(&mut self, pitch: u16, roll: u16, yaw: u16) -> Result<()> {
        let payload = RCMessage::create_set_pitch_roll_yaw_payload(pitch, roll, yaw);
        let message = RCMessage::new(RCCommand::SetPitchRollYaw, payload);
        self.send_rc_command_blocking(&message)?;
        Ok(())
    }

    /// `CMD_SETPITCH` — set pitch axis to a single RC value
    /// (`700..=2300`, or 0 to recenter). Other axes left untouched.
    pub fn set_pitch_blocking(&mut self, value: u16) -> Result<()> {
        Self::validate_rc_value(value, "Pitch")?;
        let payload = RCMessage::create_single_axis_payload(value);
        let message = RCMessage::new(RCCommand::SetPitch, payload);
        self.send_rc_command_blocking(&message)?;
        Ok(())
    }

    /// `CMD_SETROLL` — see [`Self::set_pitch_blocking`] for the value semantics.
    pub fn set_roll_blocking(&mut self, value: u16) -> Result<()> {
        Self::validate_rc_value(value, "Roll")?;
        let payload = RCMessage::create_single_axis_payload(value);
        let message = RCMessage::new(RCCommand::SetRoll, payload);
        self.send_rc_command_blocking(&message)?;
        Ok(())
    }

    /// `CMD_SETYAW` — see [`Self::set_pitch_blocking`] for the value semantics.
    pub fn set_yaw_blocking(&mut self, value: u16) -> Result<()> {
        Self::validate_rc_value(value, "Yaw")?;
        let payload = RCMessage::create_single_axis_payload(value);
        let message = RCMessage::new(RCCommand::SetYaw, payload);
        self.send_rc_command_blocking(&message)?;
        Ok(())
    }

    /// Send `CMD_SETPITCH 0` — Storm32 interprets this as "recenter".
    pub fn recenter_pitch_blocking(&mut self) -> Result<()> {
        self.set_pitch_blocking(0)
    }

    /// Send `CMD_SETROLL 0` — Storm32 interprets this as "recenter".
    pub fn recenter_roll_blocking(&mut self) -> Result<()> {
        self.set_roll_blocking(0)
    }

    /// Send `CMD_SETYAW 0` — Storm32 interprets this as "recenter".
    pub fn recenter_yaw_blocking(&mut self) -> Result<()> {
        self.set_yaw_blocking(0)
    }

    /// Refresh measured attitude. Failure preserves the cached sample without
    /// treating it as a new observation.
    pub fn update_status_blocking(&mut self) -> Result<()> {
        // Request IMU1 angles (0x0020 = LIVEDATA_IMU1ANGLES).
        let bitmask = 0x0020u16;
        let payload = RCMessage::create_get_data_fields_payload(bitmask);
        let message = RCMessage::new(RCCommand::GetDataFields, payload);

        let response = self.send_rc_command_blocking(&message)?;
        if response.is_empty() {
            return Err(Storm32Error::ProtocolError(
                "GETDATAFIELDS returned no measurement".to_string(),
            ));
        }
        self.current_status.pv = Some(parse_imu1_angles_payload(&response)?);
        Ok(())
    }

    /// `CMD_SETPANMODE` — `mode` is the 0..=5 pan-mode byte from the
    /// STorM32 RC spec (see `daemon::models::PanMode` when the `daemon`
    /// feature is enabled). Anything outside that range is rejected with
    /// [`Storm32Error::ValidationError`].
    pub fn set_pan_mode_blocking(&mut self, mode: u8) -> Result<()> {
        Self::validate_pan_mode(mode)?;
        let payload = RCMessage::create_set_pan_mode_payload(mode);
        let message = RCMessage::new(RCCommand::SetPanMode, payload);
        self.send_rc_command_blocking(&message)?;
        Ok(())
    }

    /// `CMD_SETSTANDBY` — toggle the gimbal between active and standby.
    pub fn set_standby_blocking(&mut self, enabled: bool) -> Result<()> {
        let state = if enabled { 1u8 } else { 0u8 };
        let payload = RCMessage::create_set_standby_payload(state);
        let message = RCMessage::new(RCCommand::SetStandby, payload);
        self.send_rc_command_blocking(&message)?;
        Ok(())
    }

    /// Decoded firmware version string via `CMD_GETVERSION`.
    pub fn get_version_blocking(&mut self) -> Result<String> {
        let message = RCMessage::new(RCCommand::GetVersion, Vec::new());

        let response = self.send_rc_command_blocking(&message)?;
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
    pub fn get_version_string_blocking(&mut self) -> Result<String> {
        let message = RCMessage::new(RCCommand::GetVersionStr, Vec::new());

        let response = self.send_rc_command_blocking(&message)?;
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
}

/// Decode a `CMD_GETDATAFIELDS` payload that was requested with bitmask
/// `0x0020` (LIVEDATA_IMU1ANGLES).
///
/// `response` is the bytes returned by [`super::frame::RCMessage::parse_response`],
/// i.e. the framing (start, length, command echo, CRC) is already stripped.
/// Layout: `bitmask(2) + pitch_i16(2) + roll_i16(2) + yaw_i16(2)`, each axis
/// in units of `0.01°`.
fn parse_imu1_angles_payload(response: &[u8]) -> Result<GimbalAngles> {
    if response.len() < 8 {
        return Err(Storm32Error::ProtocolError(format!(
            "IMU1ANGLES payload too short: got {} bytes, need 8 \
             (bitmask + pitch + roll + yaw)",
            response.len()
        )));
    }
    let bitmask = u16::from_le_bytes([response[0], response[1]]);
    if bitmask != 0x0020 {
        return Err(Storm32Error::ProtocolError(format!(
            "IMU1ANGLES response field mask {bitmask:#06x} differs from requested 0x0020"
        )));
    }
    let pitch_raw = i16::from_le_bytes([response[2], response[3]]);
    let roll_raw = i16::from_le_bytes([response[4], response[5]]);
    let yaw_raw = i16::from_le_bytes([response[6], response[7]]);
    Ok(GimbalAngles {
        pitch: f32::from(pitch_raw) / 100.0,
        roll: f32::from(roll_raw) / 100.0,
        yaw: f32::from(yaw_raw) / 100.0,
    })
}

#[cfg(test)]
mod tests;
