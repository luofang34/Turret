//! Typed command wrappers around [`Storm32Gimbal::send_rc_command`].
//!
//! Each method here builds an [`RCMessage`], hands it to the I/O kernel
//! in [`super::driver`], and decodes the response into a typed result.
//! The kernel itself, the validation helpers, and the connection
//! lifecycle live next door — this file is purely "STorM32 RC command
//! → struct field" plumbing.

use super::driver::{GimbalAngles, Storm32Gimbal};
use super::error::{Result, Storm32Error};
use super::frame::{RCCommand, RCMessage};
use tracing::debug;

impl Storm32Gimbal {
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

    /// `CMD_SETPITCH` — set pitch axis to a single RC value
    /// (`700..=2300`, or 0 to recenter). Other axes left untouched.
    pub fn set_pitch(&mut self, value: u16) -> Result<()> {
        Self::validate_rc_value(value, "Pitch")?;
        let payload = RCMessage::create_single_axis_payload(value);
        let message = RCMessage::new(RCCommand::SetPitch, payload);
        self.send_rc_command(&message)?;
        Ok(())
    }

    /// `CMD_SETROLL` — see [`Self::set_pitch`] for the value semantics.
    pub fn set_roll(&mut self, value: u16) -> Result<()> {
        Self::validate_rc_value(value, "Roll")?;
        let payload = RCMessage::create_single_axis_payload(value);
        let message = RCMessage::new(RCCommand::SetRoll, payload);
        self.send_rc_command(&message)?;
        Ok(())
    }

    /// `CMD_SETYAW` — see [`Self::set_pitch`] for the value semantics.
    pub fn set_yaw(&mut self, value: u16) -> Result<()> {
        Self::validate_rc_value(value, "Yaw")?;
        let payload = RCMessage::create_single_axis_payload(value);
        let message = RCMessage::new(RCCommand::SetYaw, payload);
        self.send_rc_command(&message)?;
        Ok(())
    }

    /// Send `CMD_SETPITCH 0` — Storm32 interprets this as "recenter".
    pub fn recenter_pitch(&mut self) -> Result<()> {
        self.set_pitch(0)
    }

    /// Send `CMD_SETROLL 0` — Storm32 interprets this as "recenter".
    pub fn recenter_roll(&mut self) -> Result<()> {
        self.set_roll(0)
    }

    /// Send `CMD_SETYAW 0` — Storm32 interprets this as "recenter".
    pub fn recenter_yaw(&mut self) -> Result<()> {
        self.set_yaw(0)
    }

    /// Refresh `current_status.pv` using `CMD_GETDATAFIELDS`.
    ///
    /// Errors from `send_rc_command` (I/O failure, no response, garbage
    /// on the line) propagate to the caller. Earlier this method
    /// swallowed those errors with a `warn!` and returned `Ok(())`,
    /// which broke the daemon's hot-unplug detection: the attitude
    /// poll loop counts only `Err` toward `consecutive_failures`, so a
    /// silent serial bus stayed silent — `notify_lost` never fired,
    /// the reconnect supervisor never woke, and downstream consumers
    /// (IPC `status`, MAVLink `GIMBAL_DEVICE_ATTITUDE_STATUS`) kept
    /// broadcasting the last cached `pv` as fresh attitude with no
    /// `COMMS_ERROR` flag. The cache is left untouched on the error
    /// path, so a `get_status()`-only consumer can still inspect the
    /// last good reading after a failed refresh.
    pub fn update_status(&mut self) -> Result<()> {
        // Request IMU1 angles (0x0020 = LIVEDATA_IMU1ANGLES).
        let bitmask = 0x0020u16;
        let payload = RCMessage::create_get_data_fields_payload(bitmask);
        let message = RCMessage::new(RCCommand::GetDataFields, payload);

        let response = self.send_rc_command(&message)?;
        if response.is_empty() {
            // The gimbal accepted the request but had nothing to
            // report (firmware that doesn't implement GETDATAFIELDS,
            // or a known-empty bitmask reply). Not a transport
            // failure — leave the cached `pv` in place so an
            // immediately-following `get_status()` returns the last
            // good reading rather than zeros.
            debug!("GetDataFields returned empty response (possibly unsupported)");
            return Ok(());
        }
        self.current_status.pv = parse_imu1_angles_payload(&response)?;
        Ok(())
    }

    /// `CMD_SETPANMODE` — `mode` is the 0..=5 pan-mode byte from the
    /// STorM32 RC spec (see `daemon::models::PanMode` when the `daemon`
    /// feature is enabled). Anything outside that range is rejected with
    /// [`Storm32Error::ValidationError`].
    pub fn set_pan_mode(&mut self, mode: u8) -> Result<()> {
        Self::validate_pan_mode(mode)?;
        let payload = RCMessage::create_set_pan_mode_payload(mode);
        let message = RCMessage::new(RCCommand::SetPanMode, payload);
        self.send_rc_command(&message)?;
        Ok(())
    }

    /// `CMD_SETSTANDBY` — toggle the gimbal between active and standby.
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
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;

    fn build_imu1_payload(pitch_deg: f32, roll_deg: f32, yaw_deg: f32) -> Vec<u8> {
        let mut payload = Vec::with_capacity(8);
        payload.extend_from_slice(&0x0020u16.to_le_bytes());
        payload.extend_from_slice(&((pitch_deg * 100.0) as i16).to_le_bytes());
        payload.extend_from_slice(&((roll_deg * 100.0) as i16).to_le_bytes());
        payload.extend_from_slice(&((yaw_deg * 100.0) as i16).to_le_bytes());
        payload
    }

    #[test]
    fn parse_imu1_angles_round_trips_three_axes() {
        let payload = build_imu1_payload(10.0, -5.0, 42.0);
        let angles = parse_imu1_angles_payload(&payload).unwrap();
        assert!((angles.pitch - 10.0).abs() < 0.01);
        assert!((angles.roll - -5.0).abs() < 0.01);
        assert!((angles.yaw - 42.0).abs() < 0.01);
    }

    #[test]
    fn parse_imu1_angles_handles_negative_yaw() {
        let payload = build_imu1_payload(0.0, 0.0, -179.99);
        let angles = parse_imu1_angles_payload(&payload).unwrap();
        assert!((angles.yaw - -179.99).abs() < 0.01);
    }

    #[test]
    fn parse_imu1_angles_rejects_short_payload() {
        // Old (buggy) layout was 7 bytes; reject it explicitly so a
        // regression to the off-by-one decode can't slip back in.
        let short = vec![0u8; 7];
        let err = parse_imu1_angles_payload(&short).unwrap_err();
        let msg = err.to_string();
        assert!(msg.contains("IMU1ANGLES"), "unexpected error: {msg}");
    }
}
