//! STorM32 RC protocol frame format, CRC, and payload builders.

use super::error::{Result, Storm32Error};
use log::{debug, trace};

/// Start sign for frames we send to the gimbal (`0xFA`).
const RC_START_SIGN_IN: u8 = 0xFA;

/// Start sign for frames the gimbal sends back (`0xFB`).
///
/// `pub(super)` because `driver::send_rc_command` uses this to detect stale
/// / garbage bytes on the serial line before handing bytes off to
/// [`RCMessage::parse_response`].
pub(super) const RC_START_SIGN_OUT: u8 = 0xFB;

/// Command IDs from the STorM32 RC Commands protocol.
#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum RCCommand {
    GetVersion = 1,             // CMD_GETVERSION
    GetVersionStr = 2,          // CMD_GETVERSIONSTR
    GetParameter = 3,           // CMD_GETPARAMETER
    SetParameter = 4,           // CMD_SETPARAMETER
    GetData = 5,                // CMD_GETDATA
    GetDataFields = 6,          // CMD_GETDATAFIELDS
    SetPitch = 10,              // CMD_SETPITCH
    SetRoll = 11,               // CMD_SETROLL
    SetYaw = 12,                // CMD_SETYAW
    SetPanMode = 13,            // CMD_SETPANMODE
    SetStandby = 14,            // CMD_SETSTANDBY
    DoCamera = 15,              // CMD_DOCAMERA
    SetScriptControl = 16,      // CMD_SETSCRIPTCONTROL
    SetAngle = 17,              // CMD_SETANGLE
    SetPitchRollYaw = 18,       // CMD_SETPITCHROLLYAW
    SetPwmOut = 19,             // CMD_SETPWMOUT
    RestoreParameter = 20,      // CMD_RESTOREPARAMETER
    RestoreAllParameter = 21,   // CMD_RESTOREALLPARAMETER
    ActivePanModeSetting = 100, // CMD_ACTIVEPANMODESETTING
    Ack = 150,                  // CMD_ACK
}

/// One framed request/response on the RC protocol.
#[derive(Debug)]
pub struct RCMessage {
    pub command: RCCommand,
    pub payload: Vec<u8>,
}

impl RCMessage {
    pub fn new(command: RCCommand, payload: Vec<u8>) -> Self {
        Self { command, payload }
    }

    /// Payload for `CMD_SETANGLE`: float pitch/roll/yaw plus flags and type.
    ///
    /// Format: `float(4) float(4) float(4) flags(1) type(1)` — 14 bytes.
    pub fn create_set_angle_payload(pitch: f32, roll: f32, yaw: f32) -> Vec<u8> {
        let mut payload = Vec::new();

        payload.extend_from_slice(&pitch.to_le_bytes());
        payload.extend_from_slice(&roll.to_le_bytes());
        payload.extend_from_slice(&yaw.to_le_bytes());

        // Flags: 0x00 = unlimited mode (bypasses RcMin/RcMax).
        // Bit 0x01 = pitch limited, 0x02 = roll limited, 0x04 = yaw limited.
        payload.push(0x00);

        // Type byte: unused.
        payload.push(0x00);

        payload
    }

    /// Payload for `CMD_SETPITCHROLLYAW`: three u16 RC values (700-2300, 0 = recenter).
    pub fn create_set_pitch_roll_yaw_payload(pitch: u16, roll: u16, yaw: u16) -> Vec<u8> {
        let mut payload = Vec::new();
        payload.extend_from_slice(&pitch.to_le_bytes());
        payload.extend_from_slice(&roll.to_le_bytes());
        payload.extend_from_slice(&yaw.to_le_bytes());
        payload
    }

    /// Payload for `CMD_SETPITCH` / `CMD_SETROLL` / `CMD_SETYAW`: single u16 value.
    pub fn create_single_axis_payload(value: u16) -> Vec<u8> {
        value.to_le_bytes().to_vec()
    }

    /// Payload for `CMD_GETDATAFIELDS`: 16-bit bitmask selecting which fields.
    pub fn create_get_data_fields_payload(bitmask: u16) -> Vec<u8> {
        bitmask.to_le_bytes().to_vec()
    }

    /// Payload for `CMD_SETPANMODE`: single mode byte (0-5).
    pub fn create_set_pan_mode_payload(mode: u8) -> Vec<u8> {
        vec![mode]
    }

    /// Payload for `CMD_SETSTANDBY`: single state byte (0 or 1).
    pub fn create_set_standby_payload(state: u8) -> Vec<u8> {
        vec![state]
    }

    /// Serialize to wire format: `start(1) length(1) command(1) payload(N) crc(2)`.
    pub fn serialize(&self) -> Vec<u8> {
        let mut message = Vec::new();
        message.push(RC_START_SIGN_IN);
        message.push(self.payload.len() as u8);
        message.push(self.command as u8);
        message.extend_from_slice(&self.payload);

        let crc = self.calculate_crc16(&message[1..]);
        message.extend_from_slice(&crc.to_le_bytes());

        message
    }

    /// CRC16 X.25 (same as MAVLink and the official STorM32 impl).
    fn calculate_crc16(&self, data: &[u8]) -> u16 {
        let mut crc: u16 = 0xFFFF;
        for &byte in data {
            let tmp = byte ^ (crc as u8);
            let tmp2 = tmp ^ (tmp << 4);
            crc = (crc >> 8) ^ ((tmp2 as u16) << 8) ^ ((tmp2 as u16) << 3) ^ ((tmp2 as u16) >> 4);
        }
        crc
    }

    fn verify_crc16(data: &[u8], expected_crc: u16) -> bool {
        let mut crc: u16 = 0xFFFF;
        for &byte in data {
            let tmp = byte ^ (crc as u8);
            let tmp2 = tmp ^ (tmp << 4);
            crc = (crc >> 8) ^ ((tmp2 as u16) << 8) ^ ((tmp2 as u16) << 3) ^ ((tmp2 as u16) >> 4);
        }
        crc == expected_crc
    }

    /// Parse an inbound frame from the gimbal. Returns the payload bytes.
    pub fn parse_response(data: &[u8]) -> Result<Vec<u8>> {
        // Single-byte ACK (`0x4A` etc.) — common for SET commands.
        if data.len() == 1 {
            return Ok(vec![]);
        }

        if data.len() < 3 {
            return Err(Storm32Error::ProtocolError(format!(
                "Response too short: got {} bytes, need at least 3",
                data.len()
            )));
        }

        if data[0] != RC_START_SIGN_OUT {
            return Err(Storm32Error::ProtocolError(format!(
                "Invalid start sign: expected 0xFB, got 0x{:02X}",
                data[0]
            )));
        }

        let length = data[1] as usize;
        // Total = start(1) + length(1) + command(1) + payload(length) + CRC(2).
        let expected_total = 1 + 1 + 1 + length + 2;

        if data.len() < expected_total {
            return Err(Storm32Error::ProtocolError(format!(
                "Incomplete message: got {} bytes, expected {}",
                data.len(),
                expected_total
            )));
        }

        let command_byte = data[2];
        let payload_start = 3;
        let payload_end = payload_start + length;

        let payload = if length > 0 {
            data[payload_start..payload_end].to_vec()
        } else {
            vec![]
        };

        let crc_start = payload_end;
        if crc_start + 2 <= data.len() {
            let received_crc = u16::from_le_bytes([data[crc_start], data[crc_start + 1]]);

            // CRC is calculated over length + command + payload bytes.
            let crc_data = &data[1..payload_end];

            if !Self::verify_crc16(crc_data, received_crc) {
                debug!(
                    "CRC verification failed: expected valid CRC, got 0x{:04X}",
                    received_crc
                );
                debug!("  Data for CRC: {:02X?}", crc_data);
                debug!(
                    "  Command byte: 0x{:02X}, Length: {}, Payload: {:02X?}",
                    command_byte, length, payload
                );
                return Err(Storm32Error::CrcError(received_crc));
            }

            trace!("CRC verification passed: 0x{:04X}", received_crc);
        }

        Ok(payload)
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;

    #[test]
    fn test_crc16_calculation() {
        let msg = RCMessage::new(RCCommand::GetVersion, Vec::new());
        let data = vec![0u8, 1u8]; // length=0, command=1
        let crc = msg.calculate_crc16(&data);
        assert_ne!(crc, 0);
    }

    #[test]
    fn test_crc16_verification() {
        let data = vec![0u8, 1u8];
        let msg = RCMessage::new(RCCommand::GetVersion, Vec::new());
        let crc = msg.calculate_crc16(&data);

        assert!(RCMessage::verify_crc16(&data, crc));
        assert!(!RCMessage::verify_crc16(&data, crc.wrapping_add(1)));
    }

    #[test]
    fn test_set_angle_payload() {
        let payload = RCMessage::create_set_angle_payload(10.0, 0.0, -15.0);

        // 14 bytes: 3 floats (12) + flags (1) + type (1)
        assert_eq!(payload.len(), 14);

        let pitch = f32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
        assert_eq!(pitch, 10.0);

        assert_eq!(payload[12], 0x00); // flags
        assert_eq!(payload[13], 0x00); // type
    }

    #[test]
    fn test_single_axis_payload() {
        let payload = RCMessage::create_single_axis_payload(1500);
        assert_eq!(payload.len(), 2);
        let value = u16::from_le_bytes([payload[0], payload[1]]);
        assert_eq!(value, 1500);
    }

    #[test]
    fn test_message_serialization() {
        let msg = RCMessage::new(RCCommand::GetVersion, Vec::new());
        let serialized = msg.serialize();

        assert_eq!(serialized[0], RC_START_SIGN_IN);
        assert_eq!(serialized[1], 0); // length
        assert_eq!(serialized[2], RCCommand::GetVersion as u8);

        // start + length + command + crc(2) = 5
        assert_eq!(serialized.len(), 5);
    }

    #[test]
    fn test_response_parsing_ack() {
        let result = RCMessage::parse_response(&[0x4A]);
        assert!(result.is_ok());
        assert_eq!(result.unwrap().len(), 0);
    }

    #[test]
    fn test_response_parsing_invalid() {
        // Too short (less than 3 bytes, not the single-byte ACK).
        assert!(RCMessage::parse_response(&[0xFB, 0x00]).is_err());

        // Invalid start sign.
        assert!(RCMessage::parse_response(&[0xFF, 0x00, 0x00]).is_err());
    }
}
