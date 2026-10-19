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

#[test]
fn arbitrary_single_byte_is_not_an_acknowledgement() {
    assert!(RCMessage::parse_response(&[0x00]).is_err());
    assert!(RCMessage::parse_response(&[0x4A]).is_ok());
}
