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

#[test]
fn other_measurement_field_cannot_be_interpreted_as_imu_angles() {
    let mut payload = build_imu1_payload(10.0, 20.0, 30.0);
    payload[0] = 0x10;
    assert!(parse_imu1_angles_payload(&payload).is_err());
}
