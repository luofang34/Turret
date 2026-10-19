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

mod serial;
use crate::gimbal::GimbalDevice;
use turret_core::{Angle, Axes, GimbalPosition, PayloadCommand, ReferenceFrame};

fn test_device(acknowledge: bool, fail_write: bool) -> Storm32Gimbal {
    Storm32Gimbal {
        port: Box::new(serial::Port {
            writes: Default::default(),
            acknowledge,
            fail_write,
            response: None,
        }),
        current_status: GimbalStatus { pv: None, sp: None },
        write_deadline: None,
        last_rate_at: None,
        last_acknowledged: false,
        last_motion_sent_at: None,
        last_acknowledged_at: None,
    }
}

fn full_position() -> PayloadCommand {
    PayloadCommand::GimbalPosition(GimbalPosition {
        angles: Axes {
            pitch: Some(Angle::ZERO),
            roll: Some(Angle::ZERO),
            yaw: Some(Angle::ZERO),
        },
        frame: ReferenceFrame::Vehicle,
    })
}

#[test]
fn written_motion_without_ack_retains_sent_target_but_no_measurement() {
    let mut device = test_device(false, false);
    let receipt = device
        .execute_blocking(full_position(), Instant::now() + Duration::from_secs(1))
        .unwrap();
    assert!(receipt.sent_at <= Instant::now());
    assert_eq!(receipt.acknowledged, None);
    assert_eq!(receipt.acknowledged_at, None);
    assert!(device.get_status_cached().sp.is_some());
    assert!(device.get_status_cached().pv.is_none());
}

#[test]
fn write_failure_does_not_create_a_sent_target() {
    let mut device = test_device(true, true);
    assert!(device
        .execute_blocking(full_position(), Instant::now() + Duration::from_secs(1))
        .is_err());
    assert!(device.last_motion_sent_at.is_none());
    assert!(device.get_status_cached().sp.is_none());
}

#[test]
fn native_recenter_does_not_invent_zero_setpoint() {
    let mut device = test_device(true, false);
    device
        .execute_blocking(full_position(), Instant::now() + Duration::from_secs(1))
        .unwrap();
    assert!(device.current_status.sp.is_some());
    let receipt = device
        .execute_blocking(
            PayloadCommand::Recenter,
            Instant::now() + Duration::from_secs(1),
        )
        .unwrap();
    assert_eq!(receipt.acknowledged, Some(true));
    assert!(receipt.acknowledged_at.unwrap() >= receipt.sent_at);
    assert!(device.current_status.sp.is_none());
    assert!(device.current_status.pv.is_none());
}

fn with_response(response: Vec<u8>) -> Storm32Gimbal {
    let mut device = test_device(true, false);
    device.port = Box::new(serial::Port {
        writes: Default::default(),
        acknowledge: true,
        fail_write: false,
        response: Some(response),
    });
    device
}

#[test]
fn delayed_different_command_reply_cannot_become_attitude() {
    let payload = vec![0x20, 0, 0xe8, 3, 0, 0, 0, 0];
    let mut response =
        RCMessage::new(super::super::frame::RCCommand::GetVersion, payload).serialize();
    response[0] = RC_START_SIGN_OUT;
    let mut device = with_response(response);
    assert!(device.observe_blocking().is_err());
    assert!(device.get_status_cached().pv.is_none());
}

#[test]
fn matched_measurement_reply_establishes_physical_angles() {
    let payload = vec![0x20, 0, 0xe8, 3, 0, 0, 0, 0];
    let mut response =
        RCMessage::new(super::super::frame::RCCommand::GetDataFields, payload).serialize();
    response[0] = RC_START_SIGN_OUT;
    let mut device = with_response(response);
    let observed = device.observe_blocking().unwrap();
    assert!((observed.angles.pitch.unwrap().as_radians().to_degrees() - 10.0).abs() < 0.001);
    assert!(device.get_status_cached().sp.is_none());
}

#[test]
fn recenter_ack_requires_each_axis_acknowledgement() {
    let mut device = with_response(Vec::new());
    let receipt = device
        .execute_blocking(
            PayloadCommand::Recenter,
            Instant::now() + Duration::from_secs(1),
        )
        .unwrap();
    assert!(receipt.sent_at <= Instant::now());
    assert_eq!(receipt.acknowledged, None);
    assert_eq!(receipt.acknowledged_at, None);
}
