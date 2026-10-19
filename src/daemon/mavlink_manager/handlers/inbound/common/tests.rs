use super::*;

#[test]
fn addressed_to_us_accepts_per_axis_broadcast_and_exact_match() {
    // Both axes broadcast.
    assert!(addressed_to_us(0, 0, 1, 154));
    // sysid broadcast, compid exact.
    assert!(addressed_to_us(0, 154, 1, 154));
    // sysid exact, compid broadcast.
    assert!(addressed_to_us(1, 0, 1, 154));
    // Both exact.
    assert!(addressed_to_us(1, 154, 1, 154));
}

#[test]
fn addressed_to_us_rejects_other_components_on_same_sysid() {
    // Targeting the autopilot (compid 1) on the same vehicle must NOT
    // be acted on by the gimbal manager — same-sysid component routing
    // is exactly what the compid axis is for.
    assert!(!addressed_to_us(1, 1, 1, 154));
}

#[test]
fn addressed_to_us_rejects_other_systems() {
    assert!(!addressed_to_us(2, 154, 1, 154));
    assert!(!addressed_to_us(2, 0, 1, 154));
}

#[test]
fn addressed_to_this_device_accepts_broadcast_and_exact_match() {
    // 0 = broadcast across every gimbal device under any manager.
    assert!(addressed_to_this_device(0, 154));
    // Exact compid match.
    assert!(addressed_to_this_device(154, 154));
}

#[test]
fn addressed_to_this_device_rejects_other_devices() {
    // A different gimbal under another manager (compid 155) — drop
    // silently rather than stealing motion meant for that gimbal.
    assert!(!addressed_to_this_device(155, 154));
    // The 1..=6 "non-MAVLink gimbal" range is explicitly someone
    // else's slot — never us.
    for non_mavlink in 1..=6u8 {
        assert!(
            !addressed_to_this_device(non_mavlink, 154),
            "non-MAVLink gimbal id {} must not match our compid 154",
            non_mavlink
        );
    }
}

#[test]
fn gimbal_device_id_from_param7_decodes_valid_byte() {
    assert_eq!(gimbal_device_id_from_param7(0.0), Some(0));
    assert_eq!(gimbal_device_id_from_param7(154.0), Some(154));
    // GCS may emit an integer that survives an f32 round-trip with
    // a tiny rounding error — the round() inside the helper
    // recovers the right byte.
    assert_eq!(gimbal_device_id_from_param7(154.0001), Some(154));
    assert_eq!(gimbal_device_id_from_param7(255.0), Some(255));
}

#[test]
fn gimbal_device_id_from_param7_rejects_garbage() {
    // NaN and negatives must read as "not for us" — a malformed GCS
    // can't accidentally drive this gimbal because its broken
    // param7 matched the broadcast path.
    assert_eq!(gimbal_device_id_from_param7(f32::NAN), None);
    assert_eq!(gimbal_device_id_from_param7(-1.0), None);
    assert_eq!(gimbal_device_id_from_param7(f32::INFINITY), None);
    assert_eq!(gimbal_device_id_from_param7(f32::NEG_INFINITY), None);
    assert_eq!(gimbal_device_id_from_param7(256.0), None);
    assert_eq!(gimbal_device_id_from_param7(1e9), None);
}

#[test]
fn unsupported_reference_flags_are_never_relabelled_as_vehicle_frame() {
    assert!(position_flags_supported(GimbalManagerFlags::empty()));
    assert!(rate_flags_supported(
        GimbalManagerFlags::GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME
    ));
    assert!(!position_flags_supported(
        GimbalManagerFlags::GIMBAL_MANAGER_FLAGS_PITCH_LOCK
    ));
    assert!(!position_flags_supported(
        GimbalManagerFlags::GIMBAL_MANAGER_FLAGS_NEUTRAL
    ));
    assert!(!rate_flags_supported(
        GimbalManagerFlags::GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME
    ));
}
