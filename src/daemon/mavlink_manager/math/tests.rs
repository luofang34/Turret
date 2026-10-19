use super::*;

#[test]
fn wrap_180_passes_through_in_band() {
    assert!((wrap_180_deg(0.0) - 0.0).abs() < 1e-4);
    assert!((wrap_180_deg(90.0) - 90.0).abs() < 1e-4);
    assert!((wrap_180_deg(-179.9) - -179.9).abs() < 1e-4);
    assert!((wrap_180_deg(180.0) - 180.0).abs() < 1e-4);
}

#[test]
fn wrap_180_normalises_above_180() {
    // 340° → -20° (the typical "earth_yaw - (-vehicle_yaw)" overshoot).
    assert!((wrap_180_deg(340.0) - -20.0).abs() < 1e-4);
    assert!((wrap_180_deg(181.0) - -179.0).abs() < 1e-4);
}

#[test]
fn wrap_180_normalises_below_neg_180() {
    assert!((wrap_180_deg(-340.0) - 20.0).abs() < 1e-4);
    assert!((wrap_180_deg(-181.0) - 179.0).abs() < 1e-4);
}

#[test]
fn earth_to_vehicle_yaw_subtracts_vehicle_heading() {
    // Vehicle pointing east (90° earth heading), earth-frame target
    // also east (90°): gimbal should hold yaw = 0 (look straight ahead).
    assert!((earth_to_vehicle_yaw_deg(90.0, 90.0) - 0.0).abs() < 1e-4);

    // Vehicle east (90°), earth target north (0°): gimbal yaws -90°
    // (look left from the vehicle's own forward).
    assert!((earth_to_vehicle_yaw_deg(0.0, 90.0) - -90.0).abs() < 1e-4);
}

#[test]
fn earth_to_vehicle_yaw_wraps_correctly() {
    // earth=170°, vehicle=-170° → 340° → wrap → -20°.
    assert!((earth_to_vehicle_yaw_deg(170.0, -170.0) - -20.0).abs() < 1e-4);
    // earth=-170°, vehicle=170° → -340° → wrap → 20°.
    assert!((earth_to_vehicle_yaw_deg(-170.0, 170.0) - 20.0).abs() < 1e-4);
}

fn approx_eq(a: f32, b: f32, eps: f32) -> bool {
    (a - b).abs() < eps
}

#[test]
fn quaternion_wxyz_identity_is_zero_euler() {
    let (pitch, roll, yaw) = quaternion_wxyz_to_euler_deg([1.0, 0.0, 0.0, 0.0]);
    assert!(approx_eq(pitch, 0.0, 1e-4));
    assert!(approx_eq(roll, 0.0, 1e-4));
    assert!(approx_eq(yaw, 0.0, 1e-4));
}

#[test]
fn quaternion_wxyz_roundtrips_per_axis() {
    for (p, r, y) in [(30.0, 0.0, 0.0), (0.0, 45.0, 0.0), (0.0, 0.0, 60.0)] {
        let q = euler_deg_to_quaternion_wxyz(p, r, y);
        let (pp, rr, yy) = quaternion_wxyz_to_euler_deg(q);
        assert!(approx_eq(pp, p, 1e-3), "pitch: got {}, want {}", pp, p);
        assert!(approx_eq(rr, r, 1e-3), "roll: got {}, want {}", rr, r);
        assert!(approx_eq(yy, y, 1e-3), "yaw: got {}, want {}", yy, y);
    }
}

#[test]
fn quaternion_wxyz_handles_pitch_near_gimbal_lock() {
    // Pure pitch of +90°. f32 rounding of sqrt(2)/2 leaves sinp just shy
    // of 1.0, so the explicit lock branch doesn't fire — but the regular
    // asin path still resolves to within 0.03° of the true value.
    let q = euler_deg_to_quaternion_wxyz(90.0, 0.0, 0.0);
    let (pitch, _roll, _yaw) = quaternion_wxyz_to_euler_deg(q);
    assert!(
        approx_eq(pitch, 90.0, 5e-2),
        "pitch: got {}, want ~90",
        pitch
    );
}

#[test]
fn clamp_axis_passes_in_range_value() {
    assert_eq!(clamp_axis(45.0, -90.0, 90.0, "pitch"), 45.0);
    assert_eq!(clamp_axis(-89.999, -90.0, 90.0, "pitch"), -89.999);
    assert_eq!(clamp_axis(0.0, -90.0, 90.0, "pitch"), 0.0);
}

#[test]
fn clamp_axis_clamps_out_of_range() {
    assert_eq!(clamp_axis(120.0, -90.0, 90.0, "pitch"), 90.0);
    assert_eq!(clamp_axis(-200.0, -180.0, 180.0, "yaw"), -180.0);
}

#[test]
fn clamp_axis_passes_nan_through() {
    assert!(clamp_axis(f32::NAN, -90.0, 90.0, "pitch").is_nan());
}

#[test]
fn quaternion_wxyz_clamps_non_unit_beyond_sin_range() {
    // Force sinp past 1.0 with a deliberately non-unit quaternion. Without
    // the `abs() >= 1.0` guard this would return NaN from asin.
    let q = [0.8, 0.0, 0.8, 0.0];
    let (pitch, _roll, _yaw) = quaternion_wxyz_to_euler_deg(q);
    assert!(pitch.is_finite(), "pitch should be clamped, got {}", pitch);
    assert!(approx_eq(pitch, 90.0, 1e-3));
}
