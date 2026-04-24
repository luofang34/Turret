//! Stateless math helpers for the MAVLink manager.

/// Convert a MAVLink quaternion `[w, x, y, z]` to aerospace Euler angles
/// (roll about X, pitch about Y, yaw about Z) in degrees.
///
/// MAVLink stores quaternions with the scalar component first (`w` at index
/// 0); callers that receive a quaternion from the wire should not reorder
/// before calling this. The helper applies the standard Z-Y-X intrinsic
/// rotation decomposition, with explicit handling at the ±90° pitch
/// singularity ("gimbal lock").
///
/// Returns `(pitch_deg, roll_deg, yaw_deg)`.
pub(super) fn quaternion_wxyz_to_euler_deg(q: [f32; 4]) -> (f32, f32, f32) {
    let w = q[0];
    let x = q[1];
    let y = q[2];
    let z = q[3];

    // Roll (rotation about x-axis)
    let sinr_cosp = 2.0 * (w * x + y * z);
    let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    let roll = sinr_cosp.atan2(cosr_cosp);

    // Pitch (rotation about y-axis). Clamp at ±1 to keep asin defined when a
    // non-unit quaternion comes in over the wire.
    let sinp = 2.0 * (w * y - z * x);
    let pitch = if sinp.abs() >= 1.0 {
        std::f32::consts::FRAC_PI_2.copysign(sinp)
    } else {
        sinp.asin()
    };

    // Yaw (rotation about z-axis)
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    let yaw = siny_cosp.atan2(cosy_cosp);

    (pitch.to_degrees(), roll.to_degrees(), yaw.to_degrees())
}

/// Convert a `COMMAND_LONG` param (`f32`) to a MAVLink sysid/compid byte.
///
/// The MAVLink v2 spec reserves negative values (-3, -2, -1) for special
/// semantics on `MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE`. They are not yet
/// honored here — any non-positive value collapses to 0 ("unset").
///
/// TODO(item-5): implement -3 (retain current) / -2 (reset to default) /
/// -1 (no primary) per MAVLink spec.
pub(super) fn to_id_byte(v: f32) -> u8 {
    if !v.is_finite() || v < 0.0 {
        return 0;
    }
    let r = v.round();
    if r > 255.0 {
        255
    } else {
        r as u8
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;

    #[test]
    fn to_id_byte_clamps_and_rejects_special_values() {
        assert_eq!(to_id_byte(7.0), 7);
        assert_eq!(to_id_byte(7.4), 7);
        assert_eq!(to_id_byte(7.6), 8);
        assert_eq!(to_id_byte(0.0), 0);
        assert_eq!(to_id_byte(300.0), 255);
        assert_eq!(to_id_byte(-1.0), 0);
        assert_eq!(to_id_byte(-3.0), 0);
        assert_eq!(to_id_byte(f32::NAN), 0);
        assert_eq!(to_id_byte(f32::INFINITY), 0);
        assert_eq!(to_id_byte(f32::NEG_INFINITY), 0);
    }

    fn quat_from_euler_deg(pitch_deg: f32, roll_deg: f32, yaw_deg: f32) -> [f32; 4] {
        let (hp, hr, hy) = (
            pitch_deg.to_radians() / 2.0,
            roll_deg.to_radians() / 2.0,
            yaw_deg.to_radians() / 2.0,
        );
        let (cp, sp) = (hp.cos(), hp.sin());
        let (cr, sr) = (hr.cos(), hr.sin());
        let (cy, sy) = (hy.cos(), hy.sin());
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]
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
            let q = quat_from_euler_deg(p, r, y);
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
        let q = quat_from_euler_deg(90.0, 0.0, 0.0);
        let (pitch, _roll, _yaw) = quaternion_wxyz_to_euler_deg(q);
        assert!(
            approx_eq(pitch, 90.0, 5e-2),
            "pitch: got {}, want ~90",
            pitch
        );
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
}
