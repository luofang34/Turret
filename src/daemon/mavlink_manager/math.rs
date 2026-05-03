//! Stateless math helpers for the MAVLink manager.

use tracing::warn;

/// Clamp `value` to `[min, max]` and warn the operator if the original
/// value was outside the range. `NaN` is left unchanged so that NaN-as-
/// "leave-axis-alone" still propagates to the caller.
pub(super) fn clamp_axis(value: f32, min: f32, max: f32, axis: &str) -> f32 {
    if value.is_nan() {
        return value;
    }
    let clamped = value.clamp(min, max);
    if (clamped - value).abs() > f32::EPSILON {
        warn!(
            "Clamping {} from {:.2}° to {:.2}° (limits {:.2}°..={:.2}°)",
            axis, value, clamped, min, max
        );
    }
    clamped
}

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

/// Inverse of [`quaternion_wxyz_to_euler_deg`]: build a unit quaternion in
/// MAVLink `[w, x, y, z]` order from aerospace Euler angles in degrees.
///
/// Useful for outbound `GIMBAL_DEVICE_ATTITUDE_STATUS` frames where the
/// device gives us Euler angles but the wire format wants a quaternion.
pub(super) fn euler_deg_to_quaternion_wxyz(
    pitch_deg: f32,
    roll_deg: f32,
    yaw_deg: f32,
) -> [f32; 4] {
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

/// One-shot integration of a rate command into a position target.
///
/// `current_deg` is the current commanded (or measured) angle for one
/// axis. `rate_deg_per_s` is the commanded angular velocity. `dt` is
/// the elapsed time since the previous rate tick from the same source
/// (clamped by the caller — see [`clamp_rate_dt`]).
///
/// Returns `current_deg + rate_deg_per_s * dt`. Caller is responsible
/// for clamping the result to the configured device limits.
///
/// STorM32 has no native rate command, so the manager simulates it by
/// resending position commands as the GCS streams rate ticks. With the
/// MAVLink spec's typical 10–50 Hz rate-command cadence this gives a
/// usable joystick experience without a closed-loop rate controller.
pub(super) fn compute_rate_target_deg(current_deg: f32, rate_deg_per_s: f32, dt: f32) -> f32 {
    if !rate_deg_per_s.is_finite() || rate_deg_per_s == 0.0 {
        return current_deg;
    }
    current_deg + rate_deg_per_s * dt
}

/// Clamp the inter-tick `dt` between 10 ms and 500 ms.
///
/// Lower bound: a too-short tick (rapid GCS retransmits) collapses the
/// integration step toward zero and the gimbal stops moving — pin to
/// 10 ms (= 100 Hz) so even a high-rate stream still produces motion.
/// Upper bound: a too-long tick (GCS dropped a packet, daemon was
/// blocked) produces a huge jump — pin to 500 ms so a recovered stream
/// doesn't snap the gimbal across half a second of accumulated rate.
pub(super) fn clamp_rate_dt(dt: std::time::Duration) -> std::time::Duration {
    const MIN: std::time::Duration = std::time::Duration::from_millis(10);
    const MAX: std::time::Duration = std::time::Duration::from_millis(500);
    dt.clamp(MIN, MAX)
}

/// Wrap an angle in degrees to the canonical `[-180, 180]` range.
///
/// The earth-frame ↔ vehicle-frame conversion does plain subtraction,
/// which can leave the result outside that range (e.g. earth=170°,
/// vehicle=-170° → 340°). Wrap so the gimbal driver and downstream
/// clamps don't see surprising magnitudes.
pub(super) fn wrap_180_deg(deg: f32) -> f32 {
    let mut x = deg % 360.0;
    if x > 180.0 {
        x -= 360.0;
    } else if x < -180.0 {
        x += 360.0;
    }
    x
}

/// Earth-frame yaw → vehicle-frame yaw given the vehicle's current heading.
///
/// `earth_yaw_deg` is what the GCS commanded with
/// `GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME` set; `vehicle_yaw_deg` is
/// the autopilot's current heading from `AUTOPILOT_STATE_FOR_GIMBAL_DEVICE`.
/// Result is the gimbal-frame yaw to send to the device.
pub(super) fn earth_to_vehicle_yaw_deg(earth_yaw_deg: f32, vehicle_yaw_deg: f32) -> f32 {
    wrap_180_deg(earth_yaw_deg - vehicle_yaw_deg)
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
    fn compute_rate_target_integrates_constant_rate() {
        // 30°/s × 0.1 s = +3°
        let target = compute_rate_target_deg(10.0, 30.0, 0.1);
        assert!((target - 13.0).abs() < 1e-4);
    }

    #[test]
    fn compute_rate_target_negative_rate_decreases() {
        // -45°/s × 0.2 s = -9° from 5° = -4°
        let target = compute_rate_target_deg(5.0, -45.0, 0.2);
        assert!((target - -4.0).abs() < 1e-4);
    }

    #[test]
    fn compute_rate_target_zero_rate_is_noop() {
        assert_eq!(compute_rate_target_deg(42.0, 0.0, 0.5), 42.0);
    }

    #[test]
    fn compute_rate_target_nan_rate_is_noop() {
        // NaN rate per spec means "leave axis alone" — the rate path
        // collapses to the existing position. Important so a
        // partial-axis rate command (e.g. only yaw) doesn't drag the
        // unaffected axes off their current targets.
        assert_eq!(compute_rate_target_deg(42.0, f32::NAN, 0.5), 42.0);
    }

    #[test]
    fn clamp_rate_dt_pins_below_min() {
        let clamped = clamp_rate_dt(std::time::Duration::from_micros(1));
        assert_eq!(clamped, std::time::Duration::from_millis(10));
    }

    #[test]
    fn clamp_rate_dt_pins_above_max() {
        let clamped = clamp_rate_dt(std::time::Duration::from_secs(5));
        assert_eq!(clamped, std::time::Duration::from_millis(500));
    }

    #[test]
    fn clamp_rate_dt_passes_through_in_band() {
        let dt = std::time::Duration::from_millis(100);
        assert_eq!(clamp_rate_dt(dt), dt);
    }

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
}
