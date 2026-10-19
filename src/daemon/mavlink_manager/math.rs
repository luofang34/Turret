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

/// Wrap an angle in degrees to the canonical `[-180, 180]` range.
///
/// The earth-frame ↔ vehicle-frame conversion does plain subtraction,
/// which can leave the result outside that range (e.g. earth=170°,
/// vehicle=-170° → 340°). Wrap so the gimbal driver and downstream
/// clamps don't see surprising magnitudes.
pub(crate) fn wrap_180_deg(deg: f32) -> f32 {
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

#[cfg(test)]
mod tests;
