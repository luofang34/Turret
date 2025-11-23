use crate::protocols::GimbalAngles;

/// Convert degrees to quaternion for MAVLink
pub fn angles_to_quaternion(pitch: f32, roll: f32, yaw: f32) -> [f32; 4] {
    let pitch_rad = pitch.to_radians() / 2.0;
    let roll_rad = roll.to_radians() / 2.0;
    let yaw_rad = yaw.to_radians() / 2.0;

    let cp = pitch_rad.cos();
    let sp = pitch_rad.sin();
    let cr = roll_rad.cos();
    let sr = roll_rad.sin();
    let cy = yaw_rad.cos();
    let sy = yaw_rad.sin();

    [
        cr * cp * cy + sr * sp * sy, // q1 (x)
        sr * cp * cy - cr * sp * sy, // q2 (y)
        cr * sp * cy + sr * cp * sy, // q3 (z)
        cr * cp * sy - sr * sp * cy, // q4 (w)
    ]
}

/// Convert quaternion to degrees for display
pub fn quaternion_to_angles(q: [f32; 4]) -> GimbalAngles {
    let x = q[0];
    let y = q[1];
    let z = q[2];
    let w = q[3];

    // Convert quaternion to Euler angles
    let sin_r_cp = 2.0 * (w * x + y * z);
    let cos_r_cp = 1.0 - 2.0 * (x * x + y * y);
    let roll = sin_r_cp.atan2(cos_r_cp);

    let sin_p = 2.0 * (w * y - z * x);
    let pitch = if sin_p.abs() >= 1.0 {
        std::f32::consts::FRAC_PI_2.copysign(sin_p)
    } else {
        sin_p.asin()
    };

    let sin_y_cp = 2.0 * (w * z + x * y);
    let cos_y_cp = 1.0 - 2.0 * (y * y + z * z);
    let yaw = sin_y_cp.atan2(cos_y_cp);

    GimbalAngles {
        pitch: pitch.to_degrees(),
        roll: roll.to_degrees(),
        yaw: yaw.to_degrees(),
    }
}