use super::super::super::math::quaternion_wxyz_to_euler_deg;
use super::*;

fn approx_eq(a: f32, b: f32, eps: f32) -> bool {
    (a - b).abs() < eps
}

#[test]
fn build_attitude_status_round_trips_quaternion() {
    let attitude = Attitude {
        pitch: 12.0,
        roll: -7.5,
        yaw: 33.25,
    };
    let data = build_attitude_status_data(&attitude, 1234, GimbalDeviceErrorFlags::empty());

    assert_eq!(data.time_boot_ms, 1234);

    // Decoding the quaternion back must recover the input Euler angles.
    // This catches both the math.rs helper and our use of it.
    let (p, r, y) = quaternion_wxyz_to_euler_deg(data.q);
    assert!(approx_eq(p, attitude.pitch, 1e-3), "pitch: got {p}");
    assert!(approx_eq(r, attitude.roll, 1e-3), "roll: got {r}");
    assert!(approx_eq(y, attitude.yaw, 1e-3), "yaw: got {y}");
}

#[test]
fn build_attitude_status_uses_broadcast_addressing_and_vehicle_frame() {
    let data = build_attitude_status_data(
        &Attitude {
            pitch: 0.0,
            roll: 0.0,
            yaw: 0.0,
        },
        1234,
        GimbalDeviceErrorFlags::empty(),
    );

    // Manager broadcasts to every peer, not a single addressed system.
    assert_eq!(data.target_system, 0);
    assert_eq!(data.target_component, 0);

    // Honest about what we support: vehicle-frame yaw, no earth-frame
    // tracking, no rate path.
    assert!(data
        .flags
        .contains(GimbalDeviceFlags::GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME));
    assert!(data.angular_velocity_x.is_nan());
    assert!(data.angular_velocity_y.is_nan());
    assert!(data.angular_velocity_z.is_nan());

    // No failures supplied → frame is honestly clean. The persistent
    // "device is dead" failure mode is expressed by stopping the
    // broadcast entirely (see attitude_loop), not by setting flags
    // on a stale frame.
    assert!(data.failure_flags.is_empty());
}

fn test_mavlink_config() -> MavlinkConfig {
    MavlinkConfig {
        enabled: true,
        transport: Default::default(),
        bind_addr: "0.0.0.0".parse().unwrap(),
        udp_port: 14550,
        sysid: 1,
        compid: 154,
        limits: Default::default(),
    }
}

#[test]
fn build_manager_status_active_advertises_yaw_in_vehicle_frame() {
    // Regression: `GimbalManagerFlags::default()` is rust-mavlink's
    // first-bitflag default, which is `GIMBAL_MANAGER_FLAGS_RETRACT`.
    // Letting it leak out makes every periodic / on-demand status
    // frame falsely tell GCSs that the gimbal is retracted.
    let cfg = test_mavlink_config();
    let state = StateManager::new();
    // Default state has standby = false.
    let data = build_gimbal_manager_status_data(&state, &cfg, Instant::now());

    assert!(
        !data
            .flags
            .contains(GimbalManagerFlags::GIMBAL_MANAGER_FLAGS_RETRACT),
        "flags must not advertise RETRACT while active (got {:?})",
        data.flags
    );
    assert!(
        data.flags
            .contains(GimbalManagerFlags::GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME),
        "flags should mirror GIMBAL_DEVICE_ATTITUDE_STATUS frame (got {:?})",
        data.flags
    );
}

#[test]
fn standby_does_not_claim_a_measured_retract_pose() {
    let state = StateManager::new();
    state.update_standby(true);
    let data = build_gimbal_manager_status_data(&state, &test_mavlink_config(), Instant::now());
    assert!(!data
        .flags
        .contains(GimbalManagerFlags::GIMBAL_MANAGER_FLAGS_RETRACT));
    assert!(data
        .flags
        .contains(GimbalManagerFlags::GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME));
}

#[test]
fn build_attitude_status_propagates_caller_failure_flags() {
    // The attitude loop sets COMMS_ERROR for ~1 s after any failed
    // poll so a flaky-link recovery doesn't slip through as a fully
    // healthy frame.
    let data = build_attitude_status_data(
        &Attitude {
            pitch: 0.0,
            roll: 0.0,
            yaw: 0.0,
        },
        1234,
        GimbalDeviceErrorFlags::GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR,
    );
    assert!(data
        .failure_flags
        .contains(GimbalDeviceErrorFlags::GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR));
}
