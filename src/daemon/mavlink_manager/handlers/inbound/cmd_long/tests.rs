//! Unit tests for `ConfigureField` — the decode + per-field apply
//! logic that determines how `param1..param4` of CONFIGURE map onto
//! primary / secondary sysid+compid changes — and `flags_from_param5`,
//! the bitmask decoder used by the PITCHYAW command. The async
//! wrappers around them (`on_cmd_configure`, `on_cmd_pitchyaw`) are
//! harder to test in isolation without a mock RxCtx; the math here
//! is what actually decides ownership and frame interpretation.

use super::*;

#[test]
fn decode_negative_one_is_no_change() {
    assert!(matches!(
        ConfigureField::decode(-1.0),
        ConfigureField::NoChange
    ));
    // ±0.5 rounding tolerance — float -1.0 might arrive as -0.999.
    assert!(matches!(
        ConfigureField::decode(-0.999),
        ConfigureField::NoChange
    ));
    assert!(matches!(
        ConfigureField::decode(-1.001),
        ConfigureField::NoChange
    ));
}

#[test]
fn decode_negative_two_is_claim_self() {
    assert!(matches!(
        ConfigureField::decode(-2.0),
        ConfigureField::ClaimSelf
    ));
}

#[test]
fn decode_negative_three_is_release_if_mine() {
    assert!(matches!(
        ConfigureField::decode(-3.0),
        ConfigureField::ReleaseIfMine
    ));
}

#[test]
fn decode_zero_is_release_unconditional() {
    assert!(matches!(
        ConfigureField::decode(0.0),
        ConfigureField::ReleaseUnconditional
    ));
}

#[test]
fn decode_positive_byte_is_literal() {
    assert!(matches!(
        ConfigureField::decode(154.0),
        ConfigureField::Literal(154)
    ));
    assert!(matches!(
        ConfigureField::decode(1.0),
        ConfigureField::Literal(1)
    ));
    assert!(matches!(
        ConfigureField::decode(255.0),
        ConfigureField::Literal(255)
    ));
}

#[test]
fn decode_out_of_range_is_no_change() {
    // Negative but not -1/-2/-3 — old code collapsed to 0
    // (silent release); new code keeps the value unchanged and
    // logs a warn.
    assert!(matches!(
        ConfigureField::decode(-7.0),
        ConfigureField::NoChange
    ));
    assert!(matches!(
        ConfigureField::decode(256.0),
        ConfigureField::NoChange
    ));
}

#[test]
fn decode_non_finite_is_no_change() {
    assert!(matches!(
        ConfigureField::decode(f32::NAN),
        ConfigureField::NoChange
    ));
    assert!(matches!(
        ConfigureField::decode(f32::INFINITY),
        ConfigureField::NoChange
    ));
}

#[test]
fn apply_no_change_preserves_current() {
    assert_eq!(ConfigureField::NoChange.apply(42, 99), 42);
}

#[test]
fn apply_claim_self_substitutes_requester() {
    // Even if a value was already set, `-2` always overwrites
    // with the requester.
    assert_eq!(ConfigureField::ClaimSelf.apply(42, 99), 99);
    assert_eq!(ConfigureField::ClaimSelf.apply(0, 7), 7);
}

#[test]
fn apply_release_if_mine_only_clears_on_match() {
    // `-3` clears only when the current owner matches the
    // requester — prevents one client from bumping another off
    // by impersonation.
    assert_eq!(ConfigureField::ReleaseIfMine.apply(99, 99), 0);
    assert_eq!(ConfigureField::ReleaseIfMine.apply(42, 99), 42);
}

#[test]
fn apply_release_unconditional_always_clears() {
    assert_eq!(ConfigureField::ReleaseUnconditional.apply(42, 99), 0);
    assert_eq!(ConfigureField::ReleaseUnconditional.apply(0, 99), 0);
}

#[test]
fn apply_literal_overwrites() {
    assert_eq!(ConfigureField::Literal(154).apply(42, 99), 154);
}

#[test]
fn flags_from_param5_decodes_valid_bits() {
    let f = flags_from_param5(64.0);
    assert!(f.contains(GimbalManagerFlags::GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME));
}

#[test]
fn flags_from_param5_rounds_near_integer_floats() {
    // GCS may emit 64.0 as 63.9999… through f32 round-trip.
    let f = flags_from_param5(63.9999);
    assert!(f.contains(GimbalManagerFlags::GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME));
}

#[test]
fn flags_from_param5_rejects_nan() {
    assert_eq!(flags_from_param5(f32::NAN), GimbalManagerFlags::empty());
}

#[test]
fn flags_from_param5_rejects_negative() {
    assert_eq!(flags_from_param5(-1.0), GimbalManagerFlags::empty());
    assert_eq!(
        flags_from_param5(f32::NEG_INFINITY),
        GimbalManagerFlags::empty()
    );
}

#[test]
fn flags_from_param5_rejects_overflow() {
    assert_eq!(
        flags_from_param5(f32::INFINITY),
        GimbalManagerFlags::empty()
    );
    // 2^33 — beyond u32::MAX.
    assert_eq!(
        flags_from_param5(8_589_934_592.0),
        GimbalManagerFlags::empty()
    );
}

#[test]
fn flags_from_param5_zero_is_empty_default_vehicle_frame() {
    assert_eq!(flags_from_param5(0.0), GimbalManagerFlags::empty());
}
