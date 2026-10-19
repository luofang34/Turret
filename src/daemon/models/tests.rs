//! Wire-format lockdowns for the IPC schema. These tests exist to
//! catch accidental renames — the daemon's JSON surface is a public
//! contract, and a typo in a `#[serde(rename_all = ...)]` attribute
//! would silently break every existing client.

use super::*;

#[test]
fn deserialize_set_command() {
    let cmd: IpcCommand =
        serde_json::from_str(r#"{"cmd":"set","pitch":10.0,"roll":-2.5,"yaw":42.0}"#).unwrap();
    match cmd {
        IpcCommand::Set { pitch, roll, yaw } => {
            assert!((pitch - 10.0).abs() < 1e-6);
            assert!((roll + 2.5).abs() < 1e-6);
            assert!((yaw - 42.0).abs() < 1e-6);
        }
        other => panic!("expected Set, got {other:?}"),
    }
}

#[test]
fn deserialize_pan_mode_uses_snake_case_discriminator() {
    // Wire form is `pan_mode`, not `pan-mode` (kebab) or `panmode`
    // (the `rename_all = "lowercase"` would-be default).
    let cmd: IpcCommand = serde_json::from_str(r#"{"cmd":"pan_mode","mode":3}"#).unwrap();
    match cmd {
        IpcCommand::PanMode { mode } => assert_eq!(mode, 3),
        other => panic!("expected PanMode, got {other:?}"),
    }
}

#[test]
fn deserialize_standby_uses_enabled_field() {
    // Wire form is `enabled` (boolean state), not `enable` (verb).
    let cmd: IpcCommand = serde_json::from_str(r#"{"cmd":"standby","enabled":true}"#).unwrap();
    match cmd {
        IpcCommand::Standby { enabled } => assert!(enabled),
        other => panic!("expected Standby, got {other:?}"),
    }
}

#[test]
fn old_kebab_pan_mode_is_rejected() {
    // The kebab-case form was the previous wire shape; rejecting it
    // outright (rather than silently mis-parsing) helps clients
    // discover the rename quickly.
    let result: Result<IpcCommand, _> = serde_json::from_str(r#"{"cmd":"pan-mode","mode":3}"#);
    assert!(
        result.is_err(),
        "kebab-case pan-mode must not parse (got {result:?})"
    );
}

#[test]
fn old_enable_field_is_rejected() {
    let result: Result<IpcCommand, _> = serde_json::from_str(r#"{"cmd":"standby","enable":true}"#);
    assert!(
        result.is_err(),
        "old `enable` field must not parse (got {result:?})"
    );
}

#[test]
fn serialize_pan_mode_round_trips_to_snake_case() {
    let cmd = IpcCommand::PanMode { mode: 2 };
    let s = serde_json::to_string(&cmd).unwrap();
    assert!(
        s.contains(r#""cmd":"pan_mode""#),
        "expected snake_case discriminator, got {s}"
    );
    assert!(s.contains(r#""mode":2"#), "got {s}");
}

#[test]
fn serialize_standby_round_trips_to_enabled_field() {
    let cmd = IpcCommand::Standby { enabled: false };
    let s = serde_json::to_string(&cmd).unwrap();
    assert!(s.contains(r#""cmd":"standby""#), "got {s}");
    assert!(
        s.contains(r#""enabled":false"#),
        "expected `enabled`, got {s}"
    );
}

#[test]
fn deserialize_calibrate_yaw() {
    let cmd: IpcCommand = serde_json::from_str(r#"{"cmd":"calibrate_yaw"}"#).unwrap();
    assert!(matches!(cmd, IpcCommand::CalibrateYaw));
}

#[test]
fn deserialize_set_yaw_offset() {
    let cmd: IpcCommand = serde_json::from_str(r#"{"cmd":"set_yaw_offset","deg":-4.5}"#).unwrap();
    match cmd {
        IpcCommand::SetYawOffset { deg } => assert!((deg - -4.5).abs() < 1e-6),
        other => panic!("expected SetYawOffset, got {other:?}"),
    }
}

#[test]
fn serialize_set_yaw_offset_uses_snake_case() {
    let cmd = IpcCommand::SetYawOffset { deg: 1.25 };
    let s = serde_json::to_string(&cmd).unwrap();
    assert!(
        s.contains(r#""cmd":"set_yaw_offset""#),
        "expected snake_case discriminator, got {s}"
    );
    assert!(s.contains(r#""deg":1.25"#), "got {s}");
}

#[test]
fn deserialize_selftest() {
    let cmd: IpcCommand = serde_json::from_str(r#"{"cmd":"selftest"}"#).unwrap();
    assert!(matches!(cmd, IpcCommand::Selftest));
}
