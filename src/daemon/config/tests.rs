use super::*;

/// Minimal TOML used by the corrector-knob tests — contains only
/// the fields the parser actually requires (no `#[serde(default)]`
/// on them), so we can exercise the `[gimbal]` table in isolation.
fn toml_without_gimbal_section() -> &'static str {
    r#"
[device]
path = "auto"
[ipc]
socket_path = "/tmp/turret.sock"
[mavlink]
enabled = true
udp_port = 14550
sysid = 1
compid = 154
"#
}

#[test]
fn yaw_corrector_enabled_defaults_to_true_when_section_omitted() {
    // Operators upgrading from a pre-knob config must keep the
    // existing default-on behavior — no surprise disable.
    let cfg: DaemonConfig = toml::from_str(toml_without_gimbal_section()).unwrap();
    assert!(cfg.gimbal.yaw_corrector_enabled);
}

#[test]
fn yaw_corrector_enabled_defaults_to_true_when_section_empty() {
    // An empty `[gimbal]` table parses cleanly and inherits the
    // default — covers the case where an operator added the header
    // for a future field but didn't set the corrector flag.
    let mut text = String::from(toml_without_gimbal_section());
    text.push_str("[gimbal]\n");
    let cfg: DaemonConfig = toml::from_str(&text).unwrap();
    assert!(cfg.gimbal.yaw_corrector_enabled);
}

#[test]
fn yaw_corrector_enabled_can_be_explicitly_disabled() {
    let mut text = String::from(toml_without_gimbal_section());
    text.push_str("[gimbal]\nyaw_corrector_enabled = false\n");
    let cfg: DaemonConfig = toml::from_str(&text).unwrap();
    assert!(!cfg.gimbal.yaw_corrector_enabled);
}

#[test]
fn daemon_config_default_enables_corrector() {
    // The non-TOML default path (used by `load_for_startup` when
    // the default `turret.toml` is absent) must agree with the
    // file-default behavior.
    assert!(DaemonConfig::default().gimbal.yaw_corrector_enabled);
}

/// Build a temp directory we can `cd` to without races between
/// parallel tests poking at `turret.toml` in the package root.
fn temp_workdir() -> std::path::PathBuf {
    let pid = std::process::id();
    let n = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos())
        .unwrap_or(0);
    let p = std::env::temp_dir().join(format!("turret-cfg-test-{pid}-{n}"));
    std::fs::create_dir_all(&p).unwrap();
    p
}

#[test]
fn load_for_startup_explicit_missing_file_errors() {
    // `--config /no/such/path.toml` must fail loudly. The
    // operator asked for *that* file; substituting defaults
    // would be a silent surprise.
    let dir = temp_workdir();
    let missing = dir.join("nope.toml");
    let r = DaemonConfig::load_for_startup(Some(missing.to_str().unwrap()));
    assert!(r.is_err(), "explicit missing file must error, got Ok");
}

#[test]
fn load_for_startup_default_missing_file_uses_defaults() {
    // No `--config`, no `turret.toml` in CWD: built-in defaults
    // is the right behavior (first run on a fresh install).
    let dir = temp_workdir();
    let prev = std::env::current_dir().unwrap();
    // `set_current_dir` is process-wide; we restore in a deferred
    // step so a panic doesn't leave the test runner in /tmp.
    std::env::set_current_dir(&dir).unwrap();
    let r = DaemonConfig::load_for_startup(None);
    std::env::set_current_dir(prev).unwrap();
    std::fs::remove_dir_all(&dir).ok();
    let cfg = r.expect("missing default file must produce defaults, not error");
    assert_eq!(cfg.device.path, "auto");
}

#[test]
fn load_for_startup_malformed_toml_errors() {
    // A typo in `turret.toml` must NOT silently degrade to
    // defaults — that risk-masks `device.path = "auto"` and the
    // default socket the operator didn't pick. Same shape
    // regardless of whether the path was explicit.
    let dir = temp_workdir();
    let path = dir.join("bad.toml");
    // Unclosed table header — TOML's parser rejects this with a
    // line/col-tagged error.
    std::fs::write(&path, b"[device\npath = \"auto\"\n").unwrap();
    let r = DaemonConfig::load_for_startup(Some(path.to_str().unwrap()));
    std::fs::remove_dir_all(&dir).ok();
    assert!(r.is_err(), "malformed TOML must error, got Ok");
}
