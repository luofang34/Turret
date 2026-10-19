use super::*;

/// Build a temporary `XDG_STATE_HOME` for the duration of one test.
/// Restores the previous env on drop and removes the temp dir.
/// Sequential — env mutations are process-wide, so all callers
/// acquire a shared mutex first.
struct EnvFixture {
    _guard: std::sync::MutexGuard<'static, ()>,
    tmp: std::path::PathBuf,
    prev_xdg: Option<std::ffi::OsString>,
    prev_home: Option<std::ffi::OsString>,
}

impl EnvFixture {
    fn new() -> Self {
        // Sequence env-mutating tests so they don't race each other.
        static LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
        let guard = LOCK.lock().unwrap_or_else(|p| p.into_inner());
        let pid = std::process::id();
        let n = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_nanos())
            .unwrap_or(0);
        let tmp = std::env::temp_dir().join(format!("turret-cal-test-{pid}-{n}"));
        std::fs::create_dir_all(&tmp).unwrap();
        let prev_xdg = std::env::var_os("XDG_STATE_HOME");
        let prev_home = std::env::var_os("HOME");
        std::env::set_var("XDG_STATE_HOME", &tmp);
        // Clear HOME so we provably go through XDG_STATE_HOME.
        std::env::remove_var("HOME");
        Self {
            _guard: guard,
            tmp,
            prev_xdg,
            prev_home,
        }
    }
}

impl Drop for EnvFixture {
    fn drop(&mut self) {
        match self.prev_xdg.take() {
            Some(v) => std::env::set_var("XDG_STATE_HOME", v),
            None => std::env::remove_var("XDG_STATE_HOME"),
        }
        match self.prev_home.take() {
            Some(v) => std::env::set_var("HOME", v),
            None => std::env::remove_var("HOME"),
        }
        // Best-effort cleanup; ignore errors (test already passed/failed).
        // `let_underscore_must_use` is forbidden workspace-wide, so
        // discharge the `Result` via an `if let` instead of `let _`.
        if std::fs::remove_dir_all(&self.tmp).is_err() {
            // No remediation we can do from a Drop; the OS will
            // reclaim /tmp anyway.
        }
    }
}

#[test]
fn load_returns_default_when_file_missing() {
    let fx = EnvFixture::new();
    let cal = Calibration::load_blocking();
    assert_eq!(cal, Calibration::default());
    // Confirm we did look under the fixture's temp dir, not somewhere
    // else.
    let path = state_file_path().unwrap();
    assert!(path.starts_with(&fx.tmp));
    assert!(!path.exists());
}

#[test]
fn save_then_load_round_trips() {
    let _fx = EnvFixture::new();
    let cal = Calibration {
        yaw_offset_deg: -4.5,
    };
    cal.save_blocking().unwrap();
    let loaded = Calibration::load_blocking();
    assert!((loaded.yaw_offset_deg - -4.5).abs() < 1e-6);
}

#[test]
fn load_rejects_non_finite_yaw_offset() {
    let _fx = EnvFixture::new();
    let path = state_file_path().unwrap();
    std::fs::create_dir_all(path.parent().unwrap()).unwrap();
    // TOML's float grammar accepts `nan` and `inf` as keywords. The
    // defensive non-finite check in `load()` rejects them and falls
    // back to the default.
    std::fs::write(&path, b"yaw_offset_deg = nan\n").unwrap();
    assert_eq!(Calibration::load_blocking(), Calibration::default());

    std::fs::write(&path, b"yaw_offset_deg = inf\n").unwrap();
    assert_eq!(Calibration::load_blocking(), Calibration::default());

    std::fs::write(&path, b"yaw_offset_deg = -inf\n").unwrap();
    assert_eq!(Calibration::load_blocking(), Calibration::default());
}

#[test]
fn load_returns_default_on_malformed_toml() {
    let _fx = EnvFixture::new();
    let path = state_file_path().unwrap();
    std::fs::create_dir_all(path.parent().unwrap()).unwrap();
    std::fs::write(&path, b"this is = not = valid = toml").unwrap();
    assert_eq!(Calibration::load_blocking(), Calibration::default());
}

#[test]
fn missing_yaw_offset_field_uses_default() {
    let _fx = EnvFixture::new();
    let path = state_file_path().unwrap();
    std::fs::create_dir_all(path.parent().unwrap()).unwrap();
    // Empty file is valid TOML (an empty document). With
    // `#[serde(default)]` on the field, this loads cleanly to
    // `yaw_offset_deg: 0.0`, so future schema additions don't
    // break old files.
    std::fs::write(&path, b"").unwrap();
    assert_eq!(Calibration::load_blocking().yaw_offset_deg, 0.0);
}
