//! Persisted device calibration. Today the only calibrated value is a
//! yaw zero-offset between the operator-facing command frame and the
//! IMU-reported readback frame; the file on disk is forward-compatible
//! with future per-axis trims.
//!
//! The file lives under `$XDG_STATE_HOME/turret/calibration.toml` (defaults
//! to `~/.local/state/turret/`), separate from `turret.toml` —
//! `turret.toml` is human-edited config, the state file is daemon-managed
//! runtime data that gets rewritten on every calibrate / trim. Mixing
//! the two would mean the daemon overwrites the user's config file,
//! which is exactly the surprise we want to avoid.
//!
//! TOML for the body matches `turret.toml` and `Cargo.toml` — a single
//! parser across every config-shaped file in the project.
//!
//! Reads are lossy: a missing file, malformed TOML, or any I/O error
//! falls back to `Calibration::default()` (zeroed) with a warn. The
//! daemon should boot to a usable state even when nothing on disk is
//! readable yet — first run, fresh container, permissions glitch, etc.
//! Writes propagate the underlying error so the IPC handler that
//! triggered the save can surface it to the caller.

use serde::{Deserialize, Serialize};
use std::path::PathBuf;
use tracing::{debug, info, warn};

/// Calibration values applied transparently in the daemon's I/O path.
///
/// `yaw_offset_deg` is the constant bias between the operator-facing yaw
/// frame (where SP=0 should mean PV=0) and the device's IMU-reported yaw
/// frame. The daemon adds it to outgoing commands and subtracts it from
/// incoming measurements, so callers (IPC, MAVLink) never see the raw
/// device-frame yaw.
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize, PartialEq)]
pub struct Calibration {
    /// Yaw zero-offset in degrees. Positive means "the device IMU
    /// reads N° when the device is physically commanded to 0° in
    /// operator frame." Stored in degrees because that's the unit
    /// every consumer thinks in; the in-memory representation in
    /// `GimbalHandle` is centi-degrees for lock-free atomic access.
    #[serde(default)]
    pub yaw_offset_deg: f32,
}

/// Resolve the calibration file path under XDG state directory.
///
/// Honors `$XDG_STATE_HOME` when set, falls back to `~/.local/state` per
/// the XDG Base Directory spec. Only fails when neither `$HOME` nor
/// `$XDG_STATE_HOME` is set — at that point we have nowhere safe to
/// write, and the caller has to decide whether to error out or skip
/// persistence.
pub fn state_file_path() -> Option<PathBuf> {
    let base = if let Some(xdg) = std::env::var_os("XDG_STATE_HOME").filter(|s| !s.is_empty()) {
        PathBuf::from(xdg)
    } else {
        let home = std::env::var_os("HOME").filter(|s| !s.is_empty())?;
        PathBuf::from(home).join(".local/state")
    };
    Some(base.join("turret").join("calibration.toml"))
}

impl Calibration {
    /// Load calibration from the XDG state file. Missing file, missing
    /// `$HOME`, malformed TOML, or any I/O error returns
    /// `Calibration::default()` with a `warn!`-level log line — the
    /// daemon must boot to a usable state regardless of disk content.
    pub fn load() -> Self {
        let Some(path) = state_file_path() else {
            warn!("No HOME or XDG_STATE_HOME set; calibration cannot be loaded, using defaults");
            return Self::default();
        };

        let text = match std::fs::read_to_string(&path) {
            Ok(s) => s,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
                debug!(
                    "No calibration file at {} — using defaults (this is expected on first run)",
                    path.display()
                );
                return Self::default();
            }
            Err(e) => {
                warn!(
                    "Failed to read calibration from {}: {} — using defaults",
                    path.display(),
                    e
                );
                return Self::default();
            }
        };

        match toml::from_str::<Self>(&text) {
            Ok(cal) => {
                if !cal.yaw_offset_deg.is_finite() {
                    warn!(
                        "Calibration file {} contained non-finite yaw_offset_deg ({}); \
                         using defaults",
                        path.display(),
                        cal.yaw_offset_deg
                    );
                    return Self::default();
                }
                info!(
                    "Loaded calibration from {}: yaw_offset_deg={:.3}",
                    path.display(),
                    cal.yaw_offset_deg
                );
                cal
            }
            Err(e) => {
                warn!(
                    "Calibration file {} is malformed: {} — using defaults",
                    path.display(),
                    e
                );
                Self::default()
            }
        }
    }

    /// Persist this calibration to the XDG state file. Creates the
    /// `turret/` subdirectory if missing. Errors propagate to the
    /// caller so an IPC handler can surface them in its response.
    pub fn save(&self) -> std::io::Result<()> {
        let path = state_file_path().ok_or_else(|| {
            std::io::Error::new(
                std::io::ErrorKind::NotFound,
                "no $HOME or $XDG_STATE_HOME — cannot pick a state-file path",
            )
        })?;
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        let payload = toml::to_string_pretty(self).map_err(std::io::Error::other)?;
        std::fs::write(&path, payload)?;
        debug!(
            "Persisted calibration to {}: yaw_offset_deg={:.3}",
            path.display(),
            self.yaw_offset_deg
        );
        Ok(())
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
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
        let cal = Calibration::load();
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
        cal.save().unwrap();
        let loaded = Calibration::load();
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
        assert_eq!(Calibration::load(), Calibration::default());

        std::fs::write(&path, b"yaw_offset_deg = inf\n").unwrap();
        assert_eq!(Calibration::load(), Calibration::default());

        std::fs::write(&path, b"yaw_offset_deg = -inf\n").unwrap();
        assert_eq!(Calibration::load(), Calibration::default());
    }

    #[test]
    fn load_returns_default_on_malformed_toml() {
        let _fx = EnvFixture::new();
        let path = state_file_path().unwrap();
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(&path, b"this is = not = valid = toml").unwrap();
        assert_eq!(Calibration::load(), Calibration::default());
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
        assert_eq!(Calibration::load().yaw_offset_deg, 0.0);
    }
}
