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
    pub fn load_blocking() -> Self {
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
    pub fn save_blocking(&self) -> std::io::Result<()> {
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
mod tests;
