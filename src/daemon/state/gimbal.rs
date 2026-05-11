//! The "what the gimbal is doing right now" store: measured attitude,
//! pan mode, standby flag, firmware string. Single lock; all writes are
//! short, fixed-size mutations of the inner [`GimbalState`].
//!
//! Writers: the attitude poll loop ([`GimbalStateStore::update_measured_angles`]),
//! IPC / MAVLink handlers for pan-mode / standby, daemon startup for
//! the firmware string. Readers: every consumer that wants to see what
//! the gimbal currently looks like — IPC `status`, the
//! `GIMBAL_DEVICE_ATTITUDE_STATUS` broadcaster, the
//! `GIMBAL_MANAGER_STATUS` builder.

use crate::daemon::models::{GimbalState, PanMode};
use parking_lot::RwLock;
use std::sync::Arc;

/// Thread-safe wrapper around [`GimbalState`].
///
/// Uses `parking_lot::RwLock` rather than `std::sync::RwLock` so a
/// panicking writer can't poison readers and bring the daemon down.
#[derive(Clone, Default)]
pub struct GimbalStateStore {
    inner: Arc<RwLock<GimbalState>>,
}

impl GimbalStateStore {
    /// New store seeded with [`GimbalState::default`] (zero attitude,
    /// `measured_at: None`, no firmware string yet).
    pub fn new() -> Self {
        Self::default()
    }

    /// Snapshot the current state.
    pub fn snapshot(&self) -> GimbalState {
        self.inner.read().clone()
    }

    /// Record a fresh attitude measurement from the device.
    pub fn update_measured_angles(&self, yaw: f32, pitch: f32, roll: f32) {
        self.inner.write().update_measured_angles(yaw, pitch, roll);
    }

    /// Record the most recent pan-mode commit.
    pub fn update_pan_mode(&self, mode: PanMode) {
        self.inner.write().update_pan_mode(mode);
    }

    /// Record the standby toggle the daemon last issued.
    pub fn update_standby(&self, standby: bool) {
        self.inner.write().update_standby(standby);
    }

    /// Stash the firmware version string captured at startup.
    pub fn update_firmware_version(&self, version: String) {
        self.inner.write().firmware_version = Some(version);
    }

    /// True if the daemon last commanded the gimbal into standby.
    /// Tracks toggles the daemon issued — out-of-band standby commands
    /// from another host aren't reflected here.
    pub fn is_standby(&self) -> bool {
        self.inner.read().standby
    }
}
