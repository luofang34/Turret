//! The "what the gimbal is doing right now" store: measured attitude,
//! pan mode, standby flag, firmware string. Single lock; all writes are
//! short, fixed-size mutations of the inner [`GimbalState`].
//!
//! Writers: the attitude poll loop ([`GimbalStateStore::update_observation`]),
//! IPC / MAVLink handlers for pan-mode / standby, daemon startup for
//! the firmware string. Readers: every consumer that wants to see what
//! the gimbal currently looks like — IPC `status`, the
//! `GIMBAL_DEVICE_ATTITUDE_STATUS` broadcaster, the
//! `GIMBAL_MANAGER_STATUS` builder.

use crate::daemon::models::{GimbalState, PanMode};
use parking_lot::RwLock;
use std::sync::Arc;
use std::time::{Instant, SystemTime};

#[derive(Default)]
struct MeasuredState {
    state: GimbalState,
    sampled_at: Option<Instant>,
}

/// Thread-safe wrapper around [`GimbalState`].
///
/// Uses `parking_lot::RwLock` rather than `std::sync::RwLock` so a
/// panicking writer can't poison readers and bring the daemon down.
#[derive(Clone, Default)]
pub struct GimbalStateStore {
    inner: Arc<RwLock<MeasuredState>>,
}

impl GimbalStateStore {
    /// New store seeded with [`GimbalState::default`] (unknown attitude,
    /// `measured_at: None`, no firmware string yet).
    pub fn new() -> Self {
        Self::default()
    }

    /// Snapshot the current state.
    pub fn snapshot_cached(&self) -> GimbalState {
        self.inner.read().state.clone()
    }

    /// Mark retained measurements unavailable after a device connection changes.
    pub fn invalidate(&self, epoch: u64) {
        let mut measured = self.inner.write();
        measured.sampled_at = None;
        let state = &mut measured.state;
        state.measured_at = None;
        state.pitch = None;
        state.roll = None;
        state.yaw = None;
        state.connection_epoch = epoch;
        state.firmware_version = None;
    }

    /// Accept strictly newer, fresh vehicle-frame measurements on this connection.
    ///
    /// Rejected samples retain the prior value and acquisition time, so repeated
    /// polling cannot make stale or estimated feedback appear freshly measured.
    pub fn update_observation(&self, sample: &crate::gimbal::DeviceObservation) -> bool {
        if sample.frame != turret_core::ReferenceFrame::Vehicle {
            return false;
        }
        let mut measured = self.inner.write();
        let now = Instant::now();
        let wall_now = SystemTime::now();
        let Some(sampled_at) =
            sample.measurement_time(now, crate::daemon::attitude::MAX_MEASUREMENT_AGE)
        else {
            return false;
        };
        if sample.epoch != measured.state.connection_epoch
            || measured.sampled_at.is_some_and(|last| sampled_at <= last)
        {
            return false;
        }
        measured.sampled_at = Some(sampled_at);
        let state = &mut measured.state;
        state.pitch = sample.angles.pitch.map(|v| v.as_radians().to_degrees());
        state.roll = sample.angles.roll.map(|v| v.as_radians().to_degrees());
        state.yaw = sample.angles.yaw.map(|v| v.as_radians().to_degrees());
        state.measured_at = wall_now.checked_sub(now.duration_since(sampled_at));
        true
    }

    /// Record a fresh attitude measurement from the device.
    pub fn update_measured_angles(&self, yaw: f32, pitch: f32, roll: f32) {
        let mut measured = self.inner.write();
        measured.sampled_at = Some(Instant::now());
        measured.state.update_measured_angles(yaw, pitch, roll);
    }

    /// Record the most recent pan-mode commit.
    pub fn update_pan_mode(&self, mode: PanMode) {
        self.inner.write().state.update_pan_mode(mode);
    }

    /// Record the standby toggle the daemon last issued.
    pub fn update_standby(&self, standby: bool) {
        self.inner.write().state.update_standby(standby);
    }

    /// Stash the firmware version string captured at startup.
    pub fn update_firmware_version(&self, version: String) {
        self.inner.write().state.firmware_version = Some(version);
    }

    /// True if the daemon last commanded the gimbal into standby.
    /// Tracks toggles the daemon issued — out-of-band standby commands
    /// from another host aren't reflected here.
    pub fn is_standby(&self) -> bool {
        self.inner.read().state.standby
    }
}

#[cfg(test)]
mod tests;
