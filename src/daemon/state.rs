//! Daemon state for measurements, sent targets, ownership, and vehicle heading.

pub mod commands;
pub mod gimbal;
pub mod primary;
mod source_map;
pub mod vehicle;

pub use commands::CommandStore;
pub use gimbal::GimbalStateStore;
pub use primary::PrimaryStore;
pub use vehicle::VehicleContext;

use super::models::{ControlSource, GimbalCommand, GimbalState, PanMode, PrimaryControl};
use std::time::Duration;

/// Shared state with independent measurement, command, and ownership stores.
#[derive(Clone, Default)]
pub struct StateManager {
    /// The gimbal's reported state.
    pub gimbal: GimbalStateStore,
    /// In-flight command + per-source rate-limit timestamps.
    pub commands: CommandStore,
    /// MAVLink primary / secondary control ownership.
    pub primary: PrimaryStore,
    /// Autopilot vehicle-state snapshot.
    pub vehicle: VehicleContext,
}

impl StateManager {
    /// Construct a fresh state manager with unknown observations and
    /// no per-source rate limit. Cheap to `clone()` — every sub-store
    /// is `Arc<RwLock<...>>` so all clones share the same buffers.
    pub fn new() -> Self {
        Self::with_min_command_interval(Duration::ZERO)
    }

    /// Construct a state manager with a per-source rate-limit floor on
    /// command acceptance. Two commands from the same `ControlSource`
    /// within `interval` will see the second one rejected. `ZERO`
    /// disables the check (matching [`Self::new`]).
    pub fn with_min_command_interval(interval: Duration) -> Self {
        Self {
            gimbal: GimbalStateStore::new(),
            commands: CommandStore::with_min_interval(interval),
            primary: PrimaryStore::new(),
            vehicle: VehicleContext::new(),
        }
    }

    // -----------------------------------------------------------------
    // Vehicle context (autopilot snapshot)
    // -----------------------------------------------------------------

    /// Record the autopilot's most recent vehicle heading (earth-frame
    /// yaw, degrees). Called from the
    /// `AUTOPILOT_STATE_FOR_GIMBAL_DEVICE` inbound handler.
    pub fn update_vehicle_yaw(&self, yaw_deg: f32) {
        self.vehicle.update_yaw(yaw_deg);
    }

    /// Vehicle yaw if we've heard from the autopilot within `max_age`,
    /// otherwise `None`.
    pub fn vehicle_yaw_deg(&self, max_age: Duration) -> Option<f32> {
        self.vehicle.yaw_deg(max_age)
    }

    // -----------------------------------------------------------------
    // Gimbal state (the gimbal's reported attitude / pan / standby)
    // -----------------------------------------------------------------

    /// Snapshot the current gimbal state.
    pub fn state_cached(&self) -> GimbalState {
        self.gimbal.snapshot_cached()
    }

    /// Record a fresh attitude measurement from the device.
    ///
    /// Called by the attitude poll loop. The arbitrator must not call
    /// this — it should only ever record `last_command`.
    pub fn update_measured_angles(&self, yaw: f32, pitch: f32, roll: f32) {
        self.gimbal.update_measured_angles(yaw, pitch, roll);
    }

    /// Update pan mode.
    pub fn update_pan_mode(&self, mode: PanMode) {
        self.gimbal.update_pan_mode(mode);
    }

    /// Update standby state.
    pub fn update_standby(&self, standby: bool) {
        self.gimbal.update_standby(standby);
    }

    /// Update firmware version.
    pub fn update_firmware_version(&self, version: String) {
        self.gimbal.update_firmware_version(version);
    }

    /// Whether the daemon last commanded the gimbal into standby.
    /// Tracks toggles the daemon itself issues; not a poll of the
    /// device, so it can drift if some other host puts the gimbal
    /// into standby out of band.
    pub fn is_standby(&self) -> bool {
        self.gimbal.is_standby()
    }

    // -----------------------------------------------------------------
    // Command store + cross-store helpers
    // -----------------------------------------------------------------

    /// Get the in-flight command.
    pub fn last_command_cached(&self) -> Option<GimbalCommand> {
        self.commands.last_sent_cached()
    }

    /// Run a command through arbitration: per-source rate limit first,
    /// then priority. Returns `true` if accepted.
    pub fn set_command(&self, command: GimbalCommand) -> bool {
        self.commands.try_accept(command)
    }

    /// Known selected-axis baseline; neither accepted targets nor missing data become zero.
    pub fn axis_baseline_cached(&self) -> (Option<f32>, Option<f32>, Option<f32>) {
        let last = self.commands.last_sent_cached();
        let measured = self.gimbal.snapshot_cached();
        let measured_valid = measured
            .measured_at
            .and_then(|at| at.elapsed().ok())
            .is_some_and(|age| age <= Duration::from_secs(1));
        let position = last
            .as_ref()
            .filter(|c| matches!(c.operation, super::models::CommandOperation::Position));
        let pick = |sent: Option<f32>, measured: Option<f32>| {
            sent.or_else(|| measured_valid.then_some(measured).flatten())
        };
        (
            pick(position.and_then(|c| c.pitch), measured.pitch),
            pick(position.and_then(|c| c.roll), measured.roll),
            pick(position.and_then(|c| c.yaw), measured.yaw),
        )
    }

    /// Invalidate feedback and targets which belong to a disconnected device.
    pub fn clear_device(&self, epoch: u64) {
        self.commands.clear_device();
        self.gimbal.invalidate(epoch);
    }

    // -----------------------------------------------------------------
    // Primary control (MAVLink ownership)
    // -----------------------------------------------------------------

    /// Current primary/secondary control ownership.
    pub fn primary_control_cached(&self) -> PrimaryControl {
        self.primary.get()
    }

    /// Set primary + secondary control. All-zero means "unset / open".
    pub fn set_primary_control(&self, pc: PrimaryControl) {
        self.primary.set(pc);
    }

    /// True if `(sysid, compid)` is authorized to drive the gimbal
    /// under the current primary-control configuration.
    pub fn is_primary_allowed(&self, sysid: u8, compid: u8) -> bool {
        self.primary.is_allowed(sysid, compid)
    }

    // -----------------------------------------------------------------
    // Cross-store cleanup
    // -----------------------------------------------------------------

    /// Drop the per-source tracking entries for `source`. Called by
    /// the IPC server's slot allocator on client disconnect so a
    /// future client claiming the same slot index inherits a clean
    /// slate instead of the previous tenant's rate-limit timestamp.
    pub fn clear_source(&self, source: &ControlSource) {
        self.commands.clear(source);
    }
}

#[cfg(test)]
mod tests;
