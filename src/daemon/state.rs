//! Shared in-memory daemon state.
//!
//! Split into five focused sub-stores under `daemon/state/`:
//!
//! - [`GimbalStateStore`] — the gimbal's reported state (measured
//!   attitude, pan mode, standby flag, firmware string).
//! - [`CommandStore`] — last-accepted command + per-source
//!   rate-limit timestamps that gate new commands.
//! - [`PrimaryStore`] — MAVLink primary / secondary control
//!   ownership.
//! - [`RateTracker`] — per-source rate-tick timestamps for the
//!   rate→position integrator.
//! - [`VehicleContext`] — autopilot vehicle-state snapshot
//!   (currently just yaw heading).
//!
//! Each sub-store owns one lock and one concern. They're individually
//! `Clone`able `Arc`-wrappers so any subsystem that only needs one
//! slice can take ownership of just that slice without dragging the
//! whole daemon state along.
//!
//! [`StateManager`] is the thin coordinator that composes all five.
//! Existing consumers go through its delegating methods for backward
//! compatibility; new code can reach into the sub-stores directly via
//! the public fields.

#[path = "state/commands.rs"]
pub mod commands;
#[path = "state/gimbal.rs"]
pub mod gimbal;
#[path = "state/primary.rs"]
pub mod primary;
#[path = "state/rate.rs"]
pub mod rate;
#[path = "state/source_map.rs"]
mod source_map;
#[path = "state/vehicle.rs"]
pub mod vehicle;

pub use commands::CommandStore;
pub use gimbal::GimbalStateStore;
pub use primary::PrimaryStore;
pub use rate::RateTracker;
pub use vehicle::VehicleContext;

use super::models::{ControlSource, GimbalCommand, GimbalState, PanMode, PrimaryControl};
use std::time::{Duration, Instant};

/// Thread-safe gimbal state manager. Coordinates the five focused
/// sub-stores ([`GimbalStateStore`], [`CommandStore`], [`PrimaryStore`],
/// [`RateTracker`], [`VehicleContext`]).
///
/// Each sub-store is `Clone` and shareable on its own — consumers that
/// only need one slice (a future gimbal-state observer that doesn't
/// care about arbitration, a primary-control admin tool that doesn't
/// touch attitude) can reach into the named field directly. The
/// delegating methods on `StateManager` keep existing call sites
/// working unchanged.
#[derive(Clone, Default)]
pub struct StateManager {
    /// The gimbal's reported state.
    pub gimbal: GimbalStateStore,
    /// In-flight command + per-source rate-limit timestamps.
    pub commands: CommandStore,
    /// MAVLink primary / secondary control ownership.
    pub primary: PrimaryStore,
    /// Per-source rate-tick timestamps.
    pub rate_tracker: RateTracker,
    /// Autopilot vehicle-state snapshot.
    pub vehicle: VehicleContext,
}

impl StateManager {
    /// Construct a fresh state manager with default zero values and
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
            rate_tracker: RateTracker::new(),
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
    pub fn get_state(&self) -> GimbalState {
        self.gimbal.snapshot()
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
    /// into standby out of band. Used by `GIMBAL_MANAGER_STATUS` to
    /// advertise `RETRACT`.
    pub fn is_standby(&self) -> bool {
        self.gimbal.is_standby()
    }

    // -----------------------------------------------------------------
    // Command store + cross-store helpers
    // -----------------------------------------------------------------

    /// Get the in-flight command.
    pub fn get_last_command(&self) -> Option<GimbalCommand> {
        self.commands.last()
    }

    /// Run a command through arbitration: per-source rate limit first,
    /// then priority. Returns `true` if accepted.
    pub fn set_command(&self, command: GimbalCommand) -> bool {
        self.commands.try_accept(command)
    }

    /// Per-axis baseline for "this axis is not in the current command —
    /// keep it where I last set it."
    ///
    /// MAVLink GMv2 lets a SET_PITCHYAW / DO_GIMBAL_MANAGER_PITCHYAW /
    /// SET_MANUAL_CONTROL NaN-out an angle field to mean "ignore this
    /// axis"; the same intent is encoded as `None` on a `GimbalCommand`
    /// axis throughout the daemon (IPC `pitch` / `roll` / `yaw`
    /// subcommands are also single-axis). Substituting `0.0` for an
    /// omitted axis would silently snap that axis to center on every
    /// partial command — the opposite of what the spec asks for.
    ///
    /// Per-axis resolution order:
    ///   1. `last_command.<axis>` if `Some(_)`. The most recent
    ///      *commanded* value is what the user intended; it survives
    ///      IMU noise and yaw drift correction nudges.
    ///   2. `measured.<axis>` if any successful poll has happened.
    ///      Used when no command has touched this axis yet (e.g. the
    ///      first partial command after boot).
    ///   3. `0.0` as a final fallback (cold boot, no measurement, no
    ///      command). Keeps behavior defined when nothing else is.
    ///
    /// Returned in `(pitch, roll, yaw)` order to match
    /// `inbound::common::clamp_pry` and the arbitrator's
    /// `Outcome::Execute`.
    ///
    /// This is the one cross-store read in the daemon: pulls from
    /// both [`Self::commands`] and [`Self::gimbal`].
    pub fn axis_baseline(&self) -> (f32, f32, f32) {
        let last = self.commands.last();
        let measured = self.gimbal.snapshot();
        let measured_valid = measured.measured_at.is_some();
        let pick = |last_axis: Option<f32>, measured_axis: f32| -> f32 {
            last_axis
                .or_else(|| measured_valid.then_some(measured_axis))
                .unwrap_or(0.0)
        };
        (
            pick(last.as_ref().and_then(|c| c.pitch), measured.pitch),
            pick(last.as_ref().and_then(|c| c.roll), measured.roll),
            pick(last.as_ref().and_then(|c| c.yaw), measured.yaw),
        )
    }

    // -----------------------------------------------------------------
    // Primary control (MAVLink ownership)
    // -----------------------------------------------------------------

    /// Current primary/secondary control ownership.
    pub fn get_primary_control(&self) -> PrimaryControl {
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
    // Rate tracker (per-source rate-tick timing)
    // -----------------------------------------------------------------

    /// Most recent rate-command timestamp for `source`, if any.
    pub fn last_rate_tick(&self, source: &ControlSource) -> Option<Instant> {
        self.rate_tracker.last_tick(source)
    }

    /// Mark `source` as having issued a rate command at `now`.
    pub fn record_rate_tick(&self, source: ControlSource, now: Instant) {
        self.rate_tracker.record_tick(source, now);
    }

    // -----------------------------------------------------------------
    // Cross-store cleanup
    // -----------------------------------------------------------------

    /// Drop the per-source tracking entries for `source`. Called by
    /// the IPC server's slot allocator on client disconnect so a
    /// future client claiming the same slot index inherits a clean
    /// slate instead of the previous tenant's rate-limit timestamp /
    /// rate-tick `dt` baseline.
    pub fn clear_source(&self, source: &ControlSource) {
        self.commands.clear(source);
        self.rate_tracker.clear(source);
    }
}

#[cfg(test)]
#[path = "state/tests.rs"]
mod tests;
