//! Shared in-memory daemon state: measured attitude, last-accepted
//! command, primary/secondary control ownership. All access goes through
//! `parking_lot::RwLock` so a panicking writer can't poison readers.

use super::models::{ControlSource, GimbalCommand, GimbalState, PrimaryControl};
use parking_lot::RwLock;
use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, Instant};
use tracing::warn;

/// How long a per-source `Instant` stays in `last_command_at` /
/// `last_rate_at` before lazy eviction drops it on the next write.
/// The "normal cleanup" knob — most workloads never reach the hard
/// cap, so the TTL is what keeps map size proportional to active
/// sources. After this much silence the source is treated as fresh:
/// the rate-limit window resets and the rate integrator's `dt`
/// falls back to the minimum.
const SOURCE_TRACKING_TTL: Duration = Duration::from_secs(300);

/// Snapshot of the autopilot's vehicle state from the most recent
/// `AUTOPILOT_STATE_FOR_GIMBAL_DEVICE` message. Stored in-memory for
/// consumers that need richer context than just earth-frame yaw —
/// horizon-lock pan modes, velocity-aware lead/lag, yaw-rate
/// feed-forward on earth-frame rate commands.
///
/// All fields are decoded into the conventions the gimbal pipeline
/// uses (Euler degrees, m/s) so consumers don't have to repeat the
/// quaternion math. NaN per axis on velocity / yaw_rate is preserved —
/// the message spec says "NaN if unknown" and consumers must handle
/// that themselves; we don't substitute a fake zero.
#[derive(Debug, Clone, Copy)]
pub struct VehicleState {
    /// Vehicle pitch (deg), decoded from `q`.
    pub pitch_deg: f32,
    /// Vehicle roll (deg), decoded from `q`.
    pub roll_deg: f32,
    /// Vehicle yaw / heading (deg, earth frame), decoded from `q`.
    /// This is the field used by `yaw_to_vehicle_frame` to translate
    /// earth-frame yaw commands.
    pub yaw_deg: f32,
    /// Feed-forward yaw-rate (deg/s, earth frame). `None` if the
    /// autopilot reported NaN — meaning "not yawing actively" or
    /// "estimator can't tell." Distinct from "0.0" which is a positive
    /// claim of zero rate.
    pub yaw_rate_deg_s: Option<f32>,
    /// NED-frame velocity (m/s). NaN per axis if the autopilot reports
    /// it unknown — preserved as-is so consumers can branch on per-axis
    /// validity without losing the partial information.
    pub ned_velocity_m_s: (f32, f32, f32),
}

/// Hard upper bound on per-source tracking map size. The TTL alone
/// only protects against long-term accumulation; under a burst that
/// churns through IPC client IDs faster than the TTL can clear them
/// (a buggy or malicious client looping connect/disconnect, an
/// automated test harness, etc.) the map could still spike. When at
/// this many entries an insert evicts the single oldest one.
///
/// 256 matches MAVLink's sysid space (`u8`), so even if every
/// possible MAVLink sysid is alive we never evict a "real" MAVLink
/// source — only the rest of the map (IPC slots) churns. At
/// `~32 B/entry`, the bound is ~8 KB per map, ~16 KB across both —
/// negligible on any platform we ship to.
///
/// Eviction of an active source is functionally minor:
///   - `last_command_at`: the source's next command is treated as
///     "first time", skipping one rate-limit hit. Not security-
///     critical (gimbal-limit clamping happens earlier in the
///     pipeline; rate-limit is a fairness knob, not a safety gate).
///   - `last_rate_at`: the next rate tick falls back to the 10 ms
///     `MIN_DT`, under-shooting one position increment. Operator
///     feels a one-frame jitter; the next tick recovers.
const MAX_TRACKED_SOURCES: usize = 256;

/// Thread-safe gimbal state manager.
///
/// Uses `parking_lot::RwLock` (not `std::sync::RwLock`) so a panicking
/// writer can never poison the lock and bring the daemon down with it.
#[derive(Clone)]
pub struct StateManager {
    state: Arc<RwLock<GimbalState>>,
    last_command: Arc<RwLock<Option<GimbalCommand>>>,
    primary_control: Arc<RwLock<PrimaryControl>>,
    /// When each `ControlSource` last had a command accepted. Used by
    /// the per-source rate limiter inside [`Self::set_command`]. Empty
    /// map + `min_command_interval == ZERO` means "no throttle".
    last_command_at: Arc<RwLock<HashMap<ControlSource, Instant>>>,
    /// When each `ControlSource` last issued a *rate* command. Used by
    /// the rate→position integrator in `mavlink_manager::handlers::inbound`
    /// to compute `dt` between successive rate ticks from the same source.
    /// Distinct from `last_command_at` so a position command from the same
    /// source doesn't reset the rate timer.
    last_rate_at: Arc<RwLock<HashMap<ControlSource, Instant>>>,
    /// Most recent vehicle state from `AUTOPILOT_STATE_FOR_GIMBAL_DEVICE`,
    /// paired with its arrival timestamp. Used to translate earth-frame
    /// yaw commands to vehicle-frame and (for future consumers) to
    /// access pitch / roll / NED velocity / feed-forward yaw rate.
    /// `None` until at least one such message arrives.
    vehicle_state: Arc<RwLock<Option<(VehicleState, Instant)>>>,
    /// Minimum delta between two accepted commands from the same source.
    /// `Duration::ZERO` disables the check.
    min_command_interval: Duration,
}

impl StateManager {
    /// Construct a fresh state manager with default zero values and no
    /// per-source rate limit. Cheap to `clone()` — the inner state is
    /// `Arc<RwLock<...>>` so all clones share the same underlying buffers.
    pub fn new() -> Self {
        Self::with_min_command_interval(Duration::ZERO)
    }

    /// Construct a state manager with a per-source rate-limit floor.
    /// Two commands from the same `ControlSource` within `interval` will
    /// see the second one rejected by [`Self::set_command`]. `ZERO`
    /// disables the check (matching [`Self::new`]).
    pub fn with_min_command_interval(interval: Duration) -> Self {
        Self {
            state: Arc::new(RwLock::new(GimbalState::default())),
            last_command: Arc::new(RwLock::new(None)),
            primary_control: Arc::new(RwLock::new(PrimaryControl::default())),
            last_command_at: Arc::new(RwLock::new(HashMap::new())),
            last_rate_at: Arc::new(RwLock::new(HashMap::new())),
            vehicle_state: Arc::new(RwLock::new(None)),
            min_command_interval: interval,
        }
    }

    /// Record the autopilot's most recent vehicle state. Called from the
    /// `AUTOPILOT_STATE_FOR_GIMBAL_DEVICE` inbound handler. Replaces the
    /// previous `update_vehicle_yaw` entry-point; the yaw component is
    /// still exposed via [`Self::vehicle_yaw_deg`] for the earth-frame
    /// transform path.
    pub fn update_vehicle_state(&self, state: VehicleState) {
        *self.vehicle_state.write() = Some((state, Instant::now()));
    }

    /// Full vehicle state if we've heard from the autopilot within
    /// `max_age`, otherwise `None`. Use this when consumers need richer
    /// context than yaw — pitch / roll / yaw rate / NED velocity all
    /// come from the same packet.
    pub fn vehicle_state(&self, max_age: Duration) -> Option<VehicleState> {
        let snapshot = *self.vehicle_state.read();
        snapshot.and_then(|(s, t)| (t.elapsed() <= max_age).then_some(s))
    }

    /// Vehicle yaw if we've heard from the autopilot within `max_age`,
    /// otherwise `None`. Earth-frame yaw commands fall back to vehicle
    /// frame (with a warn) when this returns `None`. Thin convenience
    /// over [`Self::vehicle_state`] for the common single-field consumer.
    pub fn vehicle_yaw_deg(&self, max_age: Duration) -> Option<f32> {
        self.vehicle_state(max_age).map(|s| s.yaw_deg)
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
    ///   2. `measured.<axis>` if any successful poll has happened. Used
    ///      when no command has touched this axis yet (e.g. the first
    ///      partial command after boot).
    ///   3. `0.0` as a final fallback (cold boot, no measurement, no
    ///      command). Keeps behavior defined when nothing else is.
    ///
    /// Returned in `(pitch, roll, yaw)` order to match
    /// `inbound::common::clamp_pry` and the arbitrator's `Outcome::Execute`.
    pub fn axis_baseline(&self) -> (f32, f32, f32) {
        let last = self.last_command.read().clone();
        let measured = self.state.read();
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

    /// Most recent rate-command timestamp for the given source, if any.
    /// Used by the rate→position integrator to compute `dt` between
    /// consecutive rate ticks from the same source.
    pub fn last_rate_tick(&self, source: &ControlSource) -> Option<Instant> {
        self.last_rate_at.read().get(source).copied()
    }

    /// Mark `source` as having issued a rate command at `now`. The rate
    /// integrator calls this after computing the new target so the next
    /// rate tick from the same source sees the right `dt`.
    ///
    /// Bounds map growth on every write: TTL sweep (5-min entries dropped)
    /// then a hard cap that evicts the oldest entry if the map is at
    /// capacity. See `check_and_record_rate` for the same pattern on
    /// `last_command_at`.
    pub fn record_rate_tick(&self, source: ControlSource, now: Instant) {
        let mut map = self.last_rate_at.write();
        evict_for_insert(&mut map, now);
        map.insert(source, now);
    }

    /// Get current gimbal state (blocking read)
    pub fn get_state(&self) -> GimbalState {
        self.state.read().clone()
    }

    /// Record a fresh attitude measurement from the device.
    ///
    /// Called by the attitude poll loop in `mavlink_manager`. The arbitrator
    /// must not call this — it should only ever record `last_command`.
    pub fn update_measured_angles(&self, yaw: f32, pitch: f32, roll: f32) {
        let mut state = self.state.write();
        state.update_measured_angles(yaw, pitch, roll);
    }

    /// Update pan mode
    pub fn update_pan_mode(&self, mode: super::models::PanMode) {
        let mut state = self.state.write();
        state.update_pan_mode(mode);
    }

    /// Update standby state
    pub fn update_standby(&self, standby: bool) {
        let mut state = self.state.write();
        state.update_standby(standby);
    }

    /// Update firmware version
    pub fn update_firmware_version(&self, version: String) {
        let mut state = self.state.write();
        state.firmware_version = Some(version);
    }

    /// Get last command
    pub fn get_last_command(&self) -> Option<GimbalCommand> {
        self.last_command.read().clone()
    }

    /// Run a command through arbitration: per-source rate limit first,
    /// then the priority check. Returns `true` if accepted (in which case
    /// the command becomes the new `last_command` and the source's
    /// `last_command_at` advances).
    ///
    /// The rate-limit check runs on its own lock so the (more contended)
    /// `last_command` lock isn't held across the time arithmetic.
    pub fn set_command(&self, command: GimbalCommand) -> bool {
        if !self.check_and_record_rate(&command.source) {
            return false;
        }

        let mut last_cmd = self.last_command.write();

        // Check if new command has higher priority than current
        let should_accept = match &*last_cmd {
            None => true, // No previous command, accept
            Some(prev) => {
                // Accept if:
                // 1. New command has higher priority, OR
                // 2. Same priority and newer timestamp, OR
                // 3. Previous command is stale
                if prev.is_stale() {
                    true
                } else {
                    let new_priority = command.source.priority();
                    let prev_priority = prev.source.priority();

                    new_priority > prev_priority
                        || (new_priority == prev_priority && command.timestamp > prev.timestamp)
                }
            }
        };

        if should_accept {
            *last_cmd = Some(command);
            true
        } else {
            false
        }
    }

    /// Per-source rate-limit check. Returns `true` if the command is
    /// allowed to proceed; on `false` the command is rejected before the
    /// priority arbitration sees it.
    ///
    /// On `true` the source's `last_command_at` advances to `now`. The
    /// check is no-op when `min_command_interval == ZERO`. Different
    /// sources are tracked independently — a noisy autopilot doesn't
    /// throttle an IPC client.
    ///
    /// Map size is bounded by two complementary mechanisms applied on
    /// every write: a 5-minute TTL sweep (normal-path cleanup) plus a
    /// 256-entry hard cap (burst-path safety net) that evicts the
    /// oldest entry when full. This keeps a long-running daemon's
    /// memory bounded even under pathological IPC churn.
    fn check_and_record_rate(&self, source: &ControlSource) -> bool {
        if self.min_command_interval.is_zero() {
            return true;
        }
        let now = Instant::now();
        let mut map = self.last_command_at.write();
        evict_for_insert(&mut map, now);
        if let Some(prev) = map.get(source) {
            if now.duration_since(*prev) < self.min_command_interval {
                warn!(
                    "Rate-limited command from {:?}: {} ms since last (min {} ms)",
                    source,
                    now.duration_since(*prev).as_millis(),
                    self.min_command_interval.as_millis(),
                );
                return false;
            }
        }
        map.insert(source.clone(), now);
        true
    }

    /// Current primary/secondary control ownership.
    pub fn get_primary_control(&self) -> PrimaryControl {
        *self.primary_control.read()
    }

    /// Set primary + secondary control. All-zero means "unset / open".
    pub fn set_primary_control(&self, pc: PrimaryControl) {
        *self.primary_control.write() = pc;
    }

    /// True if `(sysid, compid)` is authorized to drive the gimbal under the
    /// current primary-control configuration.
    pub fn is_primary_allowed(&self, sysid: u8, compid: u8) -> bool {
        self.get_primary_control().is_allowed(sysid, compid)
    }

    /// Drop the per-source tracking entries for `source`. Called by the
    /// IPC server's slot allocator on client disconnect so a future
    /// client claiming the same slot index inherits a clean slate
    /// instead of the previous tenant's rate-limit timestamp / rate-tick
    /// `dt` baseline.
    pub fn clear_source(&self, source: &ControlSource) {
        self.last_command_at.write().remove(source);
        self.last_rate_at.write().remove(source);
    }

    /// Whether the daemon last commanded the gimbal into standby. Tracks the
    /// standby toggles the daemon itself issues; not a poll of the device, so
    /// it can drift if some other host puts the gimbal into standby out of
    /// band. Used by `GIMBAL_MANAGER_STATUS` to advertise `RETRACT`.
    pub fn is_standby(&self) -> bool {
        self.state.read().standby
    }
}

/// Bound the per-source map's size before an insert. Two passes:
///
/// 1. TTL sweep: drop every entry older than the 5-minute tracking TTL.
///    Cheap O(n) `retain`. Handles the steady-state case where the
///    working set is tiny but stale entries from disconnected clients
///    would otherwise linger.
/// 2. Hard-cap pass: if the map is *still* at capacity (every entry is
///    fresh — burst workload), drop the single oldest entry to make
///    room for the new one. O(n) find-min, only fires under the burst.
///
/// Together these guarantee post-insert `len ≤ MAX_TRACKED_SOURCES`
/// regardless of workload. See `MAX_TRACKED_SOURCES` for the rationale
/// on cap value and the (minor) functional cost of evicting an active
/// source.
fn evict_for_insert(map: &mut HashMap<ControlSource, Instant>, now: Instant) {
    map.retain(|_, t| now.duration_since(*t) <= SOURCE_TRACKING_TTL);
    if map.len() >= MAX_TRACKED_SOURCES {
        if let Some(oldest_key) = map.iter().min_by_key(|(_, t)| *t).map(|(k, _)| k.clone()) {
            map.remove(&oldest_key);
        }
    }
}

impl Default for StateManager {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
#[path = "state/tests.rs"]
mod tests;
