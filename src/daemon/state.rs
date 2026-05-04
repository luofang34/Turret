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
/// `last_rate_at` before lazy eviction drops it. After this much
/// silence the source is treated as fresh — the rate-limit window
/// resets and the rate integrator's `dt` falls back to the minimum.
///
/// The maps would otherwise grow without bound under workloads that
/// churn IPC client IDs (each new connection gets a unique
/// `UnixSocket(u32)`) — at 1000 clients/day a 30-day daemon
/// accumulates ~30k entries (~1 MB). Five minutes is long enough
/// that the same active source isn't churned in and out of the map
/// during normal use, short enough that disconnected-client entries
/// don't linger past a typical operator session.
const SOURCE_TRACKING_TTL: Duration = Duration::from_secs(300);

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
    /// Most recent vehicle yaw (degrees, earth frame) reported by an
    /// `AUTOPILOT_STATE_FOR_GIMBAL_DEVICE` message, plus its arrival
    /// timestamp. Used to translate earth-frame yaw commands to
    /// vehicle-frame before sending to the device. `None` until at
    /// least one such message arrives.
    vehicle_attitude: Arc<RwLock<Option<(f32, Instant)>>>,
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
            vehicle_attitude: Arc::new(RwLock::new(None)),
            min_command_interval: interval,
        }
    }

    /// Record the autopilot's reported vehicle yaw (degrees, earth frame).
    /// Called from the `AUTOPILOT_STATE_FOR_GIMBAL_DEVICE` inbound handler.
    pub fn update_vehicle_yaw(&self, yaw_deg: f32) {
        *self.vehicle_attitude.write() = Some((yaw_deg, Instant::now()));
    }

    /// Vehicle yaw if we've heard from the autopilot within `max_age`,
    /// otherwise `None`. Earth-frame yaw commands fall back to vehicle
    /// frame (with a warn) when this returns `None`.
    pub fn vehicle_yaw_deg(&self, max_age: Duration) -> Option<f32> {
        let snapshot = *self.vehicle_attitude.read();
        snapshot.and_then(|(yaw, t)| (t.elapsed() <= max_age).then_some(yaw))
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
    /// Lazily evicts entries older than the per-source tracking TTL
    /// (5 min) on every write to bound map growth. See
    /// `check_and_record_rate` for the same pattern on `last_command_at`.
    pub fn record_rate_tick(&self, source: ControlSource, now: Instant) {
        let mut map = self.last_rate_at.write();
        evict_stale(&mut map, now);
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
    /// Map size is bounded via lazy eviction (entries older than the
    /// 5-minute per-source tracking TTL are dropped on every write),
    /// so a long-running daemon serving many transient IPC clients
    /// approximates "size = currently-active sources" instead of
    /// "size = sources ever seen".
    fn check_and_record_rate(&self, source: &ControlSource) -> bool {
        if self.min_command_interval.is_zero() {
            return true;
        }
        let now = Instant::now();
        let mut map = self.last_command_at.write();
        evict_stale(&mut map, now);
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
}

/// Drop entries from a `(ControlSource, Instant)` map whose timestamp is
/// older than [`SOURCE_TRACKING_TTL`]. Called on every write to the
/// per-source tracking maps so the daemon doesn't accumulate one entry
/// per IPC client lifetime over a multi-week run.
///
/// `retain` is O(map_size); for typical workloads (handful of active
/// sources) this is microseconds, well below the cost of the serial
/// transaction that follows.
fn evict_stale(map: &mut HashMap<ControlSource, Instant>, now: Instant) {
    map.retain(|_, t| now.duration_since(*t) <= SOURCE_TRACKING_TTL);
}

impl Default for StateManager {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
#[path = "state/tests.rs"]
mod tests;
