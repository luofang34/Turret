//! Partial device observation polling and complete-orientation broadcasts.

use crate::daemon::calibration::Calibration;
use crate::daemon::gimbal_handle::GimbalHandle;
use crate::daemon::state::StateManager;
use crate::daemon::yaw_corrector::YawCorrector;
use crate::gimbal::DeviceObservation;
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::sync::{broadcast, Notify};
use tokio::time;
use tracing::{info, warn};

/// One successful poll's worth of attitude, plus a flag derived from
/// a recent failure-linger window. Front-ends translate
/// `link_recently_failed` into protocol-specific health bits
/// (MAVLink `GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR`, …).
#[derive(Debug, Clone, Copy)]
pub struct AttitudeTick {
    /// Physical sample in operator coordinates, retaining acquisition time and source.
    pub observation: DeviceObservation,
    /// True when at least one poll within the last 1 s
    /// (`COMMS_ERROR_LINGER`) failed. Solid links clear this on the
    /// next successful tick; flaky links keep it set.
    pub link_recently_failed: bool,
}

/// Broadcast channel buffer size. 16 ticks at the ~4 Hz poll cadence
/// is ~4 s of slack — enough to ride out a sluggish broadcast loop
/// without dropping ticks, while still reporting `Lagged` rather
/// than blocking the publisher if a subscriber stalls indefinitely.
const ATTITUDE_CHANNEL_CAPACITY: usize = 16;

/// Per-tick poll cadence. ~4 Hz: low enough that the gimbal mutex
/// isn't perpetually held against MAVLink/IPC writers, high enough
/// that a MAVLink GCS sees attitude updates at a useful rate.
const POLL_INTERVAL: Duration = Duration::from_millis(250);

/// Maximum acquisition age for cached and broadcast measured attitude.
pub(crate) const MAX_MEASUREMENT_AGE: Duration = Duration::from_secs(1);

/// Number of consecutive poll failures before the supervisor is
/// pinged. 8 ticks × 250 ms ≈ 2 s — long enough to ride out a
/// transient serial hiccup, short enough that an unplug is noticed.
const FAILURE_THRESHOLD: u32 = 8;

/// How long after a failed poll the next *successful* tick still
/// carries `link_recently_failed = true`. Translates downstream into
/// `COMMS_ERROR` so a flaky link reads as "degraded" rather than
/// "healthy on every other frame."
const COMMS_ERROR_LINGER: Duration = Duration::from_secs(1);

/// Yaw corrector persistence: only write to disk after the offset has
/// slid by more than this much from the last persisted value.
const PERSIST_THRESHOLD_DEG: f32 = 0.05;

/// Yaw corrector persistence: minimum interval between two successful
/// disk writes. Caps disk write rate at 1 / minute even during a long
/// initial convergence pass.
const PERSIST_INTERVAL: Duration = Duration::from_secs(60);

/// Build the attitude broadcast channel. Returns `(tx, rx)` — `tx`
/// goes to the poll loop, `rx` (and any number of `tx.subscribe()`
/// receivers) goes to interested broadcasters.
pub fn attitude_channel() -> (
    broadcast::Sender<Arc<AttitudeTick>>,
    broadcast::Receiver<Arc<AttitudeTick>>,
) {
    broadcast::channel(ATTITUDE_CHANNEL_CAPACITY)
}

/// Poll measured state, broadcast complete orientations, and signal lost links.
pub async fn attitude_poll_loop(
    gimbal: GimbalHandle,
    state_manager: StateManager,
    notify_lost: Arc<Notify>,
    corrector: Option<YawCorrector>,
    tx: broadcast::Sender<Arc<AttitudeTick>>,
) {
    let mut interval = time::interval(POLL_INTERVAL);
    interval.set_missed_tick_behavior(time::MissedTickBehavior::Skip);
    let mut health = PollHealth::default();
    let mut persistence = DriftPersistence {
        offset: gimbal.yaw_offset_deg(),
        saved_at: None,
    };
    loop {
        interval.tick().await;
        let observation = match gimbal.observe().await {
            Ok(observation) => {
                health.recovered();
                observation
            }
            Err(error) => {
                health.failed(&error, &notify_lost);
                continue;
            }
        };
        if !state_manager.gimbal.update_observation(&observation) {
            continue;
        }
        let Ok(attitude) = observation.complete_attitude() else {
            continue;
        };
        apply_correction(
            &gimbal,
            &state_manager,
            corrector,
            attitude.yaw,
            &mut persistence,
        )
        .await;
        let tick = AttitudeTick {
            observation,
            link_recently_failed: health
                .last_failure
                .is_some_and(|at| at.elapsed() < COMMS_ERROR_LINGER),
        };
        tx.send(Arc::new(tick)).ok();
    }
}

#[derive(Default)]
struct PollHealth {
    failures: u32,
    last_failure: Option<Instant>,
}

impl PollHealth {
    fn recovered(&mut self) {
        if self.failures > 0 {
            info!(failures = self.failures, "attitude readback recovered");
        }
        self.failures = 0;
    }

    fn failed(&mut self, error: &crate::error::Error, notify_lost: &Notify) {
        self.failures = self.failures.wrapping_add(1);
        self.last_failure = Some(Instant::now());
        if self.failures == 1 || self.failures.is_multiple_of(20) {
            warn!(failures = self.failures, %error, "attitude readback failed");
        }
        if self.failures == FAILURE_THRESHOLD {
            notify_lost.notify_one();
        }
    }
}

struct DriftPersistence {
    offset: f32,
    saved_at: Option<Instant>,
}

async fn apply_correction(
    gimbal: &GimbalHandle,
    state: &StateManager,
    corrector: Option<YawCorrector>,
    yaw: f32,
    persistence: &mut DriftPersistence,
) {
    let last = state
        .last_command_cached()
        .filter(|command| matches!(command.operation, super::models::CommandOperation::Position));
    let target = last.as_ref().and_then(|command| command.yaw);
    let elapsed = last.as_ref().map(|command| command.received_at.elapsed());
    let Some(step) = corrector.and_then(|corrector| corrector.step(yaw, target, elapsed)) else {
        return;
    };
    let offset = gimbal.yaw_offset_deg() + step.correction_deg;
    gimbal.set_yaw_offset_deg(offset);
    if (offset - persistence.offset).abs() <= PERSIST_THRESHOLD_DEG
        || persistence
            .saved_at
            .is_some_and(|at| at.elapsed() < PERSIST_INTERVAL)
    {
        return;
    }
    let result = tokio::task::spawn_blocking(move || {
        Calibration {
            yaw_offset_deg: offset,
        }
        .save_blocking()
    })
    .await;
    match result {
        Ok(Ok(())) => {
            persistence.offset = offset;
            persistence.saved_at = Some(Instant::now());
            info!(offset, "yaw calibration persisted");
        }
        Ok(Err(error)) => warn!(%error, "yaw calibration persistence failed"),
        Err(error) => warn!(%error, "yaw calibration worker failed"),
    }
}

#[cfg(test)]
mod tests;
