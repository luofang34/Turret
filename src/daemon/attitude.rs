//! Hardware attitude polling — the protocol-agnostic core that owns
//! the only writes to `state.measured_*`, the failure-counter that
//! signals the reconnect supervisor, and the per-tick yaw drift
//! corrector.
//!
//! Per the protocol-pluggability stance (see the `daemon` module
//! docstring): polling, state updates, and reconnect signaling are
//! protocol-agnostic concerns. They run as a daemon-level task so an
//! operator who disables MAVLink (`mavlink.enabled: false`) still
//! gets a fresh `state.measured_*` on every IPC `status` and a
//! working hot-unplug path. Earlier the loop lived inside
//! `MavlinkManager` and was conditional on `mavlink.enabled`, so a
//! MAVLink-off deployment had a half-broken daemon.
//!
//! Front-ends that want to broadcast attitude (MAVLink's
//! `GIMBAL_DEVICE_ATTITUDE_STATUS`, a future CCSDS peer, anything
//! else) subscribe to the [`AttitudeTick`] broadcast channel
//! produced by [`attitude_channel`]. Each successful poll publishes
//! one tick; subscribers translate the protocol-agnostic
//! `link_recently_failed` flag into protocol-specific health bits.

use crate::daemon::calibration::Calibration;
use crate::daemon::gimbal_handle::GimbalHandle;
use crate::daemon::state::StateManager;
use crate::daemon::yaw_corrector::YawCorrector;
use crate::gimbal::Attitude;
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
    /// Operator-frame attitude (yaw already calibration-corrected by
    /// `GimbalHandle`).
    pub attitude: Attitude,
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
    broadcast::Sender<AttitudeTick>,
    broadcast::Receiver<AttitudeTick>,
) {
    broadcast::channel(ATTITUDE_CHANNEL_CAPACITY)
}

/// Hardware attitude poll loop. Runs forever; protocol-agnostic.
///
/// On every successful tick:
///   1. Writes `state.measured_*`.
///   2. Steps the optional yaw drift corrector and persists the
///      offset (throttled to ≤ 1 disk write per minute, ≥ 0.05° change).
///   3. Publishes an [`AttitudeTick`] for any subscribed front-end.
///
/// On failure:
///   - Logs at `warn` (rate-limited: 1st failure, then every 20th).
///   - Pings `notify_lost` exactly once when reaching the failure
///     threshold (8 consecutive ticks ≈ 2 s of silence), handing
///     recovery to the daemon's reconnect supervisor.
///   - Skips state updates and the broadcast for this tick.
pub async fn attitude_poll_loop(
    gimbal: GimbalHandle,
    state_manager: StateManager,
    notify_lost: Arc<Notify>,
    corrector: Option<YawCorrector>,
    tx: broadcast::Sender<AttitudeTick>,
) {
    let mut interval = time::interval(POLL_INTERVAL);
    interval.set_missed_tick_behavior(time::MissedTickBehavior::Skip);
    // We've waited POLL_INTERVAL for the first tick — fire immediately instead.
    interval.tick().await;

    let mut consecutive_failures: u32 = 0;
    let mut last_failure: Option<Instant> = None;
    // Yaw corrector persistence state. Seeded with the offset that was
    // already loaded into `GimbalHandle` at boot (from the XDG state
    // file via `Calibration::load()`); the corrector only triggers a
    // disk write when its accumulated drift correction moves the
    // offset materially away from that seed.
    let mut last_persisted_offset: f32 = gimbal.yaw_offset_deg();
    let mut last_persist_at: Option<Instant> = None;

    loop {
        interval.tick().await;

        let attitude = match gimbal.get_attitude().await {
            Ok(a) => {
                if consecutive_failures > 0 {
                    info!(
                        "Attitude readback recovered after {} consecutive failures",
                        consecutive_failures
                    );
                }
                consecutive_failures = 0;
                a
            }
            Err(e) => {
                consecutive_failures = consecutive_failures.wrapping_add(1);
                last_failure = Some(Instant::now());
                if consecutive_failures == 1 || consecutive_failures % 20 == 0 {
                    warn!(
                        "Attitude readback failed (#{}): {}",
                        consecutive_failures, e
                    );
                }
                if consecutive_failures == FAILURE_THRESHOLD {
                    warn!(
                        "Gimbal silent for {} consecutive polls; signaling reconnect",
                        FAILURE_THRESHOLD
                    );
                    notify_lost.notify_one();
                }
                continue;
            }
        };

        state_manager.update_measured_angles(attitude.yaw, attitude.pitch, attitude.roll);

        // Continuous yaw drift correction. When the gimbal has been
        // settled at the most-recent SP for at least `SETTLE_AFTER`,
        // the operator-frame `PV - SP` gap is treated as accumulated
        // drift and an `α`-weighted slice of it is added to the
        // calibration offset. See `daemon::yaw_corrector` for the
        // math + non-goals. No-op until at least one `set` has gone
        // through the arbitrator. Skipped entirely when the operator
        // has set `gimbal.yaw_corrector_enabled = false` in the TOML
        // (typical for actively-yawing flight in pan-mode PAN).
        let last_command = state_manager.get_last_command();
        let last_sp_yaw = last_command.as_ref().and_then(|c| c.yaw);
        let time_since_set = last_command
            .as_ref()
            .and_then(|c| c.timestamp.elapsed().ok());
        if let Some(step) =
            corrector.and_then(|c| c.step(attitude.yaw, last_sp_yaw, time_since_set))
        {
            let new_offset = gimbal.yaw_offset_deg() + step.correction_deg;
            gimbal.set_yaw_offset_deg(new_offset);
            // Throttled persistence — disk write only when the offset
            // has slid more than a noise-band's worth from what's on
            // disk, AND the last save was a while ago. Bounded
            // disk-write rate (~1/min worst case) regardless of
            // correction activity. Persistence failure is non-fatal —
            // the in-memory offset is still applied for this run.
            if (new_offset - last_persisted_offset).abs() > PERSIST_THRESHOLD_DEG
                && last_persist_at
                    .map(|t| t.elapsed() >= PERSIST_INTERVAL)
                    .unwrap_or(true)
            {
                let cal = Calibration {
                    yaw_offset_deg: new_offset,
                };
                match cal.save() {
                    Ok(()) => {
                        info!(
                            "Yaw corrector: persisted offset {:.3}° (was {:.3}°, error {:.3}°)",
                            new_offset, last_persisted_offset, step.error_deg
                        );
                        last_persisted_offset = new_offset;
                        last_persist_at = Some(Instant::now());
                    }
                    Err(e) => {
                        warn!(
                            "Yaw corrector: persist failed (in-memory offset still applied): {}",
                            e
                        );
                    }
                }
            }
        }

        // A *recent* failure (this poll succeeded but one within the
        // last second didn't) means the link is flaky — surface that
        // to subscribers via `link_recently_failed`. Front-ends
        // translate to whatever wire-level health bit applies.
        let link_recently_failed = matches!(
            last_failure,
            Some(t) if t.elapsed() < COMMS_ERROR_LINGER
        );

        // Best-effort send. `Err` means there are no active
        // subscribers — expected when `mavlink.enabled: false` and no
        // other front-end has subscribed; the poll loop is still the
        // source of truth for `state` writes and `notify_lost`, both
        // of which have already happened above. Match-discard rather
        // than `let _ = …` so `let_underscore_must_use` stays loud
        // for actual oversights.
        let tick = AttitudeTick {
            attitude,
            link_recently_failed,
        };
        match tx.send(tick) {
            Ok(_) | Err(_) => {}
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;

    #[test]
    fn attitude_channel_publishes_to_multiple_subscribers() {
        // The channel must support fan-out: MAVLink today, plus
        // future protocol broadcasters (CCSDS, custom mission-bus)
        // tomorrow. Verify two independent receivers each see the
        // same tick, and that a third late-subscribe sees only
        // future ticks (broadcast semantics).
        let (tx, rx_a) = attitude_channel();
        let mut rx_a = rx_a;
        let mut rx_b = tx.subscribe();

        let tick_one = AttitudeTick {
            attitude: Attitude {
                pitch: 1.0,
                roll: 2.0,
                yaw: 3.0,
            },
            link_recently_failed: false,
        };
        tx.send(tick_one).unwrap();

        let received_a = rx_a.try_recv().unwrap();
        let received_b = rx_b.try_recv().unwrap();
        assert_eq!(received_a.attitude.pitch, 1.0);
        assert_eq!(received_b.attitude.pitch, 1.0);

        // Late subscriber misses `tick_one` (broadcast = "from now on").
        let mut rx_c = tx.subscribe();
        assert!(rx_c.try_recv().is_err());

        let tick_two = AttitudeTick {
            attitude: Attitude {
                pitch: 4.0,
                roll: 5.0,
                yaw: 6.0,
            },
            link_recently_failed: true,
        };
        tx.send(tick_two).unwrap();
        assert_eq!(rx_a.try_recv().unwrap().attitude.pitch, 4.0);
        assert_eq!(rx_b.try_recv().unwrap().attitude.pitch, 4.0);
        let third = rx_c.try_recv().unwrap();
        assert_eq!(third.attitude.pitch, 4.0);
        assert!(third.link_recently_failed);
    }

    #[test]
    fn attitude_channel_send_with_no_subscribers_does_not_block_or_error_silently() {
        // The poll loop drops the initial receiver after splitting the
        // tx; subsequent sends with zero subscribers must return Err
        // (broadcast semantics) without blocking. The poll loop
        // discards the result, so this is just confirming the
        // contract we rely on.
        let (tx, rx) = attitude_channel();
        drop(rx);
        let tick = AttitudeTick {
            attitude: Attitude {
                pitch: 0.0,
                roll: 0.0,
                yaw: 0.0,
            },
            link_recently_failed: false,
        };
        // Err on no-subscribers, but no panic — exactly what the poll
        // loop's `let _ = tx.send(...)` relies on.
        assert!(tx.send(tick).is_err());
    }
}
