//! Tokio-friendly handle around a sync [`crate::gimbal::GimbalDevice`].
//!
//! The serial driver under [`crate::protocols::storm32_rc`] uses
//! `std::thread::sleep` between writes and reads. Calling such a method
//! while holding a `tokio::sync::Mutex` blocks the runtime worker thread
//! for the full duration of the serial exchange (5 ms ack delay + up to
//! 100 ms read timeout per command). Under load that starves other tasks
//! on the same worker.
//!
//! `GimbalHandle` solves this by:
//!
//!   1. Storing the boxed device behind an `Arc<parking_lot::Mutex<...>>`.
//!      `parking_lot::Mutex` doesn't poison and is held only briefly.
//!   2. Running every device call inside [`tokio::task::spawn_blocking`].
//!      The blocking thread pool absorbs the sleep; the runtime worker
//!      that originated the call returns to scheduling other tasks
//!      immediately.
//!
//! Cheap to clone (just bumps an Arc). The handle is the unit of
//! sharing across IPC, MAVLink, attitude poll, and reconnect tasks —
//! nobody else touches the underlying `Box<dyn GimbalDevice>`.

use crate::error::{Error, Result};
use crate::gimbal::{Attitude, GimbalDevice};
use parking_lot::Mutex;
use std::sync::atomic::{AtomicI32, Ordering};
use std::sync::Arc;
use tokio::task;

/// Scale factor between operator-frame degrees and the in-memory
/// integer encoding. We store offsets as centi-degrees (×100) so a
/// lock-free `AtomicI32` covers ±21 million degrees of range with
/// 0.01° resolution — well past anything physically meaningful, and
/// enough precision to be invisible to the user (the storm32 driver
/// itself transmits float-degrees and reports IMU yaw at 0.01°
/// granularity, so anything finer would round-trip nowhere useful).
const CENTI_DEGREE_SCALE: f32 = 100.0;

/// Async handle to a single shared gimbal device.
///
/// All methods that touch the device run via [`tokio::task::spawn_blocking`]
/// so the calling runtime task only awaits a thread-pool slot, not the
/// underlying serial transaction. The handle is `Clone` (`Arc` inside);
/// the device behind it is never actually duplicated.
///
/// # Yaw calibration model (read-side correction)
///
/// The handle owns the daemon's yaw zero-offset. The bias being corrected
/// is `imu_yaw - device_yaw_command` — i.e. the value the IMU reports
/// when the device is commanded to operator-zero. Common cause is IMU
/// calibration drift relative to the gimbal's mechanical home.
///
/// We correct on the *read* side only:
///
/// - [`Self::set_attitude`] passes the yaw command through unchanged, so
///   the gimbal physically moves to exactly the operator-commanded angle.
/// - [`Self::get_attitude`] subtracts the offset from the IMU's report,
///   so a freshly-commanded `set_attitude(0,0,yaw)` round-trips to a
///   status read of `yaw` (within IMU noise) once the gimbal has settled.
///
/// The alternative ("add to commands, leave readback alone") moves the
/// physical gimbal past the operator's intended angle to make the IMU
/// land on it — usually the wrong trade because IMU drift is far more
/// common than a miscalibrated mechanical home.
///
/// The offset is a lock-free `AtomicI32` of centi-degrees so the 4 Hz
/// attitude poll path doesn't contend on a `Mutex`.
#[derive(Clone)]
pub struct GimbalHandle {
    inner: Arc<Mutex<Box<dyn GimbalDevice>>>,
    /// Yaw offset in centi-degrees (operator frame to device frame).
    /// `Arc` so cheap `Clone` of the handle keeps every consumer in
    /// sync with the latest calibration.
    yaw_offset_centi_deg: Arc<AtomicI32>,
}

impl GimbalHandle {
    /// Wrap an opened gimbal device for sharing across tokio tasks.
    /// Yaw offset starts at zero — load and apply persisted calibration
    /// via [`Self::set_yaw_offset_deg`] right after construction.
    pub fn new(device: Box<dyn GimbalDevice>) -> Self {
        Self {
            inner: Arc::new(Mutex::new(device)),
            yaw_offset_centi_deg: Arc::new(AtomicI32::new(0)),
        }
    }

    /// Construct a handle pre-loaded with a yaw offset (operator-frame
    /// degrees). Equivalent to `new(device)` followed by
    /// `set_yaw_offset_deg(offset)`, but expresses the "load calibration
    /// from disk before any I/O happens" intent at the call site.
    pub fn with_yaw_offset_deg(device: Box<dyn GimbalDevice>, offset_deg: f32) -> Self {
        let h = Self::new(device);
        h.set_yaw_offset_deg(offset_deg);
        h
    }

    /// Update the yaw zero-offset in operator-frame degrees. Cheap and
    /// lock-free: a single atomic store. Persistence is the caller's
    /// concern (see `daemon::calibration::Calibration::save`).
    pub fn set_yaw_offset_deg(&self, offset_deg: f32) {
        let centi = if offset_deg.is_finite() {
            (offset_deg * CENTI_DEGREE_SCALE).round() as i32
        } else {
            0
        };
        self.yaw_offset_centi_deg.store(centi, Ordering::Relaxed);
    }

    /// Read the current yaw zero-offset in operator-frame degrees.
    pub fn yaw_offset_deg(&self) -> f32 {
        self.yaw_offset_centi_deg.load(Ordering::Relaxed) as f32 / CENTI_DEGREE_SCALE
    }

    /// Command an absolute Euler attitude (degrees, operator frame).
    /// Passes through to the device unchanged — the yaw calibration is
    /// applied on the read side (see [`Self::get_attitude`]) so the
    /// physical gimbal always lands on the operator's requested angle.
    pub async fn set_attitude(&self, pitch: f32, roll: f32, yaw: f32) -> Result<()> {
        let inner = self.inner.clone();
        task::spawn_blocking(move || inner.lock().set_attitude(pitch, roll, yaw))
            .await
            .map_err(join_err)?
    }

    /// Read the device's current attitude. Yaw has the calibration offset
    /// subtracted, so a freshly-commanded `set_attitude(0,0,0)` round-trips
    /// to a status read of `(0,0,0)` once the gimbal has settled
    /// (within IMU noise).
    pub async fn get_attitude(&self) -> Result<Attitude> {
        let inner = self.inner.clone();
        let raw = task::spawn_blocking(move || inner.lock().get_attitude())
            .await
            .map_err(join_err)??;
        Ok(Attitude {
            yaw: raw.yaw - self.yaw_offset_deg(),
            ..raw
        })
    }

    /// Read the device's current attitude *raw* — without subtracting
    /// the yaw calibration offset. The calibration handler uses this to
    /// capture the bias when computing a fresh offset; nothing else
    /// should call it. Operator-facing code paths must go through
    /// [`Self::get_attitude`] so the calibration is honored.
    pub async fn get_attitude_raw(&self) -> Result<Attitude> {
        let inner = self.inner.clone();
        task::spawn_blocking(move || inner.lock().get_attitude())
            .await
            .map_err(join_err)?
    }

    /// Set pan-mode byte (0..=5 per the STorM32 RC spec). Drivers that
    /// don't support this return [`Error::Unsupported`].
    pub async fn set_pan_mode(&self, mode: u8) -> Result<()> {
        let inner = self.inner.clone();
        task::spawn_blocking(move || inner.lock().set_pan_mode(mode))
            .await
            .map_err(join_err)?
    }

    /// Toggle the device between active and standby. Drivers that don't
    /// support this return [`Error::Unsupported`].
    pub async fn set_standby(&self, enabled: bool) -> Result<()> {
        let inner = self.inner.clone();
        task::spawn_blocking(move || inner.lock().set_standby(enabled))
            .await
            .map_err(join_err)?
    }

    /// Read the device's firmware version string.
    pub async fn get_version(&self) -> Result<String> {
        let inner = self.inner.clone();
        task::spawn_blocking(move || inner.lock().get_version())
            .await
            .map_err(join_err)?
    }

    /// Protocol-name string from the underlying driver. Used for log
    /// messages on startup. Cheap (no I/O) but goes through the
    /// blocking pool for uniformity with the rest of the API.
    pub async fn protocol_name(&self) -> &'static str {
        let inner = self.inner.clone();
        task::spawn_blocking(move || inner.lock().protocol_name())
            .await
            .unwrap_or("unknown")
    }

    /// Replace the underlying device with a freshly-opened one. Used by
    /// the daemon's hot-plug recovery path: scan + re-open happen
    /// without holding the lock; only the swap itself is brief.
    pub async fn replace(&self, new_device: Box<dyn GimbalDevice>) {
        let inner = self.inner.clone();
        // A blocking-pool join error here means the swap task panicked
        // (only possible if the `Box` drop on the previous gimbal
        // panicked). Surface it as a daemon-level warn rather than
        // unwinding the caller — the new device is already in place
        // either way, since `*inner.lock()` is the last sync op.
        if let Err(e) = task::spawn_blocking(move || {
            *inner.lock() = new_device;
        })
        .await
        {
            tracing::warn!("Gimbal swap task panicked: {e}");
        }
    }
}

/// Convert a `tokio::task::JoinError` (from `spawn_blocking`) into our
/// crate error type. Only fires when the blocking task itself panicked
/// — typically a bug in a `GimbalDevice` impl.
fn join_err(e: task::JoinError) -> Error {
    Error::Daemon(format!("gimbal blocking task: {e}"))
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    //! Cover the calibration application: the operator-frame yaw the
    //! handle sends down (via `set_attitude`) and reports back up (via
    //! `get_attitude`) must round-trip correctly through whatever
    //! offset is set, while pitch/roll pass through untouched.

    use super::*;
    use crate::error::Result as TurretResult;

    /// Dual-role mock: records the most recent `set_attitude` call so
    /// the test can assert what reached the device, and lets the test
    /// preload a `get_attitude` reading to assert what came back to the
    /// caller.
    struct MockGimbal {
        last_set: Arc<Mutex<Option<(f32, f32, f32)>>>,
        next_get: Arc<Mutex<Attitude>>,
    }

    impl GimbalDevice for MockGimbal {
        fn protocol_name(&self) -> &'static str {
            "mock"
        }
        fn set_attitude(&mut self, pitch: f32, roll: f32, yaw: f32) -> TurretResult<()> {
            *self.last_set.lock() = Some((pitch, roll, yaw));
            Ok(())
        }
        fn get_attitude(&mut self) -> TurretResult<Attitude> {
            Ok(*self.next_get.lock())
        }
    }

    /// `mock_handles()` return bundle. Named struct so `clippy::type_complexity`
    /// stays happy and so each test reads `mh.handle / mh.last_set / mh.next_get`
    /// instead of unpacking a 3-tuple.
    struct MockHandles {
        handle: GimbalHandle,
        last_set: Arc<Mutex<Option<(f32, f32, f32)>>>,
        next_get: Arc<Mutex<Attitude>>,
    }

    fn mock_handles() -> MockHandles {
        let last_set = Arc::new(Mutex::new(None));
        let next_get = Arc::new(Mutex::new(Attitude {
            pitch: 0.0,
            roll: 0.0,
            yaw: 0.0,
        }));
        let dev = Box::new(MockGimbal {
            last_set: last_set.clone(),
            next_get: next_get.clone(),
        });
        MockHandles {
            handle: GimbalHandle::new(dev),
            last_set,
            next_get,
        }
    }

    #[tokio::test]
    async fn set_attitude_passes_yaw_through_unchanged() {
        // Read-side correction model: commands hit the device exactly
        // as the operator wrote them. The gimbal physically goes to
        // the asked-for angle.
        let mh = mock_handles();
        mh.handle.set_yaw_offset_deg(-4.5);

        mh.handle.set_attitude(0.0, 0.0, 0.0).await.unwrap();
        let (p, r, y) = mh.last_set.lock().expect("set_attitude was called");
        assert_eq!(p, 0.0);
        assert_eq!(r, 0.0);
        assert_eq!(y, 0.0, "device yaw should equal operator yaw");

        mh.handle.set_attitude(0.0, 0.0, 30.0).await.unwrap();
        let (_, _, y) = mh.last_set.lock().expect("set_attitude was called");
        assert_eq!(y, 30.0, "device yaw should equal operator yaw");
    }

    #[tokio::test]
    async fn get_attitude_subtracts_yaw_offset_from_device_read() {
        // After calibration captures bias=-4.5° (the IMU's reading at
        // device-zero), reading the IMU at any operator command should
        // round-trip: imu reports `command + bias`, get_attitude returns
        // `imu - bias = command`.
        let mh = mock_handles();
        mh.handle.set_yaw_offset_deg(-4.5);

        // After commanding 0: IMU reports raw -4.5 (the bias). Operator
        // sees 0.
        *mh.next_get.lock() = Attitude {
            pitch: 0.0,
            roll: 0.0,
            yaw: -4.5,
        };
        let att = mh.handle.get_attitude().await.unwrap();
        assert_eq!(att.pitch, 0.0);
        assert_eq!(att.roll, 0.0);
        assert!(
            att.yaw.abs() < 1e-4,
            "operator-frame yaw should be ~0, got {}",
            att.yaw
        );

        // After commanding 20: IMU reports raw 15.5 (= 20 + bias).
        // Operator sees 20.
        *mh.next_get.lock() = Attitude {
            pitch: 0.0,
            roll: 0.0,
            yaw: 15.5,
        };
        let att = mh.handle.get_attitude().await.unwrap();
        assert!(
            (att.yaw - 20.0).abs() < 1e-4,
            "operator-frame yaw should be 20, got {}",
            att.yaw
        );
    }

    #[tokio::test]
    async fn pitch_and_roll_are_passthrough() {
        let mh = mock_handles();
        mh.handle.set_yaw_offset_deg(7.0);

        // Pitch and roll are not calibrated — pass through both directions.
        mh.handle.set_attitude(12.0, -3.0, 0.0).await.unwrap();
        let (p, r, _) = mh.last_set.lock().unwrap();
        assert_eq!(p, 12.0);
        assert_eq!(r, -3.0);

        *mh.next_get.lock() = Attitude {
            pitch: 42.0,
            roll: -8.0,
            yaw: 7.0, // raw IMU reading; with offset=7, operator-frame yaw is 0
        };
        let att = mh.handle.get_attitude().await.unwrap();
        assert_eq!(att.pitch, 42.0);
        assert_eq!(att.roll, -8.0);
        assert!(att.yaw.abs() < 1e-4);
    }

    #[tokio::test]
    async fn get_attitude_raw_skips_offset() {
        let mh = mock_handles();
        mh.handle.set_yaw_offset_deg(-4.5);
        *mh.next_get.lock() = Attitude {
            pitch: 0.0,
            roll: 0.0,
            yaw: -4.5,
        };

        // Calibration code path needs to see device-frame yaw to compute
        // a fresh offset.
        let raw = mh.handle.get_attitude_raw().await.unwrap();
        assert!(
            (raw.yaw - -4.5).abs() < 1e-4,
            "raw yaw should be -4.5, got {}",
            raw.yaw
        );
    }

    #[test]
    fn non_finite_offset_is_clamped_to_zero() {
        // Defensive — a malformed disk file or a typo'd IPC payload
        // shouldn't poison the offset and break every subsequent yaw
        // command. NaN/inf coerce to zero, the safe identity.
        let mh = mock_handles();
        mh.handle.set_yaw_offset_deg(f32::NAN);
        assert_eq!(mh.handle.yaw_offset_deg(), 0.0);
        mh.handle.set_yaw_offset_deg(f32::INFINITY);
        assert_eq!(mh.handle.yaw_offset_deg(), 0.0);
        mh.handle.set_yaw_offset_deg(f32::NEG_INFINITY);
        assert_eq!(mh.handle.yaw_offset_deg(), 0.0);

        mh.handle.set_yaw_offset_deg(-3.25);
        assert!((mh.handle.yaw_offset_deg() - -3.25).abs() < 1e-4);
    }
}
