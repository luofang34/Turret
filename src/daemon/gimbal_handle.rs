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
use std::sync::Arc;
use tokio::task;

/// Async handle to a single shared gimbal device.
///
/// All methods that touch the device run via [`tokio::task::spawn_blocking`]
/// so the calling runtime task only awaits a thread-pool slot, not the
/// underlying serial transaction. The handle is `Clone` (`Arc` inside);
/// the device behind it is never actually duplicated.
#[derive(Clone)]
pub struct GimbalHandle {
    inner: Arc<Mutex<Box<dyn GimbalDevice>>>,
}

impl GimbalHandle {
    /// Wrap an opened gimbal device for sharing across tokio tasks.
    pub fn new(device: Box<dyn GimbalDevice>) -> Self {
        Self {
            inner: Arc::new(Mutex::new(device)),
        }
    }

    /// Command an absolute Euler attitude (degrees).
    pub async fn set_attitude(&self, pitch: f32, roll: f32, yaw: f32) -> Result<()> {
        let inner = self.inner.clone();
        task::spawn_blocking(move || inner.lock().set_attitude(pitch, roll, yaw))
            .await
            .map_err(join_err)?
    }

    /// Read the device's current attitude.
    pub async fn get_attitude(&self) -> Result<Attitude> {
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
