//! Autopilot vehicle-state snapshot.
//!
//! Only the heading (earth-frame yaw) is kept today; it's the input to
//! the earth→vehicle yaw transform that translates GCS-issued
//! earth-frame yaw commands into the body-frame the gimbal accepts.
//! Other fields of `AUTOPILOT_STATE_FOR_GIMBAL_DEVICE` (pitch, roll,
//! NED velocity, feed-forward yaw rate) are dropped on ingest — re-add
//! them here alongside a real consumer.

use parking_lot::RwLock;
use std::sync::Arc;
use std::time::{Duration, Instant};

/// Last-heard vehicle heading plus the instant it arrived. Tuple
/// (heading degrees, when it was reported).
#[derive(Clone, Default)]
pub struct VehicleContext {
    inner: Arc<RwLock<Option<(f32, Instant)>>>,
}

impl VehicleContext {
    /// Empty context — `vehicle_yaw_deg(_)` returns `None` until the
    /// first update.
    pub fn new() -> Self {
        Self::default()
    }

    /// Record the autopilot's most recent vehicle heading (degrees,
    /// earth frame). The arrival timestamp is captured at call time.
    pub fn update_yaw(&self, yaw_deg: f32) {
        *self.inner.write() = Some((yaw_deg, Instant::now()));
    }

    /// Vehicle yaw if we've heard from the autopilot within `max_age`,
    /// otherwise `None`. Earth-frame yaw commands fall back to
    /// vehicle-frame interpretation (with a warn) when this returns
    /// `None`.
    pub fn yaw_deg(&self, max_age: Duration) -> Option<f32> {
        let snapshot = *self.inner.read();
        snapshot.and_then(|(y, t)| (t.elapsed() <= max_age).then_some(y))
    }
}
