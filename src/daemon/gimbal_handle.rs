//! Single device owner integrating portable execution with host I/O.

use crate::error::{Error, Result};
use crate::gimbal::{DeviceObservation, GimbalDevice, PayloadLane};
use parking_lot::Mutex;
use std::sync::atomic::{AtomicI32, Ordering};
use std::sync::Arc;
use std::time::Instant;
use tokio::task;
use turret_core::{CommandProgress, Effect, PayloadCapabilities, PayloadCommand};

mod execution;
mod lanes;
mod lifecycle;
mod observation;
use lanes::Fences;
mod timing;
use execution::DeviceSlot;
pub use timing::HostCommandTiming;
#[cfg(test)]
mod tests;

/// One shared payload, one transport owner, and one local execution clock.
#[derive(Clone)]
pub struct GimbalHandle {
    fences: Arc<Fences>,
    inner: Arc<Mutex<DeviceSlot>>,
    yaw_offset_centi_deg: Arc<AtomicI32>,
}

impl GimbalHandle {
    /// Construct a handle after checking the backend's capability contract.
    pub fn new(device: Box<dyn GimbalDevice>) -> Result<Self> {
        let fences = Arc::new(Fences::default());
        Ok(Self {
            fences: fences.clone(),
            inner: Arc::new(Mutex::new(DeviceSlot::new(device, fences)?)),
            yaw_offset_centi_deg: Arc::new(AtomicI32::new(0)),
        })
    }

    /// Construct a handle with a persisted read-side yaw calibration.
    pub fn with_yaw_offset_deg(device: Box<dyn GimbalDevice>, offset_deg: f32) -> Result<Self> {
        let handle = Self::new(device)?;
        handle.set_yaw_offset_deg(offset_deg);
        Ok(handle)
    }

    /// Record an explicitly supplied calibration; no device motion occurs.
    pub fn set_yaw_offset_deg(&self, offset_deg: f32) {
        if offset_deg.is_finite() {
            self.yaw_offset_centi_deg
                .store((offset_deg * 100.0).round() as i32, Ordering::Relaxed);
        }
    }

    /// Current read-side yaw calibration in degrees.
    pub fn yaw_offset_deg(&self) -> f32 {
        self.yaw_offset_centi_deg.load(Ordering::Relaxed) as f32 / 100.0
    }

    /// Execute before the original ingress deadline, including time waiting for I/O.
    pub async fn execute(
        &self,
        payload: PayloadCommand,
        timing: HostCommandTiming,
    ) -> Result<CommandProgress> {
        let inner = self.inner.clone();
        let lane = PayloadLane::for_command(payload);
        let expected = self.fences.snapshot(lane);
        task::spawn_blocking(move || {
            let mut slot = inner.lock();
            if !slot.matches_fence(lane, expected) {
                return Err(turret_core::Error::StaleIdentity.into());
            }
            slot.execute_blocking(payload, timing)
        })
        .await
        .map_err(join_err)?
    }

    /// Read partial measured state and preserve its provenance.
    pub async fn observe(&self) -> Result<DeviceObservation> {
        let mut observation = self.observe_raw().await?;
        if let Some(yaw) = observation
            .angles
            .yaw
            .filter(|_| self.yaw_offset_deg() != 0.0)
        {
            observation.angles.yaw = Some(turret_core::Angle::radians(
                yaw.as_radians() - self.yaw_offset_deg().to_radians(),
            )?);
            observation.source = turret_core::ObservationSource::TransformedMeasurement;
        }
        Ok(observation)
    }

    /// Read device coordinates for explicit calibration workflows.
    pub async fn observe_raw(&self) -> Result<DeviceObservation> {
        let inner = self.inner.clone();
        task::spawn_blocking(move || inner.lock().observe_blocking())
            .await
            .map_err(join_err)?
    }

    /// Retained command evidence; polling and observations may have advanced its stages.
    pub async fn last_progress_cached(&self, lane: PayloadLane) -> Result<Option<CommandProgress>> {
        let inner = self.inner.clone();
        task::spawn_blocking(move || inner.lock().lane(lane).progress_cached())
            .await
            .map_err(join_err)
    }

    /// Actual backend capabilities, including unavailable and unknown operations.
    pub async fn capabilities(&self) -> Result<PayloadCapabilities> {
        let inner = self.inner.clone();
        task::spawn_blocking(move || inner.lock().capabilities_blocking())
            .await
            .map_err(join_err)?
    }

    /// Query firmware identification through the sole device owner.
    pub async fn firmware_version(&self) -> Result<String> {
        let inner = self.inner.clone();
        task::spawn_blocking(move || inner.lock().device.firmware_version_blocking())
            .await
            .map_err(join_err)?
    }

    /// Poll execution deadlines and attempt each required local stop or cancel.
    pub async fn poll_execution(&self) -> Result<()> {
        let inner = self.inner.clone();
        task::spawn_blocking(move || inner.lock().poll_blocking())
            .await
            .map_err(join_err)?
    }

    /// Revoke pending and active commands before releasing this device owner.
    pub async fn revoke(&self) -> Result<()> {
        self.fences.advance_device();
        let inner = self.inner.clone();
        task::spawn_blocking(move || inner.lock().revoke_blocking())
            .await
            .map_err(join_err)?
    }

    /// Fence queued work and replace the physical device after reconnection.
    pub async fn replace(&self, device: Box<dyn GimbalDevice>) -> Result<u64> {
        self.fences.advance_device();
        let inner = self.inner.clone();
        task::spawn_blocking(move || inner.lock().replace_blocking(device))
            .await
            .map_err(join_err)?
    }

    /// Execute an explicitly STorM32-specific extension without implying generic support.
    pub async fn execute_storm32_extension(
        &self,
        action: crate::protocols::storm32_rc::Storm32Action,
    ) -> Result<()> {
        let inner = self.inner.clone();
        let lane = PayloadLane::Gimbal;
        let expected = self.fences.snapshot(lane);
        let deadline = Instant::now() + std::time::Duration::from_millis(100);
        task::spawn_blocking(move || {
            let mut slot = inner.lock();
            if !slot.matches_fence(lane, expected) {
                return Err(turret_core::Error::StaleIdentity.into());
            }
            let protocol = slot.device.protocol_name();
            let device = slot
                .device
                .as_any_mut()
                .and_then(|device| {
                    device.downcast_mut::<crate::protocols::storm32_rc::Storm32Gimbal>()
                })
                .ok_or(Error::Unsupported {
                    protocol,
                    op: "STorM32 extension",
                })?;
            device.execute_extension_blocking(action, deadline)
        })
        .await
        .map_err(join_err)?
    }

    /// Fence and stop the selected lane without cancelling the other lane.
    pub async fn stop_lane(&self, lane: PayloadLane, effect: Effect) -> Result<()> {
        self.fences.advance_lane(lane);
        let inner = self.inner.clone();
        task::spawn_blocking(move || inner.lock().stop_lane_blocking(lane, effect))
            .await
            .map_err(join_err)?
    }

    /// Explicitly fence both lanes and attempt a device-wide stop or cancel.
    pub async fn stop_all(&self, effect: Effect) -> Result<()> {
        self.fences.advance_device();
        let inner = self.inner.clone();
        task::spawn_blocking(move || inner.lock().stop_all_blocking(effect))
            .await
            .map_err(join_err)?
    }
}

fn join_err(error: task::JoinError) -> Error {
    Error::Worker {
        operation: "payload",
        source: error,
    }
}
