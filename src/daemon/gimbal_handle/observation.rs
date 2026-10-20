//! Measured mount angles and camera zoom advance only their own command evidence.

use super::execution::DeviceSlot;
use crate::error::Result;
use crate::gimbal::{DeviceObservation, PayloadLane};
use std::time::Instant;
use turret_core::*;

impl DeviceSlot {
    pub(super) fn observe_blocking(&mut self) -> Result<DeviceObservation> {
        self.poll_blocking()?;
        let result = self.device.observe_blocking();
        self.poll_blocking()?;
        let mut observation = result?;
        observation.epoch = self.lane(PayloadLane::Gimbal).executor.epoch();
        let sampled_at = observation
            .sampled_at
            .map(|time| self.tick(time))
            .transpose()?;
        let received_at = self.tick(observation.received_at)?;
        let mut sample = Observation {
            value: PayloadState::Gimbal(observation.angles),
            source: observation.source,
            frame: Some(observation.frame),
            sampled_at,
            received_at,
            epoch: observation.epoch,
        };
        let mut tolerance = CompletionTolerance {
            angle: Angle::radians(0.01)?,
            zoom: None,
            settle_time_us: 300_000,
            maximum_sample_gap_us: 500_000,
        };
        self.observe_lane(PayloadLane::Gimbal, &sample, tolerance)?;
        if let Some(zoom) = observation.zoom {
            sample.value = PayloadState::Zoom(zoom);
            sample.frame = None;
            // Without a backend calibration, only an exact value in the same unit
            // establishes arrival at an absolute zoom target.
            tolerance.zoom = Some((zoom.unit(), 0.0));
            self.observe_lane(PayloadLane::Zoom, &sample, tolerance)?;
        }
        Ok(observation)
    }

    fn observe_lane(
        &mut self,
        lane: PayloadLane,
        sample: &Observation<PayloadState>,
        tolerance: CompletionTolerance,
    ) -> Result<()> {
        let Some(identity) = self.lane(lane).last_identity else {
            return Ok(());
        };
        let now = self.tick(Instant::now())?;
        match self
            .lane_mut(lane)
            .executor
            .observe_completion(identity, sample, now, 1_000_000, tolerance)
        {
            Ok(_) | Err(turret_core::Error::Ended | turret_core::Error::NoCompletionTarget) => {}
            Err(error) => {
                tracing::debug!(%error, ?lane, "measurement did not establish completion")
            }
        }
        Ok(())
    }
}
