//! Device-side lifecycle writes checked after the blocking queue and device lock.

use super::lanes::{FenceStamp, Fences, Lane};
use super::HostCommandTiming;
use crate::error::Result;
use crate::gimbal::{GimbalDevice, PayloadLane};
use std::sync::Arc;
use std::time::Instant;
use turret_core::*;

pub(super) struct DeviceSlot {
    pub(super) fence: u64,
    pub(super) device: Box<dyn GimbalDevice>,
    pub(super) fences: Arc<Fences>,
    gimbal: Lane,
    zoom: Lane,
    start: Instant,
    sequence: u64,
}

impl DeviceSlot {
    pub(super) fn new(device: Box<dyn GimbalDevice>, fences: Arc<Fences>) -> Result<Self> {
        let capabilities = device.capabilities()?;
        Ok(Self {
            fence: 0,
            device,
            fences,
            gimbal: Lane::new(capabilities),
            zoom: Lane::new(capabilities),
            start: Instant::now(),
            sequence: 0,
        })
    }

    pub(super) fn lane(&self, lane: PayloadLane) -> &Lane {
        match lane {
            PayloadLane::Gimbal => &self.gimbal,
            PayloadLane::Zoom => &self.zoom,
        }
    }

    pub(super) fn lane_mut(&mut self, lane: PayloadLane) -> &mut Lane {
        match lane {
            PayloadLane::Gimbal => &mut self.gimbal,
            PayloadLane::Zoom => &mut self.zoom,
        }
    }

    pub(super) fn matches_fence(&self, lane: PayloadLane, expected: FenceStamp) -> bool {
        self.fences.snapshot(lane) == expected
            && self.fence == expected.device
            && self.lane(lane).fence == expected.lane
    }

    pub(super) fn tick(&self, instant: Instant) -> Result<Tick> {
        let elapsed = instant
            .checked_duration_since(self.start)
            .ok_or(turret_core::Error::ClockRegression)?;
        let micros =
            u64::try_from(elapsed.as_micros()).map_err(|_| turret_core::Error::InvalidTiming)?;
        Ok(Tick(micros))
    }

    pub(super) fn execute_blocking(
        &mut self,
        payload: PayloadCommand,
        timing: HostCommandTiming,
    ) -> Result<CommandProgress> {
        let lane = PayloadLane::for_command(payload);
        let expected = self.fences.snapshot(lane);
        self.prepare_lane_blocking(lane)?;
        if !self.matches_fence(lane, expected) {
            return Err(turret_core::Error::StaleIdentity.into());
        }
        if Instant::now() >= timing.send_before {
            return Err(turret_core::Error::Expired.into());
        }
        let identity = CommandIdentity {
            epoch: self.lane(lane).executor.epoch(),
            generation: self.lane(lane).executor.generation(),
            id: self.sequence,
            sequence: self.sequence,
        };
        self.sequence = self.sequence.wrapping_add(1);
        let command = Command {
            identity,
            capability_revision: self.lane(lane).executor.capabilities().revision,
            payload,
            timing: self.command_timing(timing)?,
        };
        let now = self.tick(Instant::now())?;
        self.lane_mut(lane).executor.submit(command, now)?;
        let now = self.tick(Instant::now())?;
        self.lane_mut(lane)
            .executor
            .prepare_send(now)?
            .ok_or(turret_core::Error::NotFound)?;
        self.write_blocking(lane, command, timing.send_before)?;
        self.poll_lane_blocking(lane)?;
        self.lane(lane)
            .executor
            .progress(identity)
            .ok_or_else(|| turret_core::Error::NotFound.into())
    }

    fn command_timing(&self, timing: HostCommandTiming) -> Result<CommandTiming> {
        Ok(CommandTiming {
            send_before: self.tick(timing.send_before)?,
            valid_until: timing
                .valid_until
                .map(|value| self.tick(value))
                .transpose()?,
            ack_before: timing
                .ack_before
                .map(|value| self.tick(value))
                .transpose()?,
            completion_before: timing
                .completion_before
                .map(|value| self.tick(value))
                .transpose()?,
        })
    }

    fn write_blocking(
        &mut self,
        lane: PayloadLane,
        command: Command,
        before: Instant,
    ) -> Result<()> {
        let identity = command.identity;
        let receipt = match self.device.execute_blocking(command.payload, before) {
            Ok(receipt) => receipt,
            Err(error) => {
                let now = self.tick(Instant::now())?;
                let failed = self
                    .lane_mut(lane)
                    .executor
                    .record_write_failure(identity, now)?;
                self.fence_lane(lane);
                self.lane_mut(lane).last_identity = Some(identity);
                for event in [failed.active, Some(failed.attempted)]
                    .into_iter()
                    .flatten()
                {
                    self.lane_mut(lane)
                        .require_stop(event.effect, Some(event.progress.identity));
                }
                if let Err(stop_error) = self.retry_stops_blocking([lane]) {
                    tracing::warn!(%stop_error, ?lane, "physical state remains unknown after write failure");
                }
                return Err(error);
            }
        };
        let now = self.tick(receipt.sent_at)?;
        let sent = self.lane_mut(lane).executor.record_sent(identity, now)?;
        self.lane_mut(lane).last_identity = Some(identity);
        self.lane_mut(lane)
            .require_stop(sent.current.effect, Some(identity));
        // Receipt facts are independent of whether the required stop succeeds.
        let acknowledgement = self.record_acknowledgement(lane, identity, receipt);
        let stopping = self.retry_stops_blocking([lane]);
        acknowledgement.and(stopping)
    }

    fn record_acknowledgement(
        &mut self,
        lane: PayloadLane,
        identity: CommandIdentity,
        receipt: crate::gimbal::DeviceReceipt,
    ) -> Result<()> {
        if let Some(accepted) = receipt.acknowledged {
            let at = self.tick(
                receipt
                    .acknowledged_at
                    .ok_or(turret_core::Error::InvalidObservation)?,
            )?;
            let event = self
                .lane_mut(lane)
                .executor
                .acknowledge(identity, accepted, at)?;
            self.lane_mut(lane)
                .require_stop(event.effect, Some(identity));
        }
        Ok(())
    }

    pub(super) fn fence_lane(&mut self, lane: PayloadLane) {
        self.fences.advance_lane(lane);
        self.lane_mut(lane).fence = self.fences.snapshot(lane).lane;
    }

    pub(super) fn apply_effect_blocking(
        &mut self,
        lane: PayloadLane,
        effect: Effect,
        identity: Option<CommandIdentity>,
    ) -> Result<()> {
        if effect == Effect::None {
            return Ok(());
        }
        if self.lane_mut(lane).require_stop(effect, identity) {
            self.attempt_stop_blocking(lane, effect)
        } else {
            self.lane(lane).ensure_stopped(lane)
        }
    }

    pub(super) fn attempt_stop_blocking(
        &mut self,
        lane: PayloadLane,
        effect: Effect,
    ) -> Result<()> {
        let pending = self
            .lane(lane)
            .pending_stops()
            .into_iter()
            .find_map(|(candidate, pending)| (candidate == effect).then_some(pending).flatten());
        let Some(pending) = pending else {
            return Ok(());
        };
        match self.device.stop_blocking(lane, effect) {
            Ok(()) => {
                self.lane_mut(lane).clear_stop(effect);
                Ok(())
            }
            Err(error) => Err(pending.error(lane, effect, Some(error))),
        }
    }
}
