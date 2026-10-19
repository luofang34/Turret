//! Device-side lifecycle writes checked after the blocking queue and device lock.

use super::HostCommandTiming;
use crate::error::Result;
use crate::gimbal::{DeviceObservation, GimbalDevice};
use std::time::Instant;
use turret_core::*;

pub(super) struct DeviceSlot {
    pub(super) fence: u64,
    pub(super) device: Box<dyn GimbalDevice>,
    pub(super) executor: Executor,
    pub(super) last_identity: Option<CommandIdentity>,
    start: Instant,
    sequence: u64,
}

impl DeviceSlot {
    pub(super) fn new(device: Box<dyn GimbalDevice>) -> Result<Self> {
        let capabilities = device.capabilities()?;
        Ok(Self {
            fence: 0,
            device,
            executor: Executor::new(capabilities, 0, 0),
            last_identity: None,
            start: Instant::now(),
            sequence: 0,
        })
    }

    fn tick(&self, instant: Instant) -> Result<Tick> {
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
        self.poll_blocking()?;
        if Instant::now() >= timing.send_before {
            return Err(turret_core::Error::Expired.into());
        }
        let identity = CommandIdentity {
            epoch: self.executor.epoch(),
            generation: self.executor.generation(),
            id: self.sequence,
            sequence: self.sequence,
        };
        self.sequence = self.sequence.wrapping_add(1);
        let command = Command {
            identity,
            capability_revision: self.executor.capabilities().revision,
            payload,
            timing: CommandTiming {
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
            },
        };
        self.executor.submit(command, self.tick(Instant::now())?)?;
        self.executor
            .prepare_send(self.tick(Instant::now())?)?
            .ok_or(turret_core::Error::NotFound)?;
        let receipt = match self.device.execute_blocking(payload, timing.send_before) {
            Ok(receipt) => receipt,
            Err(error) => {
                let failed = self
                    .executor
                    .record_write_failure(identity, self.tick(Instant::now())?)?;
                if let Some(active) = failed.active {
                    if let Err(stop_error) = self.apply_effect_blocking(active.effect) {
                        tracing::warn!(%stop_error, "active command stopping state is unknown after write failure");
                    }
                }
                if let Err(stop_error) = self.apply_effect_blocking(failed.attempted.effect) {
                    tracing::warn!(%stop_error, "physical state remains unknown after write failure");
                }
                return Err(error);
            }
        };
        let sent = self
            .executor
            .record_sent(identity, self.tick(receipt.sent_at)?)?;
        self.apply_effect_blocking(sent.current.effect)?;
        self.last_identity = Some(identity);
        if let Some(accepted) = receipt.acknowledged {
            let event = self.executor.acknowledge(
                identity,
                accepted,
                self.tick(
                    receipt
                        .acknowledged_at
                        .ok_or(turret_core::Error::InvalidObservation)?,
                )?,
            )?;
            self.apply_effect_blocking(event.effect)?;
        }
        self.poll_blocking()?;
        self.executor
            .progress(identity)
            .ok_or_else(|| turret_core::Error::NotFound.into())
    }

    pub(super) fn observe_blocking(&mut self) -> Result<DeviceObservation> {
        self.poll_blocking()?;
        let result = self.device.observe_blocking();
        self.poll_blocking()?;
        let mut observation = result?;
        observation.epoch = self.executor.epoch();
        if let Some(identity) = self.last_identity {
            let sampled_at = observation
                .sampled_at
                .map(|time| self.tick(time))
                .transpose()?;
            let sample = Observation {
                value: PayloadState::Gimbal(observation.angles),
                source: observation.source,
                frame: Some(observation.frame),
                sampled_at,
                received_at: self.tick(observation.received_at)?,
                epoch: self.executor.epoch(),
            };
            let tolerance = CompletionTolerance {
                angle: Angle::radians(0.01)?,
                zoom: None,
                settle_time_us: 300_000,
                maximum_sample_gap_us: 500_000,
            };
            match self.executor.observe_completion(
                identity,
                &sample,
                self.tick(Instant::now())?,
                1_000_000,
                tolerance,
            ) {
                Ok(_) | Err(turret_core::Error::Ended | turret_core::Error::NoCompletionTarget) => {
                }
                Err(error) => tracing::debug!(%error, "measurement did not establish completion"),
            }
        }
        Ok(observation)
    }

    pub(super) fn poll_blocking(&mut self) -> Result<()> {
        while let Some(event) = self.executor.poll(self.tick(Instant::now())?)? {
            tracing::debug!(?event.progress, "payload execution deadline");
            self.apply_effect_blocking(event.effect)?;
        }
        Ok(())
    }

    pub(super) fn apply_effect_blocking(&mut self, effect: Effect) -> Result<()> {
        if effect == Effect::None {
            return Ok(());
        }
        self.device.stop_blocking(effect)
    }
}
