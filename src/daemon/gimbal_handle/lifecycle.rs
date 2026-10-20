//! Independent execution deadlines and explicit device-wide rebinding.

use super::execution::DeviceSlot;
use crate::error::{Error, Result};
use crate::gimbal::{GimbalDevice, PayloadLane};
use std::time::Instant;
use turret_core::{CommandIdentity, Effect, PayloadCapabilities};

impl DeviceSlot {
    pub(super) fn capabilities_blocking(&mut self) -> Result<PayloadCapabilities> {
        let capabilities = self.device.capabilities()?;
        if self.lane(PayloadLane::Gimbal).executor.capabilities() != &capabilities {
            let effects = PayloadLane::ALL.map(|lane| {
                self.fence_lane(lane);
                let state = self.lane_mut(lane);
                let invalidated = state.executor.update_capabilities(capabilities);
                (
                    lane,
                    invalidated
                        .active
                        .map_or(Effect::None, |event| event.effect),
                    invalidated.active.map(|event| event.progress.identity),
                )
            });
            self.apply_effects_blocking(effects)?;
        }
        Ok(capabilities)
    }

    pub(super) fn poll_blocking(&mut self) -> Result<()> {
        let pending = self.retry_stops_blocking(PayloadLane::ALL);
        let deadlines = self.poll_deadlines_blocking();
        pending.and(deadlines)
    }

    pub(super) fn poll_deadlines_blocking(&mut self) -> Result<()> {
        let mut first_error = self.capabilities_blocking().err();
        for lane in PayloadLane::ALL {
            if let Err(error) = self.poll_lane_blocking(lane) {
                if first_error.is_none() {
                    first_error = Some(error);
                } else {
                    tracing::warn!(%error, ?lane, "payload deadline effect failed");
                }
            }
        }
        first_error.map_or(Ok(()), Err)
    }

    pub(super) fn prepare_lane_blocking(&mut self, lane: PayloadLane) -> Result<()> {
        self.lane(lane).ensure_stopped(lane)?;
        match self.capabilities_blocking() {
            Err(Error::StopUnconfirmed { lane: failed, .. }) if failed != lane => {}
            result => {
                result?;
            }
        }
        self.poll_lane_blocking(lane)?;
        self.lane(lane).ensure_stopped(lane)
    }

    pub(super) fn poll_lane_blocking(&mut self, lane: PayloadLane) -> Result<()> {
        loop {
            let now = self.tick(Instant::now())?;
            let Some(event) = self.lane_mut(lane).executor.poll(now)? else {
                return Ok(());
            };
            tracing::debug!(?lane, ?event.progress, "payload execution deadline");
            self.apply_effect_blocking(lane, event.effect, Some(event.progress.identity))?;
        }
    }

    pub(super) fn revoke_blocking(&mut self) -> Result<()> {
        self.fence = self.fences.snapshot(PayloadLane::Gimbal).device;
        for lane in PayloadLane::ALL {
            self.revoke_lane(lane);
        }
        self.retry_stops_blocking(PayloadLane::ALL)
    }

    pub(super) fn stop_lane_blocking(
        &mut self,
        lane: PayloadLane,
        requested: Effect,
    ) -> Result<()> {
        self.lane_mut(lane).fence = self.fences.snapshot(lane).lane;
        self.revoke_lane(lane);
        self.lane_mut(lane).require_stop(requested, None);
        self.retry_stops_blocking([lane])
    }

    pub(super) fn stop_all_blocking(&mut self, requested: Effect) -> Result<()> {
        self.fence = self.fences.snapshot(PayloadLane::Gimbal).device;
        for lane in PayloadLane::ALL {
            self.revoke_lane(lane);
            self.lane_mut(lane).require_stop(requested, None);
        }
        self.retry_stops_blocking(PayloadLane::ALL)
    }

    pub(super) fn replace_blocking(&mut self, device: Box<dyn GimbalDevice>) -> Result<u64> {
        self.fence = self.fences.snapshot(PayloadLane::Gimbal).device;
        let capabilities = device.capabilities()?;
        // Discovery does not establish physical identity. A replacement cannot
        // discharge responsibility that belongs to the current device owner.
        self.revoke_blocking()?;
        for lane in PayloadLane::ALL {
            let _invalidated = self.lane_mut(lane).executor.reconnect(capabilities);
        }
        self.device = device;
        Ok(self.lane(PayloadLane::Gimbal).executor.epoch())
    }

    fn revoke_lane(&mut self, lane: PayloadLane) {
        let invalidated = self.lane_mut(lane).executor.revoke();
        if let Some(event) = invalidated.active {
            self.lane_mut(lane)
                .require_stop(event.effect, Some(event.progress.identity));
        }
    }

    pub(super) fn retry_stops_blocking(
        &mut self,
        lanes: impl IntoIterator<Item = PayloadLane>,
    ) -> Result<()> {
        let mut first_error = None;
        for lane in lanes {
            for (effect, pending) in self.lane(lane).pending_stops() {
                if pending.is_some() {
                    if let Err(error) = self.attempt_stop_blocking(lane, effect) {
                        if first_error.is_none() {
                            first_error = Some(error);
                        } else {
                            tracing::warn!(%error, ?lane, "payload stopping state is unknown");
                        }
                    }
                }
            }
        }
        first_error.map_or(Ok(()), Err)
    }

    fn apply_effects_blocking(
        &mut self,
        effects: impl IntoIterator<Item = (PayloadLane, Effect, Option<CommandIdentity>)>,
    ) -> Result<()> {
        let mut first_error = None;
        for (lane, effect, identity) in effects {
            if let Err(error) = self.apply_effect_blocking(lane, effect, identity) {
                if first_error.is_none() {
                    first_error = Some(error);
                } else {
                    tracing::warn!(%error, ?lane, "payload stopping state is unknown");
                }
            }
        }
        first_error.map_or(Ok(()), Err)
    }
}
