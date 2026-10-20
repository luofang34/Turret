//! Independent execution deadlines and explicit device-wide rebinding.

use super::execution::DeviceSlot;
use crate::error::Result;
use crate::gimbal::{GimbalDevice, PayloadLane};
use std::time::Instant;
use turret_core::{Effect, PayloadCapabilities};

impl DeviceSlot {
    pub(super) fn capabilities_blocking(&mut self) -> Result<PayloadCapabilities> {
        let capabilities = self.device.capabilities()?;
        if self.lane(PayloadLane::Gimbal).executor.capabilities() != &capabilities {
            let effects = PayloadLane::ALL.map(|lane| {
                self.fence_lane(lane);
                let state = self.lane_mut(lane);
                let invalidated = state.executor.update_capabilities(capabilities);
                state.last_identity = None;
                (
                    lane,
                    invalidated
                        .active
                        .map_or(Effect::None, |active| active.effect),
                )
            });
            self.apply_effects_blocking(effects)?;
        }
        Ok(capabilities)
    }

    pub(super) fn poll_blocking(&mut self) -> Result<()> {
        self.capabilities_blocking()?;
        let mut first_error = None;
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

    fn poll_lane_blocking(&mut self, lane: PayloadLane) -> Result<()> {
        loop {
            let now = self.tick(Instant::now())?;
            let Some(event) = self.lane_mut(lane).executor.poll(now)? else {
                return Ok(());
            };
            tracing::debug!(?lane, ?event.progress, "payload execution deadline");
            self.apply_effect_blocking(lane, event.effect)?;
        }
    }

    pub(super) fn revoke_blocking(&mut self) -> Result<()> {
        self.fence = self.fences.snapshot(PayloadLane::Gimbal).device;
        let effects = PayloadLane::ALL.map(|lane| {
            let invalidated = self.lane_mut(lane).executor.revoke();
            (
                lane,
                invalidated
                    .active
                    .map_or(Effect::None, |active| active.effect),
            )
        });
        self.apply_effects_blocking(effects)
    }

    pub(super) fn stop_lane_blocking(
        &mut self,
        lane: PayloadLane,
        requested: Effect,
    ) -> Result<()> {
        self.lane_mut(lane).fence = self.fences.snapshot(lane).lane;
        let invalidated = self.lane_mut(lane).executor.revoke();
        let required = invalidated
            .active
            .map_or(Effect::None, |active| active.effect);
        let extra = if required == requested {
            Effect::None
        } else {
            requested
        };
        self.apply_effects_blocking([(lane, required), (lane, extra)])
    }

    pub(super) fn stop_all_blocking(&mut self, requested: Effect) -> Result<()> {
        self.fence = self.fences.snapshot(PayloadLane::Gimbal).device;
        let effects = PayloadLane::ALL.map(|lane| {
            let invalidated = self.lane_mut(lane).executor.revoke();
            let required = invalidated
                .active
                .map_or(Effect::None, |active| active.effect);
            let extra = if required == requested {
                Effect::None
            } else {
                requested
            };
            [(lane, required), (lane, extra)]
        });
        self.apply_effects_blocking(effects.into_iter().flatten())
    }

    pub(super) fn replace_blocking(&mut self, device: Box<dyn GimbalDevice>) -> Result<u64> {
        let capabilities = device.capabilities()?;
        let effects = PayloadLane::ALL.map(|lane| {
            let state = self.lane_mut(lane);
            let invalidated = state.executor.reconnect(capabilities);
            state.last_identity = None;
            (
                lane,
                invalidated
                    .active
                    .map_or(Effect::None, |active| active.effect),
            )
        });
        if let Err(error) = self.apply_effects_blocking(effects) {
            tracing::warn!(%error, "old device stopping state is unknown during reconnect");
        }
        self.device = device;
        self.fence = self.fences.snapshot(PayloadLane::Gimbal).device;
        Ok(self.lane(PayloadLane::Gimbal).executor.epoch())
    }

    fn apply_effects_blocking(
        &mut self,
        effects: impl IntoIterator<Item = (PayloadLane, Effect)>,
    ) -> Result<()> {
        let mut first_error = None;
        for (lane, effect) in effects {
            if let Err(error) = self.apply_effect_blocking(lane, effect) {
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
