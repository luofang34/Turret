//! Execution and admission fences belong to an independent payload lane.

use crate::gimbal::PayloadLane;
use std::sync::atomic::{AtomicU64, Ordering};
use turret_core::{CommandIdentity, CommandProgress, Effect, Executor, PayloadCapabilities};

#[derive(Default)]
pub(super) struct Fences {
    device: AtomicU64,
    gimbal: AtomicU64,
    zoom: AtomicU64,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub(super) struct FenceStamp {
    pub(super) device: u64,
    pub(super) lane: u64,
}

impl Fences {
    fn lane(&self, lane: PayloadLane) -> &AtomicU64 {
        match lane {
            PayloadLane::Gimbal => &self.gimbal,
            PayloadLane::Zoom => &self.zoom,
        }
    }

    pub(super) fn snapshot(&self, lane: PayloadLane) -> FenceStamp {
        FenceStamp {
            device: self.device.load(Ordering::Acquire),
            lane: self.lane(lane).load(Ordering::Acquire),
        }
    }

    pub(super) fn advance_lane(&self, lane: PayloadLane) {
        self.lane(lane).fetch_add(1, Ordering::AcqRel);
    }

    pub(super) fn advance_device(&self) {
        self.device.fetch_add(1, Ordering::AcqRel);
    }
}

pub(super) struct Lane {
    pub(super) executor: Executor,
    pub(super) last_identity: Option<CommandIdentity>,
    pub(super) fence: u64,
    continuous_stop: Option<StopResponsibility>,
    action_cancel: Option<StopResponsibility>,
}

#[derive(Clone, Copy)]
pub(super) struct StopResponsibility {
    pub(super) epoch: u64,
    pub(super) commands: [Option<CommandIdentity>; 2],
}

impl StopResponsibility {
    pub(super) fn error(
        self,
        lane: PayloadLane,
        effect: Effect,
        source: Option<crate::Error>,
    ) -> crate::Error {
        crate::Error::StopUnconfirmed {
            lane,
            effect,
            epoch: self.epoch,
            commands: self.commands,
            source: source.map(Box::new),
        }
    }
}

impl Lane {
    pub(super) fn new(capabilities: PayloadCapabilities) -> Self {
        Self {
            executor: Executor::new(capabilities, 0, 0),
            last_identity: None,
            fence: 0,
            continuous_stop: None,
            action_cancel: None,
        }
    }

    pub(super) fn progress_cached(&self) -> Option<CommandProgress> {
        self.last_identity
            .and_then(|identity| self.executor.progress(identity))
    }

    pub(super) fn pending_stops(&self) -> [(Effect, Option<StopResponsibility>); 2] {
        [
            (Effect::StopContinuous, self.continuous_stop),
            (Effect::CancelAction, self.action_cancel),
        ]
    }

    pub(super) fn require_stop(
        &mut self,
        effect: Effect,
        identity: Option<CommandIdentity>,
    ) -> bool {
        let epoch = identity.map_or(self.executor.epoch(), |command| command.epoch);
        let slot = match effect {
            Effect::None => return false,
            Effect::StopContinuous => &mut self.continuous_stop,
            Effect::CancelAction => &mut self.action_cancel,
        };
        let fresh = slot.is_none();
        let responsibility = slot.get_or_insert(StopResponsibility {
            epoch,
            commands: [None, None],
        });
        if identity.is_some() && !responsibility.commands.contains(&identity) {
            // A failed write can implicate both the active and attempted command.
            // New writes are blocked until these lane-wide effects succeed.
            if let Some(empty) = responsibility
                .commands
                .iter_mut()
                .find(|entry| entry.is_none())
            {
                *empty = identity;
            }
        }
        fresh
    }

    pub(super) fn clear_stop(&mut self, effect: Effect) {
        match effect {
            Effect::None => {}
            Effect::StopContinuous => self.continuous_stop = None,
            Effect::CancelAction => self.action_cancel = None,
        }
    }

    pub(super) fn ensure_stopped(&self, lane: PayloadLane) -> crate::Result<()> {
        for (effect, pending) in self.pending_stops() {
            if let Some(pending) = pending {
                return Err(pending.error(lane, effect, None));
            }
        }
        Ok(())
    }
}
