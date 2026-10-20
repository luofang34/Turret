//! Execution and admission fences belong to an independent payload lane.

use crate::gimbal::PayloadLane;
use std::sync::atomic::{AtomicU64, Ordering};
use turret_core::{CommandIdentity, CommandProgress, Executor, PayloadCapabilities};

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
}

impl Lane {
    pub(super) fn new(capabilities: PayloadCapabilities) -> Self {
        Self {
            executor: Executor::new(capabilities, 0, 0),
            last_identity: None,
            fence: 0,
        }
    }

    pub(super) fn progress_cached(&self) -> Option<CommandProgress> {
        self.last_identity
            .and_then(|identity| self.executor.progress(identity))
    }
}
