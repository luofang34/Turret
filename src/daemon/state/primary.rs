//! MAVLink primary / secondary control ownership.
//!
//! Set by `MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE` and re-broadcast in
//! every `GIMBAL_MANAGER_STATUS`. Pure MAVLink concern — IPC clients
//! don't have ownership semantics today.

use crate::daemon::models::PrimaryControl;
use parking_lot::RwLock;
use std::sync::Arc;

/// Owner of the [`PrimaryControl`] tuple.
#[derive(Clone, Default)]
pub struct PrimaryStore {
    inner: Arc<RwLock<PrimaryControl>>,
}

impl PrimaryStore {
    /// Empty store — primary/secondary both unset, anyone can drive
    /// the gimbal.
    pub fn new() -> Self {
        Self::default()
    }

    /// Snapshot the current ownership tuple.
    pub fn get(&self) -> PrimaryControl {
        *self.inner.read()
    }

    /// Replace the ownership tuple. All-zero means "unset / open".
    pub fn set(&self, pc: PrimaryControl) {
        *self.inner.write() = pc;
    }

    /// True if `(sysid, compid)` is authorized to drive the gimbal
    /// under the current primary-control configuration.
    pub fn is_allowed(&self, sysid: u8, compid: u8) -> bool {
        self.get().is_allowed(sysid, compid)
    }
}
