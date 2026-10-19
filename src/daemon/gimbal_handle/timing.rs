//! Execution-local deadlines stamped once by the ingress authority.

use std::time::{Duration, Instant};
use turret_core::PayloadCommand;

/// Four independent deadlines on this host's monotonic clock.
#[derive(Debug, Clone, Copy)]
pub struct HostCommandTiming {
    /// Latest permitted start of the transport write.
    pub send_before: Instant,
    /// End of execution authority; expiry requires stopping or cancelling.
    pub valid_until: Option<Instant>,
    /// Latest expected device acknowledgement, independent of execution expiry.
    pub ack_before: Option<Instant>,
    /// Latest expected observed completion, independent of execution expiry.
    pub completion_before: Option<Instant>,
}

impl HostCommandTiming {
    /// Local IPC/MAVLink policy: bounded queue age and bounded continuous authority.
    pub fn local_ingress(payload: PayloadCommand, received_at: Instant) -> Self {
        Self {
            send_before: received_at + Duration::from_millis(100),
            valid_until: payload
                .is_continuous()
                .then_some(received_at + Duration::from_millis(200)),
            ack_before: Some(received_at + Duration::from_millis(400)),
            completion_before: None,
        }
    }
}
