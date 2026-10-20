//! Typed payload intent, executor-local deadlines, and progress evidence.

use crate::{Angle, AngularRate, Axes, Error, ReferenceFrame, Tick, ZoomCommand};

/// Absolute targets for selected axes. Omitted axes remain untouched.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GimbalPosition {
    /// Selected targets in radians.
    pub angles: Axes<Option<Angle>>,
    /// Reference shared by the selected targets.
    pub frame: ReferenceFrame,
}

/// Rates for selected axes. Omitted axes remain untouched.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GimbalRate {
    /// Selected demands in radians per second.
    pub rates: Axes<Option<AngularRate>>,
    /// Reference shared by the selected demands.
    pub frame: ReferenceFrame,
}

/// Device-independent payload commands; vehicle motion is deliberately separate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PayloadCommand {
    /// An absolute target for selected mount axes.
    GimbalPosition(GimbalPosition),
    /// A continuous demand which must expire unless renewed.
    GimbalRate(GimbalRate),
    /// Invoke the backend-declared recenter operation.
    Recenter,
    /// One camera zoom operation.
    Zoom(ZoomCommand),
}

impl PayloadCommand {
    /// Whether the operation represents a continuous demand.
    pub const fn is_continuous(self) -> bool {
        matches!(
            self,
            Self::GimbalRate(_) | Self::Zoom(ZoomCommand::Continuous(_))
        )
    }
}

/// Correlation and fencing fields supplied by the local authority boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CommandIdentity {
    /// Connection epoch; reconnecting invalidates previous device assumptions.
    pub epoch: u64,
    /// Control-authority generation; revocation invalidates older commands.
    pub generation: u64,
    /// Opaque caller-assigned correlation identifier.
    pub id: u64,
    /// Wrapping order within this epoch and generation.
    pub sequence: u64,
}

/// Deadlines in the final executor's monotonic clock domain.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CommandTiming {
    /// Exclusive latest time at which a device write may begin.
    pub send_before: Tick,
    /// Exclusive end of execution validity; mandatory for continuous demands.
    pub valid_until: Option<Tick>,
    /// Exclusive deadline for a device acknowledgement, if expected.
    pub ack_before: Option<Tick>,
    /// Exclusive deadline for measured completion, if required.
    pub completion_before: Option<Tick>,
}

impl CommandTiming {
    /// Check consistent deadlines and immediate admissibility.
    pub fn validate(self, command: PayloadCommand, now: Tick) -> Result<(), Error> {
        if command.is_continuous() && self.valid_until.is_none() {
            return Err(Error::MissingValidity);
        }
        if command.is_continuous() && self.completion_before.is_some() {
            return Err(Error::InvalidTiming);
        }
        for deadline in [self.valid_until, self.ack_before, self.completion_before]
            .into_iter()
            .flatten()
        {
            if deadline < self.send_before {
                return Err(Error::InvalidTiming);
            }
        }
        if let (Some(ack), Some(complete)) = (self.ack_before, self.completion_before) {
            if ack > complete {
                return Err(Error::InvalidTiming);
            }
        }
        if now >= self.send_before {
            return Err(Error::Expired);
        }
        Ok(())
    }
}

/// A bounded command with explicit capability assumptions.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Command {
    /// Correlation and fencing identity.
    pub identity: CommandIdentity,
    /// Capability revision used when constructing the command.
    pub capability_revision: u64,
    /// Payload operation.
    pub payload: PayloadCommand,
    /// Final executor-local deadlines.
    pub timing: CommandTiming,
}

/// Why a command no longer participates in execution.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EndReason {
    /// Its write deadline elapsed while still pending.
    ExpiredBeforeSend,
    /// Its execution validity elapsed.
    ValidityExpired,
    /// The requested measured completion did not arrive in time.
    CompletionTimeout,
    /// Authority was explicitly revoked.
    Revoked,
    /// Device connection epoch changed.
    Reconnected,
    /// A newer command replaced it.
    Superseded,
    /// The device explicitly rejected it.
    DeviceRejected,
    /// Fresh physical feedback established the target was reached.
    ObservedComplete,
    /// The transport reported a failed or partial write.
    TransportFailure,
}

/// Evidence retained separately for each lifecycle stage.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CommandProgress {
    /// Correlation and fencing identity.
    pub identity: CommandIdentity,
    /// Time the executor admitted the command.
    pub accepted_at: Tick,
    /// Most recent successful immediate pre-write validation.
    pub prepared_at: Option<Tick>,
    /// Time the transport reported the full write completed.
    pub sent_at: Option<Tick>,
    /// Time the device positively acknowledged the operation.
    pub acknowledged_at: Option<Tick>,
    /// Time of the first device rejection, retained through later invalidation.
    pub rejected_at: Option<Tick>,
    /// Whether an expected acknowledgement deadline was missed.
    pub acknowledgement_timed_out: bool,
    /// Time fresh measurements established completion.
    pub observed_complete_at: Option<Tick>,
    /// Terminal outcome; neither sent nor acknowledged implies completion.
    pub ended: Option<EndReason>,
}

/// Action the platform must execute as a result of lifecycle invalidation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Effect {
    /// No further I/O is requested; this says nothing about physical movement.
    None,
    /// Issue the backend's stop operation for a continuous demand.
    StopContinuous,
    /// Attempt to cancel a bounded position or discrete action.
    CancelAction,
}

/// One lifecycle event, returned to the caller for I/O and durable recording.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[must_use = "execute the requested device effect; an event is not a physical stop"]
pub struct LifecycleEvent {
    /// Current evidence for the affected command.
    pub progress: CommandProgress,
    /// Action requested at the device boundary.
    pub effect: Effect,
}

/// Effects of a connection or authority fence changing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[must_use = "invalidate pending writes and execute active stop or cancel effects"]
pub struct Invalidation {
    /// Pending command discarded without a device write.
    pub pending: Option<CommandProgress>,
    /// Active command ended, with any required device action.
    pub active: Option<LifecycleEvent>,
}
