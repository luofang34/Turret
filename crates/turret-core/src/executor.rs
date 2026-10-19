//! A bounded device-side execution lifecycle for one independent command channel.
//!
//! Use separate executors for concurrently controlled gimbal and zoom channels.
//! A platform owns I/O and must call `prepare_send` at the actual write boundary,
//! record complete or failed writes, and execute returned stop/cancel effects.

use crate::{
    Command, CommandIdentity, CommandProgress, CompletionTolerance, Effect, EndReason, Error,
    Invalidation, LifecycleEvent, Observation, PayloadCapabilities, PayloadState, Tick,
};

mod completion;
mod deadline;
#[cfg(test)]
mod tests;
mod validation;

#[derive(Debug, Clone, Copy)]
struct Record {
    command: Command,
    progress: CommandProgress,
    settle_started_at: Option<Tick>,
    last_matching_sample: Option<Tick>,
}

impl Record {
    fn finish(&mut self, reason: EndReason) -> LifecycleEvent {
        self.progress.ended = Some(reason);
        let effect = if self.progress.sent_at.is_none() {
            Effect::None
        } else if self.command.payload.is_continuous() {
            Effect::StopContinuous
        } else {
            Effect::CancelAction
        };
        LifecycleEvent {
            progress: self.progress,
            effect,
        }
    }
}

/// Result of recording an actual full transport write.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[must_use = "inspect requested device effects and record command evidence"]
pub struct SendOutcome {
    /// Prior active command superseded by this full write.
    pub superseded: Option<CommandProgress>,
    /// New command's evidence and any immediate expiry action.
    pub current: LifecycleEvent,
}

/// A failed write may have partially reached the device and needs explicit action.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[must_use = "uncertain writes require the returned stop or cancel effects"]
pub struct WriteFailure {
    /// Evidence for the attempted write; `sent_at` remains absent.
    pub attempted: LifecycleEvent,
    /// Previously active command also invalidated by uncertain transport state.
    pub active: Option<LifecycleEvent>,
}

/// One pending command, one active command, and one retained terminal status.
///
/// No background task, clock source, allocation, or transport is owned here.
#[derive(Debug)]
pub struct Executor {
    capabilities: PayloadCapabilities,
    epoch: u64,
    generation: u64,
    last_sequence: Option<u64>,
    last_tick: Option<Tick>,
    pending: Option<Record>,
    active: Option<Record>,
    terminal: Option<CommandProgress>,
}

impl Executor {
    /// Construct an executor for the supplied local connection and authority.
    pub const fn new(capabilities: PayloadCapabilities, epoch: u64, generation: u64) -> Self {
        Self {
            capabilities,
            epoch,
            generation,
            last_sequence: None,
            last_tick: None,
            pending: None,
            active: None,
            terminal: None,
        }
    }

    /// Current capability contract.
    pub const fn capabilities(&self) -> &PayloadCapabilities {
        &self.capabilities
    }

    /// Current connection epoch.
    pub const fn epoch(&self) -> u64 {
        self.epoch
    }

    /// Current authority generation.
    pub const fn generation(&self) -> u64 {
        self.generation
    }

    /// Admit a command, replacing only an unsent pending command.
    ///
    /// An active physical command remains tracked until a replacement is sent.
    pub fn submit(
        &mut self,
        command: Command,
        now: Tick,
    ) -> Result<Option<CommandProgress>, Error> {
        self.advance_clock(now)?;
        self.validate_identity(command.identity)?;
        if command.capability_revision != self.capabilities.revision {
            return Err(Error::CapabilityRevision);
        }
        if let Some(last) = self.last_sequence {
            let difference = command.identity.sequence.wrapping_sub(last);
            if difference == 0 || difference >= (1_u64 << 63) {
                return Err(Error::StaleSequence);
            }
        }
        command.timing.validate(command.payload, now)?;
        validation::validate_payload(&self.capabilities, command.payload)?;
        let previous = self
            .pending
            .take()
            .map(|mut old| old.finish(EndReason::Superseded).progress);
        let progress = CommandProgress {
            identity: command.identity,
            accepted_at: now,
            prepared_at: None,
            sent_at: None,
            acknowledged_at: None,
            acknowledgement_timed_out: false,
            observed_complete_at: None,
            ended: None,
        };
        self.last_sequence = Some(command.identity.sequence);
        self.pending = Some(Record {
            command,
            progress,
            settle_started_at: None,
            last_matching_sample: None,
        });
        if previous.is_some() {
            self.terminal = previous;
        }
        Ok(previous)
    }

    /// Validate immediately before starting a transport write.
    ///
    /// A queued copy is not a permit: call this again after any wait for I/O.
    /// The platform must not begin a write at or beyond `send_before`.
    pub fn prepare_send(&mut self, now: Tick) -> Result<Option<Command>, Error> {
        self.advance_clock(now)?;
        let Some(record) = self.pending.as_mut() else {
            return Ok(None);
        };
        if now >= record.command.timing.send_before {
            self.terminal = Some(record.finish(EndReason::ExpiredBeforeSend).progress);
            self.pending = None;
            return Err(Error::Expired);
        }
        record.progress.prepared_at = Some(now);
        Ok(Some(record.command))
    }

    /// Record a full write; this does not claim device acceptance or movement.
    ///
    /// A write completing after its validity must still be recorded truthfully;
    /// the result then asks the platform to stop or cancel that late action.
    pub fn record_sent(
        &mut self,
        identity: CommandIdentity,
        now: Tick,
    ) -> Result<SendOutcome, Error> {
        self.advance_clock(now)?;
        self.validate_identity(identity)?;
        let pending = self.pending.as_ref().ok_or(Error::NotFound)?;
        if pending.command.identity != identity {
            return Err(Error::NotFound);
        }
        if pending.progress.prepared_at.is_none() {
            return Err(Error::NotPrepared);
        }
        let mut record = self.pending.take().ok_or(Error::NotFound)?;
        record.progress.sent_at = Some(now);
        let superseded = self
            .active
            .take()
            .map(|mut old| old.finish(EndReason::Superseded).progress);
        let current = if record
            .command
            .timing
            .valid_until
            .is_some_and(|deadline| now >= deadline)
        {
            record.finish(EndReason::ValidityExpired)
        } else {
            LifecycleEvent {
                progress: record.progress,
                effect: Effect::None,
            }
        };
        if superseded.is_some() {
            self.terminal = superseded;
        }
        self.active = Some(record);
        Ok(SendOutcome {
            superseded,
            current,
        })
    }

    /// Record a failed or partial write and invalidate continuous motion.
    ///
    /// Partial transport writes may have device effects, so even a pending
    /// command produces a stop/cancel request rather than claiming no movement.
    pub fn record_write_failure(
        &mut self,
        identity: CommandIdentity,
        now: Tick,
    ) -> Result<WriteFailure, Error> {
        self.advance_clock(now)?;
        self.validate_identity(identity)?;
        if self
            .pending
            .as_ref()
            .is_none_or(|record| record.command.identity != identity)
        {
            return Err(Error::NotFound);
        }
        let mut pending = self.pending.take().ok_or(Error::NotFound)?;
        let mut attempted = pending.finish(EndReason::TransportFailure);
        attempted.effect = if pending.command.payload.is_continuous() {
            Effect::StopContinuous
        } else {
            Effect::CancelAction
        };
        let active = self.invalidate(EndReason::TransportFailure).active;
        self.terminal = Some(attempted.progress);
        self.generation = self.generation.wrapping_add(1);
        self.last_sequence = None;
        Ok(WriteFailure { attempted, active })
    }

    /// Record a device ACK separately from measured completion.
    pub fn acknowledge(
        &mut self,
        identity: CommandIdentity,
        accepted: bool,
        now: Tick,
    ) -> Result<LifecycleEvent, Error> {
        self.advance_clock(now)?;
        self.validate_identity(identity)?;
        let record = self.active.as_mut().ok_or(Error::NotSent)?;
        if record.command.identity != identity {
            return Err(Error::NotFound);
        }
        if accepted {
            record.progress.acknowledged_at = Some(now);
            if record
                .command
                .timing
                .ack_before
                .is_some_and(|deadline| now >= deadline)
            {
                record.progress.acknowledgement_timed_out = true;
            }
            Ok(LifecycleEvent {
                progress: record.progress,
                effect: Effect::None,
            })
        } else {
            Ok(record.finish(EndReason::DeviceRejected))
        }
    }

    /// Compare fresh measured values with a sent absolute target.
    ///
    /// An ACK, an estimate, an operator assumption, or stale feedback cannot
    /// complete a command. Continuous and relative operations do not complete
    /// through an absolute-value comparison.
    pub fn observe_completion(
        &mut self,
        identity: CommandIdentity,
        observation: &Observation<PayloadState>,
        now: Tick,
        maximum_age_us: u64,
        tolerance: CompletionTolerance,
    ) -> Result<bool, Error> {
        self.advance_clock(now)?;
        self.validate_identity(identity)?;
        let valid = observation.validate_measurement(now, self.epoch, maximum_age_us);
        let recenter = self.capabilities.gimbal.recenter;
        let record = self.active_mut(identity)?;
        if let Err(error) = valid {
            record.settle_started_at = None;
            record.last_matching_sample = None;
            return Err(error);
        }
        if observation
            .sampled_at
            .is_none_or(|sample| record.progress.sent_at.is_none_or(|sent| sample < sent))
        {
            record.settle_started_at = None;
            record.last_matching_sample = None;
            return Err(Error::InvalidObservation);
        }
        if record
            .command
            .timing
            .completion_before
            .is_some_and(|deadline| now >= deadline)
            || record
                .command
                .timing
                .valid_until
                .is_some_and(|deadline| now >= deadline)
        {
            return Err(Error::Expired);
        }
        let comparison =
            completion::matches_target(record.command.payload, recenter, observation, tolerance);
        match comparison {
            Ok(true) => {}
            other => {
                record.settle_started_at = None;
                record.last_matching_sample = None;
                return other;
            }
        }
        completion::settle(
            record,
            observation.sampled_at.ok_or(Error::InvalidObservation)?,
            tolerance,
            now,
        )
    }

    /// Return the next deadline event. Drain until `None` before doing more I/O.
    ///
    /// ACK timeout is evidence only; it does not fabricate device rejection or
    /// stop an already sent position target. Execution expiry returns an effect.
    pub fn poll(&mut self, now: Tick) -> Result<Option<LifecycleEvent>, Error> {
        self.advance_clock(now)?;
        if let Some(record) = self.active.as_mut() {
            if let Some(event) = deadline::active_deadline(record, now) {
                return Ok(Some(event));
            }
        }
        if let Some(record) = self.pending.as_mut() {
            if now >= record.command.timing.send_before {
                let event = record.finish(EndReason::ExpiredBeforeSend);
                self.terminal = Some(event.progress);
                self.pending = None;
                return Ok(Some(event));
            }
        }
        Ok(None)
    }

    /// Revoke current control, incrementing the local fencing generation.
    pub fn revoke(&mut self) -> Invalidation {
        let result = self.invalidate(EndReason::Revoked);
        self.generation = self.generation.wrapping_add(1);
        self.last_sequence = None;
        result
    }

    /// Replace a contract on the same connection and fence all previous authority.
    pub fn update_capabilities(&mut self, capabilities: PayloadCapabilities) -> Invalidation {
        let result = self.revoke();
        self.capabilities = capabilities;
        result
    }

    /// Fence all commands and assumptions when the physical connection changes.
    pub fn reconnect(&mut self, capabilities: PayloadCapabilities) -> Invalidation {
        let result = self.invalidate(EndReason::Reconnected);
        self.epoch = self.epoch.wrapping_add(1);
        self.generation = self.generation.wrapping_add(1);
        self.last_sequence = None;
        self.capabilities = capabilities;
        result
    }

    /// Find bounded retained evidence without inferring device state.
    pub fn progress(&self, identity: CommandIdentity) -> Option<CommandProgress> {
        self.pending
            .iter()
            .chain(self.active.iter())
            .map(|r| r.progress)
            .chain(self.terminal)
            .find(|progress| progress.identity == identity)
    }

    fn advance_clock(&mut self, now: Tick) -> Result<(), Error> {
        if self.last_tick.is_some_and(|last| now < last) {
            return Err(Error::ClockRegression);
        }
        self.last_tick = Some(now);
        Ok(())
    }

    fn validate_identity(&self, identity: CommandIdentity) -> Result<(), Error> {
        if identity.epoch != self.epoch || identity.generation != self.generation {
            return Err(Error::StaleIdentity);
        }
        Ok(())
    }

    fn active_mut(&mut self, identity: CommandIdentity) -> Result<&mut Record, Error> {
        let record = self.active.as_mut().ok_or(Error::NotSent)?;
        if record.command.identity != identity {
            return Err(Error::NotFound);
        }
        if record.progress.ended.is_some() {
            return Err(Error::Ended);
        }
        Ok(record)
    }

    fn invalidate(&mut self, reason: EndReason) -> Invalidation {
        let pending = self
            .pending
            .take()
            .map(|mut record| record.finish(reason).progress);
        let active = self.active.take().map(|mut record| record.finish(reason));
        self.terminal = active
            .map(|event| event.progress)
            .or(pending)
            .or(self.terminal);
        Invalidation { pending, active }
    }
}
