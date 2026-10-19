//! Finite Mini 3 accessory keyframe requests; transport acceptance is not video evidence.

use turret_core::Tick;

mod encoding;
#[cfg(test)]
mod tests;

/// Minimum delay after the previous complete frame was accepted, in microseconds.
pub const MINI3_KEYFRAME_INTERVAL_US: u64 = 50_000;

/// Why a finite keyframe request cannot continue.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum KeyframeAbort {
    /// The absolute deadline is inconsistent or cannot represent the next delay.
    #[error("invalid Mini 3 keyframe timing")]
    InvalidTiming,
    /// The current connection is not the connection that admitted the request.
    #[error("Mini 3 keyframe session changed")]
    SessionChanged,
    /// The request's exclusive absolute deadline elapsed.
    #[error("Mini 3 keyframe deadline expired")]
    DeadlineExpired,
    /// The caller explicitly cancelled the request.
    #[error("Mini 3 keyframe request cancelled")]
    Cancelled,
    /// The accessory connection closed.
    #[error("Mini 3 accessory session closed")]
    Closed,
    /// A supplied monotonic timestamp regressed.
    #[error("Mini 3 keyframe clock regressed")]
    ClockRegression,
    /// The transport claimed more accepted bytes than were supplied.
    #[error("Mini 3 keyframe write count exceeds supplied suffix")]
    InvalidWriteCount,
    /// The transport returned an error; the same request must not retry.
    #[error("Mini 3 keyframe transport write failed")]
    WriteFailed,
}

/// Independent transport state; no variant certifies a decoded picture.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KeyframeState {
    /// No frame suffix is currently in flight; the next write may have a delay.
    Waiting,
    /// Only a prefix of the current logical frame has been accepted.
    Partial,
    /// All three frames were accepted by the transport at this endpoint-local time.
    TransportAccepted(Tick),
    /// Terminal failure; no subsequent call can replay any suffix.
    Aborted(KeyframeAbort),
}

/// Result supplied by the host after its nonblocking write returns.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct KeyframeWriteReceipt {
    /// Number of bytes actually accepted from the supplied suffix; zero means no progress.
    pub accepted: usize,
    /// Fresh endpoint-monotonic timestamp sampled after the write returns.
    pub completed_at: Tick,
}

/// Failure retains the actual transport error while latching a terminal request state.
#[derive(Debug, thiserror::Error)]
pub enum KeyframeWriteError<E> {
    /// Connection, deadline, or write-receipt validation failed.
    #[error(transparent)]
    State(#[from] KeyframeAbort),
    /// The host transport failed; its error remains available to the caller.
    #[error("Mini 3 keyframe transport failed: {0}")]
    Transport(#[source] E),
}

/// One explicitly authorized Mini 3 request on the upper accessory mux channel.
///
/// The caller supplies a unique connection token, an earliest-write time and an
/// exclusive absolute deadline. Closing or replacing the connection must call
/// [`Self::close`] or supply the new token. The host owns readiness and scheduling;
/// this object has no I/O, allocator, retries, or lower USB CDC integration.
#[derive(Debug)]
pub struct Mini3AccessoryKeyframe {
    frames: [[u8; 32]; 3],
    session: [u8; 16],
    deadline: Tick,
    not_before: Tick,
    last_checked: Option<Tick>,
    accepted: usize,
    state: KeyframeState,
}

impl Mini3AccessoryKeyframe {
    /// Bind a new one-shot request to one connection and its monotonic clock.
    ///
    /// `first_sequence` and the next two wrapping sequence numbers must be reserved
    /// by the host. The fixed payload does not inherit any flight or camera state.
    pub fn new(
        session: [u8; 16],
        first_sequence: u16,
        ready_at: Tick,
        deadline: Tick,
    ) -> Result<Self, KeyframeAbort> {
        if deadline <= ready_at {
            return Err(KeyframeAbort::InvalidTiming);
        }
        Ok(Self {
            frames: encoding::frames(first_sequence),
            session,
            deadline,
            not_before: ready_at,
            last_checked: None,
            accepted: 0,
            state: KeyframeState::Waiting,
        })
    }

    /// Retained transport evidence, unaffected by polling.
    #[must_use]
    pub const fn state(&self) -> KeyframeState {
        self.state
    }

    /// Total accepted bytes, including any prefix accepted before a terminal failure.
    #[must_use]
    pub const fn accepted_bytes(&self) -> usize {
        self.accepted
    }

    /// Earliest next write opportunity, or none after terminal completion or failure.
    #[must_use]
    pub fn next_write_at(&self) -> Option<Tick> {
        self.active().then_some(self.not_before)
    }

    /// Permanently discard any unwritten suffix without replaying the clear frame.
    pub fn cancel(&mut self) {
        self.abort(KeyframeAbort::Cancelled);
    }

    /// End this request when its accessory session closes; reconnect requires a new request.
    pub fn close(&mut self) {
        self.abort(KeyframeAbort::Closed);
    }

    /// Poll deadline/session validity without writing; waiting never refreshes a deadline.
    pub fn poll(&mut self, session: [u8; 16], now: Tick) -> Result<KeyframeState, KeyframeAbort> {
        self.validate(session, now)?;
        Ok(self.state)
    }

    /// Perform at most one nonblocking write of only the current frame's remaining suffix.
    ///
    /// Call this at the final write boundary with current session/time. The host
    /// must serialize it with close/cancel and report completion time after I/O.
    /// A zero-byte receipt preserves the suffix; a transport error ends the request.
    /// A successful return states accepted bytes, never device ACK or video readiness.
    pub fn write_available<E>(
        &mut self,
        session: [u8; 16],
        now: Tick,
        write: impl FnOnce(&[u8]) -> Result<KeyframeWriteReceipt, E>,
    ) -> Result<usize, KeyframeWriteError<E>> {
        self.validate(session, now)?;
        if !self.active() || now < self.not_before {
            return Ok(0);
        }
        let index = self.accepted / 32;
        let offset = self.accepted % 32;
        let suffix = &self.frames[index][offset..];
        let receipt = match write(suffix) {
            Ok(receipt) => receipt,
            Err(error) => {
                self.abort(KeyframeAbort::WriteFailed);
                return Err(KeyframeWriteError::Transport(error));
            }
        };
        if receipt.accepted > suffix.len() {
            self.abort(KeyframeAbort::InvalidWriteCount);
            return Err(KeyframeAbort::InvalidWriteCount.into());
        }
        self.accepted = self.accepted.wrapping_add(receipt.accepted);
        self.validate(session, receipt.completed_at)?;
        if receipt.accepted == 0 {
            return Ok(0);
        }
        if self.accepted == 96 {
            self.state = KeyframeState::TransportAccepted(receipt.completed_at);
        } else if self.accepted.is_multiple_of(32) {
            let Some(next) = receipt
                .completed_at
                .0
                .checked_add(MINI3_KEYFRAME_INTERVAL_US)
            else {
                self.abort(KeyframeAbort::InvalidTiming);
                return Err(KeyframeAbort::InvalidTiming.into());
            };
            self.not_before = Tick(next);
            self.state = KeyframeState::Waiting;
        } else {
            self.state = KeyframeState::Partial;
        }
        Ok(receipt.accepted)
    }

    fn validate(&mut self, session: [u8; 16], now: Tick) -> Result<(), KeyframeAbort> {
        if let KeyframeState::Aborted(reason) = self.state {
            return Err(reason);
        }
        if !self.active() {
            return Ok(());
        }
        let error = if session != self.session {
            Some(KeyframeAbort::SessionChanged)
        } else if self.last_checked.is_some_and(|previous| now < previous) {
            Some(KeyframeAbort::ClockRegression)
        } else if now >= self.deadline {
            Some(KeyframeAbort::DeadlineExpired)
        } else {
            None
        };
        if let Some(error) = error {
            self.abort(error);
            return Err(error);
        }
        self.last_checked = Some(now);
        Ok(())
    }

    fn active(&self) -> bool {
        matches!(self.state, KeyframeState::Waiting | KeyframeState::Partial)
    }

    fn abort(&mut self, reason: KeyframeAbort) {
        if self.active() {
            self.state = KeyframeState::Aborted(reason);
        }
    }
}
