//! Finite pitch targets for a one-way, initially horizontal bench attachment.

use crate::{GimbalPitchTarget, Mini3ControlProfile, PitchTargetError};
use core::time::Duration;

/// Maximum interval integrated from a live input stream.
pub const MAX_INPUT_INTERVAL: Duration = Duration::from_millis(100);

/// An input outside the selected pitch rate policy.
#[derive(Clone, Copy, Debug, PartialEq, thiserror::Error)]
#[error("pitch rate {requested} rad/s is not finite or exceeds configured limit {maximum} rad/s")]
pub struct PitchRateError {
    /// Rejected demand in radians per second.
    pub requested: f32,
    /// Configured maximum absolute demand in radians per second.
    pub maximum: f32,
}

/// Tracks commanded pitch, without producing a measured attitude.
///
/// Construction requires the host to establish an initial horizontal target.
/// Only a fully written command may advance the committed target. The caller
/// must enforce authority and invalidate queued writes when input expires.
#[derive(Debug)]
pub struct PitchPlanner {
    profile: Mini3ControlProfile,
    commanded: i16,
    remainder: f64,
    last_input: Option<Duration>,
    recentering: bool,
    write_stalled: bool,
}

impl PitchPlanner {
    /// Start a new bench attachment from an operator-confirmed horizontal pose.
    /// This creates no command and provides no sensor observation.
    #[must_use]
    pub const fn from_confirmed_horizontal(profile: Mini3ControlProfile) -> Self {
        Self {
            profile,
            commanded: 0,
            remainder: 0.0,
            last_input: None,
            recentering: false,
            write_stalled: false,
        }
    }

    /// Last fully written target in tenths of a degree, initially zero.
    #[must_use]
    pub const fn commanded(&self) -> i16 {
        self.commanded
    }

    /// Discard elapsed input time and fractional demand.
    pub fn discard_elapsed(&mut self, now: Duration) {
        self.last_input = Some(now);
        self.remainder = 0.0;
    }

    /// Commit a fully written target. This does not confirm movement or arrival.
    pub fn written(
        &mut self,
        pitch: GimbalPitchTarget,
        now: Duration,
    ) -> Result<(), PitchTargetError> {
        self.profile.pitch_target(pitch.tenths())?;
        self.commanded = pitch.tenths();
        if self.write_stalled {
            self.discard_elapsed(now);
        }
        self.write_stalled = false;
        if self.commanded == 0 {
            self.recentering = false;
        }
        Ok(())
    }

    /// Exclude waiting time when the transport cannot accept a target.
    pub fn stalled(&mut self, now: Duration, pending: bool) {
        self.write_stalled = pending;
        self.discard_elapsed(now);
    }

    /// Exclude blocking transport time from future target integration.
    pub fn exclude_io(&mut self, started: Duration, completed: Duration) {
        if let Some(last) = self.last_input
            && let Some(blocked) = completed.checked_sub(started.max(last))
        {
            self.last_input = Some(last.saturating_add(blocked));
        }
    }

    /// Resolve a live rate or an explicitly requested gradual recenter.
    ///
    /// A nonzero manual rate cancels recentering. A zero rate permits an active
    /// recenter to progress while its host-owned execution lease remains valid.
    pub fn demand(
        &mut self,
        rate: f32,
        recenter: bool,
        now: Duration,
    ) -> Result<f32, PitchRateError> {
        self.profile.validate_rate(rate)?;
        if !recenter
            && self.last_input.is_some_and(|last| {
                now.checked_sub(last)
                    .is_none_or(|elapsed| elapsed > MAX_INPUT_INTERVAL)
            })
        {
            self.cancel(now);
        }
        if rate != 0.0 && !recenter {
            self.recentering = false;
        }
        self.recentering = (recenter || self.recentering) && self.commanded != 0;
        Ok(if self.recentering {
            self.profile.rate_limit().as_radians_per_second()
        } else if recenter {
            0.0
        } else {
            rate
        })
    }

    /// Cancel an active recenter and discard elapsed demand.
    pub fn cancel(&mut self, now: Duration) {
        self.recentering = false;
        self.stalled(now, false);
    }

    /// Compute a bounded target without committing it as sent or measured.
    pub fn target(
        &mut self,
        rate: f32,
        now: Duration,
    ) -> Result<Option<GimbalPitchTarget>, turret_core::Error> {
        self.profile
            .validate_rate(rate)
            .map_err(|_| turret_core::Error::OutOfRange)?;
        let elapsed = self.last_input.and_then(|last| now.checked_sub(last));
        self.last_input = Some(now);
        let Some(elapsed) = elapsed.filter(|elapsed| *elapsed <= MAX_INPUT_INTERVAL) else {
            self.remainder = 0.0;
            return Ok(None);
        };
        let change = f64::from(rate).to_degrees() * 10.0 * elapsed.as_secs_f64();
        let desired = (f64::from(self.commanded) + self.remainder + change)
            .clamp(f64::from(self.profile.minimum_tenths()), 0.0);
        // The envelope is nonpositive; integer conversion truncates toward zero.
        let rounded = (desired - 0.5) as i16;
        self.remainder = desired - f64::from(rounded);
        if rounded == self.commanded {
            return Ok(None);
        }
        self.profile
            .pitch_target(rounded)
            .map(Some)
            .map_err(|_| turret_core::Error::OutOfRange)
    }
}

#[cfg(test)]
mod tests;
