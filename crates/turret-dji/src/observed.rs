//! Pitch planning from qualified device feedback and continuous operator input.

use core::time::Duration;

use crate::{GimbalPitchTarget, MAX_INPUT_INTERVAL, Mini3ControlProfile};

/// A finite target whose receipt belongs to one plan from this planner.
///
/// The host must also match the transport command identity before recording
/// a write. A plan is not a receipt or evidence of physical movement.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ObservedPitchPlan {
    id: u64,
    target: GimbalPitchTarget,
}

impl ObservedPitchPlan {
    /// The profile-validated target to encode at the final transport boundary.
    pub const fn target(self) -> GimbalPitchTarget {
        self.target
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Motion {
    Idle,
    Manual,
    Recenter,
}

/// Plans finite pitch steps without repeating a fully written target.
///
/// The host must qualify feedback, authority, and write deadlines before
/// planning. Each recenter requires continuous valid input and fresh feedback.
/// The configured rate bounds target planning, not measured device speed.
#[derive(Debug)]
pub struct ObservedPitchPlanner {
    profile: Mini3ControlProfile,
    motion: Motion,
    hold: Option<GimbalPitchTarget>,
    last_input: Option<Duration>,
    last_written: Option<GimbalPitchTarget>,
    pending: Option<ObservedPitchPlan>,
    next_id: u64,
}

impl ObservedPitchPlanner {
    /// Start without an assumed attitude, active motion, or write history.
    pub const fn new(profile: Mini3ControlProfile) -> Self {
        Self {
            profile,
            motion: Motion::Idle,
            hold: None,
            last_input: None,
            last_written: None,
            pending: None,
            next_id: 0,
        }
    }

    /// Plan from a qualified observation and a currently admitted input.
    ///
    /// A recenter input starts a new operation. Neutral inputs continue that
    /// operation. A manual rate cancels it. An input gap cancels prior motion.
    /// A repeated plan retains its identity and must not extend a queued write.
    pub fn plan(
        &mut self,
        pitch_tenths: i16,
        rate: f32,
        recenter: bool,
        now: Duration,
    ) -> Result<Option<ObservedPitchPlan>, turret_core::Error> {
        if self.profile.validate_rate(rate).is_err() {
            self.cancel();
            return Err(turret_core::Error::OutOfRange);
        }
        let observed = match self.profile.pitch_target(pitch_tenths) {
            Ok(observed) => observed,
            Err(_) => {
                self.cancel();
                return Err(turret_core::Error::InvalidObservation);
            }
        };
        if self.last_input.is_some_and(|last| {
            now.checked_sub(last)
                .is_none_or(|elapsed| elapsed > MAX_INPUT_INTERVAL)
        }) {
            self.cancel();
        }
        self.last_input = Some(now);
        self.update_motion(observed, rate, recenter);
        let target = match self.motion {
            Motion::Manual => Some(self.profile.observed_pitch_target(
                pitch_tenths,
                rate,
                false,
            )?),
            Motion::Recenter => Some(self.profile.observed_pitch_target(
                pitch_tenths,
                0.0,
                true,
            )?),
            Motion::Idle => self.hold,
        };
        let Some(target) = target.filter(|target| Some(*target) != self.last_written) else {
            self.pending = None;
            return Ok(None);
        };
        if let Some(pending) = self.pending.filter(|pending| pending.target == target) {
            return Ok(Some(pending));
        }
        let plan = ObservedPitchPlan {
            id: self.next_id,
            target,
        };
        self.next_id = self.next_id.wrapping_add(1);
        self.pending = Some(plan);
        Ok(Some(plan))
    }

    /// Commit a full write receipt matched by the host to this exact plan.
    ///
    /// Returns false for a superseded or cancelled plan. Neither result
    /// establishes device acceptance, arrival at the target, or physical stop.
    pub fn written(&mut self, plan: ObservedPitchPlan) -> bool {
        if self.pending != Some(plan) {
            return false;
        }
        self.last_written = Some(plan.target);
        self.pending = None;
        if self.motion == Motion::Idle {
            self.hold = None;
        }
        true
    }

    /// Cancel motion and receipt history after an authority or validity failure.
    pub fn cancel(&mut self) {
        self.motion = Motion::Idle;
        self.hold = None;
        self.last_input = None;
        self.last_written = None;
        self.pending = None;
    }

    fn update_motion(&mut self, observed: GimbalPitchTarget, rate: f32, recenter: bool) {
        if recenter {
            self.begin(Motion::Recenter);
        } else if rate != 0.0 {
            if self.motion != Motion::Manual {
                self.begin(Motion::Manual);
            }
        } else if self.motion == Motion::Manual {
            self.motion = Motion::Idle;
            self.hold = Some(observed);
        }
        if self.motion == Motion::Recenter && observed.tenths() == 0 {
            self.motion = Motion::Idle;
            self.hold = None;
        }
    }

    fn begin(&mut self, motion: Motion) {
        self.motion = motion;
        self.hold = None;
        self.last_written = None;
        self.pending = None;
    }
}

#[cfg(test)]
mod tests;
