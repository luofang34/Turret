//! Manager-client claim cadence and stream/recenter coordination.

use turret_core::*;

mod authority;
use authority::Authority;

const STALE_DEMAND_US: u64 = 300_000;

/// Small policy state driven by the final writer's actual send receipts.
#[derive(Debug)]
pub struct ManagerPolicy {
    pitch_maximum: AngularRate,
    yaw_maximum: AngularRate,
    authority: Authority,
    last_motion_sent: Option<Tick>,
    streaming: bool,
    recenter_sent: Option<Tick>,
}

impl ManagerPolicy {
    /// Construct normalized-stick scaling with explicit configured rate limits.
    pub fn new(pitch_maximum: AngularRate, yaw_maximum: AngularRate) -> Result<Self, Error> {
        if pitch_maximum.as_radians_per_second() <= 0.0
            || yaw_maximum.as_radians_per_second() <= 0.0
        {
            return Err(Error::InvalidLimits);
        }
        Ok(Self {
            pitch_maximum,
            yaw_maximum,
            authority: Authority::default(),
            last_motion_sent: None,
            streaming: false,
            recenter_sent: None,
        })
    }

    /// Scale canonical stick directions; idle never overwrites an unresolved sent recenter.
    pub fn rate_intent(
        &self,
        pitch: f32,
        yaw: f32,
        now: Tick,
    ) -> Result<Option<PayloadCommand>, Error> {
        if !pitch.is_finite()
            || !yaw.is_finite()
            || !(-1.0..=1.0).contains(&pitch)
            || !(-1.0..=1.0).contains(&yaw)
        {
            return Err(Error::InvalidValue);
        }
        if !self.has_authority(now)? {
            return Err(Error::Unavailable);
        }
        if pitch == 0.0 && yaw == 0.0 && self.recenter_sent.is_some() {
            return Ok(None);
        }
        Ok(Some(PayloadCommand::GimbalRate(GimbalRate {
            rates: Axes {
                pitch: Some(AngularRate::radians_per_second(
                    pitch * self.pitch_maximum.as_radians_per_second(),
                )?),
                roll: None,
                yaw: Some(AngularRate::radians_per_second(
                    yaw * self.yaw_maximum.as_radians_per_second(),
                )?),
            },
            frame: ReferenceFrame::HorizonVehicleYaw,
        })))
    }

    /// Explicit recenter intent under freshly confirmed manager ownership.
    pub fn recenter_intent(&self, now: Tick) -> Result<PayloadCommand, Error> {
        if self.has_authority(now)? {
            Ok(PayloadCommand::Recenter)
        } else {
            Err(Error::Unavailable)
        }
    }

    /// Begin an explicitly authorized acquisition; a fresh manager status is still required.
    pub fn request_claim(&mut self, system_id: u8, component_id: u8) -> Result<(), Error> {
        self.authority.request_claim(system_id, component_id)
    }

    /// Cache actual GIMBAL_MANAGER_STATUS primary-owner fields from the expected manager.
    pub fn observe_status(
        &mut self,
        system_id: u8,
        component_id: u8,
        now: Tick,
    ) -> Result<(), Error> {
        self.authority.observe_status(system_id, component_id, now)
    }

    /// True only for one unclaimed-manager request in this explicit acquisition.
    pub fn claim_due(&self, now: Tick) -> Result<bool, Error> {
        self.authority.claim_due(now)
    }

    /// Whether a current manager status must be requested before proceeding.
    pub fn status_needed(&self, now: Tick) -> Result<bool, Error> {
        self.authority.status_needed(now)
    }

    /// Whether fresh manager feedback confirms this client as primary.
    pub fn has_authority(&self, now: Tick) -> Result<bool, Error> {
        self.authority.has_authority(now)
    }

    /// Record a complete claim write; this alone does not confirm primary control.
    pub fn record_claim_sent(&mut self, now: Tick) -> Result<(), Error> {
        self.authority.record_claim_sent(now)
    }

    /// Advance stream/recenter policy only after the payload's entire transport write succeeds.
    pub fn record_sent(&mut self, command: PayloadCommand, now: Tick) -> Result<(), Error> {
        if let Some(previous) = self.last_motion_sent {
            now.elapsed_since(previous)?;
        }
        match command {
            PayloadCommand::Recenter => {
                self.recenter_sent = Some(now);
                self.streaming = false;
            }
            PayloadCommand::GimbalRate(rate) => {
                check_rate_frame(rate.frame)?;
                self.recenter_sent = None;
                self.streaming = rate.rates.pitch.is_some_and(|r| r != AngularRate::ZERO)
                    || rate.rates.yaw.is_some_and(|r| r != AngularRate::ZERO);
            }
            PayloadCommand::GimbalPosition(_) => {
                self.recenter_sent = None;
                self.streaming = false;
            }
            PayloadCommand::Zoom(_) => return Ok(()),
        }
        self.last_motion_sent = Some(now);
        Ok(())
    }

    /// Fallback for un-timed legacy streams. Explicit Executor validity takes precedence.
    pub fn stale_stop_due(&self, now: Tick) -> Result<bool, Error> {
        if !self.streaming {
            return Ok(false);
        }
        self.last_motion_sent.map_or(Ok(false), |sent| {
            now.elapsed_since(sent)
                .map(|elapsed| elapsed >= STALE_DEMAND_US)
        })
    }

    /// Forget connection/authority-dependent policy; the owner must separately enact Executor effects.
    pub fn revoke(&mut self) {
        self.authority = Authority::default();
        self.last_motion_sent = None;
        self.streaming = false;
        self.recenter_sent = None;
    }
}

fn check_rate_frame(frame: ReferenceFrame) -> Result<(), Error> {
    if frame == ReferenceFrame::HorizonVehicleYaw {
        Ok(())
    } else {
        Err(Error::UnsupportedFrame)
    }
}

#[cfg(test)]
mod tests;
