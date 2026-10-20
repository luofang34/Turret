//! One configured Mini 3 pitch envelope for commands and capability declarations.

use turret_core::{
    Angle, AngleRange, AngularRate, Axes, AxisCapabilities, Capability, FeedbackCapabilities,
    Frames, GimbalCapabilities, GimbalPosition, Implementation, PayloadCapabilities,
    PositionCapability, RateCapability, RecenterCapability, ReferenceFrame, StopGuarantee,
};

/// Pitch limits selected by the host for a tested Mini 3 attachment.
///
/// The rate limit is an operating policy, not a measured hardware maximum.
/// Construction does not establish feedback, stopping behavior, or device identity.
/// A profile change requires the host to update executor capabilities and fence queued writes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mini3ControlProfile {
    minimum_tenths: i16,
    rate_limit: AngularRate,
}

impl Mini3ControlProfile {
    /// Select a lower pitch bound between -90 and zero degrees and a positive rate limit.
    pub fn new(minimum_tenths: i16, rate_limit: AngularRate) -> Result<Self, turret_core::Error> {
        if !(-900..=0).contains(&minimum_tenths) || rate_limit.as_radians_per_second() <= 0.0 {
            return Err(turret_core::Error::InvalidLimits);
        }
        Ok(Self {
            minimum_tenths,
            rate_limit,
        })
    }

    /// Inclusive lower target bound in tenths of a degree; the upper bound is zero.
    pub const fn minimum_tenths(self) -> i16 {
        self.minimum_tenths
    }

    /// Configured maximum absolute rate of the synthesized pitch demand.
    pub const fn rate_limit(self) -> AngularRate {
        self.rate_limit
    }

    /// Validate an absolute target against this attachment's configured envelope.
    pub fn pitch_target(
        self,
        tenths: i16,
    ) -> Result<crate::GimbalPitchTarget, crate::PitchTargetError> {
        crate::GimbalPitchTarget::new(self, tenths)
    }

    /// Revalidate a target at encoding, including targets created with another profile.
    ///
    /// The host must still check authority and deadlines at the actual write boundary.
    pub fn encode_pitch(
        self,
        target: crate::GimbalPitchTarget,
        sequence: u16,
    ) -> Result<[u8; 21], crate::PitchTargetError> {
        Ok(self
            .pitch_target(target.tenths())?
            .encode_app_command(sequence))
    }

    /// Convert an observed pitch and live demand into a one-second position target.
    ///
    /// The host must first verify observation route, mode, continuity, and age.
    /// Recenter returns one rate-limited step toward zero. The host must continue
    /// planning while authority remains valid; this target does not prove arrival.
    pub fn observed_pitch_target(
        self,
        pitch_tenths: i16,
        rate: f32,
        recenter: bool,
    ) -> Result<crate::GimbalPitchTarget, turret_core::Error> {
        self.validate_rate(rate)
            .map_err(|_| turret_core::Error::OutOfRange)?;
        self.pitch_target(pitch_tenths)
            .map_err(|_| turret_core::Error::InvalidObservation)?;
        let demand = if recenter {
            self.rate_limit.as_radians_per_second()
        } else {
            rate
        };
        let desired = f64::from(pitch_tenths) + f64::from(demand).to_degrees() * 10.0;
        let bounded = desired.clamp(f64::from(self.minimum_tenths), 0.0);
        self.pitch_target((bounded - 0.5) as i16)
            .map_err(|_| turret_core::Error::OutOfRange)
    }

    pub(crate) fn validate_rate(self, rate: f32) -> Result<(), crate::PitchRateError> {
        if rate.is_finite() && rate.abs() <= self.rate_limit.as_radians_per_second() {
            Ok(())
        } else {
            Err(crate::PitchRateError {
                requested: rate,
                maximum: self.rate_limit.as_radians_per_second(),
            })
        }
    }

    /// Declare the same configured limits used by target planning and encoding.
    ///
    /// Feedback, zoom, native position speed, and stopping behavior remain unknown.
    pub fn capabilities(self) -> Result<PayloadCapabilities, turret_core::Error> {
        capabilities(self)
    }
}

fn capabilities(profile: Mini3ControlProfile) -> Result<PayloadCapabilities, turret_core::Error> {
    let frames = Frames {
        mount: true,
        ..Frames::default()
    };
    let feedback = FeedbackCapabilities {
        angle: Capability::Unknown,
        rate: Capability::Unknown,
    };
    let unavailable_axis = AxisCapabilities {
        position: Capability::Unsupported,
        rate: Capability::Unsupported,
        feedback,
    };
    Ok(PayloadCapabilities {
        revision: 1,
        gimbal: GimbalCapabilities {
            axes: Axes {
                pitch: AxisCapabilities {
                    position: Capability::Available(PositionCapability {
                        implementation: Implementation::Native,
                        range: AngleRange {
                            minimum: Angle::radians(
                                f32::from(profile.minimum_tenths) * core::f32::consts::PI / 1800.0,
                            )?,
                            maximum: Angle::ZERO,
                        },
                        frames,
                        maximum_rate: None,
                    }),
                    rate: Capability::Available(RateCapability {
                        implementation: Implementation::Synthesized,
                        maximum_rate: profile.rate_limit,
                        frames,
                    }),
                    feedback,
                },
                roll: unavailable_axis,
                yaw: unavailable_axis,
            },
            recenter: Capability::Available(RecenterCapability {
                implementation: Implementation::Synthesized,
                target: Some(GimbalPosition {
                    angles: Axes {
                        pitch: Some(Angle::ZERO),
                        roll: None,
                        yaw: None,
                    },
                    frame: ReferenceFrame::Mount,
                }),
            }),
            stop: StopGuarantee::Unknown,
        },
        zoom: Capability::Unknown,
    })
}

/// Encode the read-only RC version query used to identify a connected endpoint.
pub fn rc_version_request() -> [u8; 13] {
    let mut bytes = [0x55, 13, 4, 0, 10, 6, 1, 0, 0x40, 0, 1, 0, 0];
    bytes[3] = crate::header_checksum(&bytes[..3]);
    let checksum = crate::frame_checksum(&bytes[..11]).to_le_bytes();
    bytes[11..].copy_from_slice(&checksum);
    bytes
}

#[cfg(test)]
mod tests;
