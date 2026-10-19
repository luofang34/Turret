//! The verified experimental pitch command envelope.

use turret_core::{
    Angle, AngleRange, AngularRate, Axes, AxisCapabilities, Capability, FeedbackCapabilities,
    Frames, GimbalCapabilities, GimbalPosition, Implementation, PayloadCapabilities,
    PositionCapability, RateCapability, RecenterCapability, ReferenceFrame, StopGuarantee,
};

/// Build the bounded pitch contract. Feedback and zoom support remain unknown.
///
/// The caller selects a tested lower pitch bound between -90 and zero degrees.
/// A finite position target does not establish a bounded stopping time.
pub fn pitch_capabilities(minimum_tenths: i16) -> Result<PayloadCapabilities, turret_core::Error> {
    if !(-900..=0).contains(&minimum_tenths) {
        return Err(turret_core::Error::InvalidLimits);
    }
    let maximum_rate = AngularRate::radians_per_second(crate::MAX_PITCH_RATE)?;
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
                                f32::from(minimum_tenths) * core::f32::consts::PI / 1800.0,
                            )?,
                            maximum: Angle::ZERO,
                        },
                        frames,
                        maximum_rate: Some(maximum_rate),
                    }),
                    rate: Capability::Available(RateCapability {
                        implementation: Implementation::Synthesized,
                        maximum_rate,
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
