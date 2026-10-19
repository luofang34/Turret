//! Protocol operations are separate from unknown attached-device geometry.

use turret_core::*;

/// Advertise the manager-client operations implemented for a pitch/yaw mount.
/// Device position limits and feedback remain unknown until independently discovered.
pub fn manager_capabilities(
    revision: u64,
    pitch_maximum: AngularRate,
    yaw_maximum: AngularRate,
) -> Result<PayloadCapabilities, Error> {
    let axis = |maximum: AngularRate| -> Result<AxisCapabilities, Error> {
        if maximum.as_radians_per_second() <= 0.0 {
            return Err(Error::InvalidLimits);
        }
        Ok(AxisCapabilities {
            position: Capability::Unknown,
            rate: Capability::Available(RateCapability {
                implementation: Implementation::Native,
                maximum_rate: maximum,
                frames: Frames {
                    horizon_vehicle_yaw: true,
                    ..Frames::default()
                },
            }),
            feedback: FeedbackCapabilities {
                angle: Capability::Unknown,
                rate: Capability::Unknown,
            },
        })
    };
    Ok(PayloadCapabilities {
        revision,
        gimbal: GimbalCapabilities {
            axes: Axes {
                pitch: axis(pitch_maximum)?,
                roll: AxisCapabilities::UNSUPPORTED,
                yaw: axis(yaw_maximum)?,
            },
            recenter: Capability::Available(RecenterCapability {
                implementation: Implementation::Native,
                target: None,
            }),
            stop: StopGuarantee::BestEffort,
        },
        zoom: Capability::Unknown,
    })
}

/// Advertise camera zoom only from its BASIC_ZOOM capability.
/// Continuous motion and physical-unit calibration are not inferred from that flag.
pub fn camera_capabilities(revision: u64, basic_zoom: bool) -> Result<PayloadCapabilities, Error> {
    let unknown_absolute = AbsoluteZoomCapabilities {
        ratio: Capability::Unknown,
        travel_fraction: Capability::Unknown,
        focal_length_mm: Capability::Unknown,
        horizontal_fov_radians: Capability::Unknown,
    };
    let zoom = if basic_zoom {
        Capability::Available(ZoomCapabilities {
            absolute: AbsoluteZoomCapabilities {
                travel_fraction: Capability::Available(ZoomRange {
                    minimum: ZoomAbsolute::new(ZoomUnit::TravelFraction, 0.0)?,
                    maximum: ZoomAbsolute::new(ZoomUnit::TravelFraction, 1.0)?,
                    implementation: Implementation::Native,
                }),
                ..unknown_absolute
            },
            step: Capability::Available(ZoomStepCapability {
                implementation: Implementation::Native,
                maximum_steps: 1,
            }),
            continuous: Capability::Unknown,
            feedback: unknown_absolute,
            stop: StopGuarantee::Unknown,
        })
    } else {
        Capability::Unknown
    };
    Ok(PayloadCapabilities {
        revision,
        gimbal: GimbalCapabilities {
            axes: Axes {
                pitch: AxisCapabilities::UNSUPPORTED,
                roll: AxisCapabilities::UNSUPPORTED,
                yaw: AxisCapabilities::UNSUPPORTED,
            },
            recenter: Capability::Unsupported,
            stop: StopGuarantee::Unknown,
        },
        zoom,
    })
}

#[cfg(test)]
mod tests;
