//! Advertised device support is distinct from implemented client operations.

use turret_core::*;

mod manager;
pub use manager::ManagerInformation;

/// Describe one selected manager from its information message, or unknown discovery.
///
/// The host validates sender, device binding, connection epoch, and report ordering.
/// Rate maxima are host operating policy: the information message carries angle limits,
/// not measured speed limits. Device information cannot replace manager information,
/// because a manager may add capabilities beyond those of its attached device.
pub fn manager_capabilities(
    revision: u64,
    information: Option<ManagerInformation>,
    pitch_maximum: AngularRate,
    yaw_maximum: AngularRate,
) -> Result<PayloadCapabilities, Error> {
    manager::capabilities(revision, information, pitch_maximum, yaw_maximum)
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
