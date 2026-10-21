//! The operations implemented by the STorM32 payload binding.

use turret_core::*;

/// Build the backend contract; device limits may further constrain these bounds.
pub fn storm32_capabilities() -> Result<PayloadCapabilities, Error> {
    let frames = Frames {
        mount: true,
        vehicle: true,
        earth_ned: false,
        horizon_vehicle_yaw: false,
    };
    let position = PositionCapability {
        implementation: Implementation::Native,
        range: AngleRange {
            minimum: Angle::radians(-core::f32::consts::PI)?,
            maximum: Angle::radians(core::f32::consts::PI)?,
        },
        frames,
        maximum_rate: None,
    };
    let axis = AxisCapabilities {
        mechanics: AxisMechanics::UNKNOWN,
        position: Capability::Available(position),
        rate: Capability::Available(RateCapability {
            implementation: Implementation::Synthesized,
            maximum_rate: AngularRate::radians_per_second(core::f32::consts::FRAC_PI_2)?,
            frames,
        }),
        feedback: FeedbackCapabilities {
            angle: Capability::Available(()),
            rate: Capability::Unsupported,
        },
    };
    Ok(PayloadCapabilities {
        revision: 1,
        gimbal: GimbalCapabilities {
            axes: Axes {
                pitch: axis,
                roll: axis,
                yaw: axis,
            },
            recenter: Capability::Available(RecenterCapability {
                implementation: Implementation::Native,
                target: None,
            }),
            stop: StopGuarantee::BestEffort,
        },
        zoom: Capability::Unsupported,
    })
}
