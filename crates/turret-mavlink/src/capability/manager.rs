//! GIMBAL_MANAGER_INFORMATION semantics without transport or attachment identity.

use turret_core::*;

const HAS_NEUTRAL: u32 = 2;
const HAS_ROLL_AXIS: u32 = 4;
const HAS_ROLL_FOLLOW: u32 = 8;
const HAS_ROLL_LOCK: u32 = 16;
const HAS_PITCH_AXIS: u32 = 32;
const HAS_PITCH_FOLLOW: u32 = 64;
const HAS_PITCH_LOCK: u32 = 128;
const HAS_YAW_AXIS: u32 = 256;
const HAS_YAW_FOLLOW: u32 = 512;
const HAS_YAW_LOCK: u32 = 1024;

/// Semantic fields from the selected GIMBAL_MANAGER_INFORMATION (280) report.
///
/// Values use the [MAVLink common definitions](https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_INFORMATION).
/// The host must bind this report to the selected manager and device before use.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ManagerInformation {
    /// GIMBAL_MANAGER_CAP_FLAGS; unrecognized bits do not grant other operations.
    pub cap_flags: u32,
    /// Minimum reported roll angle, radians; NaN preserves an unknown limit.
    pub roll_min: f32,
    /// Maximum reported roll angle, radians; NaN preserves an unknown limit.
    pub roll_max: f32,
    /// Minimum reported pitch angle, radians; NaN preserves an unknown limit.
    pub pitch_min: f32,
    /// Maximum reported pitch angle, radians; NaN preserves an unknown limit.
    pub pitch_max: f32,
    /// Minimum reported yaw angle, radians; NaN preserves an unknown limit.
    pub yaw_min: f32,
    /// Maximum reported yaw angle, radians; NaN preserves an unknown limit.
    pub yaw_max: f32,
}

pub(super) fn capabilities(
    revision: u64,
    information: Option<ManagerInformation>,
    pitch_maximum: AngularRate,
    yaw_maximum: AngularRate,
) -> Result<PayloadCapabilities, Error> {
    if pitch_maximum.as_radians_per_second() <= 0.0 || yaw_maximum.as_radians_per_second() <= 0.0 {
        return Err(Error::InvalidLimits);
    }
    let mut result = PayloadCapabilities {
        revision,
        gimbal: GimbalCapabilities {
            axes: Axes {
                pitch: AxisCapabilities::UNKNOWN,
                roll: AxisCapabilities::UNKNOWN,
                yaw: AxisCapabilities::UNKNOWN,
            },
            recenter: Capability::Unknown,
            stop: StopGuarantee::Unknown,
        },
        zoom: Capability::Unknown,
    };
    let Some(info) = information else {
        return Ok(result);
    };
    let flags = info.cap_flags;
    result.gimbal.axes.pitch = axis(
        flags & HAS_PITCH_AXIS != 0,
        frames(flags, HAS_PITCH_FOLLOW, HAS_PITCH_LOCK, false),
        (info.pitch_min, info.pitch_max),
        Some(pitch_maximum),
    )?;
    result.gimbal.axes.yaw = axis(
        flags & HAS_YAW_AXIS != 0,
        frames(flags, HAS_YAW_FOLLOW, HAS_YAW_LOCK, true),
        (info.yaw_min, info.yaw_max),
        Some(yaw_maximum),
    )?;
    result.gimbal.axes.roll = axis(
        flags & HAS_ROLL_AXIS != 0,
        frames(flags, HAS_ROLL_FOLLOW, HAS_ROLL_LOCK, false),
        (info.roll_min, info.roll_max),
        None,
    )?;
    result.gimbal.recenter = if flags & HAS_NEUTRAL != 0 {
        Capability::Available(RecenterCapability {
            implementation: Implementation::Native,
            target: None,
        })
    } else {
        Capability::Unsupported
    };
    result.gimbal.stop = StopGuarantee::BestEffort;
    Ok(result)
}

fn frames(flags: u32, follow: u32, lock: u32, yaw: bool) -> Frames {
    Frames {
        vehicle: flags & follow != 0,
        earth_ned: flags & lock != 0,
        horizon_vehicle_yaw: flags & if yaw { follow } else { lock } != 0,
        ..Frames::default()
    }
}

fn axis(
    present: bool,
    supported: Frames,
    limits: (f32, f32),
    maximum: Option<AngularRate>,
) -> Result<AxisCapabilities, Error> {
    if !present {
        return Ok(AxisCapabilities::UNSUPPORTED);
    }
    let implemented = maximum.is_some()
        && (supported.vehicle || supported.horizon_vehicle_yaw || supported.earth_ned);
    let position = match angle_range(limits)? {
        Some(range) => position_capability(range, supported, implemented)?,
        None if implemented => Capability::Unknown,
        None => Capability::Unimplemented { capability: None },
    };
    let rate = maximum.map_or(
        Capability::Unimplemented { capability: None },
        |maximum_rate| {
            availability(
                RateCapability {
                    implementation: Implementation::Native,
                    maximum_rate,
                    frames: supported,
                },
                implemented,
            )
        },
    );
    Ok(AxisCapabilities {
        // Manager operations may include vehicle motion; flags do not identify mechanics.
        mechanics: AxisMechanics::UNKNOWN,
        position,
        rate,
        feedback: FeedbackCapabilities {
            angle: Capability::Unknown,
            rate: Capability::Unknown,
        },
    })
}

fn position_capability(
    reported: AngleRange,
    frames: Frames,
    implemented: bool,
) -> Result<Capability<PositionCapability>, Error> {
    // COMMAND_LONG 1000 constrains targets to ±180°, independently of mechanical travel.
    let minimum = reported.minimum.as_radians().max(-core::f32::consts::PI);
    let maximum = reported.maximum.as_radians().min(core::f32::consts::PI);
    let encodable = minimum <= maximum;
    let range = if encodable {
        AngleRange {
            minimum: Angle::radians(minimum)?,
            maximum: Angle::radians(maximum)?,
        }
    } else {
        reported
    };
    Ok(availability(
        PositionCapability {
            implementation: Implementation::Native,
            range,
            frames,
            maximum_rate: None,
        },
        implemented && encodable,
    ))
}

fn availability<T>(capability: T, implemented: bool) -> Capability<T> {
    if implemented {
        Capability::Available(capability)
    } else {
        Capability::Unimplemented {
            capability: Some(capability),
        }
    }
}

fn angle_range((minimum, maximum): (f32, f32)) -> Result<Option<AngleRange>, Error> {
    if minimum.is_infinite() || maximum.is_infinite() {
        return Err(Error::InvalidLimits);
    }
    if minimum.is_nan() || maximum.is_nan() {
        return Ok(None);
    }
    if minimum > maximum {
        return Err(Error::InvalidLimits);
    }
    Ok(Some(AngleRange {
        minimum: Angle::radians(minimum)?,
        maximum: Angle::radians(maximum)?,
    }))
}
