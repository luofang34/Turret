//! Unit-preserving MAVLink demand encoding; the executor checks advertised support.

use turret_core::*;

/// MAVLink ROLL_LOCK | PITCH_LOCK with vehicle-follow yaw.
pub const HORIZON_VEHICLE_YAW_FLAGS: u32 = 4 | 8;

/// Semantic packet fields for the host's MAVLink serializer and transport.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Demand {
    /// One COMMAND_LONG operation. Routing and protocol sequence belong to the host.
    CommandLong {
        /// MAV_CMD identifier.
        command: u16,
        /// MAVLink parameters in their protocol-defined units.
        params: [f32; 7],
    },
    /// GIMBAL_MANAGER_SET_ATTITUDE with ignored quaternion and selected angular rates.
    AttitudeRate {
        /// Explicit gimbal reference flags.
        flags: u32,
        /// Gimbal device component or manager-local device identifier; zero addresses all.
        gimbal_device_id: u8,
        /// Pitch rate, radians per second; NaN means omitted.
        pitch_rps: f32,
        /// Roll rate, radians per second; NaN means omitted.
        roll_rps: f32,
        /// Yaw rate, radians per second; NaN means omitted.
        yaw_rps: f32,
    },
}

/// Request primary control; queuing this packet does not establish ownership.
pub fn claim_command(system_id: u8, component_id: u8, gimbal_device_id: u8) -> Demand {
    Demand::CommandLong {
        command: 1001,
        params: [
            f32::from(system_id),
            f32::from(component_id),
            -1.0,
            -1.0,
            0.0,
            0.0,
            f32::from(gimbal_device_id),
        ],
    }
}

/// Release primary ownership only if this MAVLink sender still owns it.
pub fn release_command(gimbal_device_id: u8) -> Demand {
    Demand::CommandLong {
        command: 1001,
        params: [
            -3.0,
            -3.0,
            -1.0,
            -1.0,
            0.0,
            0.0,
            f32::from(gimbal_device_id),
        ],
    }
}

/// Request manager status; receiving a transport ACK is not an ownership report.
pub fn request_manager_status() -> Demand {
    Demand::CommandLong {
        command: 512,
        params: [281.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    }
}

/// Encode validated payload intent without converting unrelated zoom coordinates.
/// Camera ID is the MAV_CMD_SET_CAMERA_ZOOM parameter, independently of target component.
pub fn encode_payload(
    command: PayloadCommand,
    gimbal_device_id: u8,
    camera_id: u8,
) -> Result<Demand, Error> {
    match command {
        PayloadCommand::Recenter => Ok(Demand::CommandLong {
            command: 1000,
            params: [
                0.0,
                0.0,
                f32::NAN,
                f32::NAN,
                12.0,
                0.0,
                f32::from(gimbal_device_id),
            ],
        }),
        PayloadCommand::GimbalPosition(position) => {
            check_frame(position.frame)?;
            if position.angles.roll.is_some() {
                return Err(Error::Unsupported);
            }
            if position.angles.pitch.is_none() && position.angles.yaw.is_none() {
                return Err(Error::EmptyCommand);
            }
            for angle in [position.angles.pitch, position.angles.yaw]
                .into_iter()
                .flatten()
            {
                if angle.as_radians().abs() > core::f32::consts::PI {
                    return Err(Error::OutOfRange);
                }
            }
            Ok(Demand::CommandLong {
                command: 1000,
                params: [
                    position
                        .angles
                        .pitch
                        .map_or(f32::NAN, |v| v.as_radians().to_degrees()),
                    position
                        .angles
                        .yaw
                        .map_or(f32::NAN, |v| v.as_radians().to_degrees()),
                    f32::NAN,
                    f32::NAN,
                    12.0,
                    0.0,
                    f32::from(gimbal_device_id),
                ],
            })
        }
        PayloadCommand::GimbalRate(rate) => {
            check_frame(rate.frame)?;
            if rate.rates.roll.is_some() {
                return Err(Error::Unsupported);
            }
            if rate.rates.pitch.is_none() && rate.rates.yaw.is_none() {
                return Err(Error::EmptyCommand);
            }
            Ok(Demand::AttitudeRate {
                flags: HORIZON_VEHICLE_YAW_FLAGS,
                gimbal_device_id,
                pitch_rps: rate
                    .rates
                    .pitch
                    .map_or(f32::NAN, AngularRate::as_radians_per_second),
                roll_rps: f32::NAN,
                yaw_rps: rate
                    .rates
                    .yaw
                    .map_or(f32::NAN, AngularRate::as_radians_per_second),
            })
        }
        PayloadCommand::Zoom(zoom) => encode_zoom(zoom, camera_id),
    }
}

fn check_frame(frame: ReferenceFrame) -> Result<(), Error> {
    if frame == ReferenceFrame::HorizonVehicleYaw {
        Ok(())
    } else {
        Err(Error::UnsupportedFrame)
    }
}

fn encode_zoom(command: ZoomCommand, camera_id: u8) -> Result<Demand, Error> {
    let (kind, value) = match command {
        ZoomCommand::Step(step) => {
            if ![-1, 1].contains(&step) {
                return Err(Error::OutOfRange);
            }
            (0.0, f32::from(step))
        }
        ZoomCommand::Continuous(rate) => (1.0, rate.value()),
        ZoomCommand::Absolute(value) => match value.unit() {
            ZoomUnit::Ratio => return Err(Error::Unsupported),
            ZoomUnit::TravelFraction => (2.0, value.value() * 100.0),
            ZoomUnit::FocalLengthMm => (3.0, value.value()),
            ZoomUnit::HorizontalFovRadians => (4.0, value.value().to_degrees()),
        },
    };
    Ok(Demand::CommandLong {
        command: 531,
        params: [kind, value, f32::from(camera_id), 0.0, 0.0, 0.0, 0.0],
    })
}

/// Best-effort zero-rate request for the affected operation; never a measured stop.
pub fn stop_command(command: PayloadCommand) -> Result<PayloadCommand, Error> {
    match command {
        PayloadCommand::Zoom(ZoomCommand::Continuous(_)) => Ok(PayloadCommand::Zoom(
            ZoomCommand::Continuous(NormalizedZoomRate::new(0.0)?),
        )),
        PayloadCommand::Zoom(_) => Err(Error::Unsupported),
        _ => Ok(PayloadCommand::GimbalRate(GimbalRate {
            rates: Axes {
                pitch: Some(AngularRate::ZERO),
                roll: None,
                yaw: Some(AngularRate::ZERO),
            },
            frame: ReferenceFrame::HorizonVehicleYaw,
        })),
    }
}

#[cfg(test)]
mod tests;
