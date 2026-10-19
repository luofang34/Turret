//! Validate intent against the actual advertised operations.

use crate::{
    AxisCapabilities, Error, GimbalPosition, PayloadCapabilities, PayloadCommand, ReferenceFrame,
    ZoomCommand,
};

pub(super) fn validate_payload(
    capabilities: &PayloadCapabilities,
    payload: PayloadCommand,
) -> Result<(), Error> {
    match payload {
        PayloadCommand::GimbalPosition(position) => validate_position(capabilities, position),
        PayloadCommand::GimbalRate(rate) => {
            let mut any = false;
            for (value, axis) in [
                (rate.rates.pitch, &capabilities.gimbal.axes.pitch),
                (rate.rates.roll, &capabilities.gimbal.axes.roll),
                (rate.rates.yaw, &capabilities.gimbal.axes.yaw),
            ] {
                if let Some(value) = value {
                    any = true;
                    let mode = axis.rate.available()?;
                    if !mode.frames.contains(rate.frame) {
                        return Err(Error::UnsupportedFrame);
                    }
                    let maximum = mode.maximum_rate.as_radians_per_second();
                    if maximum <= 0.0 {
                        return Err(Error::InvalidLimits);
                    }
                    if value.as_radians_per_second().abs() > maximum {
                        return Err(Error::OutOfRange);
                    }
                }
            }
            if any {
                Ok(())
            } else {
                Err(Error::EmptyCommand)
            }
        }
        PayloadCommand::Recenter => {
            let recenter = capabilities.gimbal.recenter.available()?;
            match recenter.target {
                Some(target) => validate_position(capabilities, target),
                None if recenter.implementation == crate::Implementation::Native => Ok(()),
                None => Err(Error::NoCompletionTarget),
            }
        }
        PayloadCommand::Zoom(command) => {
            let zoom = capabilities.zoom.available()?;
            match command {
                ZoomCommand::Absolute(target) => zoom
                    .absolute
                    .get(target.unit())
                    .available()?
                    .validate(target),
                ZoomCommand::Step(steps) => {
                    let limits = zoom.step.available()?;
                    if steps.unsigned_abs() > limits.maximum_steps {
                        return Err(Error::OutOfRange);
                    }
                    if limits.maximum_steps == 0 {
                        return Err(Error::InvalidLimits);
                    }
                    Ok(())
                }
                ZoomCommand::Continuous(_) => zoom.continuous.available().map(|_| ()),
            }
        }
    }
}

fn validate_position(
    capabilities: &PayloadCapabilities,
    position: GimbalPosition,
) -> Result<(), Error> {
    let mut any = false;
    for (value, axis) in [
        (position.angles.pitch, &capabilities.gimbal.axes.pitch),
        (position.angles.roll, &capabilities.gimbal.axes.roll),
        (position.angles.yaw, &capabilities.gimbal.axes.yaw),
    ] {
        if let Some(value) = value {
            any = true;
            validate_axis_position(axis, value, position.frame)?;
        }
    }
    if any {
        Ok(())
    } else {
        Err(Error::EmptyCommand)
    }
}

fn validate_axis_position(
    axis: &AxisCapabilities,
    value: crate::Angle,
    frame: ReferenceFrame,
) -> Result<(), Error> {
    let mode = axis.position.available()?;
    if !mode.frames.contains(frame) {
        return Err(Error::UnsupportedFrame);
    }
    if mode
        .maximum_rate
        .is_some_and(|rate| rate.as_radians_per_second() <= 0.0)
    {
        return Err(Error::InvalidLimits);
    }
    mode.range.validate(value)
}
