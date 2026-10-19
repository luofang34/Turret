//! Completion is a comparison with fresh measurements, never command echo.

use crate::{
    Angle, Axes, Capability, CompletionTolerance, Error, GimbalPosition, Observation,
    PayloadCommand, PayloadState, RecenterCapability, ZoomCommand,
};

pub(super) fn matches_target(
    command: PayloadCommand,
    recenter: Capability<RecenterCapability>,
    observation: &Observation<PayloadState>,
    tolerance: CompletionTolerance,
) -> Result<bool, Error> {
    if tolerance.angle.as_radians() < 0.0 {
        return Err(Error::InvalidValue);
    }
    match command {
        PayloadCommand::GimbalPosition(target) => {
            compare_gimbal(target, observation, tolerance.angle)
        }
        PayloadCommand::Recenter => compare_gimbal(
            recenter
                .available()?
                .target
                .ok_or(Error::NoCompletionTarget)?,
            observation,
            tolerance.angle,
        ),
        PayloadCommand::Zoom(ZoomCommand::Absolute(target)) => {
            let PayloadState::Zoom(measured) = observation.value else {
                return Err(Error::InvalidObservation);
            };
            let (unit, amount) = tolerance.zoom.ok_or(Error::InvalidValue)?;
            if !amount.is_finite() || amount < 0.0 {
                return Err(Error::InvalidValue);
            }
            if observation.frame.is_some()
                || unit != target.unit()
                || measured.unit() != target.unit()
            {
                return Err(Error::InvalidObservation);
            }
            Ok((target.value() - measured.value()).abs() <= amount)
        }
        _ => Err(Error::NoCompletionTarget),
    }
}

fn compare_gimbal(
    target: GimbalPosition,
    observation: &Observation<PayloadState>,
    tolerance: Angle,
) -> Result<bool, Error> {
    let PayloadState::Gimbal(measured) = observation.value else {
        return Err(Error::InvalidObservation);
    };
    if observation.frame != Some(target.frame) {
        return Err(Error::InvalidObservation);
    }
    compare_axes(target.angles, measured, tolerance)
}

fn compare_axes(
    target: Axes<Option<Angle>>,
    measured: Axes<Option<Angle>>,
    tolerance: Angle,
) -> Result<bool, Error> {
    for (wanted, actual) in [
        (target.pitch, measured.pitch),
        (target.roll, measured.roll),
        (target.yaw, measured.yaw),
    ] {
        if let Some(wanted) = wanted {
            let actual = actual.ok_or(Error::InvalidObservation)?;
            if (actual.as_radians() - wanted.as_radians()).abs() > tolerance.as_radians() {
                return Ok(false);
            }
        }
    }
    Ok(true)
}

pub(super) fn settle(
    record: &mut super::Record,
    sampled: crate::Tick,
    tolerance: CompletionTolerance,
    now: crate::Tick,
) -> Result<bool, Error> {
    if record
        .last_matching_sample
        .is_some_and(|last| sampled <= last)
    {
        record.settle_started_at = None;
        record.last_matching_sample = None;
        return Err(Error::InvalidObservation);
    }
    let contiguous = record
        .last_matching_sample
        .is_some_and(|last| sampled.0.saturating_sub(last.0) <= tolerance.maximum_sample_gap_us);
    if !contiguous {
        record.settle_started_at = Some(sampled);
    }
    record.last_matching_sample = Some(sampled);
    let settled = record
        .settle_started_at
        .is_some_and(|start| sampled.0.saturating_sub(start.0) >= tolerance.settle_time_us);
    if settled {
        record.progress.observed_complete_at = Some(now);
        record.progress.ended = Some(crate::EndReason::ObservedComplete);
    }
    Ok(settled)
}
