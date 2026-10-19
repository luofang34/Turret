//! Measurement provenance and freshness independent of command acknowledgements.

use crate::{Angle, Axes, Error, ReferenceFrame, Tick, ZoomAbsolute, ZoomUnit};

/// How a value was obtained.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ObservationSource {
    /// A measurement reported by the physical device.
    DeviceMeasured,
    /// A coordinate transform of physical measurements.
    TransformedMeasurement,
    /// Model prediction or commanded-value estimate.
    Estimated,
    /// An operator-provided assumption, not instrumented feedback.
    OperatorAssumed,
}

/// One value with explicit connection provenance and age.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Observation<T> {
    /// Observed or estimated value.
    pub value: T,
    /// Provenance of the value.
    pub source: ObservationSource,
    /// Angular reference, absent for zoom observations.
    pub frame: Option<ReferenceFrame>,
    /// Sample time converted to the executor clock when that conversion is known.
    pub sampled_at: Option<Tick>,
    /// Local receipt time; does not replace an unknown sample timestamp.
    pub received_at: Tick,
    /// Device connection epoch in which the sample was obtained.
    pub epoch: u64,
}

/// Freshness is reported independently of the value itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Freshness {
    /// Sample belongs to an obsolete device connection.
    WrongEpoch,
    /// Sample time is not established, so actual age is unknown.
    UnknownAge,
    /// Timestamp is inconsistent with the local clock.
    InvalidTime,
    /// Measurement is within the maximum accepted sample age.
    Fresh,
    /// The actual sample is older than the accepted age.
    Stale,
}

impl<T> Observation<T> {
    /// Evaluate sample age; receiving a stale value again does not refresh it.
    pub fn freshness(&self, now: Tick, epoch: u64, maximum_age_us: u64) -> Freshness {
        if self.epoch != epoch {
            return Freshness::WrongEpoch;
        }
        let Some(sampled) = self.sampled_at else {
            return Freshness::UnknownAge;
        };
        if sampled > self.received_at || self.received_at > now {
            return Freshness::InvalidTime;
        }
        match now.elapsed_since(sampled) {
            Ok(age) if age <= maximum_age_us => Freshness::Fresh,
            Ok(_) => Freshness::Stale,
            Err(_) => Freshness::InvalidTime,
        }
    }

    /// Only fresh physical measurements can establish observed completion.
    pub fn validate_measurement(
        &self,
        now: Tick,
        epoch: u64,
        maximum_age_us: u64,
    ) -> Result<(), Error> {
        if !matches!(
            self.source,
            ObservationSource::DeviceMeasured | ObservationSource::TransformedMeasurement
        ) || self.freshness(now, epoch, maximum_age_us) != Freshness::Fresh
        {
            return Err(Error::InvalidObservation);
        }
        Ok(())
    }
}

/// Physical quantities which may establish completion of absolute commands.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PayloadState {
    /// Available measured axes; missing axes remain unknown.
    Gimbal(Axes<Option<Angle>>),
    /// Actual zoom in one explicitly declared coordinate.
    Zoom(ZoomAbsolute),
}

/// Target tolerance in the relevant physical coordinate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CompletionTolerance {
    /// Nonnegative angular tolerance, in radians.
    pub angle: Angle,
    /// Unit and nonnegative tolerance for absolute zoom, if checking zoom.
    pub zoom: Option<(ZoomUnit, f32)>,
    /// Minimum span of continuously matching samples.
    pub settle_time_us: u64,
    /// Maximum gap between matching samples before settling restarts.
    pub maximum_sample_gap_us: u64,
}
