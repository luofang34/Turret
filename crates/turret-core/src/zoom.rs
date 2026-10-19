//! Zoom coordinates are explicit and never converted without calibration.

use crate::{Capability, Error, Implementation, StopGuarantee};

/// Coordinate used for absolute zoom control and feedback.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ZoomUnit {
    /// Magnification relative to the camera's documented reference.
    Ratio,
    /// Normalized lens travel, unrelated to magnification without calibration.
    TravelFraction,
    /// Focal length in millimeters.
    FocalLengthMm,
    /// Horizontal field of view in radians.
    HorizontalFovRadians,
}

/// An absolute zoom value whose unit cannot be confused with another one.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ZoomAbsolute {
    unit: ZoomUnit,
    value: f32,
}

impl ZoomAbsolute {
    /// Validate an absolute value in the supplied zoom coordinate.
    pub fn new(unit: ZoomUnit, value: f32) -> Result<Self, Error> {
        let valid = match unit {
            ZoomUnit::Ratio | ZoomUnit::FocalLengthMm => value > 0.0,
            ZoomUnit::TravelFraction => (0.0..=1.0).contains(&value),
            ZoomUnit::HorizontalFovRadians => value > 0.0 && value < core::f32::consts::PI,
        };
        if !value.is_finite() || !valid {
            return Err(Error::InvalidValue);
        }
        Ok(Self { unit, value })
    }
    /// Explicit coordinate of this value.
    pub const fn unit(self) -> ZoomUnit {
        self.unit
    }
    /// Value expressed in its declared coordinate.
    pub const fn value(self) -> f32 {
        self.value
    }
}

/// Continuous zoom demand in [-1, 1], not magnification per second.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NormalizedZoomRate(f32);

impl NormalizedZoomRate {
    /// Positive zooms in; negative zooms out; zero requests no zoom motion.
    pub fn new(value: f32) -> Result<Self, Error> {
        if !value.is_finite() || !(-1.0..=1.0).contains(&value) {
            return Err(Error::InvalidValue);
        }
        Ok(Self(value))
    }
    /// Normalized signed demand.
    pub const fn value(self) -> f32 {
        self.0
    }
}

/// Advertised range in exactly one absolute zoom coordinate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ZoomRange {
    /// Inclusive lower bound.
    pub minimum: ZoomAbsolute,
    /// Inclusive upper bound.
    pub maximum: ZoomAbsolute,
    /// Native device operation or calibrated backend synthesis.
    pub implementation: Implementation,
}

impl ZoomRange {
    /// Validate both the declared range and a target without unit conversion.
    pub fn validate(self, target: ZoomAbsolute) -> Result<(), Error> {
        if self.minimum.unit() != self.maximum.unit() || self.minimum.value() > self.maximum.value()
        {
            return Err(Error::InvalidLimits);
        }
        if target.unit() != self.minimum.unit() {
            return Err(Error::Unsupported);
        }
        if target.value() < self.minimum.value() || target.value() > self.maximum.value() {
            return Err(Error::OutOfRange);
        }
        Ok(())
    }
}

/// Absolute commands supported independently for each zoom coordinate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AbsoluteZoomCapabilities {
    /// Magnification ratio support.
    pub ratio: Capability<ZoomRange>,
    /// Lens travel support.
    pub travel_fraction: Capability<ZoomRange>,
    /// Focal length support.
    pub focal_length_mm: Capability<ZoomRange>,
    /// Horizontal field of view support.
    pub horizontal_fov_radians: Capability<ZoomRange>,
}

impl AbsoluteZoomCapabilities {
    /// Select support for the supplied coordinate, without converting it.
    pub const fn get(&self, unit: ZoomUnit) -> &Capability<ZoomRange> {
        match unit {
            ZoomUnit::Ratio => &self.ratio,
            ZoomUnit::TravelFraction => &self.travel_fraction,
            ZoomUnit::FocalLengthMm => &self.focal_length_mm,
            ZoomUnit::HorizontalFovRadians => &self.horizontal_fov_radians,
        }
    }
}

/// Discrete zoom increments expressed in device-defined detents.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ZoomStepCapability {
    /// Native or synthesized step operation.
    pub implementation: Implementation,
    /// Maximum absolute number of detents in one command.
    pub maximum_steps: u16,
}

/// Zoom operations, feedback, and stopping contract for one camera.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ZoomCapabilities {
    /// Each absolute coordinate is independently declared.
    pub absolute: AbsoluteZoomCapabilities,
    /// Device-defined step commands.
    pub step: Capability<ZoomStepCapability>,
    /// Normalized continuous zoom demand.
    pub continuous: Capability<Implementation>,
    /// Coordinates in which actual zoom is reported.
    pub feedback: AbsoluteZoomCapabilities,
    /// Behavior when a continuous zoom stream disappears.
    pub stop: StopGuarantee,
}

/// Commands whose types preserve their actual zoom meaning.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ZoomCommand {
    /// Set one absolute, explicitly typed coordinate.
    Absolute(ZoomAbsolute),
    /// Move by a signed number of device detents.
    Step(i16),
    /// Renew a continuous normalized demand.
    Continuous(NormalizedZoomRate),
}
