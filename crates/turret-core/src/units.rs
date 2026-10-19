//! Finite angular quantities, axes, and executor-local time.

use crate::Error;

/// A finite angle in radians.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Angle(f32);

impl Angle {
    /// Construct a finite angle; no wrapping or range clamping is implicit.
    pub fn radians(value: f32) -> Result<Self, Error> {
        value
            .is_finite()
            .then_some(Self(value))
            .ok_or(Error::InvalidValue)
    }
    /// Zero angle.
    pub const ZERO: Self = Self(0.0);
    /// Angle in radians.
    pub const fn as_radians(self) -> f32 {
        self.0
    }
}

/// A finite angular rate in radians per second.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AngularRate(f32);

impl AngularRate {
    /// Construct a finite angular rate.
    pub fn radians_per_second(value: f32) -> Result<Self, Error> {
        value
            .is_finite()
            .then_some(Self(value))
            .ok_or(Error::InvalidValue)
    }
    /// Zero angular rate.
    pub const ZERO: Self = Self(0.0);
    /// Rate in radians per second.
    pub const fn as_radians_per_second(self) -> f32 {
        self.0
    }
}

/// Values for independently controllable mount axes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Axes<T> {
    /// Pitch: positive raises the camera boresight.
    pub pitch: T,
    /// Roll: positive rotates clockwise looking along the boresight.
    pub roll: T,
    /// Yaw: positive turns right in the stated frame.
    pub yaw: T,
}

/// Explicit reference for an angular target or measurement.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReferenceFrame {
    /// Relative to the physical mount's configured zero.
    Mount,
    /// Relative to the vehicle's forward, right, down axes.
    Vehicle,
    /// Earth north, east, down reference.
    EarthNed,
    /// Roll and pitch lock to the horizon; yaw follows the vehicle heading.
    HorizonVehicleYaw,
}

/// Supported angular reference frames.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Frames {
    /// Mount-relative control is supported.
    pub mount: bool,
    /// Vehicle-relative control is supported.
    pub vehicle: bool,
    /// Earth-relative control is supported.
    pub earth_ned: bool,
    /// Horizon roll/pitch with vehicle-follow yaw is supported.
    pub horizon_vehicle_yaw: bool,
}

impl Frames {
    /// Whether a frame is in this set.
    pub const fn contains(self, frame: ReferenceFrame) -> bool {
        match frame {
            ReferenceFrame::Mount => self.mount,
            ReferenceFrame::Vehicle => self.vehicle,
            ReferenceFrame::EarthNed => self.earth_ned,
            ReferenceFrame::HorizonVehicleYaw => self.horizon_vehicle_yaw,
        }
    }
}

/// Microseconds in the final executor's monotonic clock domain.
///
/// This is not a transferable wall-clock timestamp. A transport must convert
/// remaining validity into this domain without renewing an old command.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Tick(pub u64);

impl Tick {
    /// Elapsed local microseconds, rejecting backwards time.
    pub fn elapsed_since(self, earlier: Self) -> Result<u64, Error> {
        self.0.checked_sub(earlier.0).ok_or(Error::ClockRegression)
    }
}
