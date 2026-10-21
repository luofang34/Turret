//! Physical mechanics, endpoint operations, and current execution guarantees.

use crate::{Angle, AngularRate, Axes, Error, Frames, GimbalPosition, ZoomCapabilities};

/// How an operation is implemented by its backend.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Implementation {
    /// A device command directly expresses the operation.
    Native,
    /// The backend synthesizes it from other device operations.
    Synthesized,
}

/// Why supported functionality cannot presently be used.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UnavailableReason {
    /// No active device connection exists.
    Disconnected,
    /// This connection does not carry the required protocol.
    Transport,
    /// The device's current mode does not permit the operation.
    DeviceMode,
    /// Required state feedback is missing or stale.
    MissingFeedback,
    /// Required calibration has not been established.
    MissingCalibration,
    /// The operation requires control authority not currently held.
    NoAuthority,
    /// A configured operating policy disables the operation.
    Policy,
}

/// Support and current usability of one endpoint operation.
///
/// Operations can be synthesized. They do not establish mechanical axis presence,
/// stabilization, or measured camera orientation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Capability<T> {
    /// Support has not been established for this device and firmware.
    Unknown,
    /// The endpoint does not support this operation.
    Unsupported,
    /// The endpoint supports the operation, but this backend cannot execute it.
    Unimplemented {
        /// Known operation contract, absent when its limits or coordinates are unknown.
        capability: Option<T>,
    },
    /// The operation can currently be used.
    Available(T),
    /// The backend implements it, but a condition prevents current use.
    Unavailable {
        /// The supported operation's contract.
        capability: T,
        /// Condition preventing use.
        reason: UnavailableReason,
    },
}

impl<T> Capability<T> {
    /// Return the presently usable contract without collapsing unknown support.
    pub fn available(&self) -> Result<&T, Error> {
        match self {
            Self::Available(value) => Ok(value),
            Self::Unknown => Err(Error::UnknownCapability),
            Self::Unsupported => Err(Error::Unsupported),
            Self::Unimplemented { .. } => Err(Error::NotImplemented),
            Self::Unavailable { .. } => Err(Error::Unavailable),
        }
    }
}

/// An absolute, finite angular interval in radians.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AngleRange {
    /// Inclusive lower bound.
    pub minimum: Angle,
    /// Inclusive upper bound.
    pub maximum: Angle,
}

impl AngleRange {
    /// Check the interval and a requested position.
    pub fn validate(self, value: Angle) -> Result<(), Error> {
        if self.minimum.as_radians() > self.maximum.as_radians() {
            return Err(Error::InvalidLimits);
        }
        if value.as_radians() < self.minimum.as_radians()
            || value.as_radians() > self.maximum.as_radians()
        {
            return Err(Error::OutOfRange);
        }
        Ok(())
    }
}

/// Absolute-position control on one mount axis.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PositionCapability {
    /// Native device support or backend synthesis.
    pub implementation: Implementation,
    /// Allowed target range.
    pub range: AngleRange,
    /// Supported coordinate frames.
    pub frames: Frames,
    /// Known speed limit, if supplied by the backend.
    pub maximum_rate: Option<AngularRate>,
}

/// Continuous rate control on one mount axis.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RateCapability {
    /// Native device support or backend synthesis.
    pub implementation: Implementation,
    /// Inclusive maximum absolute angular rate; must be positive.
    pub maximum_rate: AngularRate,
    /// Supported coordinate frames.
    pub frames: Frames,
}

/// Available measurements are independent of commanded axes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FeedbackCapabilities {
    /// Whether the device reports this axis's measured angle.
    pub angle: Capability<()>,
    /// Whether the device reports this axis's measured angular rate.
    pub rate: Capability<()>,
}

/// Established physical support, independent of backend implementation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HardwareSupport {
    /// Physical support has not been established for this attachment.
    Unknown,
    /// The hardware does not provide the feature.
    Unsupported,
    /// The hardware provides the feature.
    Supported,
}

/// Physical axis properties; these never grant command authority or feedback.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AxisMechanics {
    /// Whether the mount can physically move about this axis.
    pub movement: HardwareSupport,
    /// Whether the mount can stabilize the optical payload about this axis.
    pub stabilization: HardwareSupport,
}

impl AxisMechanics {
    /// No physical properties have been established.
    pub const UNKNOWN: Self = Self {
        movement: HardwareSupport::Unknown,
        stabilization: HardwareSupport::Unknown,
    };
}

/// One axis's physical, command, and observation contracts.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxisCapabilities {
    /// Physical properties independent of logical operation support.
    pub mechanics: AxisMechanics,
    /// Absolute position support.
    pub position: Capability<PositionCapability>,
    /// Continuous rate support.
    pub rate: Capability<RateCapability>,
    /// Measurements available independently of control support.
    pub feedback: FeedbackCapabilities,
}

impl AxisCapabilities {
    /// No physical properties, operations, or feedback have been established.
    pub const UNKNOWN: Self = Self {
        mechanics: AxisMechanics::UNKNOWN,
        position: Capability::Unknown,
        rate: Capability::Unknown,
        feedback: FeedbackCapabilities {
            angle: Capability::Unknown,
            rate: Capability::Unknown,
        },
    };

    /// No endpoint control or feedback; physical properties remain unknown.
    pub const UNSUPPORTED: Self = Self {
        mechanics: AxisMechanics::UNKNOWN,
        position: Capability::Unsupported,
        rate: Capability::Unsupported,
        feedback: FeedbackCapabilities {
            angle: Capability::Unsupported,
            rate: Capability::Unsupported,
        },
    };
}

/// Physical stopping behavior when the control stream disappears.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StopGuarantee {
    /// No stopping behavior has been established.
    Unknown,
    /// The host can attempt a stop; loss of the host provides no guarantee.
    BestEffort,
    /// Every emitted device action has bounded duration.
    FiniteMotion {
        /// Maximum duration of a final emitted action in microseconds.
        maximum_duration_us: u64,
    },
    /// A watchdog beside or within the actuator stops unrenewed motion.
    DeviceWatchdog {
        /// Maximum silence before the watchdog acts, in microseconds.
        maximum_silence_us: u64,
    },
}

/// Recenter support with independently known target geometry.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RecenterCapability {
    /// Native or synthesized operation.
    pub implementation: Implementation,
    /// Target for measured completion, or unknown when defined by device configuration.
    pub target: Option<GimbalPosition>,
}

/// Gimbal capabilities for one independently controlled mount.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GimbalCapabilities {
    /// Per-axis control and feedback.
    pub axes: Axes<AxisCapabilities>,
    /// Recenter operation with an explicit reference pose.
    pub recenter: Capability<RecenterCapability>,
    /// Stopping behavior independent of command acknowledgements.
    pub stop: StopGuarantee,
}

/// Capabilities exposed by one camera and its associated mount.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PayloadCapabilities {
    /// Changes when capability assumptions change within a connection.
    pub revision: u64,
    /// Gimbal support.
    pub gimbal: GimbalCapabilities,
    /// Camera zoom support.
    pub zoom: Capability<ZoomCapabilities>,
}
