//! Bounded DJI framing and pitch control without an operating-system dependency.
#![no_std]

#[cfg(test)]
extern crate std;

mod checksum;
mod duml;
mod gimbal;
mod keyframe;
mod observed;
mod pitch;
mod profile;
mod stream;

#[cfg(test)]
mod capture;

pub use checksum::{frame as frame_checksum, header as header_checksum};
pub use duml::{DumlFrame, DumlParser, RouteAddress};
pub use gimbal::{GimbalAngles, GimbalPitchTarget, PitchTargetError, pitch_response};
pub use stream::ParseStats;

pub use observed::{ObservedPitchPlan, ObservedPitchPlanner};
pub use pitch::{MAX_INPUT_INTERVAL, PitchPlanner, PitchRateError};
pub use profile::{Mini3ControlProfile, rc_version_request};

pub use keyframe::{
    KeyframeAbort, KeyframeState, KeyframeWriteError, KeyframeWriteReceipt,
    MINI3_KEYFRAME_INTERVAL_US, Mini3AccessoryKeyframe,
};
