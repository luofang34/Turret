//! Bounded DJI framing and pitch control without an operating-system dependency.
#![no_std]

#[cfg(test)]
extern crate std;

mod checksum;
mod duml;
mod gimbal;
mod pitch;
mod profile;
mod stream;

#[cfg(test)]
mod capture;

pub use checksum::{frame as frame_checksum, header as header_checksum};
pub use duml::{DumlFrame, DumlParser, RouteAddress};
pub use gimbal::{GimbalAngles, GimbalPitchTarget, PitchTargetError, pitch_response};
pub use stream::ParseStats;

pub use pitch::{
    MAX_INPUT_INTERVAL, MAX_PITCH_RATE, PitchPlanner, PitchRateError, observed_pitch_target,
};
pub use profile::{pitch_capabilities, rc_version_request};
