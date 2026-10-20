//! Pitch commands and raw gimbal observations.

use crate::{DumlFrame, RouteAddress, checksum};

const APP: u8 = 2;
const GIMBAL: u8 = 4;

/// A pitch target outside the selected control profile.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
#[error("gimbal pitch {tenths} tenths of a degree is outside {minimum_tenths}..=0")]
pub struct PitchTargetError {
    /// The rejected target in tenths of a degree.
    pub tenths: i16,
    /// The selected profile's inclusive lower bound.
    pub minimum_tenths: i16,
}

/// A profile-validated pitch target, encoded with a one-second movement duration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GimbalPitchTarget {
    tenths: i16,
}

impl GimbalPitchTarget {
    pub(crate) fn new(
        profile: crate::Mini3ControlProfile,
        tenths: i16,
    ) -> Result<Self, PitchTargetError> {
        if (profile.minimum_tenths()..=0).contains(&tenths) {
            Ok(Self { tenths })
        } else {
            Err(PitchTargetError {
                tenths,
                minimum_tenths: profile.minimum_tenths(),
            })
        }
    }

    /// Returns the target in tenths of a degree.
    #[must_use]
    pub const fn tenths(self) -> i16 {
        self.tenths
    }

    /// Encodes a DUML APP-to-gimbal absolute pitch command.
    ///
    /// Yaw and roll are ignored. This returns bytes only; it does not open a
    /// transport or authorize a write. The caller supplies the wire sequence.
    #[must_use]
    pub(crate) fn encode_app_command(self, sequence: u16) -> [u8; 21] {
        let sequence = sequence.to_le_bytes();
        let pitch = self.tenths.to_le_bytes();
        let mut bytes = [
            0x55,
            21,
            4,
            0,
            APP,
            GIMBAL,
            sequence[0],
            sequence[1],
            0x40,
            4,
            0x14,
            0,
            0,
            0,
            0,
            pitch[0],
            pitch[1],
            0x07,
            10,
            0,
            0,
        ];
        bytes[3] = checksum::header(&bytes[..3]);
        let crc = checksum::frame(&bytes[..19]).to_le_bytes();
        bytes[19..].copy_from_slice(&crc);
        bytes
    }
}

/// The first three angles in a gimbal push, in tenths of a degree.
///
/// The yaw reference and device mode are not established by these fields.
/// This value is not a vehicle attitude or a freshness guarantee.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GimbalAngles {
    /// Pitch in tenths of a degree.
    pub pitch_tenths: i16,
    /// Roll in tenths of a degree.
    pub roll_tenths: i16,
    /// Yaw in tenths of a degree, with no declared reference frame.
    pub yaw_tenths: i16,
    /// The push sequence, which can skip values shared with other messages.
    pub sequence: u16,
}

impl GimbalAngles {
    /// Decodes a version-one 04/05 push to the specified recipient.
    ///
    /// An ACK, another route, or a short payload produces no observation.
    #[must_use]
    pub fn from_push(frame: &DumlFrame, recipient: RouteAddress) -> Option<Self> {
        if !matches_gimbal(frame, recipient, 0, 5) {
            return None;
        }
        let bytes = frame.payload().get(..6)?;
        Some(Self {
            pitch_tenths: i16::from_le_bytes([bytes[0], bytes[1]]),
            roll_tenths: i16::from_le_bytes([bytes[2], bytes[3]]),
            yaw_tenths: i16::from_le_bytes([bytes[4], bytes[5]]),
            sequence: frame.sequence(),
        })
    }
}

/// Returns a matching APP pitch-command response code.
///
/// Zero is the device's command response. It does not prove physical motion.
#[must_use]
pub fn pitch_response(frame: &DumlFrame, sequence: u16) -> Option<u8> {
    if !matches_gimbal(frame, RouteAddress(APP), 0x80, 0x14) || frame.sequence() != sequence {
        return None;
    }
    match frame.payload() {
        [code] => Some(*code),
        _ => None,
    }
}

fn matches_gimbal(frame: &DumlFrame, recipient: RouteAddress, flags: u8, command: u8) -> bool {
    frame.version() == 1
        && frame.sender() == RouteAddress(GIMBAL)
        && frame.receiver() == recipient
        && frame.flags() == flags
        && frame.command_set() == 4
        && frame.command_id() == command
}

#[cfg(test)]
mod tests;
