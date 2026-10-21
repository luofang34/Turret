use std::vec::Vec;

use super::{GimbalAngles, PitchTargetError, pitch_response};
use crate::{DumlFrame, DumlParser, Mini3ControlProfile, RouteAddress, checksum};

fn profile() -> Mini3ControlProfile {
    Mini3ControlProfile::new(
        -900,
        0,
        turret_core::AngularRate::radians_per_second(5_f32.to_radians()).unwrap(),
    )
    .unwrap()
}

// RC231 / Mini 3 capture raw SHA-256:
// 2dc370d945475cc5ee71aa04ea0cc901d88296e8b20a0e654f232891b8b8d273.
// Observation byte offsets: 429496, 1006998, 3671919; each frame is 62 bytes.
// Response byte offset: 489970. The command is the transmitted DUML frame.
const OBSERVATIONS: &[u8] = include_bytes!("fixtures/pitch-observations.bin");
const COMMAND: &[u8] = include_bytes!("fixtures/pitch-command.bin");
const RESPONSE: &[u8] = include_bytes!("fixtures/pitch-response.bin");

fn parse(bytes: &[u8]) -> Vec<DumlFrame> {
    let mut parser = DumlParser::new();
    let mut frames = Vec::new();
    for chunk in bytes.chunks(3) {
        parser.push(chunk, |frame| frames.push(frame));
    }
    assert_eq!(parser.buffered_bytes(), 0);
    assert_eq!(parser.stats().discarded_bytes, 0);
    frames
}

fn rewrite(bytes: &mut [u8]) {
    let length = bytes.len();
    bytes[3] = checksum::header(&bytes[..3]);
    let crc = checksum::frame(&bytes[..length - 2]).to_le_bytes();
    bytes[length - 2..].copy_from_slice(&crc);
}

#[test]
fn the_pitch_encoder_matches_the_captured_command() -> Result<(), PitchTargetError> {
    let target = profile().pitch_target(-50)?;
    assert_eq!(profile().encode_pitch(target, 34531)?, COMMAND);
    assert_eq!(target.tenths(), -50);
    Ok(())
}

#[test]
fn every_target_preserves_the_route_ignored_axes_and_duration() -> Result<(), PitchTargetError> {
    for tenths in -900..=0 {
        let target = profile().pitch_target(tenths)?;
        for sequence in [0, 1, u16::MAX] {
            let frames = parse(&profile().encode_pitch(target, sequence)?);
            assert_eq!(frames.len(), 1);
            let frame = &frames[0];
            assert_eq!(frame.sender(), RouteAddress(2));
            assert_eq!(frame.receiver(), RouteAddress(4));
            assert_eq!(frame.sequence(), sequence);
            assert_eq!(frame.flags(), 0x40);
            assert_eq!((frame.command_set(), frame.command_id()), (4, 0x14));
            let pitch = tenths.to_le_bytes();
            assert_eq!(frame.payload(), [0, 0, 0, 0, pitch[0], pitch[1], 7, 10]);
        }
    }
    Ok(())
}

#[test]
fn a_target_outside_the_encoder_range_cannot_be_encoded() {
    for tenths in [i16::MIN, -901, 1, i16::MAX] {
        assert_eq!(
            profile().pitch_target(tenths),
            Err(PitchTargetError {
                tenths,
                minimum_tenths: -900,
                maximum_tenths: 0,
            })
        );
    }
}

#[test]
fn actual_pushes_keep_the_three_angles_and_wire_sequence() {
    let observed: Vec<_> = parse(OBSERVATIONS)
        .iter()
        .filter_map(|frame| GimbalAngles::from_push(frame, RouteAddress(2)))
        .collect();
    let expected: Vec<_> = [(0, 8020), (-49, 9401), (-49, 16071)]
        .into_iter()
        .map(|(pitch_tenths, sequence)| GimbalAngles {
            pitch_tenths,
            roll_tenths: 0,
            yaw_tenths: 39,
            sequence,
        })
        .collect();
    assert_eq!(observed, expected);
    assert!(
        parse(OBSERVATIONS)
            .iter()
            .all(|frame| GimbalAngles::from_push(frame, RouteAddress(10)).is_none())
    );
}

#[test]
fn a_response_is_correlated_without_claiming_motion() {
    let frames = parse(RESPONSE);
    assert_eq!(frames.len(), 1);
    assert_eq!(pitch_response(&frames[0], 34531), Some(0));
    assert_eq!(pitch_response(&frames[0], 34532), None);
    assert_eq!(GimbalAngles::from_push(&frames[0], RouteAddress(2)), None);
    let mut denied = RESPONSE.to_vec();
    denied[11] = 0xe0;
    rewrite(&mut denied);
    assert_eq!(pitch_response(&parse(&denied)[0], 34531), Some(0xe0));
}

#[test]
fn another_route_version_or_kind_is_not_a_gimbal_observation() {
    for (index, value) in [(2, 8), (4, 6), (5, 10), (8, 0x80), (9, 0), (10, 0x14)] {
        let mut bytes = OBSERVATIONS[..62].to_vec();
        bytes[index] = value;
        rewrite(&mut bytes);
        let frames = parse(&bytes);
        assert_eq!(frames.len(), 1);
        assert_eq!(GimbalAngles::from_push(&frames[0], RouteAddress(2)), None);
    }
}

#[test]
fn short_pushes_do_not_create_zero_filled_angles() {
    for payload_len in 0..6 {
        let mut bytes = OBSERVATIONS[..11 + payload_len].to_vec();
        bytes.extend([0, 0]);
        bytes[1] = bytes.len() as u8;
        rewrite(&mut bytes);
        let frames = parse(&bytes);
        assert_eq!(frames.len(), 1);
        assert_eq!(GimbalAngles::from_push(&frames[0], RouteAddress(2)), None);
    }
}
