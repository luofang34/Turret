use std::{vec, vec::Vec};

use super::{DumlParser, MAX_FRAME};
use crate::{
    capture::{RC_N1C, synthetic_frame},
    checksum,
};

#[test]
fn external_capture_parses_at_every_chunk_boundary() {
    for split in 0..=RC_N1C.len() {
        let mut parser = DumlParser::new();
        let mut frames = Vec::new();
        parser.push(&RC_N1C[..split], |frame| frames.push(frame));
        parser.push(&RC_N1C[split..], |frame| frames.push(frame));
        assert_eq!(frames.len(), 1);
        let frame = &frames[0];
        assert_eq!(frame.raw(), RC_N1C);
        assert_eq!(frame.version(), 1);
        assert_eq!(frame.sender().0, 0x0a);
        assert_eq!(frame.receiver().0, 6);
        assert_eq!(frame.sequence(), 0x34eb);
        assert_eq!(frame.flags(), 0x40);
        assert_eq!(frame.command_set(), 6);
        assert_eq!(frame.command_id(), 1);
        assert!(frame.payload().is_empty());
        assert_eq!(parser.buffered_bytes(), 0);
    }
}

#[test]
fn coalesced_frames_keep_unknown_fields_payload_and_routing() {
    let bytes = synthetic_frame(&[0x55, 0xcc, 1, 2]);
    let mut parser = DumlParser::new();
    let mut frames = Vec::new();
    parser.push(&[bytes.as_slice(), &RC_N1C].concat(), |frame| {
        frames.push(frame)
    });
    assert_eq!(frames.len(), 2);
    assert_eq!(frames[0].sender().device_type(), 2);
    assert_eq!(frames[0].sender().index(), 7);
    assert_eq!(frames[0].receiver().device_type(), 3);
    assert_eq!(frames[0].receiver().index(), 3);
    assert_eq!(frames[0].flags(), 255);
    assert_eq!(frames[0].command_set(), 254);
    assert_eq!(frames[0].command_id(), 253);
    assert_eq!(frames[0].payload(), [0x55, 0xcc, 1, 2]);
}

#[test]
fn bad_lengths_header_crcs_and_body_crcs_resynchronize() {
    let mut bad_header = RC_N1C;
    bad_header[3] ^= 1;
    let mut bad_body = RC_N1C;
    bad_body[12] ^= 1;
    let input = [
        &[0xaa, 0xbb][..],
        &[0x55, 0, 4, 0],
        &bad_header,
        &bad_body,
        &RC_N1C,
    ]
    .concat();
    let mut parser = DumlParser::new();
    let mut frames = Vec::new();
    parser.push(&input, |frame| frames.push(frame));
    assert_eq!(frames.len(), 1);
    let stats = parser.stats();
    assert_eq!(stats.invalid_lengths, 1);
    assert_eq!(stats.header_crc_errors, 1);
    assert_eq!(stats.frame_crc_errors, 1);
    assert_eq!(stats.discarded_bytes, 32);
}

#[test]
fn finish_recovers_valid_frame_after_truncated_candidate() {
    let mut input = vec![0x55, 0xff, 0x07];
    input.push(checksum::header(&input));
    input.extend(RC_N1C);
    let mut parser = DumlParser::new();
    let mut frames = Vec::new();
    parser.push(&input, |frame| frames.push(frame));
    assert!(frames.is_empty());
    parser.finish(|frame| frames.push(frame));
    assert_eq!(frames.len(), 1);
    assert_eq!(parser.stats().truncated_candidates, 1);
    assert_eq!(parser.buffered_bytes(), 0);
}

#[test]
fn finish_discards_tail_and_parser_can_start_another_stream() {
    let mut parser = DumlParser::new();
    let mut emitted = Vec::new();
    parser.push(&RC_N1C[..8], |frame| emitted.push(frame));
    parser.finish(|frame| emitted.push(frame));
    assert!(emitted.is_empty());
    assert_eq!(parser.stats().discarded_bytes, 8);
    assert_eq!(parser.buffered_bytes(), 0);
    parser.push(&RC_N1C, |_| {});
    assert_eq!(parser.stats().frames, 1);
}

#[test]
fn full_ten_bit_length_and_unknown_version_are_preserved() {
    let mut bytes = synthetic_frame(&vec![0x31; MAX_FRAME - 13]);
    bytes[2] = 0xff;
    bytes[3] = checksum::header(&bytes[..3]);
    let end = bytes.len() - 2;
    let crc = checksum::frame(&bytes[..end]);
    bytes[end..].copy_from_slice(&crc.to_le_bytes());
    let mut parser = DumlParser::new();
    let mut frames = Vec::new();
    parser.push(&bytes, |frame| frames.push(frame));
    assert_eq!(frames.len(), 1);
    assert_eq!(frames[0].version(), 63);
    assert_eq!(frames[0].raw().len(), MAX_FRAME);
    assert_eq!(parser.stats().peak_buffered_bytes, MAX_FRAME);
}

#[test]
fn large_noise_input_does_not_accumulate() {
    let mut parser = DumlParser::new();
    let mut emitted = Vec::new();
    parser.push(&vec![0x55; 100_000], |frame| emitted.push(frame));
    assert!(parser.buffered_bytes() < 4);
    assert!(parser.stats().peak_buffered_bytes <= MAX_FRAME);
    parser.finish(|frame| emitted.push(frame));
    assert!(emitted.is_empty());
    assert_eq!(parser.stats().discarded_bytes, 100_000);
}

#[test]
fn noise_payloads_are_bounded_for_arbitrary_input() {
    let mut parser = DumlParser::new();
    let mut state = 7_u32;
    for _ in 0..100_000 {
        state = state.wrapping_mul(1664525).wrapping_add(1013904223);
        parser.push(&[(state >> 24) as u8], |_| {});
        assert!(parser.buffered_bytes() <= MAX_FRAME);
    }
    parser.finish(|_| {});
    assert_eq!(parser.buffered_bytes(), 0);
}

#[test]
fn local_c5_passive_capture_validates_without_inventing_payload_semantics() {
    // RC231 bottom CDC passive capture: six unknown payload bytes per frame.
    let bytes = include_bytes!("fixtures/c5-cdc-passive.bin");
    let mut parser = DumlParser::new();
    let mut frames = Vec::new();
    for chunk in bytes.chunks(7) {
        parser.push(chunk, |frame| frames.push(frame));
    }
    parser.finish(|frame| frames.push(frame));
    assert_eq!(frames.len(), 12);
    for frame in &frames {
        assert_eq!(frame.version(), 1);
        assert_eq!(frame.sender().0, 6);
        assert_eq!(frame.receiver().0, 10);
        assert_eq!(frame.sequence(), 1);
        assert_eq!(frame.flags(), 0x20);
        assert_eq!(frame.command_set(), 6);
        assert_eq!(frame.command_id(), 0x1e);
        assert_eq!(frame.payload().len(), 6);
    }
    let stats = parser.stats();
    assert_eq!(stats.header_crc_errors, 0);
    assert_eq!(stats.frame_crc_errors, 0);
    assert_eq!(stats.discarded_bytes, 0);
    assert_eq!(stats.truncated_candidates, 0);
    assert_eq!(
        frames
            .iter()
            .flat_map(|frame| frame.raw())
            .copied()
            .collect::<Vec<_>>(),
        bytes
    );
}

#[test]
fn partial_nested_frame_survives_buffer_compaction() {
    let mut bytes = synthetic_frame(&vec![0x31; MAX_FRAME - 13]);
    bytes[MAX_FRAME - 6..].copy_from_slice(&RC_N1C[..6]);
    let mut parser = DumlParser::new();
    let mut frames = Vec::new();
    parser.push(&bytes, |frame| frames.push(frame));
    assert!(frames.is_empty());
    assert_eq!(parser.buffered_bytes(), 6);
    parser.push(&RC_N1C[6..], |frame| frames.push(frame));
    assert_eq!(frames.len(), 1);
    assert_eq!(frames[0].raw(), RC_N1C);
    assert_eq!(parser.stats().frame_crc_errors, 1);
    assert_eq!(parser.stats().peak_buffered_bytes, MAX_FRAME);
    assert_eq!(parser.buffered_bytes(), 0);
}
