use core::convert::Infallible;
use std::{io, vec::Vec};

use super::*;
use crate::{DumlFrame, DumlParser, RouteAddress};

const SESSION: [u8; 16] = [1; 16];
const REQUEST_ONE: &[u8; 96] = include_bytes!("fixtures/mini3-request-1.bin");
const REQUEST_TWO: &[u8; 96] = include_bytes!("fixtures/mini3-request-2.bin");

fn request(sequence: u16, deadline: u64) -> Mini3AccessoryKeyframe {
    Mini3AccessoryKeyframe::new(SESSION, sequence, Tick(0), Tick(deadline)).unwrap()
}

fn accept(request: &mut Mini3AccessoryKeyframe, now: u64, completed: u64) -> usize {
    request
        .write_available(SESSION, Tick(now), |bytes| {
            Ok::<_, Infallible>(KeyframeWriteReceipt {
                accepted: bytes.len(),
                completed_at: Tick(completed),
            })
        })
        .unwrap()
}

fn frames(bytes: &[u8]) -> Vec<DumlFrame> {
    let mut parser = DumlParser::new();
    let mut frames = Vec::new();
    let (packets, remainder) = bytes.as_chunks::<32>();
    assert!(remainder.is_empty());
    for packet in packets {
        assert_eq!(&packet[..8], &[0x55, 0xcc, 0x49, 0x57, 24, 0, 0, 0]);
        parser.push(&packet[8..], |frame| frames.push(frame));
    }
    assert_eq!(parser.stats().header_crc_errors, 0);
    assert_eq!(parser.stats().frame_crc_errors, 0);
    assert_eq!(parser.buffered_bytes(), 0);
    assert_eq!(frames.len(), 3);
    frames
}

fn transmit(request: &mut Mini3AccessoryKeyframe) -> Vec<u8> {
    let mut bytes = Vec::new();
    while let Some(now) = request.next_write_at() {
        request
            .write_available(SESSION, now, |suffix| {
                bytes.extend_from_slice(suffix);
                Ok::<_, Infallible>(KeyframeWriteReceipt {
                    accepted: suffix.len(),
                    completed_at: now,
                })
            })
            .unwrap();
    }
    bytes
}

#[test]
fn complete_transmission_matches_both_independent_capture_fixtures() {
    for (sequence, fixture) in [(45092, REQUEST_ONE), (33924, REQUEST_TWO)] {
        let mut request = request(sequence, 1_000_000);
        let bytes = transmit(&mut request);
        assert_eq!(bytes, fixture);
        assert_eq!(request.accepted_bytes(), 96);
        assert_eq!(
            request.state(),
            KeyframeState::TransportAccepted(Tick(100_000))
        );
        for (index, frame) in frames(&bytes).iter().enumerate() {
            assert_eq!(frame.version(), 1);
            assert_eq!(frame.sender(), RouteAddress(2));
            assert_eq!(frame.receiver(), RouteAddress(0x0e));
            assert_eq!(frame.flags(), 0);
            assert_eq!(frame.command_set(), 1);
            assert_eq!(frame.command_id(), 1);
            assert_eq!(frame.sequence(), sequence.wrapping_add(index as u16));
            let request_byte = if index == 2 { 4 } else { 0x24 };
            assert_eq!(
                frame.payload(),
                &[0, 0, 0, 0, 0, request_byte, 0, 0, 0, request_byte, 0]
            );
        }
    }
}

#[test]
fn sequence_rollover_recomputes_valid_checksums() {
    let bytes = transmit(&mut request(u16::MAX, 1_000_000));
    let sequences: Vec<_> = frames(&bytes).iter().map(DumlFrame::sequence).collect();
    assert_eq!(sequences, [u16::MAX, 0, 1]);
}

#[test]
fn short_writes_resume_only_current_suffix_without_replaying_bytes() {
    let mut request = request(45092, 1_000_000);
    let mut sent = Vec::new();
    for accepted in [5, 0, 7, 20, 13, 19, 31, 1] {
        let now = request.next_write_at().unwrap();
        let offset = request.accepted_bytes();
        let frame_end = (offset / 32 + 1) * 32;
        request
            .write_available(SESSION, now, |suffix| {
                assert_eq!(suffix, &REQUEST_ONE[offset..frame_end]);
                sent.extend_from_slice(&suffix[..accepted]);
                Ok::<_, Infallible>(KeyframeWriteReceipt {
                    accepted,
                    completed_at: now,
                })
            })
            .unwrap();
    }
    assert_eq!(sent, REQUEST_ONE);
    assert_eq!(
        request.state(),
        KeyframeState::TransportAccepted(Tick(100_000))
    );
}

#[test]
fn interval_starts_after_actual_full_acceptance_not_attempt_or_partial_write() {
    let mut request = request(1, 1_000_000);
    request
        .write_available(SESSION, Tick(0), |_| {
            Ok::<_, Infallible>(KeyframeWriteReceipt {
                accepted: 10,
                completed_at: Tick(25_000),
            })
        })
        .unwrap();
    assert_eq!(request.state(), KeyframeState::Partial);
    assert_eq!(accept(&mut request, 30_000, 70_000), 22);
    assert_eq!(request.next_write_at(), Some(Tick(120_000)));
    assert_eq!(
        request
            .write_available(SESSION, Tick(119_999), |_| -> Result<_, Infallible> {
                panic!("next frame must wait for full acceptance plus 50 ms")
            })
            .unwrap(),
        0
    );
    assert_eq!(accept(&mut request, 120_000, 140_000), 32);
    assert_eq!(request.next_write_at(), Some(Tick(190_000)));
}

#[test]
fn zero_byte_write_does_not_advance_or_restart_interval() {
    let mut request = request(1, 1_000_000);
    assert_eq!(accept(&mut request, 0, 10), 32);
    request
        .write_available(SESSION, Tick(50_010), |_| {
            Ok::<_, Infallible>(KeyframeWriteReceipt {
                accepted: 0,
                completed_at: Tick(55_000),
            })
        })
        .unwrap();
    assert_eq!(request.accepted_bytes(), 32);
    assert_eq!(request.next_write_at(), Some(Tick(50_010)));
    assert_eq!(accept(&mut request, 55_000, 55_001), 32);
    assert_eq!(request.next_write_at(), Some(Tick(105_001)));
}

#[test]
fn ready_time_delays_initial_write_and_deadline_is_exclusive() {
    let mut request = Mini3AccessoryKeyframe::new(SESSION, 1, Tick(100), Tick(200)).unwrap();
    assert_eq!(
        request
            .write_available(SESSION, Tick(99), |_| -> Result<_, Infallible> {
                panic!("not ready")
            })
            .unwrap(),
        0
    );
    assert_eq!(accept(&mut request, 100, 100), 32);
    assert_eq!(
        request.poll(SESSION, Tick(200)),
        Err(KeyframeAbort::DeadlineExpired)
    );
    assert_eq!(request.accepted_bytes(), 32);
    assert_eq!(request.next_write_at(), None);
}

#[test]
fn deadline_expires_partial_frame_without_sending_remaining_suffix() {
    let mut request = request(1, 100);
    request
        .write_available(SESSION, Tick(0), |_| {
            Ok::<_, Infallible>(KeyframeWriteReceipt {
                accepted: 3,
                completed_at: Tick(1),
            })
        })
        .unwrap();
    let result = request.write_available(SESSION, Tick(100), |_| -> Result<_, Infallible> {
        panic!("expired suffix must not reach writer")
    });
    assert!(matches!(
        result,
        Err(KeyframeWriteError::State(KeyframeAbort::DeadlineExpired))
    ));
    assert_eq!(request.accepted_bytes(), 3);
}

#[test]
fn cancellation_and_close_are_permanent_after_partial_write() {
    for reason in [KeyframeAbort::Cancelled, KeyframeAbort::Closed] {
        let mut request = request(1, 1_000_000);
        request
            .write_available(SESSION, Tick(0), |_| {
                Ok::<_, Infallible>(KeyframeWriteReceipt {
                    accepted: 5,
                    completed_at: Tick(1),
                })
            })
            .unwrap();
        if reason == KeyframeAbort::Cancelled {
            request.cancel();
        } else {
            request.close();
        }
        assert_eq!(request.state(), KeyframeState::Aborted(reason));
        for session in [SESSION, [2; 16]] {
            let result = request.write_available(session, Tick(10), |_| -> Result<_, Infallible> {
                panic!("terminal request must not replay")
            });
            assert!(matches!(result, Err(KeyframeWriteError::State(actual)) if actual == reason));
        }
        assert_eq!(request.accepted_bytes(), 5);
    }
}

#[test]
fn session_change_rejects_queued_write_and_original_session_cannot_resume() {
    let mut request = request(1, 1_000_000);
    assert_eq!(accept(&mut request, 0, 0), 32);
    let result = request.write_available([2; 16], Tick(50_000), |_| -> Result<_, Infallible> {
        panic!("request belongs to a different connection")
    });
    assert!(matches!(
        result,
        Err(KeyframeWriteError::State(KeyframeAbort::SessionChanged))
    ));
    assert_eq!(
        request.poll(SESSION, Tick(60_000)),
        Err(KeyframeAbort::SessionChanged)
    );
    assert_eq!(request.accepted_bytes(), 32);
}

#[test]
fn write_failure_retains_transport_source_and_is_not_retriable() {
    let mut request = request(1, 1_000_000);
    let error = request
        .write_available(SESSION, Tick(0), |_| {
            Err(io::Error::new(
                io::ErrorKind::BrokenPipe,
                "accessory disconnected",
            ))
        })
        .unwrap_err();
    assert!(
        matches!(error, KeyframeWriteError::Transport(source) if source.kind() == io::ErrorKind::BrokenPipe)
    );
    assert_eq!(
        request.poll(SESSION, Tick(1)),
        Err(KeyframeAbort::WriteFailed)
    );
    assert_eq!(request.accepted_bytes(), 0);
}

#[test]
fn invalid_write_count_aborts_without_claiming_accepted_bytes() {
    let mut request = request(1, 1_000_000);
    let result = request.write_available(SESSION, Tick(0), |bytes| {
        Ok::<_, Infallible>(KeyframeWriteReceipt {
            accepted: bytes.len() + 1,
            completed_at: Tick(1),
        })
    });
    assert!(matches!(
        result,
        Err(KeyframeWriteError::State(KeyframeAbort::InvalidWriteCount))
    ));
    assert_eq!(request.accepted_bytes(), 0);
}

#[test]
fn post_write_expiry_retains_actual_accepted_prefix_but_ends_request() {
    let mut request = request(1, 100);
    let result = request.write_available(SESSION, Tick(99), |_| {
        Ok::<_, Infallible>(KeyframeWriteReceipt {
            accepted: 12,
            completed_at: Tick(100),
        })
    });
    assert!(matches!(
        result,
        Err(KeyframeWriteError::State(KeyframeAbort::DeadlineExpired))
    ));
    assert_eq!(request.accepted_bytes(), 12);
    assert_eq!(request.next_write_at(), None);
}

#[test]
fn clock_regression_before_or_after_write_is_terminal() {
    let mut request = request(1, 1_000_000);
    request.poll(SESSION, Tick(10)).unwrap();
    assert_eq!(
        request.poll(SESSION, Tick(9)),
        Err(KeyframeAbort::ClockRegression)
    );
    let mut request =
        super::Mini3AccessoryKeyframe::new(SESSION, 1, Tick(0), Tick(1_000_000)).unwrap();
    let result = request.write_available(SESSION, Tick(10), |_| {
        Ok::<_, Infallible>(KeyframeWriteReceipt {
            accepted: 4,
            completed_at: Tick(9),
        })
    });
    assert!(matches!(
        result,
        Err(KeyframeWriteError::State(KeyframeAbort::ClockRegression))
    ));
    assert_eq!(request.accepted_bytes(), 4);
}

#[test]
fn completed_request_never_replays_after_poll_cancel_close_or_new_session() {
    let mut request = request(1, 1_000_000);
    transmit(&mut request);
    request.cancel();
    request.close();
    assert_eq!(
        request.poll([2; 16], Tick(u64::MAX)),
        Ok(KeyframeState::TransportAccepted(Tick(100_000)))
    );
    assert_eq!(
        request
            .write_available([2; 16], Tick(0), |_| -> Result<_, Infallible> {
                panic!("complete request must not replay")
            })
            .unwrap(),
        0
    );
    assert_eq!(request.accepted_bytes(), 96);
}

#[test]
fn invalid_initial_timing_and_interval_overflow_are_rejected() {
    assert!(matches!(
        Mini3AccessoryKeyframe::new(SESSION, 1, Tick(1), Tick(1)),
        Err(KeyframeAbort::InvalidTiming)
    ));
    let mut request = request(1, u64::MAX);
    let result = request.write_available(SESSION, Tick(u64::MAX - 10), |bytes| {
        Ok::<_, Infallible>(KeyframeWriteReceipt {
            accepted: bytes.len(),
            completed_at: Tick(u64::MAX - 1),
        })
    });
    assert!(matches!(
        result,
        Err(KeyframeWriteError::State(KeyframeAbort::InvalidTiming))
    ));
    assert_eq!(request.accepted_bytes(), 32);
}
