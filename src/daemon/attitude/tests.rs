use super::*;

#[test]
fn attitude_channel_publishes_to_multiple_subscribers() {
    // The channel must support fan-out: MAVLink today, plus
    // future protocol broadcasters (CCSDS, custom mission-bus)
    // tomorrow. Verify two independent receivers each see the
    // same tick, and that a third late-subscribe sees only
    // future ticks (broadcast semantics).
    let (tx, rx_a) = attitude_channel();
    let mut rx_a = rx_a;
    let mut rx_b = tx.subscribe();

    let tick_one = AttitudeTick {
        attitude: Attitude {
            pitch: 1.0,
            roll: 2.0,
            yaw: 3.0,
        },
        link_recently_failed: false,
    };
    tx.send(Arc::new(tick_one)).unwrap();

    let received_a = rx_a.try_recv().unwrap();
    let received_b = rx_b.try_recv().unwrap();
    assert_eq!(received_a.attitude.pitch, 1.0);
    assert_eq!(received_b.attitude.pitch, 1.0);

    // Late subscriber misses `tick_one` (broadcast = "from now on").
    let mut rx_c = tx.subscribe();
    assert!(rx_c.try_recv().is_err());

    let tick_two = AttitudeTick {
        attitude: Attitude {
            pitch: 4.0,
            roll: 5.0,
            yaw: 6.0,
        },
        link_recently_failed: true,
    };
    tx.send(Arc::new(tick_two)).unwrap();
    assert_eq!(rx_a.try_recv().unwrap().attitude.pitch, 4.0);
    assert_eq!(rx_b.try_recv().unwrap().attitude.pitch, 4.0);
    let third = rx_c.try_recv().unwrap();
    assert_eq!(third.attitude.pitch, 4.0);
    assert!(third.link_recently_failed);
}

#[test]
fn attitude_channel_send_with_no_subscribers_does_not_block_or_error_silently() {
    // The poll loop drops the initial receiver after splitting the
    // tx; subsequent sends with zero subscribers must return Err
    // (broadcast semantics) without blocking. The poll loop
    // discards the result, so this is just confirming the
    // contract we rely on.
    let (tx, rx) = attitude_channel();
    drop(rx);
    let tick = AttitudeTick {
        attitude: Attitude {
            pitch: 0.0,
            roll: 0.0,
            yaw: 0.0,
        },
        link_recently_failed: false,
    };
    // Err on no-subscribers, but no panic — exactly what the poll
    // loop's `let _ = tx.send(...)` relies on.
    assert!(tx.send(Arc::new(tick)).is_err());
}
