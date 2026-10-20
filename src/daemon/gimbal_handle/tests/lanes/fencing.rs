use super::*;

#[test]
fn stopping_zoom_fences_only_queued_zoom_before_the_device_lock() {
    runtime().block_on(async {
        let (handle, probe) = camera();
        let inner = handle.inner.clone();
        let (locked_tx, locked_rx) = tokio::sync::oneshot::channel();
        let (release_tx, release_rx) = std::sync::mpsc::channel();
        let blocker = std::thread::spawn(move || {
            let guard = inner.lock();
            locked_tx.send(()).unwrap();
            release_rx.recv().unwrap();
            drop(guard);
        });
        locked_rx.await.unwrap();
        let deadline = Instant::now() + Duration::from_secs(10);
        let mut queued_gimbal = Box::pin(handle.execute(pitch_position(), timing(deadline)));
        let mut queued_zoom = Box::pin(handle.execute(zoom_position(), timing(deadline)));
        std::future::poll_fn(|cx| {
            assert!(queued_gimbal.as_mut().poll(cx).is_pending());
            assert!(queued_zoom.as_mut().poll(cx).is_pending());
            Poll::Ready(())
        })
        .await;
        let mut stop = Box::pin(handle.stop_lane(PayloadLane::Zoom, Effect::StopContinuous));
        std::future::poll_fn(|cx| {
            assert!(stop.as_mut().poll(cx).is_pending());
            Poll::Ready(())
        })
        .await;
        release_tx.send(()).unwrap();
        assert!(matches!(
            queued_zoom.await,
            Err(Error::Payload(turret_core::Error::StaleIdentity))
        ));
        queued_gimbal.await.unwrap();
        stop.await.unwrap();
        blocker.join().unwrap();
        let probe = probe.lock();
        assert_eq!(probe.writes, vec![pitch_position()]);
        assert_eq!(
            probe.stops,
            vec![(PayloadLane::Zoom, Effect::StopContinuous)]
        );
    });
}

#[test]
fn partial_zoom_write_failure_preserves_active_gimbal_and_fences_zoom() {
    runtime().block_on(async {
        let (handle, probe) = camera();
        let gimbal = handle
            .execute(
                pitch_rate(),
                continuous_timing(Instant::now() + Duration::from_secs(3)),
            )
            .await
            .unwrap();
        probe.lock().failed_write = Some(PayloadLane::Zoom);
        assert!(handle
            .execute(
                zoom_rate(),
                continuous_timing(Instant::now() + Duration::from_secs(3)),
            )
            .await
            .is_err());
        let failed = handle
            .last_progress_cached(PayloadLane::Zoom)
            .await
            .unwrap()
            .unwrap();
        assert_eq!(failed.ended, Some(EndReason::TransportFailure));
        assert_eq!(
            handle
                .last_progress_cached(PayloadLane::Gimbal)
                .await
                .unwrap(),
            Some(gimbal)
        );
        assert_eq!(
            probe.lock().stops,
            vec![(PayloadLane::Zoom, Effect::StopContinuous)]
        );
        probe.lock().failed_write = None;
        let recovered = handle
            .execute(
                zoom_position(),
                timing(Instant::now() + Duration::from_secs(1)),
            )
            .await
            .unwrap();
        assert_eq!(
            recovered.identity.generation,
            failed.identity.generation.wrapping_add(1)
        );
        handle.revoke().await.unwrap();
    });
}
