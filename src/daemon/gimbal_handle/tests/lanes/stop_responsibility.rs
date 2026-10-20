use super::*;

#[cfg(test)]
async fn send(handle: &GimbalHandle, command: PayloadCommand) -> Result<CommandProgress> {
    let now = Instant::now();
    handle
        .execute(
            command,
            HostCommandTiming {
                send_before: now + Duration::from_secs(10),
                valid_until: command
                    .is_continuous()
                    .then_some(now + Duration::from_secs(20)),
                ack_before: None,
                completion_before: None,
            },
        )
        .await
}

#[cfg(test)]
fn assert_unconfirmed(
    result: Result<impl std::fmt::Debug>,
    identity: CommandIdentity,
    expected_effect: Effect,
    attempted: bool,
) {
    match result {
        Err(Error::StopUnconfirmed {
            lane,
            effect,
            epoch,
            commands,
            source,
        }) => {
            assert_eq!(lane, PayloadLane::Gimbal);
            assert_eq!(effect, expected_effect);
            assert_eq!(epoch, identity.epoch);
            assert!(commands.contains(&Some(identity)));
            assert_eq!(source.is_some(), attempted);
        }
        other => panic!("expected unresolved stop, got {other:?}"),
    }
}

#[test]
fn failed_stop_blocks_its_lane_without_retrying_during_motion_or_observation() {
    runtime().block_on(async {
        let (handle, probe) = camera();
        let prior = send(&handle, pitch_rate()).await.unwrap();
        probe.lock().failed_stop = Some(PayloadLane::Gimbal);
        assert_unconfirmed(
            handle
                .stop_lane(PayloadLane::Gimbal, Effect::StopContinuous)
                .await,
            prior.identity,
            Effect::StopContinuous,
            true,
        );
        assert_unconfirmed(
            send(&handle, pitch_position()).await,
            prior.identity,
            Effect::StopContinuous,
            false,
        );
        send(&handle, zoom_position()).await.unwrap();
        handle.observe_raw().await.unwrap();
        assert_eq!(probe.lock().writes, vec![pitch_rate(), zoom_position()]);
        assert_eq!(probe.lock().stops.len(), 1);
        assert_unconfirmed(
            handle.poll_execution().await,
            prior.identity,
            Effect::StopContinuous,
            true,
        );
        assert_eq!(probe.lock().stops.len(), 2);
        assert_unconfirmed(
            handle.revoke().await,
            prior.identity,
            Effect::StopContinuous,
            true,
        );
        assert_eq!(probe.lock().stops.len(), 4);
        probe.lock().failed_stop = None;
        handle
            .stop_lane(PayloadLane::Gimbal, Effect::StopContinuous)
            .await
            .unwrap();
        let next = send(&handle, pitch_position()).await.unwrap();
        assert_ne!(next.identity, prior.identity);
        assert_eq!(probe.lock().stops.len(), 5);
    });
}

#[test]
fn partial_write_retains_both_required_effects_until_explicit_success() {
    runtime().block_on(async {
        let (handle, probe) = camera();
        let prior = send(&handle, pitch_rate()).await.unwrap();
        {
            let mut device = probe.lock();
            device.failed_write = Some(PayloadLane::Gimbal);
            device.failed_stop = Some(PayloadLane::Gimbal);
        }
        assert!(send(&handle, pitch_position()).await.is_err());
        let failed = handle
            .last_progress_cached(PayloadLane::Gimbal)
            .await
            .unwrap()
            .unwrap();
        assert_eq!(failed.ended, Some(EndReason::TransportFailure));
        assert_eq!(failed.sent_at, None);
        assert_eq!(
            probe.lock().stops,
            vec![
                (PayloadLane::Gimbal, Effect::StopContinuous),
                (PayloadLane::Gimbal, Effect::CancelAction),
            ]
        );
        assert_unconfirmed(
            send(&handle, pitch_position()).await,
            prior.identity,
            Effect::StopContinuous,
            false,
        );
        send(&handle, zoom_position()).await.unwrap();
        assert_eq!(probe.lock().writes.len(), 3);
        probe.lock().failed_stop = None;
        probe.lock().failed_write = None;
        handle.poll_execution().await.unwrap();
        assert_eq!(probe.lock().stops.len(), 4);
        send(&handle, pitch_position()).await.unwrap();
    });
}

#[test]
fn one_cancel_preserves_both_identities_when_a_replacement_write_is_partial() {
    runtime().block_on(async {
        let (handle, probe) = camera();
        let prior = send(&handle, pitch_position()).await.unwrap();
        probe.lock().failed_write = Some(PayloadLane::Gimbal);
        probe.lock().failed_stop = Some(PayloadLane::Gimbal);
        assert!(send(&handle, pitch_position()).await.is_err());
        let failed = handle
            .last_progress_cached(PayloadLane::Gimbal)
            .await
            .unwrap()
            .unwrap();
        match send(&handle, pitch_position()).await {
            Err(Error::StopUnconfirmed {
                commands, effect, ..
            }) => {
                assert_eq!(effect, Effect::CancelAction);
                assert_eq!(commands, [Some(prior.identity), Some(failed.identity)]);
            }
            other => panic!("expected both partial-write identities, got {other:?}"),
        }
        assert_eq!(
            probe.lock().stops,
            vec![(PayloadLane::Gimbal, Effect::CancelAction)]
        );
    });
}

#[test]
fn unresolved_owner_is_not_replaced_and_successful_explicit_stop_allows_reconnect() {
    runtime().block_on(async {
        let (handle, old) = camera();
        let prior = send(&handle, pitch_position()).await.unwrap();
        old.lock().failed_stop = Some(PayloadLane::Gimbal);
        let (_, replacement) = camera();
        for attempt in 1..=2 {
            assert_unconfirmed(
                handle
                    .replace(Box::new(CameraDevice(replacement.clone())))
                    .await,
                prior.identity,
                Effect::CancelAction,
                true,
            );
            assert_eq!(old.lock().stops.len(), attempt);
            assert!(replacement.lock().stops.is_empty());
            assert!(replacement.lock().writes.is_empty());
            assert_eq!(
                handle.observe_raw().await.unwrap().epoch,
                prior.identity.epoch
            );
        }
        assert_unconfirmed(
            send(&handle, pitch_position()).await,
            prior.identity,
            Effect::CancelAction,
            false,
        );
        send(&handle, zoom_position()).await.unwrap();
        assert_eq!(old.lock().writes, vec![pitch_position(), zoom_position()]);
        old.lock().failed_stop = None;
        handle
            .stop_lane(PayloadLane::Gimbal, Effect::CancelAction)
            .await
            .unwrap();
        let epoch = handle
            .replace(Box::new(CameraDevice(replacement.clone())))
            .await
            .unwrap();
        assert_eq!(epoch, prior.identity.epoch.wrapping_add(1));
        assert!(replacement.lock().stops.is_empty());
        let next = send(&handle, pitch_position()).await.unwrap();
        assert_eq!(next.identity.epoch, epoch);
        assert_eq!(replacement.lock().writes, vec![pitch_position()]);
    });
}

#[test]
fn explicit_stop_attempts_every_lane_and_effect_despite_failure() {
    runtime().block_on(async {
        let (handle, probe) = camera();
        let prior = send(&handle, pitch_rate()).await.unwrap();
        send(&handle, zoom_rate()).await.unwrap();
        probe.lock().failed_stop = Some(PayloadLane::Gimbal);
        assert_unconfirmed(
            handle.stop_all(Effect::CancelAction).await,
            prior.identity,
            Effect::StopContinuous,
            true,
        );
        assert_eq!(
            probe.lock().stops,
            vec![
                (PayloadLane::Gimbal, Effect::StopContinuous),
                (PayloadLane::Gimbal, Effect::CancelAction),
                (PayloadLane::Zoom, Effect::StopContinuous),
                (PayloadLane::Zoom, Effect::CancelAction),
            ]
        );
        assert_unconfirmed(
            handle.poll_execution().await,
            prior.identity,
            Effect::StopContinuous,
            true,
        );
        assert_eq!(probe.lock().stops.len(), 6);
        send(&handle, zoom_position()).await.unwrap();
    });
}

#[test]
fn capability_refresh_keeps_failed_cancel_evidence_and_other_lane_available() {
    runtime().block_on(async {
        let (handle, probe) = camera();
        let prior = send(&handle, pitch_position()).await.unwrap();
        {
            let mut device = probe.lock();
            device.failed_stop = Some(PayloadLane::Gimbal);
            device.capabilities.revision = device.capabilities.revision.wrapping_add(1);
        }
        assert_unconfirmed(
            handle.capabilities().await,
            prior.identity,
            Effect::CancelAction,
            true,
        );
        probe.lock().capabilities.revision = 3;
        handle.capabilities().await.unwrap();
        assert_eq!(probe.lock().stops.len(), 1);
        assert_unconfirmed(
            send(&handle, pitch_position()).await,
            prior.identity,
            Effect::CancelAction,
            false,
        );
        send(&handle, zoom_position()).await.unwrap();
        probe.lock().failed_stop = None;
        handle.poll_execution().await.unwrap();
        send(&handle, pitch_position()).await.unwrap();
    });
}

#[test]
fn device_extensions_cannot_bypass_an_unconfirmed_stop() {
    use crate::protocols::storm32_rc::Storm32Action;

    runtime().block_on(async {
        let (handle, probe) = camera();
        let prior = send(&handle, pitch_position()).await.unwrap();
        probe.lock().failed_stop = Some(PayloadLane::Gimbal);
        assert!(handle.revoke().await.is_err());
        for action in [Storm32Action::Standby(false), Storm32Action::PanMode(1)] {
            assert_unconfirmed(
                handle.execute_storm32_extension(action).await,
                prior.identity,
                Effect::CancelAction,
                false,
            );
        }
        assert_eq!(probe.lock().writes, vec![pitch_position()]);
        assert_eq!(probe.lock().stops.len(), 1);
        probe.lock().failed_stop = None;
        handle.poll_execution().await.unwrap();
        assert!(matches!(
            handle
                .execute_storm32_extension(Storm32Action::Standby(false))
                .await,
            Err(Error::Unsupported {
                op: "STorM32 extension",
                ..
            })
        ));
    });
}

struct LateRejectedDevice {
    device: CameraDevice,
    entered: Option<tokio::sync::oneshot::Sender<()>>,
    release: std::sync::mpsc::Receiver<()>,
}

impl GimbalDevice for LateRejectedDevice {
    fn protocol_name(&self) -> &'static str {
        "late-rejected-receipt"
    }

    fn capabilities(&self) -> Result<PayloadCapabilities> {
        self.device.capabilities()
    }

    fn execute_blocking(
        &mut self,
        command: PayloadCommand,
        before: Instant,
    ) -> Result<DeviceReceipt> {
        if Instant::now() >= before {
            return Err(turret_core::Error::Expired.into());
        }
        self.device.0.lock().writes.push(command);
        self.entered
            .take()
            .ok_or_else(|| Error::InvalidInput("late receipt reused".into()))?
            .send(())
            .map_err(|_| Error::InvalidInput("receipt receiver closed".into()))?;
        self.release
            .recv()
            .map_err(|_| Error::InvalidInput("receipt release closed".into()))?;
        let received = Instant::now();
        Ok(DeviceReceipt {
            sent_at: received,
            acknowledged_at: Some(received),
            acknowledged: Some(false),
        })
    }

    fn stop_blocking(&mut self, lane: PayloadLane, effect: Effect) -> Result<()> {
        self.device.stop_blocking(lane, effect)
    }
}

#[test]
fn a_late_write_keeps_its_negative_ack_even_when_the_required_stop_fails() {
    runtime().block_on(async {
        let (_, probe) = camera();
        probe.lock().failed_stop = Some(PayloadLane::Gimbal);
        let (entered_tx, entered_rx) = tokio::sync::oneshot::channel();
        let (release_tx, release_rx) = std::sync::mpsc::channel();
        let handle = GimbalHandle::new(Box::new(LateRejectedDevice {
            device: CameraDevice(probe.clone()),
            entered: Some(entered_tx),
            release: release_rx,
        }))
        .unwrap();
        let now = Instant::now();
        let validity = now + Duration::from_secs(2);
        let owner = handle.clone();
        let write = tokio::spawn(async move {
            owner
                .execute(
                    pitch_rate(),
                    HostCommandTiming {
                        send_before: now + Duration::from_secs(1),
                        valid_until: Some(validity),
                        ack_before: None,
                        completion_before: None,
                    },
                )
                .await
        });
        tokio::time::timeout(Duration::from_secs(3), entered_rx)
            .await
            .unwrap()
            .unwrap();
        assert!(
            tokio::time::timeout_at(validity.into(), std::future::pending::<()>(),)
                .await
                .is_err()
        );
        release_tx.send(()).unwrap();
        let result = tokio::time::timeout(Duration::from_secs(3), write)
            .await
            .unwrap()
            .unwrap();
        let progress = handle
            .last_progress_cached(PayloadLane::Gimbal)
            .await
            .unwrap()
            .unwrap();
        assert!(progress.sent_at.is_some());
        assert!(progress.rejected_at.is_some());
        assert!(progress.rejected_at >= progress.sent_at);
        assert_eq!(progress.acknowledged_at, None);
        assert_eq!(progress.ended, Some(EndReason::DeviceRejected));
        assert_unconfirmed(result, progress.identity, Effect::StopContinuous, true);
        assert_eq!(
            probe.lock().stops,
            vec![(PayloadLane::Gimbal, Effect::StopContinuous)]
        );
        assert_unconfirmed(
            send(&handle, pitch_position()).await,
            progress.identity,
            Effect::StopContinuous,
            false,
        );
    });
}
