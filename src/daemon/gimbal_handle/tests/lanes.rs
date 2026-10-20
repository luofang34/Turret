use super::*;
use crate::gimbal::PayloadLane;

mod fencing;
mod stop_responsibility;

struct Probe {
    capabilities: PayloadCapabilities,
    writes: Vec<PayloadCommand>,
    stops: Vec<(PayloadLane, Effect)>,
    pitch: Angle,
    zoom: Option<ZoomAbsolute>,
    failed_stop: Option<PayloadLane>,
    failed_write: Option<PayloadLane>,
}

struct CameraDevice(Arc<Mutex<Probe>>);

impl GimbalDevice for CameraDevice {
    fn protocol_name(&self) -> &'static str {
        "independent-lanes-test"
    }
    fn capabilities(&self) -> Result<PayloadCapabilities> {
        Ok(self.0.lock().capabilities)
    }
    fn execute_blocking(
        &mut self,
        command: PayloadCommand,
        before: Instant,
    ) -> Result<DeviceReceipt> {
        if Instant::now() >= before {
            return Err(turret_core::Error::Expired.into());
        }
        let mut probe = self.0.lock();
        probe.writes.push(command);
        if probe.failed_write == Some(PayloadLane::for_command(command)) {
            return Err(Error::MissingMeasurement {
                field: "test write receipt",
            });
        }
        Ok(DeviceReceipt {
            sent_at: Instant::now(),
            acknowledged: None,
            acknowledged_at: None,
        })
    }
    fn observe_blocking(&mut self) -> Result<DeviceObservation> {
        let probe = self.0.lock();
        let now = Instant::now();
        Ok(DeviceObservation {
            epoch: 0,
            angles: Axes {
                pitch: Some(probe.pitch),
                roll: None,
                yaw: None,
            },
            zoom: probe.zoom,
            frame: ReferenceFrame::Vehicle,
            source: ObservationSource::DeviceMeasured,
            sampled_at: Some(now),
            received_at: now,
        })
    }
    fn stop_blocking(&mut self, lane: PayloadLane, effect: Effect) -> Result<()> {
        let mut probe = self.0.lock();
        probe.stops.push((lane, effect));
        if probe.failed_stop == Some(lane) {
            return Err(Error::MissingMeasurement {
                field: "test stop receipt",
            });
        }
        Ok(())
    }
}

#[cfg(test)]
fn camera() -> (GimbalHandle, Arc<Mutex<Probe>>) {
    let range = ZoomRange {
        minimum: ZoomAbsolute::new(ZoomUnit::Ratio, 1.0).unwrap(),
        maximum: ZoomAbsolute::new(ZoomUnit::Ratio, 5.0).unwrap(),
        implementation: Implementation::Native,
    };
    let absolute = AbsoluteZoomCapabilities {
        ratio: Capability::Available(range),
        travel_fraction: Capability::Unsupported,
        focal_length_mm: Capability::Unsupported,
        horizontal_fov_radians: Capability::Unsupported,
    };
    let mut capabilities = storm32_capabilities().unwrap();
    capabilities.zoom = Capability::Available(ZoomCapabilities {
        absolute,
        feedback: absolute,
        step: Capability::Unsupported,
        continuous: Capability::Available(Implementation::Native),
        stop: StopGuarantee::BestEffort,
    });
    let probe = Arc::new(Mutex::new(Probe {
        capabilities,
        writes: Vec::new(),
        stops: Vec::new(),
        pitch: Angle::ZERO,
        zoom: Some(ZoomAbsolute::new(ZoomUnit::Ratio, 2.0).unwrap()),
        failed_stop: None,
        failed_write: None,
    }));
    let handle = GimbalHandle::new(Box::new(CameraDevice(probe.clone()))).unwrap();
    (handle, probe)
}

#[cfg(test)]
fn zoom_position() -> PayloadCommand {
    PayloadCommand::Zoom(ZoomCommand::Absolute(
        ZoomAbsolute::new(ZoomUnit::Ratio, 2.0).unwrap(),
    ))
}

#[cfg(test)]
fn zoom_rate() -> PayloadCommand {
    PayloadCommand::Zoom(ZoomCommand::Continuous(
        NormalizedZoomRate::new(0.4).unwrap(),
    ))
}

#[cfg(test)]
fn pitch_rate() -> PayloadCommand {
    PayloadCommand::GimbalRate(GimbalRate {
        rates: Axes {
            pitch: Some(AngularRate::radians_per_second(0.1).unwrap()),
            roll: None,
            yaw: None,
        },
        frame: ReferenceFrame::Vehicle,
    })
}

#[cfg(test)]
fn continuous_timing(deadline: Instant) -> HostCommandTiming {
    HostCommandTiming {
        send_before: Instant::now() + Duration::from_millis(100),
        valid_until: Some(deadline),
        ack_before: None,
        completion_before: None,
    }
}

#[test]
fn a_zoom_write_and_replacement_preserve_the_active_gimbal() {
    runtime().block_on(async {
        let (handle, probe) = camera();
        let gimbal = handle
            .execute(
                pitch_rate(),
                continuous_timing(Instant::now() + Duration::from_secs(3)),
            )
            .await
            .unwrap();
        handle
            .execute(
                zoom_rate(),
                continuous_timing(Instant::now() + Duration::from_secs(3)),
            )
            .await
            .unwrap();
        handle
            .execute(
                zoom_position(),
                timing(Instant::now() + Duration::from_secs(1)),
            )
            .await
            .unwrap();
        assert_eq!(
            handle
                .last_progress_cached(PayloadLane::Gimbal)
                .await
                .unwrap(),
            Some(gimbal)
        );
        assert!(handle
            .last_progress_cached(PayloadLane::Zoom)
            .await
            .unwrap()
            .unwrap()
            .ended
            .is_none());
        assert!(probe.lock().stops.is_empty());
        handle.revoke().await.unwrap();
    });
}

#[test]
fn zoom_expiry_stops_only_zoom_while_gimbal_keeps_its_evidence() {
    runtime().block_on(async {
        let (handle, probe) = camera();
        let gimbal = handle
            .execute(
                pitch_rate(),
                continuous_timing(Instant::now() + Duration::from_secs(3)),
            )
            .await
            .unwrap();
        let deadline = Instant::now() + Duration::from_millis(200);
        handle
            .execute(zoom_rate(), continuous_timing(deadline))
            .await
            .unwrap();
        // This timer exercises the actual expiration boundary, without polling for state.
        tokio::time::sleep_until(tokio::time::Instant::from_std(deadline)).await;
        handle.poll_execution().await.unwrap();
        assert_eq!(
            probe.lock().stops,
            vec![(PayloadLane::Zoom, Effect::StopContinuous)]
        );
        assert_eq!(
            handle
                .last_progress_cached(PayloadLane::Zoom)
                .await
                .unwrap()
                .unwrap()
                .ended,
            Some(EndReason::ValidityExpired)
        );
        assert_eq!(
            handle
                .last_progress_cached(PayloadLane::Gimbal)
                .await
                .unwrap(),
            Some(gimbal)
        );
        handle.revoke().await.unwrap();
    });
}

#[test]
fn measured_zoom_can_complete_while_the_gimbal_target_remains_unreached() {
    runtime().block_on(async {
        let (handle, probe) = camera();
        probe.lock().pitch = Angle::radians(0.2).unwrap();
        handle
            .execute(
                pitch_position(),
                timing(Instant::now() + Duration::from_secs(1)),
            )
            .await
            .unwrap();
        handle
            .execute(
                zoom_position(),
                timing(Instant::now() + Duration::from_secs(1)),
            )
            .await
            .unwrap();
        handle.observe_raw().await.unwrap();
        let settled = Instant::now() + Duration::from_millis(310);
        tokio::time::sleep_until(tokio::time::Instant::from_std(settled)).await;
        handle.observe_raw().await.unwrap();
        let zoom = handle
            .last_progress_cached(PayloadLane::Zoom)
            .await
            .unwrap()
            .unwrap();
        assert_eq!(zoom.ended, Some(EndReason::ObservedComplete));
        assert!(zoom.observed_complete_at.is_some());
        assert!(handle
            .last_progress_cached(PayloadLane::Gimbal)
            .await
            .unwrap()
            .unwrap()
            .ended
            .is_none());
        assert!(probe.lock().stops.is_empty());
        handle.revoke().await.unwrap();
    });
}

#[test]
fn stopping_one_lane_preserves_the_others_generation_and_active_command() {
    runtime().block_on(async {
        let (handle, probe) = camera();
        let gimbal = handle
            .execute(
                pitch_rate(),
                continuous_timing(Instant::now() + Duration::from_secs(3)),
            )
            .await
            .unwrap();
        let zoom = handle
            .execute(
                zoom_rate(),
                continuous_timing(Instant::now() + Duration::from_secs(3)),
            )
            .await
            .unwrap();
        handle
            .stop_lane(PayloadLane::Zoom, Effect::StopContinuous)
            .await
            .unwrap();
        assert_eq!(
            probe.lock().stops,
            vec![(PayloadLane::Zoom, Effect::StopContinuous)]
        );
        assert_eq!(
            handle
                .last_progress_cached(PayloadLane::Gimbal)
                .await
                .unwrap(),
            Some(gimbal)
        );
        assert_eq!(
            handle
                .last_progress_cached(PayloadLane::Zoom)
                .await
                .unwrap()
                .unwrap()
                .ended,
            Some(EndReason::Revoked)
        );
        let next = handle
            .execute(
                zoom_position(),
                timing(Instant::now() + Duration::from_secs(1)),
            )
            .await
            .unwrap();
        assert_eq!(
            next.identity.generation,
            zoom.identity.generation.wrapping_add(1)
        );
        handle.revoke().await.unwrap();
    });
}

#[test]
fn devicewide_revoke_attempts_both_lanes_even_when_the_first_stop_fails() {
    runtime().block_on(async {
        let (handle, probe) = camera();
        handle
            .execute(
                pitch_rate(),
                continuous_timing(Instant::now() + Duration::from_secs(3)),
            )
            .await
            .unwrap();
        handle
            .execute(
                zoom_rate(),
                continuous_timing(Instant::now() + Duration::from_secs(3)),
            )
            .await
            .unwrap();
        probe.lock().failed_stop = Some(PayloadLane::Gimbal);
        assert!(handle.revoke().await.is_err());
        assert_eq!(
            probe.lock().stops,
            vec![
                (PayloadLane::Gimbal, Effect::StopContinuous),
                (PayloadLane::Zoom, Effect::StopContinuous)
            ]
        );
        for lane in PayloadLane::ALL {
            assert_eq!(
                handle
                    .last_progress_cached(lane)
                    .await
                    .unwrap()
                    .unwrap()
                    .ended,
                Some(EndReason::Revoked)
            );
        }
    });
}

#[test]
fn capability_changes_and_reconnection_preserve_both_lanes_evidence() {
    runtime().block_on(async {
        let (handle, probe) = camera();
        let gimbal = handle
            .execute(
                pitch_position(),
                timing(Instant::now() + Duration::from_secs(1)),
            )
            .await
            .unwrap();
        let zoom = handle
            .execute(
                zoom_position(),
                timing(Instant::now() + Duration::from_secs(1)),
            )
            .await
            .unwrap();
        {
            let mut state = probe.lock();
            state.capabilities.revision = state.capabilities.revision.wrapping_add(1);
        }
        handle.capabilities().await.unwrap();
        for (lane, prior) in [(PayloadLane::Gimbal, gimbal), (PayloadLane::Zoom, zoom)] {
            assert_retained(&handle, lane, prior, EndReason::Revoked).await;
        }
        assert_eq!(
            probe.lock().stops,
            vec![
                (PayloadLane::Gimbal, Effect::CancelAction),
                (PayloadLane::Zoom, Effect::CancelAction)
            ]
        );
        let next_gimbal = handle
            .execute(
                pitch_position(),
                timing(Instant::now() + Duration::from_secs(1)),
            )
            .await
            .unwrap();
        let next_zoom = handle
            .execute(
                zoom_position(),
                timing(Instant::now() + Duration::from_secs(1)),
            )
            .await
            .unwrap();
        assert_eq!(
            next_gimbal.identity.generation,
            gimbal.identity.generation.wrapping_add(1)
        );
        assert_eq!(
            next_zoom.identity.generation,
            zoom.identity.generation.wrapping_add(1)
        );
        let (_, replacement) = camera();
        assert_eq!(
            handle
                .replace(Box::new(CameraDevice(replacement)))
                .await
                .unwrap(),
            1
        );
        for (lane, prior) in [
            (PayloadLane::Gimbal, next_gimbal),
            (PayloadLane::Zoom, next_zoom),
        ] {
            assert_retained(&handle, lane, prior, EndReason::Revoked).await;
        }
        assert_eq!(handle.observe_raw().await.unwrap().epoch, 1);
    });
}

#[cfg(test)]
async fn assert_retained(
    handle: &GimbalHandle,
    lane: PayloadLane,
    prior: CommandProgress,
    reason: EndReason,
) {
    let retained = handle.last_progress_cached(lane).await.unwrap().unwrap();
    assert_eq!(retained.identity, prior.identity);
    assert_eq!(retained.sent_at, prior.sent_at);
    assert_eq!(retained.ended, Some(reason));
}
