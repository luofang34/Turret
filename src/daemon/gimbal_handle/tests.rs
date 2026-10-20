use super::*;
use crate::error::Error;
use crate::gimbal::DeviceReceipt;
use crate::protocols::storm32_rc::storm32_capabilities;
use std::future::Future;
use std::task::Poll;
use std::time::Duration;
use turret_core::*;

struct FakeDevice {
    writes: Arc<Mutex<Vec<PayloadCommand>>>,
    angles: Axes<Option<Angle>>,
    pitch_only: bool,
}

impl GimbalDevice for FakeDevice {
    fn protocol_name(&self) -> &'static str {
        "fake"
    }
    fn capabilities(&self) -> Result<PayloadCapabilities> {
        let mut capabilities = storm32_capabilities()?;
        if self.pitch_only {
            capabilities.gimbal.axes.yaw = AxisCapabilities::UNSUPPORTED;
            capabilities.gimbal.axes.roll = AxisCapabilities::UNSUPPORTED;
            capabilities.gimbal.recenter = Capability::Unsupported;
        }
        Ok(capabilities)
    }
    fn execute_blocking(
        &mut self,
        command: PayloadCommand,
        deadline: Instant,
    ) -> Result<DeviceReceipt> {
        if Instant::now() >= deadline {
            return Err(turret_core::Error::Expired.into());
        }
        self.writes.lock().push(command);
        Ok(DeviceReceipt {
            sent_at: Instant::now(),
            acknowledged_at: None,
            acknowledged: None,
        })
    }
    fn observe_blocking(&mut self) -> Result<DeviceObservation> {
        let now = Instant::now();
        Ok(DeviceObservation {
            epoch: 0,
            angles: self.angles,
            frame: ReferenceFrame::Vehicle,
            zoom: None,
            source: ObservationSource::DeviceMeasured,
            sampled_at: Some(now),
            received_at: now,
        })
    }
    fn stop_blocking(&mut self, _lane: crate::gimbal::PayloadLane, _effect: Effect) -> Result<()> {
        Ok(())
    }
}

#[cfg(test)]
fn fixture(pitch_only: bool) -> Result<(GimbalHandle, Arc<Mutex<Vec<PayloadCommand>>>)> {
    let writes = Arc::new(Mutex::new(Vec::new()));
    let device = FakeDevice {
        writes: writes.clone(),
        pitch_only,
        angles: Axes {
            pitch: Some(Angle::ZERO),
            roll: None,
            yaw: Some(Angle::radians(0.5)?),
        },
    };
    Ok((GimbalHandle::new(Box::new(device))?, writes))
}

#[cfg(test)]
fn runtime() -> tokio::runtime::Runtime {
    tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .unwrap()
}

#[cfg(test)]
fn pitch_position() -> PayloadCommand {
    PayloadCommand::GimbalPosition(GimbalPosition {
        angles: Axes {
            pitch: Some(Angle::ZERO),
            roll: None,
            yaw: None,
        },
        frame: ReferenceFrame::Vehicle,
    })
}

#[test]
fn a_complete_write_does_not_claim_ack_or_physical_completion() {
    runtime().block_on(async {
        let (handle, writes) = fixture(true).unwrap();
        let progress = handle
            .execute(
                pitch_position(),
                timing(Instant::now() + Duration::from_secs(1)),
            )
            .await
            .unwrap();
        assert!(progress.sent_at.is_some());
        assert_eq!(progress.acknowledged_at, None);
        assert_eq!(progress.observed_complete_at, None);
        assert_eq!(writes.lock().as_slice(), &[pitch_position()]);
    });
}

#[test]
fn unsupported_yaw_is_rejected_before_device_write() {
    runtime().block_on(async {
        let (handle, writes) = fixture(true).unwrap();
        let yaw = PayloadCommand::GimbalPosition(GimbalPosition {
            angles: Axes {
                pitch: None,
                roll: None,
                yaw: Some(Angle::ZERO),
            },
            frame: ReferenceFrame::Vehicle,
        });
        assert!(matches!(
            handle
                .execute(yaw, timing(Instant::now() + Duration::from_secs(1)))
                .await,
            Err(Error::Payload(turret_core::Error::Unsupported))
        ));
        assert!(writes.lock().is_empty());
    });
}

#[test]
fn expired_ingress_deadline_prevents_any_write() {
    runtime().block_on(async {
        let (handle, writes) = fixture(true).unwrap();
        assert!(matches!(
            handle
                .execute(
                    pitch_position(),
                    timing(Instant::now() - Duration::from_millis(1))
                )
                .await,
            Err(Error::Payload(turret_core::Error::Expired))
        ));
        assert!(writes.lock().is_empty());
    });
}

#[test]
fn partial_observation_preserves_unknown_axes_and_read_side_calibration() {
    runtime().block_on(async {
        let (handle, _) = fixture(false).unwrap();
        handle.set_yaw_offset_deg(5.0);
        let observed = handle.observe().await.unwrap();
        assert_eq!(observed.angles.roll, None);
        assert!(
            (observed.angles.yaw.unwrap().as_radians() - (0.5 - 5.0_f32.to_radians())).abs()
                < 0.0001
        );
        assert!(observed.complete_attitude().is_err());
        let raw = handle.observe_raw().await.unwrap();
        assert_eq!(raw.angles.yaw.unwrap().as_radians(), 0.5);
    });
}

#[test]
fn revoke_fences_a_command_waiting_for_the_device_lock() {
    runtime().block_on(async {
        let (handle, writes) = fixture(true).unwrap();
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
        let mut queued = Box::pin(handle.execute(
            pitch_position(),
            timing(Instant::now() + Duration::from_secs(10)),
        ));
        std::future::poll_fn(|cx| {
            assert!(queued.as_mut().poll(cx).is_pending());
            Poll::Ready(())
        })
        .await;
        let mut revoke = Box::pin(handle.revoke());
        std::future::poll_fn(|cx| {
            assert!(revoke.as_mut().poll(cx).is_pending());
            Poll::Ready(())
        })
        .await;
        release_tx.send(()).unwrap();
        assert!(matches!(
            queued.await,
            Err(Error::Payload(turret_core::Error::StaleIdentity))
        ));
        revoke.await.unwrap();
        blocker.join().unwrap();
        assert!(writes.lock().is_empty());
    });
}

#[test]
fn reconnection_changes_observation_epoch() {
    runtime().block_on(async {
        let (handle, writes) = fixture(true).unwrap();
        assert_eq!(handle.observe().await.unwrap().epoch, 0);
        handle
            .replace(Box::new(FakeDevice {
                writes,
                pitch_only: true,
                angles: Axes {
                    pitch: Some(Angle::ZERO),
                    roll: None,
                    yaw: None,
                },
            }))
            .await
            .unwrap();
        assert_eq!(handle.observe().await.unwrap().epoch, 1);
    });
}

#[cfg(test)]
fn timing(send_before: Instant) -> HostCommandTiming {
    HostCommandTiming {
        send_before,
        valid_until: None,
        ack_before: None,
        completion_before: None,
    }
}

mod lanes;
