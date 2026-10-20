use super::*;
use crate::gimbal::{DeviceReceipt, GimbalDevice};
use turret_core::{
    Angle, Axes, ObservationSource, PayloadCapabilities, PayloadCommand, ReferenceFrame,
};

fn sample(pitch: f32) -> DeviceObservation {
    let now = Instant::now();
    DeviceObservation {
        epoch: 0,
        angles: Axes {
            pitch: Some(Angle::radians(pitch.to_radians()).unwrap()),
            roll: Some(Angle::ZERO),
            yaw: Some(Angle::ZERO),
        },
        frame: ReferenceFrame::Vehicle,
        zoom: None,
        source: ObservationSource::DeviceMeasured,
        sampled_at: Some(now),
        received_at: now,
    }
}

#[test]
fn attitude_channel_publishes_to_multiple_subscribers() {
    let (tx, mut rx_a) = attitude_channel();
    let mut rx_b = tx.subscribe();
    let tick_one = AttitudeTick {
        observation: sample(1.0),
        link_recently_failed: false,
    };
    tx.send(Arc::new(tick_one)).unwrap();
    for received in [rx_a.try_recv().unwrap(), rx_b.try_recv().unwrap()] {
        assert_eq!(
            received.observation.angles.pitch,
            tick_one.observation.angles.pitch
        );
        assert_eq!(
            received.observation.sampled_at,
            tick_one.observation.sampled_at
        );
    }
    let mut rx_c = tx.subscribe();
    assert!(rx_c.try_recv().is_err());
    let tick_two = AttitudeTick {
        observation: sample(4.0),
        link_recently_failed: true,
    };
    tx.send(Arc::new(tick_two)).unwrap();
    for received in [
        rx_a.try_recv().unwrap(),
        rx_b.try_recv().unwrap(),
        rx_c.try_recv().unwrap(),
    ] {
        assert_eq!(
            received.observation.angles.pitch,
            tick_two.observation.angles.pitch
        );
        assert!(received.link_recently_failed);
    }
}

#[test]
fn attitude_channel_with_no_subscribers_reports_closed_delivery() {
    let (tx, rx) = attitude_channel();
    drop(rx);
    let tick = AttitudeTick {
        observation: sample(0.0),
        link_recently_failed: false,
    };
    assert!(tx.send(Arc::new(tick)).is_err());
}

struct SourceDevice(ObservationSource);

impl GimbalDevice for SourceDevice {
    fn protocol_name(&self) -> &'static str {
        "source-test"
    }

    fn capabilities(&self) -> crate::Result<PayloadCapabilities> {
        Ok(crate::protocols::storm32_rc::storm32_capabilities()?)
    }

    fn execute_blocking(
        &mut self,
        _command: PayloadCommand,
        _send_before: Instant,
    ) -> crate::Result<DeviceReceipt> {
        Err(turret_core::Error::Unsupported.into())
    }

    fn observe_blocking(&mut self) -> crate::Result<DeviceObservation> {
        Ok(DeviceObservation {
            source: self.0,
            ..sample(0.0)
        })
    }
}

#[test]
fn yaw_calibration_does_not_promote_estimates_to_physical_measurements() {
    tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .unwrap()
        .block_on(async {
            for source in [
                ObservationSource::DeviceMeasured,
                ObservationSource::TransformedMeasurement,
                ObservationSource::Estimated,
                ObservationSource::OperatorAssumed,
            ] {
                let handle =
                    GimbalHandle::with_yaw_offset_deg(Box::new(SourceDevice(source)), 5.0).unwrap();
                let observation = handle.observe().await.unwrap();
                let expected = if source == ObservationSource::DeviceMeasured {
                    ObservationSource::TransformedMeasurement
                } else {
                    source
                };
                assert_eq!(observation.source, expected);
                assert!(
                    (observation.angles.yaw.unwrap().as_radians().to_degrees() + 5.0).abs() < 0.001
                );
            }
        });
}
