use super::*;
use crate::gimbal::DeviceObservation;
use turret_core::{Angle, Axes, ObservationSource, ReferenceFrame};

mod measurements;

fn sample(now: Instant) -> DeviceObservation {
    DeviceObservation {
        epoch: 0,
        angles: Axes {
            pitch: Some(Angle::ZERO),
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
fn lagging_broadcast_receiver_counts_actual_loss() {
    tokio::runtime::Builder::new_current_thread()
        .build()
        .unwrap()
        .block_on(async {
            let (sender, mut receiver) = broadcast::channel(1);
            let tick = Arc::new(AttitudeTick {
                observation: sample(Instant::now()),
                link_recently_failed: false,
            });
            for _ in 0..3 {
                sender.send(tick.clone()).unwrap();
            }
            let mut dropped = 0;
            assert_eq!(
                receive_attitude(&mut receiver, &mut dropped)
                    .await
                    .unwrap()
                    .observation
                    .angles
                    .pitch,
                Some(Angle::ZERO)
            );
            assert_eq!(dropped, 2);
            drop(sender);
            assert!(receive_attitude(&mut receiver, &mut dropped)
                .await
                .is_none());
        });
}
