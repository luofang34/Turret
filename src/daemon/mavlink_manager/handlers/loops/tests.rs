use super::*;
use crate::gimbal::Attitude;

#[test]
fn lagging_broadcast_receiver_counts_actual_loss() {
    tokio::runtime::Builder::new_current_thread()
        .build()
        .unwrap()
        .block_on(async {
            let (sender, mut receiver) = broadcast::channel(1);
            let tick = Arc::new(AttitudeTick {
                attitude: Attitude {
                    pitch: 1.0,
                    roll: 2.0,
                    yaw: 3.0,
                },
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
                    .attitude
                    .pitch,
                1.0
            );
            assert_eq!(dropped, 2);
            drop(sender);
            assert!(receive_attitude(&mut receiver, &mut dropped)
                .await
                .is_none());
        });
}
