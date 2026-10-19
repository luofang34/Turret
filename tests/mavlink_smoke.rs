//! UDP ingress, partial-axis device execution, and independent physical feedback.

#![cfg(test)]

use mavlink::ardupilotmega::{GimbalManagerFlags, MavMessage, GIMBAL_MANAGER_SET_PITCHYAW_DATA};
use mavlink::{read_v2_msg, write_v2_msg, MavHeader};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use tokio::net::UdpSocket;
use tokio::sync::Notify;
use turret::daemon::config::MavlinkConfig;
use turret::daemon::gimbal_handle::GimbalHandle;
use turret::daemon::mavlink_manager::MavlinkManager;
use turret::daemon::state::StateManager;
use turret::gimbal::{DeviceObservation, DeviceReceipt, GimbalDevice};
use turret::Result;
use turret_core::{Angle, Axes, PayloadCapabilities, PayloadCommand};

struct MockState {
    sent: Option<PayloadCommand>,
    measured: Axes<Option<Angle>>,
}

struct MockGimbal {
    state: Arc<Mutex<MockState>>,
    sent: Arc<Notify>,
}

impl GimbalDevice for MockGimbal {
    fn protocol_name(&self) -> &'static str {
        "mock"
    }
    fn capabilities(&self) -> Result<PayloadCapabilities> {
        Ok(turret::protocols::storm32_rc::storm32_capabilities()?)
    }
    fn execute_blocking(
        &mut self,
        command: PayloadCommand,
        deadline: Instant,
    ) -> Result<DeviceReceipt> {
        assert!(Instant::now() < deadline);
        self.state.lock().unwrap().sent = Some(command);
        self.sent.notify_one();
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
            angles: self.state.lock().unwrap().measured,
            frame: turret_core::ReferenceFrame::Vehicle,
            zoom: None,
            source: turret_core::ObservationSource::DeviceMeasured,
            sampled_at: Some(now),
            received_at: now,
        })
    }
}

fn pitchyaw_frame() -> Vec<u8> {
    let data = GIMBAL_MANAGER_SET_PITCHYAW_DATA {
        flags: GimbalManagerFlags::empty(),
        pitch: 10_f32.to_radians(),
        yaw: 20_f32.to_radians(),
        pitch_rate: f32::NAN,
        yaw_rate: f32::NAN,
        target_system: 1,
        target_component: 154,
        gimbal_device_id: 0,
    };
    let mut bytes = Vec::new();
    write_v2_msg(
        &mut bytes,
        MavHeader {
            system_id: 255,
            component_id: 1,
            sequence: 0,
        },
        &MavMessage::GIMBAL_MANAGER_SET_PITCHYAW(data),
    )
    .unwrap();
    bytes
}

#[test]
fn partial_command_and_independent_measurement_cross_udp_boundary() {
    tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .unwrap()
        .block_on(async {
            let socket = Arc::new(UdpSocket::bind("127.0.0.1:0").await.unwrap());
            let gcs = UdpSocket::bind("127.0.0.1:0").await.unwrap();
            gcs.connect(socket.local_addr().unwrap()).await.unwrap();
            let state = Arc::new(Mutex::new(MockState {
                sent: None,
                measured: Axes {
                    pitch: None,
                    roll: None,
                    yaw: None,
                },
            }));
            let sent = Arc::new(Notify::new());
            let handle = GimbalHandle::new(Box::new(MockGimbal {
                state: state.clone(),
                sent: sent.clone(),
            }))
            .unwrap();
            let (tx, rx) = turret::daemon::attitude::attitude_channel();
            let manager = MavlinkManager::new(StateManager::new(), handle.clone());
            let config = MavlinkConfig {
                sysid: 1,
                compid: 154,
                ..turret::daemon::config::DaemonConfig::default().mavlink
            };
            let task = tokio::spawn(manager.start_on_socket(config, rx, socket));
            gcs.send(&pitchyaw_frame()).await.unwrap();
            tokio::time::timeout(Duration::from_secs(2), sent.notified())
                .await
                .unwrap();
            let payload = state.lock().unwrap().sent.unwrap();
            let PayloadCommand::GimbalPosition(target) = payload else {
                panic!("unexpected command")
            };
            assert!((target.angles.pitch.unwrap().as_radians().to_degrees() - 10.0).abs() < 0.001);
            assert!((target.angles.yaw.unwrap().as_radians().to_degrees() - 20.0).abs() < 0.001);
            assert_eq!(target.angles.roll, None);
            assert!(handle.observe().await.unwrap().complete_attitude().is_err());
            state.lock().unwrap().measured = Axes {
                pitch: Some(Angle::radians(3_f32.to_radians()).unwrap()),
                roll: Some(Angle::ZERO),
                yaw: Some(Angle::ZERO),
            };
            let measured = handle.observe().await.unwrap().complete_attitude().unwrap();
            tx.send(Arc::new(turret::daemon::attitude::AttitudeTick {
                attitude: measured,
                link_recently_failed: false,
            }))
            .unwrap();
            let attitude = tokio::time::timeout(Duration::from_secs(2), receive_attitude(&gcs))
                .await
                .unwrap();
            let pitch = (2.0 * (attitude[0] * attitude[2] - attitude[3] * attitude[1]))
                .asin()
                .to_degrees();
            assert!((pitch - 3.0).abs() < 0.001);
            drop(tx);
            tokio::time::timeout(Duration::from_secs(2), task)
                .await
                .unwrap()
                .unwrap()
                .unwrap();
        });
}

async fn receive_attitude(socket: &UdpSocket) -> [f32; 4] {
    let mut bytes = [0; 1024];
    loop {
        let count = socket.recv(&mut bytes).await.unwrap();
        let mut reader = mavlink::peek_reader::PeekReader::new(&bytes[..count]);
        if let Ok((_, MavMessage::GIMBAL_DEVICE_ATTITUDE_STATUS(data))) =
            read_v2_msg::<MavMessage, _>(&mut reader)
        {
            return data.q;
        }
    }
}
