//! Device receipt evidence survives real MAVLink command and acknowledgement framing.

#![cfg(test)]
#![cfg(feature = "daemon")]

use mavlink::ardupilotmega::{MavCmd, MavMessage, MavResult, COMMAND_LONG_DATA};
use mavlink::{read_v2_msg, write_v2_msg, MavHeader};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use tokio::net::UdpSocket;
use tokio::sync::broadcast;
use tokio::task::JoinHandle;
use turret::daemon::attitude::{attitude_channel, AttitudeTick};
use turret::daemon::config::{DaemonConfig, MavlinkConfig};
use turret::daemon::gimbal_handle::GimbalHandle;
use turret::daemon::mavlink_manager::MavlinkManager;
use turret::daemon::state::StateManager;
use turret::gimbal::{DeviceReceipt, GimbalDevice, PayloadLane};
use turret::Result;
use turret_core::{CommandProgress, Effect, EndReason, PayloadCapabilities, PayloadCommand};

#[derive(Default)]
struct DeviceCalls {
    writes: Vec<PayloadCommand>,
    stops: Vec<(PayloadLane, Effect)>,
}

struct MockGimbal {
    acknowledgement: Option<bool>,
    calls: Arc<Mutex<DeviceCalls>>,
}

impl GimbalDevice for MockGimbal {
    fn protocol_name(&self) -> &'static str {
        "receipt-fixture"
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
        self.calls.lock().unwrap().writes.push(command);
        let sent_at = Instant::now();
        Ok(DeviceReceipt {
            sent_at,
            acknowledged_at: self.acknowledgement.map(|_| Instant::now()),
            acknowledged: self.acknowledgement,
        })
    }

    fn stop_blocking(&mut self, lane: PayloadLane, effect: Effect) -> Result<()> {
        self.calls.lock().unwrap().stops.push((lane, effect));
        Ok(())
    }
}

struct Session {
    socket: UdpSocket,
    handle: GimbalHandle,
    calls: Arc<Mutex<DeviceCalls>>,
    attitude: Option<broadcast::Sender<Arc<AttitudeTick>>>,
    manager: Option<JoinHandle<Result<()>>>,
}

impl Session {
    async fn start(acknowledgement: Option<bool>) -> Self {
        let manager_socket = Arc::new(UdpSocket::bind("127.0.0.1:0").await.unwrap());
        let socket = UdpSocket::bind("127.0.0.1:0").await.unwrap();
        socket
            .connect(manager_socket.local_addr().unwrap())
            .await
            .unwrap();
        let calls = Arc::new(Mutex::new(DeviceCalls::default()));
        let handle = GimbalHandle::new(Box::new(MockGimbal {
            acknowledgement,
            calls: calls.clone(),
        }))
        .unwrap();
        let (attitude, receiver) = attitude_channel();
        let manager = MavlinkManager::new(StateManager::new(), handle.clone());
        let config = MavlinkConfig {
            sysid: 1,
            compid: 154,
            ..DaemonConfig::default().mavlink
        };
        Self {
            socket,
            handle,
            calls,
            attitude: Some(attitude),
            manager: Some(tokio::spawn(manager.start_on_socket(
                config,
                receiver,
                manager_socket,
            ))),
        }
    }

    async fn dispatch(&self, rate: bool) -> (MavHeader, MavResult, CommandProgress) {
        self.socket.send(&command_frame(rate)).await.unwrap();
        let (header, result) =
            tokio::time::timeout(Duration::from_secs(2), receive_ack(&self.socket))
                .await
                .unwrap();
        let progress = self
            .handle
            .last_progress_cached(PayloadLane::Gimbal)
            .await
            .unwrap()
            .unwrap();
        (header, result, progress)
    }

    async fn shutdown(&mut self) {
        drop(self.attitude.take());
        let Some(mut task) = self.manager.take() else {
            panic!("manager already stopped");
        };
        match tokio::time::timeout(Duration::from_secs(2), &mut task).await {
            Ok(result) => result.unwrap().unwrap(),
            Err(_) => {
                task.abort();
                task.await.ok();
                panic!("MAVLink manager did not stop after its attitude channel closed");
            }
        }
    }
}

impl Drop for Session {
    fn drop(&mut self) {
        if let Some(task) = self.manager.take() {
            task.abort();
        }
    }
}

fn command_frame(rate: bool) -> Vec<u8> {
    let data = COMMAND_LONG_DATA {
        target_system: 1,
        target_component: 154,
        command: MavCmd::MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
        confirmation: 0,
        param1: if rate { f32::NAN } else { 10.0 },
        param2: f32::NAN,
        param3: if rate { 45.0 } else { f32::NAN },
        param4: f32::NAN,
        param5: 0.0,
        param6: f32::NAN,
        param7: 154.0,
    };
    let mut bytes = Vec::new();
    write_v2_msg(
        &mut bytes,
        MavHeader {
            system_id: 255,
            component_id: 1,
            sequence: 0,
        },
        &MavMessage::COMMAND_LONG(data),
    )
    .unwrap();
    bytes
}

async fn receive_ack(socket: &UdpSocket) -> (MavHeader, MavResult) {
    let mut bytes = [0; 1024];
    loop {
        let count = socket.recv(&mut bytes).await.unwrap();
        let mut reader = mavlink::peek_reader::PeekReader::new(&bytes[..count]);
        if let Ok((header, MavMessage::COMMAND_ACK(ack))) =
            read_v2_msg::<MavMessage, _>(&mut reader)
        {
            assert_eq!(ack.command, MavCmd::MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW);
            return (header, ack.result);
        }
    }
}

fn check_payload(command: PayloadCommand, rate: bool) {
    if rate {
        let PayloadCommand::GimbalRate(target) = command else {
            panic!("rate command lost its operation");
        };
        let pitch = target.rates.pitch.unwrap().as_radians_per_second();
        assert!((pitch.to_degrees() - 45.0).abs() < 0.001);
        assert!(target.rates.roll.is_none());
        assert!(target.rates.yaw.is_none());
    } else {
        let PayloadCommand::GimbalPosition(target) = command else {
            panic!("position command lost its operation");
        };
        let pitch = target.angles.pitch.unwrap().as_radians();
        assert!((pitch.to_degrees() - 10.0).abs() < 0.001);
        assert!(target.angles.roll.is_none());
        assert!(target.angles.yaw.is_none());
    }
}

async fn check_receipt(rate: bool, acknowledgement: Option<bool>) {
    let mut session = Session::start(acknowledgement).await;
    let (header, result, progress) = session.dispatch(rate).await;
    session.shutdown().await;
    assert_eq!((header.system_id, header.component_id), (1, 154));
    assert_eq!(
        result,
        if acknowledgement == Some(false) {
            MavResult::MAV_RESULT_FAILED
        } else {
            MavResult::MAV_RESULT_ACCEPTED
        }
    );
    assert!(progress.sent_at.is_some());
    assert_eq!(
        progress.acknowledged_at.is_some(),
        acknowledgement == Some(true)
    );
    assert_eq!(
        progress.rejected_at.is_some(),
        acknowledgement == Some(false)
    );
    assert!(!progress.acknowledgement_timed_out);
    assert!(progress.observed_complete_at.is_none());
    assert_eq!(
        progress.ended,
        (acknowledgement == Some(false)).then_some(EndReason::DeviceRejected)
    );
    let calls = session.calls.lock().unwrap();
    assert_eq!(calls.writes.len(), 1);
    check_payload(calls.writes[0], rate);
    let expected_stops = if acknowledgement == Some(false) {
        vec![(
            PayloadLane::Gimbal,
            if rate {
                Effect::StopContinuous
            } else {
                Effect::CancelAction
            },
        )]
    } else {
        Vec::new()
    };
    assert_eq!(calls.stops, expected_stops);
}

fn run_receipts(rate: bool) {
    tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .unwrap()
        .block_on(async {
            for acknowledgement in [None, Some(true), Some(false)] {
                check_receipt(rate, acknowledgement).await;
            }
        });
}

#[test]
fn position_command_ack_preserves_device_receipt_evidence() {
    run_receipts(false);
}

#[test]
fn rate_command_ack_preserves_device_receipt_evidence() {
    run_receipts(true);
}
