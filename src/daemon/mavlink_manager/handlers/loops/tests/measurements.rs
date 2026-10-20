use super::*;
use crate::daemon::attitude::{attitude_channel, attitude_poll_loop};
use crate::daemon::config::DaemonConfig;
use crate::daemon::gimbal_handle::GimbalHandle;
use crate::daemon::mavlink_manager::MavlinkManager;
use crate::gimbal::{DeviceReceipt, GimbalDevice};
use mavlink::ardupilotmega::{MavCmd, COMMAND_LONG_DATA, GIMBAL_DEVICE_ATTITUDE_STATUS_DATA};
use mavlink::{read_v2_msg, write_v2_msg, MavHeader};
use tokio::sync::{mpsc, oneshot, Notify};
use tokio::task::JoinHandle;
use turret_core::{PayloadCapabilities, PayloadCommand};

type Reply = oneshot::Sender<DeviceObservation>;

struct ObservedDevice {
    requests: mpsc::UnboundedSender<Reply>,
}

impl GimbalDevice for ObservedDevice {
    fn protocol_name(&self) -> &'static str {
        "measured-observation-test"
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
        let (reply, response) = oneshot::channel();
        self.requests
            .send(reply)
            .map_err(|_| crate::Error::InvalidInput("observation test ended".into()))?;
        response
            .blocking_recv()
            .map_err(|_| crate::Error::InvalidInput("observation test ended".into()))
    }
}

struct PollSession {
    requests: mpsc::UnboundedReceiver<Reply>,
    ticks: broadcast::Receiver<Arc<AttitudeTick>>,
    state: StateManager,
    socket: UdpSocket,
    start_time: Instant,
    poll: JoinHandle<()>,
    manager: JoinHandle<crate::Result<()>>,
}

impl PollSession {
    async fn start() -> Self {
        let socket = UdpSocket::bind("127.0.0.1:0").await.unwrap();
        let manager_socket = Arc::new(UdpSocket::bind("127.0.0.1:0").await.unwrap());
        socket
            .connect(manager_socket.local_addr().unwrap())
            .await
            .unwrap();
        let (request_tx, requests) = mpsc::unbounded_channel();
        let gimbal = GimbalHandle::new(Box::new(ObservedDevice {
            requests: request_tx,
        }))
        .unwrap();
        let state = StateManager::new();
        let (sender, receiver) = attitude_channel();
        let ticks = sender.subscribe();
        let manager = MavlinkManager::new(state.clone(), gimbal.clone());
        let start_time = manager.start_time;
        let config = MavlinkConfig {
            sysid: 1,
            compid: 154,
            ..DaemonConfig::default().mavlink
        };
        let manager = tokio::spawn(manager.start_on_socket(config, receiver, manager_socket));
        socket.send(&status_request()).await.unwrap();
        tokio::time::timeout(Duration::from_secs(2), wait_for_status(&socket))
            .await
            .unwrap();
        let poll = tokio::spawn(attitude_poll_loop(
            gimbal,
            state.clone(),
            Arc::new(Notify::new()),
            None,
            sender,
        ));
        Self {
            requests,
            ticks,
            state,
            socket,
            start_time,
            poll,
            manager,
        }
    }

    async fn next_poll(&mut self) -> Reply {
        tokio::time::timeout(Duration::from_secs(2), self.requests.recv())
            .await
            .unwrap()
            .unwrap()
    }

    async fn receive_measurement(&mut self, sample: DeviceObservation) {
        let tick = tokio::time::timeout(Duration::from_secs(2), self.ticks.recv())
            .await
            .unwrap()
            .unwrap();
        assert_eq!(tick.observation.sampled_at, sample.sampled_at);
        let data = tokio::time::timeout(Duration::from_secs(2), receive_attitude(&self.socket))
            .await
            .unwrap();
        let expected = sample
            .sampled_at
            .unwrap()
            .duration_since(self.start_time)
            .as_millis() as u32;
        assert_eq!(data.time_boot_ms, expected);
        let pitch = (2.0 * (data.q[0] * data.q[2] - data.q[3] * data.q[1])).asin();
        assert!((pitch - sample.angles.pitch.unwrap().as_radians()).abs() < 0.001);
    }

    async fn shutdown(mut self) {
        self.requests.close();
        while let Ok(reply) = self.requests.try_recv() {
            drop(reply);
        }
        self.poll.abort();
        let stopped = tokio::time::timeout(Duration::from_secs(2), &mut self.poll).await;
        assert!(stopped.unwrap().unwrap_err().is_cancelled());
        match tokio::time::timeout(Duration::from_secs(2), &mut self.manager).await {
            Ok(result) => result.unwrap().unwrap(),
            Err(_) => {
                self.manager.abort();
                (&mut self.manager).await.ok();
                panic!("MAVLink manager did not stop after polling ended");
            }
        }
    }
}

impl Drop for PollSession {
    fn drop(&mut self) {
        self.poll.abort();
        self.manager.abort();
    }
}

fn status_request() -> Vec<u8> {
    let data = COMMAND_LONG_DATA {
        target_system: 1,
        target_component: 154,
        confirmation: 0,
        command: MavCmd::MAV_CMD_REQUEST_MESSAGE,
        param1: 281.0,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
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

async fn receive_message(socket: &UdpSocket) -> MavMessage {
    let mut bytes = [0; 1024];
    let count = socket.recv(&mut bytes).await.unwrap();
    read_v2_msg::<MavMessage, _>(&mut mavlink::peek_reader::PeekReader::new(&bytes[..count]))
        .unwrap()
        .1
}

async fn wait_for_status(socket: &UdpSocket) {
    loop {
        if matches!(
            receive_message(socket).await,
            MavMessage::GIMBAL_MANAGER_STATUS(_)
        ) {
            return;
        }
    }
}

async fn receive_attitude(socket: &UdpSocket) -> GIMBAL_DEVICE_ATTITUDE_STATUS_DATA {
    loop {
        if let MavMessage::GIMBAL_DEVICE_ATTITUDE_STATUS(data) = receive_message(socket).await {
            return data;
        }
    }
}

fn invalid_samples(first: DeviceObservation) -> Vec<DeviceObservation> {
    let now = Instant::now();
    let newer = sample(now);
    vec![
        DeviceObservation {
            received_at: now,
            ..first
        },
        sample(now - Duration::from_secs(2)),
        DeviceObservation {
            sampled_at: None,
            ..newer
        },
        DeviceObservation {
            source: ObservationSource::Estimated,
            ..newer
        },
        DeviceObservation {
            source: ObservationSource::OperatorAssumed,
            ..newer
        },
        DeviceObservation {
            frame: ReferenceFrame::Mount,
            ..newer
        },
        DeviceObservation {
            received_at: now + Duration::from_secs(10),
            ..newer
        },
        DeviceObservation {
            sampled_at: Some(now + Duration::from_secs(10)),
            ..newer
        },
    ]
}

#[test]
fn only_new_measured_samples_cross_the_observe_poll_udp_path() {
    tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .unwrap()
        .block_on(async {
            let mut session = PollSession::start().await;
            let first = sample(Instant::now());
            session.next_poll().await.send(first).unwrap();
            session.receive_measurement(first).await;
            let cached = session.state.state_cached().measured_at;
            let mut pending = session.next_poll().await;
            for invalid in invalid_samples(first) {
                pending.send(invalid).unwrap();
                pending = session.next_poll().await;
                assert!(session.ticks.try_recv().is_err());
                assert_eq!(session.state.state_cached().measured_at, cached);
            }
            let mut final_sample = sample(Instant::now());
            final_sample.angles.pitch = Some(Angle::radians(10_f32.to_radians()).unwrap());
            pending.send(final_sample).unwrap();
            session.receive_measurement(final_sample).await;
            session.shutdown().await;
        });
}

#[test]
fn queued_broadcasts_recheck_time_epoch_source_and_duplicate_samples() {
    tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .unwrap()
        .block_on(async {
            let receiver = UdpSocket::bind("127.0.0.1:0").await.unwrap();
            let sender = Arc::new(UdpSocket::bind("127.0.0.1:0").await.unwrap());
            let peers = Arc::new(tokio::sync::Mutex::new(std::collections::HashMap::from([
                (receiver.local_addr().unwrap(), Instant::now()),
            ])));
            let now = Instant::now();
            let start_time = now - Duration::from_millis(100);
            let first = sample(now - Duration::from_millis(50));
            let second = sample(now - Duration::from_millis(40));
            let (ticks, receiver_ticks) = attitude_channel();
            let mut samples = invalid_samples(first);
            samples[0] = sample(start_time - Duration::from_nanos(1));
            samples.push(DeviceObservation { epoch: 1, ..first });
            samples.extend([first, first, second]);
            for observation in samples {
                ticks
                    .send(Arc::new(AttitudeTick {
                        observation,
                        link_recently_failed: false,
                    }))
                    .unwrap();
            }
            drop(ticks);
            let task = tokio::spawn(attitude_broadcast_loop(
                sender,
                peers,
                Arc::new(AtomicU8::new(0)),
                DaemonConfig::default().mavlink,
                start_time,
                receiver_ticks,
                StateManager::new(),
            ));
            tokio::time::timeout(Duration::from_secs(2), task)
                .await
                .unwrap()
                .unwrap();
            for expected in [50, 60] {
                let data =
                    tokio::time::timeout(Duration::from_secs(2), receive_attitude(&receiver))
                        .await
                        .unwrap();
                assert_eq!(data.time_boot_ms, expected);
            }
            let mut remaining = [0; 1024];
            assert_eq!(
                receiver.try_recv(&mut remaining).unwrap_err().kind(),
                std::io::ErrorKind::WouldBlock
            );
        });
}

#[test]
fn reconnect_while_waiting_for_peers_discards_the_queued_measurement() {
    tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .unwrap()
        .block_on(async {
            let receiver = UdpSocket::bind("127.0.0.1:0").await.unwrap();
            let sender = Arc::new(UdpSocket::bind("127.0.0.1:0").await.unwrap());
            let peers = Arc::new(tokio::sync::Mutex::new(std::collections::HashMap::from([
                (receiver.local_addr().unwrap(), Instant::now()),
            ])));
            let peer_guard = peers.lock().await;
            let state = StateManager::new();
            let start_time = Instant::now();
            let (ticks, receiver_ticks) = attitude_channel();
            ticks
                .send(Arc::new(AttitudeTick {
                    observation: sample(Instant::now()),
                    link_recently_failed: false,
                }))
                .unwrap();
            drop(ticks);
            let mut broadcast = Box::pin(attitude_broadcast_loop(
                sender,
                peers.clone(),
                Arc::new(AtomicU8::new(0)),
                DaemonConfig::default().mavlink,
                start_time,
                receiver_ticks,
                state.clone(),
            ));
            std::future::poll_fn(|context| {
                assert!(std::future::Future::poll(broadcast.as_mut(), context).is_pending());
                std::task::Poll::Ready(())
            })
            .await;
            state.gimbal.invalidate(1);
            drop(peer_guard);
            tokio::time::timeout(Duration::from_secs(2), broadcast)
                .await
                .unwrap();
            let mut remaining = [0; 1024];
            assert_eq!(
                receiver.try_recv(&mut remaining).unwrap_err().kind(),
                std::io::ErrorKind::WouldBlock
            );
        });
}
