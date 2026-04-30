//! MAVLink Gimbal Manager: task lifecycle + shared receive-side context.
//!
//! Per-message frame logic and the broadcast loops live in a private
//! `handlers` submodule alongside this file.

#[path = "mavlink_manager/handlers.rs"]
mod handlers;
#[path = "mavlink_manager/math.rs"]
mod math;

use super::arbitrator::Arbitrator;
use super::config::{MavlinkConfig, Transport};
use super::state::StateManager;
use crate::gimbal::GimbalDevice;
use std::collections::HashSet;
use std::net::SocketAddr;
use std::sync::atomic::AtomicU8;
use std::sync::Arc;
use std::time::Instant;
use tokio::net::UdpSocket;
use tokio::sync::{Mutex, Notify};
use tracing::{debug, error, info};

/// Peer set: every GCS (or autopilot) we've ever received a packet from.
/// Heartbeats and `GIMBAL_MANAGER_STATUS` broadcasts go to every entry.
pub(crate) type Peers = Arc<Mutex<HashSet<SocketAddr>>>;

/// Receive-side context: references every per-message handler needs.
///
/// Bundles `socket`, `config`, `seq`, `state`, `arbitrator`, and `gimbal`
/// into a single short-lived reference so handler signatures stay within
/// clippy's arg-count limit.
pub(crate) struct RxCtx<'a> {
    pub socket: &'a Arc<UdpSocket>,
    pub config: &'a MavlinkConfig,
    pub seq: &'a Arc<AtomicU8>,
    pub state: &'a StateManager,
    pub arbitrator: &'a Arbitrator,
    pub gimbal: &'a Arc<Mutex<Box<dyn GimbalDevice>>>,
    /// Daemon boot reference for `time_boot_ms` on on-demand frames built
    /// inside per-message handlers (the periodic loops carry their own copy).
    pub start_time: Instant,
}

/// MAVLink Gimbal Manager implementation.
pub struct MavlinkManager {
    state_manager: StateManager,
    arbitrator: Arc<Arbitrator>,
    gimbal: Arc<Mutex<Box<dyn GimbalDevice>>>,
    peers: Peers,
    /// Monotonic sequence counter for outbound frames.
    seq: Arc<AtomicU8>,
    /// Daemon start time for `time_boot_ms` computation.
    start_time: Instant,
    /// Set by the attitude poll loop after `FAILURE_THRESHOLD` consecutive
    /// readback errors. The daemon-level reconnect task awaits on this.
    notify_lost: Arc<Notify>,
}

impl MavlinkManager {
    /// Build a MAVLink Gimbal Manager. `notify_lost` is the shared
    /// `Notify` that the attitude poll loop pings when consecutive read
    /// failures cross the loop's failure threshold; the daemon-level
    /// `reconnect_task` drives recovery from there.
    pub fn new(
        state_manager: StateManager,
        gimbal: Arc<Mutex<Box<dyn GimbalDevice>>>,
        notify_lost: Arc<Notify>,
    ) -> Self {
        let arbitrator = Arc::new(Arbitrator::new(state_manager.clone()));

        Self {
            state_manager,
            arbitrator,
            gimbal,
            peers: Arc::new(Mutex::new(HashSet::new())),
            seq: Arc::new(AtomicU8::new(0)),
            start_time: Instant::now(),
            notify_lost,
        }
    }

    /// Start the MAVLink manager.
    pub async fn start(self, config: MavlinkConfig) -> crate::error::Result<()> {
        match config.transport {
            Transport::Udp => self.start_udp(config).await,
        }
    }

    /// Start UDP MAVLink transport: bind socket, spawn broadcast loops,
    /// then run the main receive loop.
    async fn start_udp(self, config: MavlinkConfig) -> crate::error::Result<()> {
        let bind_addr = format!("0.0.0.0:{}", config.udp_port);
        info!("Starting MAVLink manager on UDP {}", bind_addr);

        let socket = Arc::new(UdpSocket::bind(&bind_addr).await?);
        info!("MAVLink UDP socket bound to {}", bind_addr);

        // 1 Hz HEARTBEAT — announces us as MAV_TYPE_GIMBAL so any GCS that
        // hears the packet discovers the gimbal manager.
        {
            let socket = socket.clone();
            let peers = self.peers.clone();
            let seq = self.seq.clone();
            let config = config.clone();
            tokio::spawn(async move {
                handlers::heartbeat_loop(socket, peers, seq, config).await;
            });
        }

        // 5 Hz GIMBAL_MANAGER_STATUS — keeps GCSs in sync with primary /
        // secondary control ownership and gimbal device id.
        {
            let socket = socket.clone();
            let peers = self.peers.clone();
            let seq = self.seq.clone();
            let state = self.state_manager.clone();
            let config = config.clone();
            let start_time = self.start_time;
            tokio::spawn(async move {
                handlers::publish_status_loop(socket, peers, seq, state, config, start_time).await;
            });
        }

        // 4 Hz attitude poll + GIMBAL_DEVICE_ATTITUDE_STATUS broadcast.
        // This is the only writer of measured state.{yaw,pitch,roll} — see
        // `Arbitrator::process_command` for why commanded values stay in
        // `last_command` instead of overwriting the measured fields. After
        // `FAILURE_THRESHOLD` consecutive errors it pings `notify_lost` to
        // hand recovery off to the daemon-level reconnect task.
        {
            let socket = socket.clone();
            let peers = self.peers.clone();
            let seq = self.seq.clone();
            let state = self.state_manager.clone();
            let config = config.clone();
            let gimbal = self.gimbal.clone();
            let start_time = self.start_time;
            let notify_lost = self.notify_lost.clone();
            tokio::spawn(async move {
                handlers::attitude_loop(
                    socket,
                    peers,
                    seq,
                    state,
                    config,
                    gimbal,
                    start_time,
                    notify_lost,
                )
                .await;
            });
        }

        // Main receive loop
        let mut buf = [0u8; 1024];
        let gimbal = self.gimbal.clone();
        let arbitrator = self.arbitrator.clone();
        let state = self.state_manager.clone();
        let peers = self.peers.clone();
        let seq = self.seq.clone();
        let mavlink_config = config.clone();
        let start_time = self.start_time;

        loop {
            match socket.recv_from(&mut buf).await {
                Ok((len, src_addr)) => {
                    debug!("Received {} bytes from {}", len, src_addr);

                    // Record peer so subsequent broadcasts reach this GCS.
                    peers.lock().await.insert(src_addr);

                    let mut reader = mavlink::peek_reader::PeekReader::new(&buf[..len]);
                    match mavlink::read_v2_msg(&mut reader) {
                        Ok((header, msg)) => {
                            debug!("Received MAVLink message: {:?}", msg);
                            let ctx = RxCtx {
                                socket: &socket,
                                config: &mavlink_config,
                                seq: &seq,
                                state: &state,
                                arbitrator: &arbitrator,
                                gimbal: &gimbal,
                                start_time,
                            };
                            handlers::handle_mavlink_message(
                                header.system_id,
                                header.component_id,
                                src_addr,
                                msg,
                                &ctx,
                            )
                            .await;
                        }
                        Err(e) => debug!("Failed to parse MAVLink message: {}", e),
                    }
                }
                Err(e) => error!("UDP receive error: {}", e),
            }
        }
    }
}
