//! MAVLink Gimbal Manager: task lifecycle + shared receive-side context.
//!
//! Per-message frame logic and the broadcast loops live in a private
//! `handlers` submodule alongside this file.

#[path = "mavlink_manager/handlers.rs"]
mod handlers;
#[path = "mavlink_manager/math.rs"]
mod math;

use super::arbitrator::Arbitrator;
use super::config::MavlinkConfig;
use super::state::StateManager;
use crate::gimbal::GimbalDevice;
use log::{debug, error, info, warn};
use std::collections::HashSet;
use std::net::SocketAddr;
use std::sync::atomic::AtomicU8;
use std::sync::Arc;
use std::time::Instant;
use tokio::net::UdpSocket;
use tokio::sync::Mutex;

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
}

/// MAVLink Gimbal Manager implementation.
pub struct MavlinkManager {
    _config: MavlinkConfig,
    state_manager: StateManager,
    arbitrator: Arc<Arbitrator>,
    gimbal: Arc<Mutex<Box<dyn GimbalDevice>>>,
    peers: Peers,
    /// Monotonic sequence counter for outbound frames.
    seq: Arc<AtomicU8>,
    /// Daemon start time for `time_boot_ms` computation.
    start_time: Instant,
}

impl MavlinkManager {
    pub fn new(
        config: MavlinkConfig,
        state_manager: StateManager,
        gimbal: Arc<Mutex<Box<dyn GimbalDevice>>>,
    ) -> Self {
        let arbitrator = Arc::new(Arbitrator::new(state_manager.clone()));

        Self {
            _config: config.clone(),
            state_manager,
            arbitrator,
            gimbal,
            peers: Arc::new(Mutex::new(HashSet::new())),
            seq: Arc::new(AtomicU8::new(0)),
            start_time: Instant::now(),
        }
    }

    /// Start the MAVLink manager.
    pub async fn start(self, config: MavlinkConfig) -> crate::error::Result<()> {
        if config.transport == "udp" {
            self.start_udp(config).await
        } else {
            warn!("Serial transport not yet implemented for MAVLink");
            Ok(())
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

        // Main receive loop
        let mut buf = [0u8; 1024];
        let gimbal = self.gimbal.clone();
        let arbitrator = self.arbitrator.clone();
        let state = self.state_manager.clone();
        let peers = self.peers.clone();
        let seq = self.seq.clone();
        let mavlink_config = config.clone();

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
