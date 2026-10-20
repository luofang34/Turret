//! MAVLink Gimbal Manager: task lifecycle + shared receive-side context.
//!
//! Per-message frame logic and the broadcast loops live in a private
//! `handlers` submodule alongside this file.

mod handlers;
// `math` carries pure helpers (wrap_180, earth↔body yaw, quaternion).
// Visible to the rest of the daemon so the yaw corrector can share
// `wrap_180_deg` without duplicating it.
pub(crate) mod math;

use super::arbitrator::Arbitrator;
use super::config::{MavlinkConfig, Transport};
use super::gimbal_handle::GimbalHandle;
use super::state::StateManager;
use std::collections::HashMap;
use std::net::SocketAddr;
use std::sync::atomic::AtomicU8;
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::net::UdpSocket;
use tokio::sync::Mutex;
use tracing::{debug, error, info};

/// How long a peer may go silent before its entry in the peer map is
/// dropped on the next inbound packet. MAVLink heartbeat is 1 Hz by
/// convention, so 60 s of silence is "this GCS is gone." Until then we
/// keep broadcasting attitude / manager-status to it so a brief
/// disconnect (UDP packet loss spike, brief reboot) doesn't stop the
/// telemetry stream.
const PEER_TTL: Duration = Duration::from_secs(60);

/// Hard upper bound on the peer map size. The TTL alone covers normal
/// churn; this catches a burst that floods the manager with packets from
/// many distinct source ports faster than the TTL can clear stale
/// entries (a misconfigured client looping bind/send/close, an attacker
/// scanning, etc.). When at this many entries an insert evicts the
/// single oldest one. 256 matches MAVLink's `u8` sysid space —
/// even with every possible sysid alive there's headroom.
const MAX_PEERS: usize = 256;

/// Peer map: every GCS / autopilot we've heard from, with the
/// timestamp of the most recent packet. Heartbeats and broadcasts go
/// to every key. Bounded by `PEER_TTL` + `MAX_PEERS` via
/// [`record_peer`] on each inbound packet.
pub(crate) type Peers = Arc<Mutex<HashMap<SocketAddr, Instant>>>;

/// Insert / refresh a peer entry, with TTL eviction and a hard cap.
///
/// Two passes:
///
/// 1. TTL sweep — drop every entry older than [`PEER_TTL`]. Cheap O(n)
///    `retain`, handles the steady-state case where short-lived clients
///    come and go.
/// 2. Hard cap — if the map is *still* at capacity (a burst that all
///    landed within the TTL window), evict the single oldest entry
///    before inserting the new one. O(n) find-min, only fires under
///    the burst.
///
/// Together: post-call `len ≤ MAX_PEERS`. Same eviction shape as
/// `daemon::state::evict_for_insert` for consistency.
pub(crate) fn record_peer(
    peers: &mut HashMap<SocketAddr, Instant>,
    addr: SocketAddr,
    now: Instant,
) {
    peers.retain(|_, t| now.duration_since(*t) <= PEER_TTL);
    if !peers.contains_key(&addr) && peers.len() >= MAX_PEERS {
        if let Some(oldest) = peers.iter().min_by_key(|(_, t)| *t).map(|(addr, _)| *addr) {
            peers.remove(&oldest);
        }
    }
    peers.insert(addr, now);
}

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
    pub gimbal: &'a GimbalHandle,
    /// Daemon boot reference for `time_boot_ms` on on-demand frames built
    /// inside per-message handlers (the periodic loops carry their own copy).
    pub start_time: Instant,
}

/// MAVLink Gimbal Manager implementation.
pub struct MavlinkManager {
    state_manager: StateManager,
    arbitrator: Arc<Arbitrator>,
    gimbal: GimbalHandle,
    peers: Peers,
    /// Monotonic sequence counter for outbound frames.
    seq: Arc<AtomicU8>,
    /// Daemon start time for `time_boot_ms` computation.
    start_time: Instant,
}

impl MavlinkManager {
    /// Build a MAVLink Gimbal Manager. The hardware attitude poll +
    /// reconnect signaling + yaw corrector live in the daemon-level
    /// [`crate::daemon::attitude::attitude_poll_loop`] now (always-on,
    /// protocol-agnostic); the manager subscribes to attitude ticks
    /// through the broadcast `Receiver` passed to [`Self::start`].
    pub fn new(state_manager: StateManager, gimbal: GimbalHandle) -> Self {
        let arbitrator = Arc::new(Arbitrator::new(state_manager.clone()));

        Self {
            state_manager,
            arbitrator,
            gimbal,
            peers: Arc::new(Mutex::new(HashMap::new())),
            seq: Arc::new(AtomicU8::new(0)),
            start_time: Instant::now(),
        }
    }

    /// Start the MAVLink manager. `attitude_rx` is the manager's
    /// subscription to the daemon's attitude poll broadcast — every
    /// successful poll fires one [`AttitudeTick`](crate::daemon::attitude::AttitudeTick)
    /// the broadcast loop consumes to send `GIMBAL_DEVICE_ATTITUDE_STATUS`.
    pub async fn start(
        self,
        config: MavlinkConfig,
        attitude_rx: tokio::sync::broadcast::Receiver<Arc<crate::daemon::attitude::AttitudeTick>>,
    ) -> crate::error::Result<()> {
        match config.transport {
            Transport::Udp => self.start_udp(config, attitude_rx).await,
        }
    }

    /// Start UDP MAVLink transport: bind socket, spawn broadcast loops,
    /// then run the main receive loop.
    async fn start_udp(
        self,
        config: MavlinkConfig,
        attitude_rx: tokio::sync::broadcast::Receiver<Arc<crate::daemon::attitude::AttitudeTick>>,
    ) -> crate::error::Result<()> {
        let bind_addr = SocketAddr::new(config.bind_addr, config.udp_port);
        info!("Starting MAVLink manager on UDP {}", bind_addr);

        let socket = Arc::new(UdpSocket::bind(bind_addr).await?);
        info!("MAVLink UDP socket bound to {}", bind_addr);

        self.start_on_socket(config, attitude_rx, socket).await
    }

    /// Serve a pre-bound socket; the caller can select an ephemeral port without a bind race.
    /// All broadcast futures share this lifetime and stop when the owner is cancelled.
    pub async fn start_on_socket(
        self,
        config: MavlinkConfig,
        attitude_rx: tokio::sync::broadcast::Receiver<Arc<crate::daemon::attitude::AttitudeTick>>,
        socket: Arc<UdpSocket>,
    ) -> crate::error::Result<()> {
        tokio::select! {
            result = self.receive_loop(&config, &socket) => result,
            () = handlers::heartbeat_loop(socket.clone(), self.peers.clone(), self.seq.clone(), config.clone()) => Ok(()),
            () = handlers::publish_status_loop(socket.clone(), self.peers.clone(), self.seq.clone(), self.state_manager.clone(), config.clone(), self.start_time) => Ok(()),
            () = handlers::attitude_broadcast_loop(socket.clone(), self.peers.clone(), self.seq.clone(), config.clone(), self.start_time, attitude_rx, self.state_manager.clone()) => Ok(()),
        }
    }

    async fn receive_loop(
        &self,
        config: &MavlinkConfig,
        socket: &Arc<UdpSocket>,
    ) -> crate::error::Result<()> {
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
                    // `record_peer` enforces the TTL + hard cap so the
                    // map can't grow unboundedly under sustained churn.
                    record_peer(&mut *peers.lock().await, src_addr, Instant::now());

                    let mut reader = mavlink::peek_reader::PeekReader::new(&buf[..len]);
                    match mavlink::read_v2_msg(&mut reader) {
                        Ok((header, msg)) => {
                            debug!("Received MAVLink message: {:?}", msg);
                            let ctx = RxCtx {
                                socket,
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

#[cfg(test)]
mod tests;
