//! MAVLink Gimbal Manager: task lifecycle + shared receive-side context.
//!
//! Per-message frame logic and the broadcast loops live in a private
//! `handlers` submodule alongside this file.

#[path = "mavlink_manager/handlers.rs"]
mod handlers;
// `math` carries pure helpers (wrap_180, earth↔body yaw, quaternion).
// Visible to the rest of the daemon so the yaw corrector can share
// `wrap_180_deg` without duplicating it.
#[path = "mavlink_manager/math.rs"]
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
use tokio::sync::{Mutex, Notify};
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
        gimbal: GimbalHandle,
        notify_lost: Arc<Notify>,
    ) -> Self {
        let arbitrator = Arc::new(Arbitrator::new(state_manager.clone()));

        Self {
            state_manager,
            arbitrator,
            gimbal,
            peers: Arc::new(Mutex::new(HashMap::new())),
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
        let bind_addr = SocketAddr::new(config.bind_addr, config.udp_port);
        info!("Starting MAVLink manager on UDP {}", bind_addr);

        let socket = Arc::new(UdpSocket::bind(bind_addr).await?);
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
            // Default-on yaw drift corrector with mount_yaw_deg=0
            // (gimbal-forward-equals-vehicle-nose). When a future
            // `gimbal.mount_yaw_deg` config knob lands, plumb it
            // through here. The corrector itself is a no-op until at
            // least one `set` has gone through the arbitrator.
            let corrector = super::yaw_corrector::YawCorrector::new(0.0);
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
                    corrector,
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
                    // `record_peer` enforces the TTL + hard cap so the
                    // map can't grow unboundedly under sustained churn.
                    record_peer(&mut *peers.lock().await, src_addr, Instant::now());

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

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    //! Cover the peer-map eviction. Same shape as
    //! `daemon::state::tests::evict_for_insert_*` so a future maintainer
    //! reading either one immediately recognizes the pattern.

    use super::*;

    fn addr(port: u16) -> SocketAddr {
        SocketAddr::from(([127, 0, 0, 1], port))
    }

    #[test]
    fn record_peer_inserts_and_refreshes() {
        let mut peers = HashMap::new();
        let t0 = Instant::now();
        record_peer(&mut peers, addr(1000), t0);
        assert_eq!(peers.len(), 1);
        assert_eq!(peers[&addr(1000)], t0);

        // Re-record the same peer at a later time → entry overwritten,
        // map size unchanged.
        let t1 = t0 + Duration::from_secs(1);
        record_peer(&mut peers, addr(1000), t1);
        assert_eq!(peers.len(), 1);
        assert_eq!(peers[&addr(1000)], t1);
    }

    #[test]
    fn record_peer_drops_ttl_expired_entries() {
        let mut peers = HashMap::new();
        let t0 = Instant::now();
        record_peer(&mut peers, addr(1000), t0);
        record_peer(&mut peers, addr(1001), t0);
        assert_eq!(peers.len(), 2);

        // Advance past the TTL and insert a fresh peer. The two old
        // entries should be swept on the way in.
        let t_late = t0 + PEER_TTL + Duration::from_secs(1);
        record_peer(&mut peers, addr(1002), t_late);
        assert_eq!(peers.len(), 1, "stale peers were not evicted");
        assert!(peers.contains_key(&addr(1002)));
    }

    #[test]
    fn record_peer_respects_hard_cap_under_burst() {
        let mut peers = HashMap::new();
        let t0 = Instant::now();
        // Fill the map within the TTL window so the TTL pass evicts
        // nothing — this exercises the hard-cap path.
        for i in 0..MAX_PEERS as u16 {
            record_peer(
                &mut peers,
                addr(2000 + i),
                t0 + Duration::from_millis(i.into()),
            );
        }
        assert_eq!(peers.len(), MAX_PEERS);
        let oldest = addr(2000);
        assert!(peers.contains_key(&oldest));

        // One more insert. Cap holds, oldest goes.
        let t_new = t0 + Duration::from_millis(MAX_PEERS as u64);
        record_peer(&mut peers, addr(9999), t_new);
        assert_eq!(peers.len(), MAX_PEERS);
        assert!(
            !peers.contains_key(&oldest),
            "oldest should have been evicted"
        );
        assert!(peers.contains_key(&addr(9999)));
    }

    #[test]
    fn record_peer_burst_stays_capped() {
        // Sanity stress: insert many more than MAX_PEERS distinct
        // addresses inside one TTL window. Final size must equal the
        // cap, not the number of inserts.
        let mut peers = HashMap::new();
        let t0 = Instant::now();
        let burst = MAX_PEERS * 5;
        for i in 0..burst as u16 {
            record_peer(
                &mut peers,
                addr(10_000 + i),
                t0 + Duration::from_millis(i.into()),
            );
        }
        assert_eq!(peers.len(), MAX_PEERS, "hard cap must hold under burst");
    }
}
