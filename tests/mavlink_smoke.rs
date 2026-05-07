//! End-to-end smoke: a fake GCS sends `GIMBAL_MANAGER_SET_PITCHYAW` to
//! the manager over UDP, then verifies that the angles flow through
//! inbound dispatch → arbitrator → `GimbalDevice::set_attitude`, AND
//! that the next attitude-loop tick broadcasts a
//! `GIMBAL_DEVICE_ATTITUDE_STATUS` back encoding the same angles.
//!
//! This exercises in one shot:
//!
//! - UDP bind + recv loop in `MavlinkManager::start_udp`
//! - `target_system / target_component` validation
//! - `addressed_to_us` per-axis check
//! - primary-control gating (default = open)
//! - quaternion ↔ Euler conversion in both directions
//! - clamp-to-limits path (no clamping at the values used here)
//! - the arbitrator's accept-and-execute path
//! - the 4 Hz attitude poll + broadcast helper
//!
//! Anything that breaks any one of those will surface as a panic in
//! one of the two phase loops.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use mavlink::ardupilotmega::{GimbalManagerFlags, MavMessage, GIMBAL_MANAGER_SET_PITCHYAW_DATA};
use mavlink::{read_v2_msg, write_v2_msg, MavHeader};
use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::sync::{Arc, Mutex as StdMutex};
use std::time::{Duration, Instant};
use tokio::net::UdpSocket;
use tokio::sync::Notify;
use turret::daemon::config::{GimbalLimits, MavlinkConfig, Transport};
use turret::daemon::gimbal_handle::GimbalHandle;
use turret::daemon::mavlink_manager::MavlinkManager;
use turret::daemon::state::StateManager;
use turret::error::Result;
use turret::gimbal::{Attitude, GimbalDevice};

#[derive(Default)]
struct MockState {
    last_attitude: Option<(f32, f32, f32)>,
}

struct MockGimbal {
    state: Arc<StdMutex<MockState>>,
}

impl GimbalDevice for MockGimbal {
    fn protocol_name(&self) -> &'static str {
        "Mock"
    }

    fn set_attitude(&mut self, pitch: f32, roll: f32, yaw: f32) -> Result<()> {
        self.state.lock().unwrap().last_attitude = Some((pitch, roll, yaw));
        Ok(())
    }

    fn get_attitude(&mut self) -> Result<Attitude> {
        let s = self.state.lock().unwrap();
        let (pitch, roll, yaw) = s.last_attitude.unwrap_or((0.0, 0.0, 0.0));
        Ok(Attitude { pitch, roll, yaw })
    }
}

/// Bind once, capture the port, drop. The OS will keep handing out the
/// same port for a short window — race with other parallel tests is
/// possible but rare. Tolerable for one integration test.
async fn pick_free_udp_port() -> u16 {
    let s = UdpSocket::bind("127.0.0.1:0").await.unwrap();
    s.local_addr().unwrap().port()
}

/// Z-Y-X intrinsic Euler decomposition. Mirror of the private helper in
/// `src/daemon/mavlink_manager/math.rs` — copied here so the test can
/// decode the wire-format quaternion without reaching into crate internals.
fn quaternion_wxyz_to_euler_deg(q: [f32; 4]) -> (f32, f32, f32) {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    let sinr_cosp = 2.0 * (w * x + y * z);
    let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    let roll = sinr_cosp.atan2(cosr_cosp);
    let sinp = (2.0 * (w * y - z * x)).clamp(-1.0, 1.0);
    let pitch = sinp.asin();
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    let yaw = siny_cosp.atan2(cosy_cosp);
    (pitch.to_degrees(), roll.to_degrees(), yaw.to_degrees())
}

fn build_set_pitchyaw_frame(
    pitch_deg: f32,
    yaw_deg: f32,
    target_sysid: u8,
    target_compid: u8,
    sender_sysid: u8,
    sender_compid: u8,
) -> Vec<u8> {
    let m = GIMBAL_MANAGER_SET_PITCHYAW_DATA {
        flags: GimbalManagerFlags::default(),
        pitch: pitch_deg.to_radians(),
        yaw: yaw_deg.to_radians(),
        pitch_rate: f32::NAN,
        yaw_rate: f32::NAN,
        target_system: target_sysid,
        target_component: target_compid,
        gimbal_device_id: 0,
    };
    let header = MavHeader {
        system_id: sender_sysid,
        component_id: sender_compid,
        sequence: 0,
    };
    let mut buf = Vec::new();
    write_v2_msg(
        &mut buf,
        header,
        &MavMessage::GIMBAL_MANAGER_SET_PITCHYAW(m),
    )
    .unwrap();
    buf
}

#[tokio::test]
async fn set_pitchyaw_round_trips_through_attitude_status_broadcast() {
    let port = pick_free_udp_port().await;
    let our_sysid = 1u8;
    let our_compid = 154u8;
    // Bind loopback-only so the test never touches the host's external
    // interfaces, and so concurrent test processes on the same machine
    // don't compete for the port range outside 127.0.0.1.
    let config = MavlinkConfig {
        enabled: true,
        transport: Transport::Udp,
        bind_addr: IpAddr::V4(Ipv4Addr::LOCALHOST),
        udp_port: port,
        sysid: our_sysid,
        compid: our_compid,
        limits: GimbalLimits::default(),
    };

    let mock_state = Arc::new(StdMutex::new(MockState::default()));
    let mock = Box::new(MockGimbal {
        state: mock_state.clone(),
    }) as Box<dyn GimbalDevice>;
    let gimbal = GimbalHandle::new(mock);
    let state_manager = StateManager::new();
    let notify_lost = Arc::new(Notify::new());

    // Spawn the daemon-level attitude poll loop alongside the manager —
    // PR after the loop split (#67) made polling protocol-agnostic and
    // moved it out of MavlinkManager. Phase 2 of this test relies on
    // the broadcast loop seeing ticks on the channel below.
    let (attitude_tx, attitude_rx) = turret::daemon::attitude::attitude_channel();
    let _poll_task = tokio::spawn({
        let gimbal = gimbal.clone();
        let state = state_manager.clone();
        let notify = notify_lost.clone();
        let tx = attitude_tx.clone();
        async move {
            // No corrector — keeps the test tight (no calibration
            // persistence side-effects). The poll runs at 4 Hz; phase
            // 2's 3 s deadline gives ~12 ticks of headroom.
            turret::daemon::attitude::attitude_poll_loop(gimbal, state, notify, None, tx).await;
        }
    });

    let manager = MavlinkManager::new(state_manager, gimbal);
    let _manager_task = tokio::spawn({
        let cfg = config.clone();
        async move {
            // Manager is aborted at test-end via the tokio runtime drop;
            // a startup error here would already have failed the test
            // through the phase-1 timeout, so collapsing it to `.ok()`
            // is the correct discard.
            manager.start(cfg, attitude_rx).await.ok();
        }
    });

    // Bind a fake GCS, connect to the manager. `connect` sets the default
    // destination so subsequent `send` calls don't need an addr argument
    // and `recv` filters to packets from the connected peer.
    let gcs = UdpSocket::bind("127.0.0.1:0").await.unwrap();
    let manager_addr: SocketAddr = format!("127.0.0.1:{}", port).parse().unwrap();
    gcs.connect(manager_addr).await.unwrap();

    // Give the manager's spawned recv loop a moment to bind. Without this,
    // the first frame can race the `UdpSocket::bind` and get dropped on
    // slow CI.
    tokio::time::sleep(Duration::from_millis(150)).await;

    let frame = build_set_pitchyaw_frame(10.0, 20.0, our_sysid, our_compid, 255, 1);
    gcs.send(&frame).await.unwrap();

    // Phase 1: inbound path must reach the gimbal trait. Poll until the
    // mock's `last_attitude` is populated. SET_PITCHYAW handler forces
    // roll = 0 (the message has no roll axis) and the position-mode
    // arbitrator routes to `set_attitude(pitch, roll, yaw)`.
    let deadline = Instant::now() + Duration::from_secs(2);
    loop {
        if let Some((p, r, y)) = mock_state.lock().unwrap().last_attitude {
            assert!(
                (p - 10.0).abs() < 0.5,
                "pitch reached gimbal as {p}, want 10"
            );
            assert!(r.abs() < 0.5, "roll reached gimbal as {r}, want 0");
            assert!((y - 20.0).abs() < 0.5, "yaw reached gimbal as {y}, want 20");
            break;
        }
        if Instant::now() >= deadline {
            panic!("mock gimbal never received the SET_PITCHYAW command");
        }
        tokio::time::sleep(Duration::from_millis(25)).await;
    }

    // Phase 2: the attitude loop must broadcast back. It polls at ~4 Hz
    // and re-encodes Euler→quaternion. Drain frames (filtering out
    // HEARTBEAT / GIMBAL_MANAGER_STATUS / GIMBAL_MANAGER_INFORMATION
    // noise) until we see a GIMBAL_DEVICE_ATTITUDE_STATUS whose decoded
    // angles match what we commanded — proves the broadcast helper, the
    // peer-recording side effect of the inbound path, and the
    // build_attitude_status_data quaternion encoding all line up.
    let deadline = Instant::now() + Duration::from_secs(3);
    let mut buf = vec![0u8; 1024];
    loop {
        if Instant::now() >= deadline {
            panic!("manager never broadcast a matching GIMBAL_DEVICE_ATTITUDE_STATUS");
        }
        let recv = tokio::time::timeout(Duration::from_millis(300), gcs.recv(&mut buf)).await;
        let n = match recv {
            Ok(Ok(n)) => n,
            _ => continue,
        };
        let mut reader = mavlink::peek_reader::PeekReader::new(&buf[..n]);
        let msg = match read_v2_msg::<MavMessage, _>(&mut reader) {
            Ok((_, m)) => m,
            Err(_) => continue,
        };
        if let MavMessage::GIMBAL_DEVICE_ATTITUDE_STATUS(s) = msg {
            let (p, r, y) = quaternion_wxyz_to_euler_deg(s.q);
            if (p - 10.0).abs() < 0.5 && r.abs() < 0.5 && (y - 20.0).abs() < 0.5 {
                return;
            }
        }
    }
}
