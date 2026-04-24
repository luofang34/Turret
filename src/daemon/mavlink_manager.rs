use super::arbitrator::Arbitrator;
use super::config::MavlinkConfig;
use super::models::{CommandMode, ControlSource, GimbalCommand};
use super::state::StateManager;
use crate::gimbal::GimbalDevice;
use log::{debug, error, info, warn};
use mavlink::ardupilotmega::{
    GimbalManagerFlags, MavAutopilot, MavMessage, MavModeFlag, MavState, MavType,
    GIMBAL_MANAGER_STATUS_DATA, HEARTBEAT_DATA,
};
use std::collections::HashSet;
use std::net::SocketAddr;
use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::net::UdpSocket;
use tokio::sync::Mutex;
use tokio::time;

/// Peer set: every GCS (or autopilot) we've ever received a packet from.
/// Heartbeats and `GIMBAL_MANAGER_STATUS` broadcasts go to every entry.
type Peers = Arc<Mutex<HashSet<SocketAddr>>>;

/// MAVLink Gimbal Manager implementation
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

    /// Start the MAVLink manager
    pub async fn start(self, config: MavlinkConfig) -> crate::error::Result<()> {
        if config.transport == "udp" {
            self.start_udp(config).await
        } else {
            warn!("Serial transport not yet implemented for MAVLink");
            Ok(())
        }
    }

    /// Start UDP MAVLink transport
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
                Self::heartbeat_loop(socket, peers, seq, config).await;
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
                Self::publish_status_loop(socket, peers, seq, state, config, start_time).await;
            });
        }

        // Main receive loop
        let mut buf = [0u8; 1024];
        let gimbal = self.gimbal.clone();
        let arbitrator = self.arbitrator.clone();
        let peers = self.peers.clone();
        let seq = self.seq.clone();
        let mavlink_config = config.clone();

        loop {
            match socket.recv_from(&mut buf).await {
                Ok((len, src_addr)) => {
                    debug!("Received {} bytes from {}", len, src_addr);

                    // Record peer so subsequent HEARTBEAT / STATUS broadcasts
                    // reach this GCS even if it never sends again.
                    peers.lock().await.insert(src_addr);

                    // Parse MAVLink message
                    let mut reader = mavlink::peek_reader::PeekReader::new(&buf[..len]);
                    match mavlink::read_v2_msg(&mut reader) {
                        Ok((header, msg)) => {
                            debug!("Received MAVLink message: {:?}", msg);

                            Self::handle_mavlink_message(
                                header.system_id,
                                msg,
                                &socket,
                                src_addr,
                                &mavlink_config,
                                &seq,
                                &gimbal,
                                &arbitrator,
                            )
                            .await;
                        }
                        Err(e) => {
                            debug!("Failed to parse MAVLink message: {}", e);
                        }
                    }
                }
                Err(e) => {
                    error!("UDP receive error: {}", e);
                }
            }
        }
    }

    /// Broadcast `HEARTBEAT` at 1 Hz to every recorded peer.
    async fn heartbeat_loop(
        socket: Arc<UdpSocket>,
        peers: Peers,
        seq: Arc<AtomicU8>,
        config: MavlinkConfig,
    ) {
        let mut interval = time::interval(Duration::from_secs(1));
        loop {
            interval.tick().await;

            let hb = HEARTBEAT_DATA {
                custom_mode: 0,
                mavtype: MavType::MAV_TYPE_GIMBAL,
                autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
                base_mode: MavModeFlag::empty(),
                system_status: MavState::MAV_STATE_ACTIVE,
                mavlink_version: 3,
            };
            let msg = MavMessage::HEARTBEAT(hb);

            let targets: Vec<SocketAddr> = peers.lock().await.iter().copied().collect();
            if targets.is_empty() {
                continue;
            }
            for peer in targets {
                if let Err(e) = Self::send_mavlink_message(
                    &socket,
                    peer,
                    config.sysid,
                    config.compid,
                    &seq,
                    msg.clone(),
                )
                .await
                {
                    warn!("HEARTBEAT to {} failed: {}", peer, e);
                }
            }
        }
    }

    /// Handle incoming MAVLink message.
    ///
    /// TODO(item-5): collapse `socket` / `seq` / `config` into a send context
    /// when `COMMAND_LONG` dispatch + primary-client tracking lands.
    #[allow(clippy::too_many_arguments)]
    async fn handle_mavlink_message(
        sender_sysid: u8,
        msg: MavMessage,
        socket: &Arc<UdpSocket>,
        src_addr: std::net::SocketAddr,
        config: &MavlinkConfig,
        seq: &Arc<AtomicU8>,
        gimbal: &Arc<Mutex<Box<dyn GimbalDevice>>>,
        arbitrator: &Arbitrator,
    ) {
        match msg {
            MavMessage::GIMBAL_MANAGER_SET_ATTITUDE(set_attitude) => {
                debug!("Received GIMBAL_MANAGER_SET_ATTITUDE: {:?}", set_attitude);

                // Check if message is for us (target_system and target_component)
                if set_attitude.target_system != 0 && set_attitude.target_system != config.sysid {
                    return;
                }

                // For now, center the gimbal when receiving attitude command
                // TODO: Implement proper quaternion to Euler conversion
                let command = GimbalCommand::new(
                    ControlSource::Mavlink(sender_sysid),
                    CommandMode::Position,
                    Some(0.0),
                    Some(0.0),
                    Some(0.0),
                );

                let mut gimbal_guard = gimbal.lock().await;
                match arbitrator.process_command(command, &mut gimbal_guard) {
                    Ok(true) => info!(
                        "Executed MAVLink attitude command from sysid {}",
                        sender_sysid
                    ),
                    Ok(false) => warn!("MAVLink command rejected (lower priority)"),
                    Err(e) => error!("Failed to execute MAVLink command: {}", e),
                }
            }

            MavMessage::GIMBAL_MANAGER_SET_PITCHYAW(set_pitchyaw) => {
                debug!("Received GIMBAL_MANAGER_SET_PITCHYAW: {:?}", set_pitchyaw);

                // Check if message is for us
                if set_pitchyaw.target_system != 0 && set_pitchyaw.target_system != config.sysid {
                    return;
                }

                // Convert radians to degrees
                let pitch_deg = set_pitchyaw.pitch.to_degrees();
                let yaw_deg = set_pitchyaw.yaw.to_degrees();

                let command = GimbalCommand::new(
                    ControlSource::Mavlink(sender_sysid),
                    CommandMode::Position,
                    Some(yaw_deg),
                    Some(pitch_deg),
                    Some(0.0),
                );

                let mut gimbal_guard = gimbal.lock().await;
                match arbitrator.process_command(command, &mut gimbal_guard) {
                    Ok(true) => info!(
                        "Executed MAVLink pitch/yaw command from sysid {}",
                        sender_sysid
                    ),
                    Ok(false) => warn!("MAVLink command rejected (lower priority)"),
                    Err(e) => error!("Failed to execute MAVLink command: {}", e),
                }
            }

            MavMessage::GIMBAL_MANAGER_SET_MANUAL_CONTROL(set_manual) => {
                debug!(
                    "Received GIMBAL_MANAGER_SET_MANUAL_CONTROL: {:?}",
                    set_manual
                );

                // Check if message is for us
                if set_manual.target_system != 0 && set_manual.target_system != config.sysid {
                    return;
                }

                // Manual control uses normalized values (-1.0 to +1.0)
                // Scale to reasonable angle range (e.g., ±90 degrees)
                let pitch_deg = set_manual.pitch * 90.0;
                let yaw_deg = set_manual.yaw * 90.0;

                let command = GimbalCommand::new(
                    ControlSource::Mavlink(sender_sysid),
                    CommandMode::Manual,
                    Some(yaw_deg),
                    Some(pitch_deg),
                    Some(0.0),
                );

                let mut gimbal_guard = gimbal.lock().await;
                match arbitrator.process_command(command, &mut gimbal_guard) {
                    Ok(true) => debug!(
                        "Executed MAVLink manual control from sysid {}",
                        sender_sysid
                    ),
                    Ok(false) => debug!("MAVLink manual control rejected (lower priority)"),
                    Err(e) => error!("Failed to execute MAVLink manual control: {}", e),
                }
            }

            MavMessage::GIMBAL_MANAGER_INFORMATION(_info_req) => {
                debug!("Received GIMBAL_MANAGER_INFORMATION request");

                // Send GIMBAL_MANAGER_INFORMATION response
                Self::send_gimbal_manager_information(socket, src_addr, config, seq).await;
            }

            _ => {
                // Ignore other messages
                debug!("Ignoring MAVLink message type: {:?}", msg);
            }
        }
    }

    /// Broadcast `GIMBAL_MANAGER_STATUS` at 5 Hz to every recorded peer.
    ///
    /// Primary / secondary control fields are currently zero — real
    /// primary-client tracking lands with the `COMMAND_LONG` / `_CONFIGURE`
    /// dispatch work (roadmap item 5).
    async fn publish_status_loop(
        socket: Arc<UdpSocket>,
        peers: Peers,
        seq: Arc<AtomicU8>,
        _state_manager: StateManager,
        config: MavlinkConfig,
        start_time: Instant,
    ) {
        let mut interval = time::interval(Duration::from_millis(200)); // 5 Hz

        loop {
            interval.tick().await;

            let status = GIMBAL_MANAGER_STATUS_DATA {
                time_boot_ms: start_time.elapsed().as_millis() as u32,
                flags: GimbalManagerFlags::default(),
                gimbal_device_id: config.compid,
                primary_control_sysid: 0,
                primary_control_compid: 0,
                secondary_control_sysid: 0,
                secondary_control_compid: 0,
            };
            let msg = MavMessage::GIMBAL_MANAGER_STATUS(status);

            let targets: Vec<SocketAddr> = peers.lock().await.iter().copied().collect();
            if targets.is_empty() {
                continue;
            }
            for peer in targets {
                if let Err(e) = Self::send_mavlink_message(
                    &socket,
                    peer,
                    config.sysid,
                    config.compid,
                    &seq,
                    msg.clone(),
                )
                .await
                {
                    warn!("GIMBAL_MANAGER_STATUS to {} failed: {}", peer, e);
                }
            }
        }
    }

    /// Send GIMBAL_MANAGER_INFORMATION response
    async fn send_gimbal_manager_information(
        socket: &Arc<UdpSocket>,
        dest_addr: std::net::SocketAddr,
        config: &MavlinkConfig,
        seq: &Arc<AtomicU8>,
    ) {
        // Based on mavlink 0.13 schema for GIMBAL_MANAGER_INFORMATION
        let info = mavlink::ardupilotmega::GIMBAL_MANAGER_INFORMATION_DATA {
            time_boot_ms: 0, // TODO: Track uptime
            cap_flags: mavlink::ardupilotmega::GimbalManagerCapFlags::default(),
            gimbal_device_id: config.compid,
            roll_min: -180.0_f32.to_radians(),
            roll_max: 180.0_f32.to_radians(),
            pitch_min: -90.0_f32.to_radians(),
            pitch_max: 90.0_f32.to_radians(),
            yaw_min: -180.0_f32.to_radians(),
            yaw_max: 180.0_f32.to_radians(),
        };

        let msg = MavMessage::GIMBAL_MANAGER_INFORMATION(info);

        match Self::send_mavlink_message(socket, dest_addr, config.sysid, config.compid, seq, msg)
            .await
        {
            Ok(_) => debug!("Sent GIMBAL_MANAGER_INFORMATION"),
            Err(e) => error!("Failed to send GIMBAL_MANAGER_INFORMATION: {}", e),
        }
    }

    /// Send a MAVLink message
    async fn send_mavlink_message(
        socket: &Arc<UdpSocket>,
        dest_addr: std::net::SocketAddr,
        sysid: u8,
        compid: u8,
        seq: &Arc<AtomicU8>,
        msg: MavMessage,
    ) -> crate::error::Result<()> {
        let header = mavlink::MavHeader {
            system_id: sysid,
            component_id: compid,
            // MAVLink sequence is an 8-bit counter that wraps; overflow panics
            // in debug builds without wrapping_add, so use fetch_add (already
            // wrapping) and take the post-increment value.
            sequence: seq.fetch_add(1, Ordering::Relaxed),
        };

        let mut buf = Vec::new();
        mavlink::write_v2_msg(&mut buf, header, &msg)?;

        socket.send_to(&buf, dest_addr).await?;
        Ok(())
    }
}
