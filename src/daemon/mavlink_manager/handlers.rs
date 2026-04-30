//! Per-message MAVLink handlers and outbound helpers for
//! [`crate::daemon::mavlink_manager::MavlinkManager`].
//!
//! This module holds everything that happens *after* a UDP datagram has been
//! parsed into a `MavMessage` or during the periodic broadcast loops. The
//! parent module owns the socket / task lifecycle; this one is frame logic.

use super::math::{
    clamp_axis, euler_deg_to_quaternion_wxyz, quaternion_wxyz_to_euler_deg, to_id_byte,
};
use super::{Peers, RxCtx};
use crate::daemon::config::{GimbalLimits, MavlinkConfig};
use crate::daemon::models::{CommandMode, ControlSource, GimbalCommand, PrimaryControl};
use crate::daemon::state::StateManager;
use crate::gimbal::{Attitude, GimbalDevice};
use mavlink::ardupilotmega::{
    GimbalDeviceErrorFlags, GimbalDeviceFlags, GimbalManagerFlags, MavAutopilot, MavCmd,
    MavMessage, MavModeFlag, MavResult, MavState, MavType, COMMAND_ACK_DATA, COMMAND_LONG_DATA,
    GIMBAL_DEVICE_ATTITUDE_STATUS_DATA, GIMBAL_MANAGER_STATUS_DATA, HEARTBEAT_DATA,
};
use std::net::SocketAddr;
use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::net::UdpSocket;
use tokio::sync::Mutex;
use tokio::time;
use tracing::{debug, error, info, warn};

/// Broadcast `HEARTBEAT` at 1 Hz to every recorded peer.
pub(super) async fn heartbeat_loop(
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
            if let Err(e) = send_mavlink_message(
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

/// Broadcast `GIMBAL_MANAGER_STATUS` at 5 Hz to every recorded peer.
pub(super) async fn publish_status_loop(
    socket: Arc<UdpSocket>,
    peers: Peers,
    seq: Arc<AtomicU8>,
    state_manager: StateManager,
    config: MavlinkConfig,
    start_time: Instant,
) {
    let mut interval = time::interval(Duration::from_millis(200)); // 5 Hz
    loop {
        interval.tick().await;

        let msg = MavMessage::GIMBAL_MANAGER_STATUS(build_gimbal_manager_status_data(
            &state_manager,
            &config,
            start_time,
        ));

        let targets: Vec<SocketAddr> = peers.lock().await.iter().copied().collect();
        if targets.is_empty() {
            continue;
        }
        for peer in targets {
            if let Err(e) = send_mavlink_message(
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

/// Attitude poll + `GIMBAL_DEVICE_ATTITUDE_STATUS` broadcast loop.
///
/// Runs at a low fixed cadence (~4 Hz) and is the **only** writer of
/// `state.{yaw,pitch,roll}` — see the comment in
/// `Arbitrator::process_command` for why commanded values are deliberately
/// not mirrored into state. On poll error we skip both the state update and
/// the broadcast so consumers never see a stale-pretending-to-be-fresh
/// value, and we count consecutive failures so a full unplug can be logged
/// once instead of every tick.
///
/// `MissedTickBehavior::Skip` keeps the cadence honest if the gimbal mutex
/// is held longer than the interval (e.g. a slow `set_attitude` queueing
/// behind us) — we don't queue up backlogged ticks.
#[allow(clippy::too_many_arguments)]
pub(super) async fn attitude_loop(
    socket: Arc<UdpSocket>,
    peers: Peers,
    seq: Arc<AtomicU8>,
    state_manager: StateManager,
    config: MavlinkConfig,
    gimbal: Arc<Mutex<Box<dyn GimbalDevice>>>,
    start_time: Instant,
    notify_lost: Arc<tokio::sync::Notify>,
) {
    /// Number of consecutive poll failures (~250 ms each) before we declare
    /// the device gone and ask the reconnect task to take over. 8 ticks =
    /// ~2 seconds of silence, long enough to ride out a transient serial
    /// hiccup but short enough that an unplug is noticed promptly.
    const FAILURE_THRESHOLD: u32 = 8;
    /// How long after the last failed poll the next *successful* broadcast
    /// still carries `COMMS_ERROR`. Solid devices clear within one tick;
    /// flaky devices keep the flag set so a GCS sees `degraded` instead
    /// of `healthy` on every other frame.
    const COMMS_ERROR_LINGER: Duration = Duration::from_secs(1);

    let mut interval = time::interval(Duration::from_millis(250));
    interval.set_missed_tick_behavior(time::MissedTickBehavior::Skip);
    // We've waited 250ms for the first tick — fire immediately instead.
    interval.tick().await;
    let mut consecutive_failures: u32 = 0;
    let mut last_failure: Option<Instant> = None;

    loop {
        interval.tick().await;

        let attitude = {
            let mut g = gimbal.lock().await;
            g.get_attitude()
        };

        let attitude = match attitude {
            Ok(a) => {
                if consecutive_failures > 0 {
                    info!(
                        "Attitude readback recovered after {} consecutive failures",
                        consecutive_failures
                    );
                }
                consecutive_failures = 0;
                a
            }
            Err(e) => {
                consecutive_failures = consecutive_failures.wrapping_add(1);
                last_failure = Some(Instant::now());
                if consecutive_failures == 1 || consecutive_failures % 20 == 0 {
                    warn!(
                        "Attitude readback failed (#{}): {}",
                        consecutive_failures, e
                    );
                }
                if consecutive_failures == FAILURE_THRESHOLD {
                    warn!(
                        "Gimbal silent for {} consecutive polls; signaling reconnect",
                        FAILURE_THRESHOLD
                    );
                    notify_lost.notify_one();
                }
                continue;
            }
        };

        state_manager.update_measured_angles(attitude.yaw, attitude.pitch, attitude.roll);

        // A *recent* failure (this poll succeeded but one within the last
        // second didn't) means the link is flaky. Surface that to the GCS
        // via COMMS_ERROR rather than letting an in-between healthy frame
        // mask the instability.
        let failure_flags = match last_failure {
            Some(t) if t.elapsed() < COMMS_ERROR_LINGER => {
                GimbalDeviceErrorFlags::GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR
            }
            _ => GimbalDeviceErrorFlags::empty(),
        };

        let msg = MavMessage::GIMBAL_DEVICE_ATTITUDE_STATUS(build_attitude_status_data(
            &attitude,
            start_time,
            failure_flags,
        ));

        let targets: Vec<SocketAddr> = peers.lock().await.iter().copied().collect();
        if targets.is_empty() {
            continue;
        }
        for peer in targets {
            if let Err(e) = send_mavlink_message(
                &socket,
                peer,
                config.sysid,
                config.compid,
                &seq,
                msg.clone(),
            )
            .await
            {
                warn!("GIMBAL_DEVICE_ATTITUDE_STATUS to {} failed: {}", peer, e);
            }
        }
    }
}

/// Build a `GIMBAL_DEVICE_ATTITUDE_STATUS_DATA` from a measured attitude.
///
/// `target_system / target_component = 0` — the manager broadcasts this to
/// every recorded peer, not to a single addressed system. `flags` is set
/// to `YAW_IN_VEHICLE_FRAME` to be honest about what the manager actually
/// supports today; earth-frame yaw needs vehicle-attitude integration that
/// isn't wired up. `failure_flags` is supplied by the caller — see
/// [`attitude_loop`] for the COMMS_ERROR linger window. The persistent
/// "device is dead" failure mode is still expressed by stopping the
/// broadcast entirely, not by setting flags on a stale frame.
fn build_attitude_status_data(
    attitude: &Attitude,
    start_time: Instant,
    failure_flags: GimbalDeviceErrorFlags,
) -> GIMBAL_DEVICE_ATTITUDE_STATUS_DATA {
    let q = euler_deg_to_quaternion_wxyz(attitude.pitch, attitude.roll, attitude.yaw);
    GIMBAL_DEVICE_ATTITUDE_STATUS_DATA {
        time_boot_ms: start_time.elapsed().as_millis() as u32,
        target_system: 0,
        target_component: 0,
        flags: GimbalDeviceFlags::GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME,
        q,
        angular_velocity_x: f32::NAN,
        angular_velocity_y: f32::NAN,
        angular_velocity_z: f32::NAN,
        failure_flags,
    }
}

/// Dispatch a parsed inbound `MavMessage` to a per-type handler.
#[tracing::instrument(level = "debug", skip(msg, ctx))]
pub(super) async fn handle_mavlink_message(
    sender_sysid: u8,
    sender_compid: u8,
    src_addr: SocketAddr,
    msg: MavMessage,
    ctx: &RxCtx<'_>,
) {
    match msg {
        MavMessage::GIMBAL_MANAGER_SET_ATTITUDE(m) => {
            on_set_attitude(sender_sysid, sender_compid, m, ctx).await;
        }
        MavMessage::GIMBAL_MANAGER_SET_PITCHYAW(m) => {
            on_set_pitchyaw(sender_sysid, sender_compid, m, ctx).await;
        }
        MavMessage::GIMBAL_MANAGER_SET_MANUAL_CONTROL(m) => {
            on_set_manual(sender_sysid, sender_compid, m, ctx).await;
        }
        MavMessage::GIMBAL_MANAGER_INFORMATION(_) => {
            debug!("Received GIMBAL_MANAGER_INFORMATION request");
            send_gimbal_manager_information(
                ctx.socket,
                src_addr,
                ctx.config,
                ctx.seq,
                ctx.start_time,
            )
            .await;
        }
        MavMessage::COMMAND_LONG(cmd) => {
            on_command_long(sender_sysid, sender_compid, src_addr, cmd, ctx).await;
        }
        _ => debug!("Ignoring MAVLink message type: {:?}", msg),
    }
}

async fn on_set_attitude(
    sender_sysid: u8,
    sender_compid: u8,
    m: mavlink::ardupilotmega::GIMBAL_MANAGER_SET_ATTITUDE_DATA,
    ctx: &RxCtx<'_>,
) {
    debug!("Received GIMBAL_MANAGER_SET_ATTITUDE: {:?}", m);
    if !addressed_to_us(
        m.target_system,
        m.target_component,
        ctx.config.sysid,
        ctx.config.compid,
    ) {
        return;
    }
    if !ctx.state.is_primary_allowed(sender_sysid, sender_compid) {
        warn!(
            "Ignoring GIMBAL_MANAGER_SET_ATTITUDE from non-primary ({},{})",
            sender_sysid, sender_compid
        );
        return;
    }

    // NaN in q means "ignore quaternion, use angular_velocity instead" per
    // spec. Rate control isn't implemented yet, so just drop the command.
    if m.q.iter().any(|c| c.is_nan()) {
        debug!("GIMBAL_MANAGER_SET_ATTITUDE with NaN quaternion — rate path not yet supported");
        return;
    }

    // TODO: honor frame flags (YAW_IN_VEHICLE_FRAME vs YAW_IN_EARTH_FRAME)
    // and RETRACT / NEUTRAL shortcuts. Earth-frame yaw needs vehicle
    // attitude integration, which depends on AUTOPILOT_STATE_FOR_GIMBAL_DEVICE
    // ingestion that isn't wired up yet.
    let (pitch_deg, roll_deg, yaw_deg) = quaternion_wxyz_to_euler_deg(m.q);
    let (pitch, roll, yaw) = clamp_pry(pitch_deg, roll_deg, yaw_deg, &ctx.config.limits);
    let command = GimbalCommand::new(
        ControlSource::Mavlink(sender_sysid),
        CommandMode::Position,
        Some(yaw),
        Some(pitch),
        Some(roll),
    );
    dispatch_gimbal_command(command, ctx, "SET_ATTITUDE").await;
}

async fn on_set_pitchyaw(
    sender_sysid: u8,
    sender_compid: u8,
    m: mavlink::ardupilotmega::GIMBAL_MANAGER_SET_PITCHYAW_DATA,
    ctx: &RxCtx<'_>,
) {
    debug!("Received GIMBAL_MANAGER_SET_PITCHYAW: {:?}", m);
    if !addressed_to_us(
        m.target_system,
        m.target_component,
        ctx.config.sysid,
        ctx.config.compid,
    ) {
        return;
    }
    if !ctx.state.is_primary_allowed(sender_sysid, sender_compid) {
        warn!(
            "Ignoring GIMBAL_MANAGER_SET_PITCHYAW from non-primary ({},{})",
            sender_sysid, sender_compid
        );
        return;
    }

    let pitch_deg = m.pitch.to_degrees();
    let yaw_deg = m.yaw.to_degrees();
    let (pitch, _roll, yaw) = clamp_pry(pitch_deg, 0.0, yaw_deg, &ctx.config.limits);
    let command = GimbalCommand::new(
        ControlSource::Mavlink(sender_sysid),
        CommandMode::Position,
        Some(yaw),
        Some(pitch),
        Some(0.0),
    );
    dispatch_gimbal_command(command, ctx, "SET_PITCHYAW").await;
}

async fn on_set_manual(
    sender_sysid: u8,
    sender_compid: u8,
    m: mavlink::ardupilotmega::GIMBAL_MANAGER_SET_MANUAL_CONTROL_DATA,
    ctx: &RxCtx<'_>,
) {
    debug!("Received GIMBAL_MANAGER_SET_MANUAL_CONTROL: {:?}", m);
    if !addressed_to_us(
        m.target_system,
        m.target_component,
        ctx.config.sysid,
        ctx.config.compid,
    ) {
        return;
    }
    if !ctx.state.is_primary_allowed(sender_sysid, sender_compid) {
        debug!(
            "Ignoring GIMBAL_MANAGER_SET_MANUAL_CONTROL from non-primary ({},{})",
            sender_sysid, sender_compid
        );
        return;
    }

    // Manual control uses normalized values (-1.0..=1.0). Scale to ±90°
    // then clamp to the configured device limits in case those are
    // tighter than ±90°.
    let pitch_deg = m.pitch * 90.0;
    let yaw_deg = m.yaw * 90.0;
    let (pitch, _roll, yaw) = clamp_pry(pitch_deg, 0.0, yaw_deg, &ctx.config.limits);
    let command = GimbalCommand::new(
        ControlSource::Mavlink(sender_sysid),
        CommandMode::Manual,
        Some(yaw),
        Some(pitch),
        Some(0.0),
    );
    dispatch_gimbal_command(command, ctx, "SET_MANUAL_CONTROL").await;
}

async fn on_command_long(
    sender_sysid: u8,
    sender_compid: u8,
    src_addr: SocketAddr,
    cmd: COMMAND_LONG_DATA,
    ctx: &RxCtx<'_>,
) {
    if !addressed_to_us(
        cmd.target_system,
        cmd.target_component,
        ctx.config.sysid,
        ctx.config.compid,
    ) {
        return;
    }
    match cmd.command {
        MavCmd::MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW => {
            on_cmd_pitchyaw(sender_sysid, sender_compid, src_addr, cmd, ctx).await;
        }
        MavCmd::MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE => {
            on_cmd_configure(src_addr, cmd, ctx).await;
        }
        MavCmd::MAV_CMD_REQUEST_MESSAGE => {
            on_cmd_request_message(src_addr, cmd, ctx).await;
        }
        _ => debug!("Ignoring COMMAND_LONG: {:?}", cmd.command),
    }
}

/// Standard MAVLink message IDs the manager can serve on demand.
///
/// `GIMBAL_DEVICE_ATTITUDE_STATUS` (285) is intentionally not handled here:
/// the 4 Hz attitude_loop already broadcasts it to every recorded peer, so
/// any GCS that requests one will get the next periodic frame within
/// 250 ms. Acknowledging the request without doing anything would be a lie;
/// we'd rather fall through to `MAV_RESULT_DENIED` and let the GCS know to
/// just listen.
mod msg_id {
    pub(super) const GIMBAL_MANAGER_INFORMATION: u32 = 280;
    pub(super) const GIMBAL_MANAGER_STATUS: u32 = 281;
}

/// Convert a `COMMAND_LONG.param1` to a message id. NaN / negative / very
/// large values collapse to `None` — the caller will deny the request.
fn message_id_from_param(p: f32) -> Option<u32> {
    if !p.is_finite() || p < 0.0 || p > u32::MAX as f32 {
        return None;
    }
    Some(p.round() as u32)
}

async fn on_cmd_request_message(src_addr: SocketAddr, cmd: COMMAND_LONG_DATA, ctx: &RxCtx<'_>) {
    let result = match message_id_from_param(cmd.param1) {
        Some(msg_id::GIMBAL_MANAGER_INFORMATION) => {
            send_gimbal_manager_information(
                ctx.socket,
                src_addr,
                ctx.config,
                ctx.seq,
                ctx.start_time,
            )
            .await;
            MavResult::MAV_RESULT_ACCEPTED
        }
        Some(msg_id::GIMBAL_MANAGER_STATUS) => {
            let status = build_gimbal_manager_status_data(ctx.state, ctx.config, ctx.start_time);
            let msg = MavMessage::GIMBAL_MANAGER_STATUS(status);
            match send_mavlink_message(
                ctx.socket,
                src_addr,
                ctx.config.sysid,
                ctx.config.compid,
                ctx.seq,
                msg,
            )
            .await
            {
                Ok(()) => MavResult::MAV_RESULT_ACCEPTED,
                Err(e) => {
                    error!("Failed to send on-demand GIMBAL_MANAGER_STATUS: {}", e);
                    MavResult::MAV_RESULT_FAILED
                }
            }
        }
        Some(other) => {
            debug!("MAV_CMD_REQUEST_MESSAGE for unsupported id {}", other);
            MavResult::MAV_RESULT_DENIED
        }
        None => {
            debug!(
                "MAV_CMD_REQUEST_MESSAGE with non-integer param1 {}",
                cmd.param1
            );
            MavResult::MAV_RESULT_DENIED
        }
    };
    send_command_ack(src_addr, ctx, MavCmd::MAV_CMD_REQUEST_MESSAGE, result).await;
}

async fn on_cmd_pitchyaw(
    sender_sysid: u8,
    sender_compid: u8,
    src_addr: SocketAddr,
    cmd: COMMAND_LONG_DATA,
    ctx: &RxCtx<'_>,
) {
    if !ctx.state.is_primary_allowed(sender_sysid, sender_compid) {
        warn!(
            "Rejecting MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW from non-primary ({},{})",
            sender_sysid, sender_compid
        );
        send_command_ack(
            src_addr,
            ctx,
            MavCmd::MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            MavResult::MAV_RESULT_DENIED,
        )
        .await;
        return;
    }

    // param1 = pitch (deg), param2 = yaw (deg). param3/4 are rates and
    // ignored until a rate-control path lands. NaN means "leave axis
    // unchanged" per spec — substitute 0.0 for now.
    let pitch_in = if cmd.param1.is_nan() { 0.0 } else { cmd.param1 };
    let yaw_in = if cmd.param2.is_nan() { 0.0 } else { cmd.param2 };
    let (pitch, _roll, yaw) = clamp_pry(pitch_in, 0.0, yaw_in, &ctx.config.limits);
    let command = GimbalCommand::new(
        ControlSource::Mavlink(sender_sysid),
        CommandMode::Position,
        Some(yaw),
        Some(pitch),
        Some(0.0),
    );

    let result = {
        let mut gimbal_guard = ctx.gimbal.lock().await;
        ctx.arbitrator.process_command(command, &mut gimbal_guard)
    };
    let ack = match result {
        Ok(true) => {
            info!(
                "MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW accepted from ({},{})",
                sender_sysid, sender_compid
            );
            MavResult::MAV_RESULT_ACCEPTED
        }
        Ok(false) => {
            debug!("MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW rejected by arbitrator (priority)");
            MavResult::MAV_RESULT_TEMPORARILY_REJECTED
        }
        Err(e) => {
            error!("MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW execution failed: {}", e);
            MavResult::MAV_RESULT_FAILED
        }
    };
    send_command_ack(
        src_addr,
        ctx,
        MavCmd::MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
        ack,
    )
    .await;
}

async fn on_cmd_configure(src_addr: SocketAddr, cmd: COMMAND_LONG_DATA, ctx: &RxCtx<'_>) {
    let pc = PrimaryControl {
        primary_sysid: to_id_byte(cmd.param1),
        primary_compid: to_id_byte(cmd.param2),
        secondary_sysid: to_id_byte(cmd.param3),
        secondary_compid: to_id_byte(cmd.param4),
    };
    ctx.state.set_primary_control(pc);
    info!(
        "MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE → primary=({},{}), secondary=({},{})",
        pc.primary_sysid, pc.primary_compid, pc.secondary_sysid, pc.secondary_compid,
    );
    send_command_ack(
        src_addr,
        ctx,
        MavCmd::MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
        MavResult::MAV_RESULT_ACCEPTED,
    )
    .await;
}

/// Clamp `(pitch, roll, yaw)` to the configured device limits and log
/// per-axis if a clamp actually occurred. The order of the returned tuple
/// matches the input order, not the order arguments appear in
/// `GimbalCommand::new`.
fn clamp_pry(pitch: f32, roll: f32, yaw: f32, limits: &GimbalLimits) -> (f32, f32, f32) {
    (
        clamp_axis(pitch, limits.pitch_min_deg, limits.pitch_max_deg, "pitch"),
        clamp_axis(roll, limits.roll_min_deg, limits.roll_max_deg, "roll"),
        clamp_axis(yaw, limits.yaw_min_deg, limits.yaw_max_deg, "yaw"),
    )
}

/// Common tail for SET_* handlers: execute through the arbitrator and log.
async fn dispatch_gimbal_command(command: GimbalCommand, ctx: &RxCtx<'_>, label: &str) {
    let mut gimbal_guard = ctx.gimbal.lock().await;
    match ctx.arbitrator.process_command(command, &mut gimbal_guard) {
        Ok(true) => info!("Executed MAVLink {}", label),
        Ok(false) => warn!("MAVLink {} rejected (lower priority)", label),
        Err(e) => error!("Failed to execute MAVLink {}: {}", label, e),
    }
}

/// True if a frame whose `target_system` / `target_component` we're looking
/// at should be acted on by this manager.
///
/// Both axes obey the MAVLink "0 = broadcast, otherwise must match" rule.
/// Without the component check the manager would happily execute commands
/// addressed to the autopilot (compid 1) on the same vehicle, which is
/// what the GMv2 spec is trying to prevent when multiple components share
/// a sysid.
fn addressed_to_us(target_system: u8, target_component: u8, our_sysid: u8, our_compid: u8) -> bool {
    let sys_ok = target_system == 0 || target_system == our_sysid;
    let comp_ok = target_component == 0 || target_component == our_compid;
    sys_ok && comp_ok
}

/// Build a `GIMBAL_MANAGER_STATUS` frame from the current state. Used both
/// by the 5 Hz publish loop and the on-demand `MAV_CMD_REQUEST_MESSAGE`
/// responder.
fn build_gimbal_manager_status_data(
    state_manager: &StateManager,
    config: &MavlinkConfig,
    start_time: Instant,
) -> GIMBAL_MANAGER_STATUS_DATA {
    let pc = state_manager.get_primary_control();
    GIMBAL_MANAGER_STATUS_DATA {
        time_boot_ms: start_time.elapsed().as_millis() as u32,
        flags: GimbalManagerFlags::default(),
        gimbal_device_id: config.compid,
        primary_control_sysid: pc.primary_sysid,
        primary_control_compid: pc.primary_compid,
        secondary_control_sysid: pc.secondary_sysid,
        secondary_control_compid: pc.secondary_compid,
    }
}

async fn send_gimbal_manager_information(
    socket: &Arc<UdpSocket>,
    dest_addr: SocketAddr,
    config: &MavlinkConfig,
    seq: &Arc<AtomicU8>,
    start_time: Instant,
) {
    use mavlink::ardupilotmega::GimbalManagerCapFlags as Caps;

    // Capabilities the Storm32-backed manager actually has. We advertise
    // all three axes (Storm32 is a 3-axis gimbal), follow / lock per axis
    // (covered by SetPanMode 0..=5), retract via SetStandby, neutral via
    // recenter_*, and RC-style inputs (the protocol's native command set).
    // We do NOT advertise infinite-yaw, earth-frame yaw, or point-at-location:
    // earth-frame yaw needs vehicle attitude integration we don't have, and
    // point-at-location needs GPS / coordinate-frame plumbing.
    let cap_flags = Caps::GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT
        | Caps::GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL
        | Caps::GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS
        | Caps::GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW
        | Caps::GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK
        | Caps::GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS
        | Caps::GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW
        | Caps::GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK
        | Caps::GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS
        | Caps::GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW
        | Caps::GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK
        | Caps::GIMBAL_MANAGER_CAP_FLAGS_HAS_RC_INPUTS;

    let limits = &config.limits;
    let info = mavlink::ardupilotmega::GIMBAL_MANAGER_INFORMATION_DATA {
        time_boot_ms: start_time.elapsed().as_millis() as u32,
        cap_flags,
        gimbal_device_id: config.compid,
        roll_min: limits.roll_min_deg.to_radians(),
        roll_max: limits.roll_max_deg.to_radians(),
        pitch_min: limits.pitch_min_deg.to_radians(),
        pitch_max: limits.pitch_max_deg.to_radians(),
        yaw_min: limits.yaw_min_deg.to_radians(),
        yaw_max: limits.yaw_max_deg.to_radians(),
    };

    let msg = MavMessage::GIMBAL_MANAGER_INFORMATION(info);

    match send_mavlink_message(socket, dest_addr, config.sysid, config.compid, seq, msg).await {
        Ok(_) => debug!("Sent GIMBAL_MANAGER_INFORMATION"),
        Err(e) => error!("Failed to send GIMBAL_MANAGER_INFORMATION: {}", e),
    }
}

async fn send_command_ack(
    dest_addr: SocketAddr,
    ctx: &RxCtx<'_>,
    command: MavCmd,
    result: MavResult,
) {
    let ack = COMMAND_ACK_DATA { command, result };
    let msg = MavMessage::COMMAND_ACK(ack);
    if let Err(e) = send_mavlink_message(
        ctx.socket,
        dest_addr,
        ctx.config.sysid,
        ctx.config.compid,
        ctx.seq,
        msg,
    )
    .await
    {
        error!("Failed to send COMMAND_ACK: {}", e);
    }
}

async fn send_mavlink_message(
    socket: &Arc<UdpSocket>,
    dest_addr: SocketAddr,
    sysid: u8,
    compid: u8,
    seq: &Arc<AtomicU8>,
    msg: MavMessage,
) -> crate::error::Result<()> {
    let header = mavlink::MavHeader {
        system_id: sysid,
        component_id: compid,
        // MAVLink sequence is an 8-bit counter that wraps; fetch_add on
        // AtomicU8 wraps by definition, matching the CLAUDE.md ban on
        // plain `+= 1` on monotonic counters.
        sequence: seq.fetch_add(1, Ordering::Relaxed),
    };

    let mut buf = Vec::new();
    mavlink::write_v2_msg(&mut buf, header, &msg)?;

    socket.send_to(&buf, dest_addr).await?;
    Ok(())
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;

    #[test]
    fn addressed_to_us_accepts_per_axis_broadcast_and_exact_match() {
        // Both axes broadcast.
        assert!(addressed_to_us(0, 0, 1, 154));
        // sysid broadcast, compid exact.
        assert!(addressed_to_us(0, 154, 1, 154));
        // sysid exact, compid broadcast.
        assert!(addressed_to_us(1, 0, 1, 154));
        // Both exact.
        assert!(addressed_to_us(1, 154, 1, 154));
    }

    #[test]
    fn addressed_to_us_rejects_other_components_on_same_sysid() {
        // Targeting the autopilot (compid 1) on the same vehicle must NOT
        // be acted on by the gimbal manager — same-sysid component routing
        // is exactly what the compid axis is for.
        assert!(!addressed_to_us(1, 1, 1, 154));
    }

    #[test]
    fn addressed_to_us_rejects_other_systems() {
        assert!(!addressed_to_us(2, 154, 1, 154));
        assert!(!addressed_to_us(2, 0, 1, 154));
    }

    fn approx_eq(a: f32, b: f32, eps: f32) -> bool {
        (a - b).abs() < eps
    }

    #[test]
    fn build_attitude_status_round_trips_quaternion() {
        let attitude = Attitude {
            pitch: 12.0,
            roll: -7.5,
            yaw: 33.25,
        };
        let data =
            build_attitude_status_data(&attitude, Instant::now(), GimbalDeviceErrorFlags::empty());

        // Decoding the quaternion back must recover the input Euler angles.
        // This catches both the math.rs helper and our use of it.
        let (p, r, y) = quaternion_wxyz_to_euler_deg(data.q);
        assert!(approx_eq(p, attitude.pitch, 1e-3), "pitch: got {p}");
        assert!(approx_eq(r, attitude.roll, 1e-3), "roll: got {r}");
        assert!(approx_eq(y, attitude.yaw, 1e-3), "yaw: got {y}");
    }

    #[test]
    fn build_attitude_status_uses_broadcast_addressing_and_vehicle_frame() {
        let data = build_attitude_status_data(
            &Attitude {
                pitch: 0.0,
                roll: 0.0,
                yaw: 0.0,
            },
            Instant::now(),
            GimbalDeviceErrorFlags::empty(),
        );

        // Manager broadcasts to every peer, not a single addressed system.
        assert_eq!(data.target_system, 0);
        assert_eq!(data.target_component, 0);

        // Honest about what we support: vehicle-frame yaw, no earth-frame
        // tracking, no rate path.
        assert!(data
            .flags
            .contains(GimbalDeviceFlags::GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME));
        assert!(data.angular_velocity_x.is_nan());
        assert!(data.angular_velocity_y.is_nan());
        assert!(data.angular_velocity_z.is_nan());

        // No failures supplied → frame is honestly clean. The persistent
        // "device is dead" failure mode is expressed by stopping the
        // broadcast entirely (see attitude_loop), not by setting flags
        // on a stale frame.
        assert!(data.failure_flags.is_empty());
    }

    #[test]
    fn build_attitude_status_propagates_caller_failure_flags() {
        // The attitude_loop sets COMMS_ERROR for ~1 s after any failed
        // poll so a flaky-link recovery doesn't slip through as a fully
        // healthy frame.
        let data = build_attitude_status_data(
            &Attitude {
                pitch: 0.0,
                roll: 0.0,
                yaw: 0.0,
            },
            Instant::now(),
            GimbalDeviceErrorFlags::GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR,
        );
        assert!(data
            .failure_flags
            .contains(GimbalDeviceErrorFlags::GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR));
    }
}
