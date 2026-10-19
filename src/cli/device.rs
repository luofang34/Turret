use super::output;
use clap::ArgMatches;
use tracing::{error, info};
use turret::daemon::calibration::Calibration;
use turret::device_scanner::DeviceScanner;
use turret::error::{Error, Result};
use turret::gimbal::{self, GimbalDevice};
use turret::protocols::storm32_rc::Storm32RC;

pub(super) fn as_storm32(gimbal: &mut dyn GimbalDevice) -> Result<&mut Storm32RC> {
    let protocol_name = gimbal.protocol_name();
    gimbal
        .as_any_mut()
        .and_then(|any| any.downcast_mut::<Storm32RC>())
        .ok_or_else(|| {
            Error::InvalidInput(format!(
                "command requires a STorM32 RC device, got {protocol_name}"
            ))
        })
}

pub(super) fn resolve_device_path_blocking(matches: &ArgMatches) -> Result<String> {
    if let Some(device) = matches.get_one::<String>("device") {
        info!("Using manually specified device: {}", device);
        return Ok(device.clone());
    }
    let scanner = DeviceScanner::new();
    match scanner.find_storm32_device() {
        Some(device) => {
            info!("Auto-detected STorM32 device: {}", device);
            Ok(device)
        }
        None => {
            let devices = scanner.list_all_devices();
            if devices.is_empty() {
                error!("No serial devices found");
            } else {
                error!("No STorM32 device auto-detected; available serial devices:");
                for d in &devices {
                    error!("  {} — {:?}", d.port_name, d.port_type);
                }
                error!("Pass --device <path> to override.");
            }
            Err(Error::NotDetected {
                device: "auto".to_string(),
                tried: "STorM32 USB CDC".to_string(),
            })
        }
    }
}

pub(super) fn open_direct_blocking(matches: &ArgMatches) -> Result<Box<dyn GimbalDevice>> {
    let device_path = resolve_device_path_blocking(matches)?;
    info!("Using device: {}", device_path);
    let gimbal = gimbal::detect_gimbal_blocking(&device_path)?;
    info!("Detected protocol: {}", gimbal.protocol_name());
    Ok(gimbal)
}

pub(super) fn direct_set_blocking(
    matches: &ArgMatches,
    pitch: f32,
    roll: f32,
    yaw: f32,
) -> Result<()> {
    use turret_core::{Angle, Axes, GimbalPosition, PayloadCommand, ReferenceFrame};
    let command = PayloadCommand::GimbalPosition(GimbalPosition {
        angles: Axes {
            pitch: Some(Angle::radians(pitch.to_radians())?),
            roll: Some(Angle::radians(roll.to_radians())?),
            yaw: Some(Angle::radians(yaw.to_radians())?),
        },
        frame: ReferenceFrame::Mount,
    });
    execute_blocking(matches, command)
}

pub(super) fn direct_center_blocking(matches: &ArgMatches) -> Result<()> {
    execute_blocking(matches, turret_core::PayloadCommand::Recenter)
}

fn execute_blocking(matches: &ArgMatches, payload: turret_core::PayloadCommand) -> Result<()> {
    use std::time::{Duration, Instant};
    use turret_core::{Command, CommandIdentity, CommandTiming, Executor, Tick};
    let mut device = open_direct_blocking(matches)?;
    let capabilities = device.capabilities()?;
    let origin = Instant::now();
    let tick = || Tick(origin.elapsed().as_micros().min(u128::from(u64::MAX)) as u64);
    let mut executor = Executor::new(capabilities, 0, 0);
    let command = Command {
        identity: CommandIdentity {
            epoch: 0,
            generation: 0,
            id: 1,
            sequence: 1,
        },
        capability_revision: capabilities.revision,
        payload,
        timing: CommandTiming {
            send_before: Tick(1_000_000),
            valid_until: None,
            ack_before: None,
            completion_before: None,
        },
    };
    executor.submit(command, tick())?;
    executor
        .prepare_send(tick())?
        .ok_or(turret_core::Error::Expired)?;
    let receipt = device.execute_blocking(payload, origin + Duration::from_secs(1))?;
    let sent_at = Tick(
        receipt
            .sent_at
            .saturating_duration_since(origin)
            .as_micros()
            .min(u128::from(u64::MAX)) as u64,
    );
    let sent = executor.record_sent(command.identity, sent_at)?;
    if sent.current.effect != turret_core::Effect::None {
        device.stop_blocking(sent.current.effect)?;
    }
    match receipt.acknowledged {
        Some(true) => output(format_args!(
            "device accepted; movement completion unverified"
        )),
        Some(false) => Err(Error::Daemon("device rejected the command".to_owned())),
        None => output(format_args!(
            "sent; device acceptance and movement completion unverified"
        )),
    }
}

pub(super) fn direct_status_blocking(matches: &ArgMatches) -> Result<()> {
    let mut gimbal = open_direct_blocking(matches)?;
    let observation = gimbal.observe_blocking()?;
    let calibration = Calibration::load_blocking();
    let degrees =
        |angle: Option<turret_core::Angle>| angle.map(|value| value.as_radians().to_degrees());
    let pitch = degrees(observation.angles.pitch);
    let roll = degrees(observation.angles.roll);
    let yaw = degrees(observation.angles.yaw).map(|angle| angle - calibration.yaw_offset_deg);
    let display =
        |value: Option<f32>| value.map_or_else(|| "unknown".to_owned(), |v| format!("{v:.2}°"));
    output(format_args!(
        "pitch={}  roll={}  yaw={}",
        display(pitch),
        display(roll),
        display(yaw)
    ))?;
    output(format_args!(
        "frame={:?}  source={:?}",
        observation.frame, observation.source
    ))
}

pub(super) fn direct_version_blocking(matches: &ArgMatches) -> Result<()> {
    let mut gimbal = open_direct_blocking(matches)?;
    let version = gimbal.firmware_version_blocking()?;
    output(format_args!("{version}"))
}

pub(super) fn direct_version_str_blocking(matches: &ArgMatches) -> Result<()> {
    let mut gimbal = open_direct_blocking(matches)?;
    let storm32 = as_storm32(&mut *gimbal)?;
    let version = storm32.get_version_string_blocking()?;
    output(format_args!("{version}"))?;
    Ok(())
}

pub(super) fn direct_pan_mode_blocking(matches: &ArgMatches, mode: u8) -> Result<()> {
    let mut gimbal = open_direct_blocking(matches)?;
    info!("Setting pan mode to {mode}...");
    as_storm32(&mut *gimbal)?.set_pan_mode_blocking(mode)?;
    output(format_args!("ok"))?;
    Ok(())
}

pub(super) fn direct_standby_blocking(matches: &ArgMatches, enabled: bool) -> Result<()> {
    let mut gimbal = open_direct_blocking(matches)?;
    info!(
        "Setting standby to {}...",
        if enabled { "enabled" } else { "disabled" }
    );
    as_storm32(&mut *gimbal)?.set_standby_blocking(enabled)?;
    output(format_args!("ok"))?;
    Ok(())
}

pub(super) fn direct_single_axis_blocking(
    matches: &ArgMatches,
    axis: &str,
    value: u16,
) -> Result<()> {
    let mut gimbal = open_direct_blocking(matches)?;
    let storm32 = as_storm32(&mut *gimbal)?;
    if value == 0 {
        info!("Recentering {axis} axis...");
        match axis {
            "pitch" => storm32.recenter_pitch_blocking()?,
            "roll" => storm32.recenter_roll_blocking()?,
            "yaw" => storm32.recenter_yaw_blocking()?,
            _ => return Err(Error::InvalidInput(format!("unknown mount axis {axis}"))),
        }
    } else {
        info!("Setting {axis} to {value}...");
        match axis {
            "pitch" => storm32.set_pitch_blocking(value)?,
            "roll" => storm32.set_roll_blocking(value)?,
            "yaw" => storm32.set_yaw_blocking(value)?,
            _ => return Err(Error::InvalidInput(format!("unknown mount axis {axis}"))),
        }
    }
    output(format_args!("ok"))?;
    Ok(())
}
