//! `turret` CLI: thin wrapper over [`turret::gimbal`] / [`turret::daemon`].
//!
//! In one-shot mode (the default), it parses a subcommand, opens the
//! gimbal once, runs the operation, prints the result, and exits. With
//! `--daemon`, it instead loads `turret.yaml` and hands off to
//! [`turret::daemon::run_daemon`]. Errors propagate as
//! [`turret::error::Error`] to `main`'s return value, so the process
//! exit code reflects the failure shape.

use clap::{Arg, ArgMatches, Command};
use tracing::{debug, error, info};
use tracing_subscriber::filter::LevelFilter;

use turret::daemon;
use turret::device_scanner::DeviceScanner;
use turret::error::{Error, Result};
use turret::gimbal::{self, GimbalDevice};
use turret::protocols::storm32_rc::Storm32RC;

/// Borrow the boxed gimbal as a `Storm32RC` for protocol-specific commands
/// the trait doesn't expose.
fn as_storm32(gimbal: &mut dyn GimbalDevice) -> Result<&mut Storm32RC> {
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

/// Pull a required argument off a clap matcher. Surfaces a typed error
/// instead of unwrapping — clap normally guarantees presence for
/// `.required(true)` args, so this branch reads as "clap regressed".
fn required<'a, T: Send + Sync + Clone + 'static>(
    matches: &'a ArgMatches,
    name: &'static str,
) -> Result<&'a T> {
    matches
        .get_one::<T>(name)
        .ok_or_else(|| Error::InvalidInput(format!("missing required argument '{name}'")))
}

fn parse_angles(args: &[String]) -> Result<(f32, f32, f32)> {
    let parse = |raw: &str, axis: &str| -> Result<f32> {
        raw.trim()
            .parse::<f32>()
            .map_err(|_| Error::InvalidInput(format!("invalid {axis} value: '{raw}'")))
    };

    if args.len() == 1 {
        let input = &args[0];
        let separator = if input.contains(',') {
            ','
        } else if input.contains(':') {
            ':'
        } else {
            return Err(Error::InvalidInput(
                "single-argument form must use ',' or ':' between axes (e.g. 10,0,-15)".to_string(),
            ));
        };

        let parts: Vec<&str> = input.split(separator).collect();
        if parts.len() != 3 {
            return Err(Error::InvalidInput(format!(
                "format must contain exactly 3 values: pitch{separator}roll{separator}yaw (got {})",
                parts.len()
            )));
        }
        Ok((
            parse(parts[0], "pitch")?,
            parse(parts[1], "roll")?,
            parse(parts[2], "yaw")?,
        ))
    } else if args.len() == 3 {
        Ok((
            parse(&args[0], "pitch")?,
            parse(&args[1], "roll")?,
            parse(&args[2], "yaw")?,
        ))
    } else {
        Err(Error::InvalidInput(
            "expected 'pitch roll yaw', 'pitch,roll,yaw', or 'pitch:roll:yaw'".to_string(),
        ))
    }
}

fn parse_standby(value: &str) -> Result<bool> {
    match value.to_lowercase().as_str() {
        "true" | "1" | "on" | "enabled" => Ok(true),
        "false" | "0" | "off" | "disabled" => Ok(false),
        other => Err(Error::InvalidInput(format!(
            "invalid standby value '{other}'; use true/false, 1/0, on/off, or enabled/disabled"
        ))),
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    let matches = Command::new("turret")
        .version(env!("CARGO_PKG_VERSION"))
        .about("STorM32 Gimbal Control using RC Commands")
        .arg(Arg::new("device").short('d').long("device").help("Serial device path (auto-detects if not specified)").value_name("DEVICE"))
        .arg(Arg::new("verbose").short('v').long("verbose")
            .help("Increase logging verbosity (-v=info, -vv=debug, -vvv=trace)")
            .action(clap::ArgAction::Count))
        .arg(Arg::new("daemon")
            .long("daemon")
            .help("Run in daemon mode (background service with IPC and MAVLink)")
            .action(clap::ArgAction::SetTrue))
        .arg(Arg::new("config")
            .short('c')
            .long("config")
            .help("Path to configuration file (daemon mode only)")
            .value_name("CONFIG"))
        .subcommand(
            Command::new("set")
                .about("Set gimbal angles")
                .arg(Arg::new("angles")
                    .help("Angles: 'pitch roll yaw', 'pitch,roll,yaw', or 'pitch:roll:yaw'")
                    .required(true)
                    .num_args(1..=3)
                    .allow_hyphen_values(true)
                    .value_name("ANGLES"))
        )
        .subcommand(Command::new("center").about("Center gimbal (all axes to 0°)"))
        .subcommand(Command::new("status").about("Get current gimbal status"))
        .subcommand(Command::new("version").about("Get gimbal firmware version"))
        .subcommand(Command::new("version-str").about("Get gimbal firmware version strings"))
        .subcommand(
            Command::new("pitch")
                .about("Set pitch axis (700-2300 or 0 to recenter)")
                .arg(Arg::new("value").help("Pitch value").required(true).value_parser(clap::value_parser!(u16)))
        )
        .subcommand(
            Command::new("roll")
                .about("Set roll axis (700-2300 or 0 to recenter)")
                .arg(Arg::new("value").help("Roll value").required(true).value_parser(clap::value_parser!(u16)))
        )
        .subcommand(
            Command::new("yaw")
                .about("Set yaw axis (700-2300 or 0 to recenter)")
                .arg(Arg::new("value").help("Yaw value").required(true).value_parser(clap::value_parser!(u16)))
        )
        .subcommand(
            Command::new("pan-mode")
                .about("Set pan mode (0=off, 1=HOLDHOLDPAN, 2=HOLDHOLDHOLD, 3=PANPANPAN, 4=PANHOLDHOLD, 5=PANHOLDPAN)")
                .arg(Arg::new("mode").help("Pan mode").required(true).value_parser(clap::value_parser!(u8)))
        )
        .subcommand(
            Command::new("standby")
                .about("Set standby mode")
                .arg(Arg::new("enable").help("Enable standby (true/false)").required(true))
        )
        .get_matches();

    // Verbosity: -v / -vv / -vvv picks the global level floor, RUST_LOG
    // env var can further tune per-target filters.
    let default_level = match matches.get_count("verbose") {
        0 => LevelFilter::WARN,
        1 => LevelFilter::INFO,
        2 => LevelFilter::DEBUG,
        _ => LevelFilter::TRACE,
    };
    let env_filter = tracing_subscriber::EnvFilter::builder()
        .with_default_directive(default_level.into())
        .with_env_var("RUST_LOG")
        .from_env_lossy();
    tracing_subscriber::fmt().with_env_filter(env_filter).init();

    if matches.get_flag("daemon") {
        info!("Starting in daemon mode...");
        let config_path = matches.get_one::<String>("config").cloned();
        return daemon::run_daemon(config_path).await;
    }

    let device_path = if let Some(device) = matches.get_one::<String>("device") {
        info!("Using manually specified device: {}", device);
        device.clone()
    } else {
        let scanner = DeviceScanner::new();
        match scanner.find_storm32_device() {
            Some(device) => {
                info!("Auto-detected STorM32 device: {}", device);
                device
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
                return Err(Error::NotDetected {
                    device: "auto".to_string(),
                    tried: "STorM32 USB CDC".to_string(),
                });
            }
        }
    };

    info!("Using device: {}", device_path);
    let mut gimbal = gimbal::detect_gimbal(&device_path)?;
    info!("Detected protocol: {}", gimbal.protocol_name());

    match matches.subcommand() {
        Some(("set", sub)) => {
            let angle_args: Vec<String> = sub
                .get_many::<String>("angles")
                .ok_or_else(|| Error::InvalidInput("missing 'angles' argument".to_string()))?
                .cloned()
                .collect();
            let (pitch, roll, yaw) = parse_angles(&angle_args)?;
            info!("Setting angles: pitch={pitch:.1}°, roll={roll:.1}°, yaw={yaw:.1}°");
            gimbal.set_attitude(pitch, roll, yaw)?;
            info!("Angles set successfully");
        }
        Some(("center", _)) => {
            info!("Centering gimbal...");
            gimbal.center()?;
            info!("Gimbal centered");
        }
        Some(("status", _)) => {
            debug!("Getting gimbal status...");
            let attitude = gimbal.get_attitude()?;
            info!(
                "Current angles: pitch={:.1}°, roll={:.1}°, yaw={:.1}°",
                attitude.pitch, attitude.roll, attitude.yaw
            );
        }
        Some(("version", _)) => {
            debug!("Getting firmware version...");
            let version = gimbal.get_version()?;
            info!("Firmware version: {version}");
        }
        Some(("version-str", _)) => {
            debug!("Getting firmware version strings...");
            let storm32 = as_storm32(&mut *gimbal)?;
            let version = storm32.get_version_string()?;
            info!("{version}");
        }
        Some((axis @ ("pitch" | "roll" | "yaw"), sub)) => {
            let value = *required::<u16>(sub, "value")?;
            let storm32 = as_storm32(&mut *gimbal)?;
            if value == 0 {
                info!("Recentering {axis} axis...");
                match axis {
                    "pitch" => storm32.recenter_pitch()?,
                    "roll" => storm32.recenter_roll()?,
                    "yaw" => storm32.recenter_yaw()?,
                    _ => unreachable!("guarded by outer match"),
                }
                info!("{axis} axis recentered");
            } else {
                info!("Setting {axis} to {value}...");
                match axis {
                    "pitch" => storm32.set_pitch(value)?,
                    "roll" => storm32.set_roll(value)?,
                    "yaw" => storm32.set_yaw(value)?,
                    _ => unreachable!("guarded by outer match"),
                }
                info!("{axis} set successfully");
            }
        }
        Some(("pan-mode", sub)) => {
            let mode = *required::<u8>(sub, "mode")?;
            let mode_name = match mode {
                0 => "OFF",
                1 => "HOLDHOLDPAN",
                2 => "HOLDHOLDHOLD",
                3 => "PANPANPAN",
                4 => "PANHOLDHOLD",
                5 => "PANHOLDPAN",
                _ => "UNKNOWN",
            };
            info!("Setting pan mode to {mode} ({mode_name})...");
            gimbal.set_pan_mode(mode)?;
            info!("Pan mode set successfully");
        }
        Some(("standby", sub)) => {
            let enable = parse_standby(required::<String>(sub, "enable")?)?;
            info!(
                "Setting standby mode to {}...",
                if enable { "enabled" } else { "disabled" }
            );
            gimbal.set_standby(enable)?;
            info!("Standby mode set successfully");
        }
        _ => {
            info!("STorM32 Gimbal Control — use --help for available commands");
            info!("Examples: turret status; turret set 10,0,-15; turret center");
        }
    }

    Ok(())
}
