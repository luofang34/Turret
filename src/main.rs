//! `turret` CLI: thin wrapper that prefers the daemon's IPC socket when
//! one is reachable, falling back to a one-shot direct-device path
//! otherwise.
//!
//! ## Routing model
//!
//! Each subcommand has two execution paths:
//!
//! 1. **IPC-routed** — `pkg-default /tmp/turret.sock` (override with
//!    `--socket <path>`). When the socket exists and accepts a
//!    connection, the CLI builds the equivalent JSON command, sends it
//!    over the Unix socket, prints the response, and exits. The daemon
//!    handles arbitration, calibration, and primary-control on every
//!    command — same code path as a MAVLink GCS would take.
//!
//! 2. **Direct-device** — when the socket isn't reachable (no daemon
//!    running), the CLI opens the serial device itself and runs the
//!    command in process. Calibration is loaded from the same XDG state
//!    file the daemon would read, so behavior matches across the two
//!    paths within IMU noise.
//!
//! `--daemon` is the only mode that opens the device for a long-running
//! lifetime; everything else is one-shot.
//!
//! Errors propagate as [`turret::error::Error`] to `main`'s return
//! value. Notably, [`Error::DeviceBusy`] (rather than the generic
//! [`Error::NotDetected`]) surfaces when a daemon is holding the port —
//! the message tells the operator to use IPC or stop the daemon.

use clap::{Arg, ArgMatches, Command};
use serde_json::Value as Json;
use std::path::Path;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::UnixStream;
use tracing::{debug, error, info};
use tracing_subscriber::filter::LevelFilter;

use turret::daemon;
use turret::daemon::calibration::Calibration;
use turret::device_scanner::DeviceScanner;
use turret::error::{Error, Result};
use turret::gimbal::{self, GimbalDevice};
use turret::protocols::storm32_rc::Storm32RC;

/// Default path the daemon binds and the CLI dials.
const DEFAULT_SOCKET_PATH: &str = "/tmp/turret.sock";

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

/// Outcome of the IPC dial: either we got a response back, or the
/// socket isn't reachable (no daemon listening). "Not reachable" is the
/// common case — drives the fallback into direct-device mode. Other
/// errors propagate as `Error` so the operator sees the actual cause.
enum IpcOutcome {
    Reached(Json),
    NotReachable,
}

/// Connect to the daemon's Unix socket, send one JSON command, read the
/// JSON response. Returns [`IpcOutcome::NotReachable`] when the socket
/// file is missing or refuses connections (typical "no daemon"); other
/// errors (malformed response, write/read failure on a connected
/// socket) bubble up.
async fn try_ipc(socket_path: &Path, payload: &Json) -> Result<IpcOutcome> {
    use std::io::ErrorKind;

    let stream = match UnixStream::connect(socket_path).await {
        Ok(s) => s,
        Err(e) if matches!(e.kind(), ErrorKind::NotFound | ErrorKind::ConnectionRefused) => {
            return Ok(IpcOutcome::NotReachable);
        }
        Err(e) => return Err(e.into()),
    };

    let (reader, mut writer) = stream.into_split();
    let mut bytes = serde_json::to_vec(payload)?;
    bytes.push(b'\n');
    writer.write_all(&bytes).await?;
    writer.flush().await?;

    let mut reader = BufReader::new(reader);
    let mut line = String::new();
    reader.read_line(&mut line).await?;
    let response: Json = serde_json::from_str(line.trim())?;
    Ok(IpcOutcome::Reached(response))
}

/// Pretty-print an IPC response for terminal use. Errors come through
/// the daemon as `{"status":"error","message":"..."}`; success
/// responses are `{"status":"ok"}` or `{"status":"ok","data":{...}}`.
/// We special-case `data` shapes the user actually reads; for anything
/// else fall back to formatted JSON so even unfamiliar response shapes
/// are legible.
fn print_ipc_response(cmd: &str, response: &Json) -> Result<()> {
    let status = response.get("status").and_then(Json::as_str);
    match status {
        Some("error") => {
            let msg = response
                .get("message")
                .and_then(Json::as_str)
                .unwrap_or("(no message)");
            Err(Error::Daemon(format!("ipc {cmd}: {msg}")))
        }
        Some("ok") => {
            if let Some(data) = response.get("data") {
                pretty_print_data(cmd, data);
            } else {
                println!("ok");
            }
            Ok(())
        }
        _ => Err(Error::Daemon(format!(
            "ipc {cmd}: unexpected response: {response}"
        ))),
    }
}

/// Format the `data` payload of an OK response. Per-command pretty
/// shapes for the most-used responses, plus a JSON-pretty fallback so
/// we don't lose information for anything unrecognized.
fn pretty_print_data(cmd: &str, data: &Json) {
    match cmd {
        "status" => {
            let p = data.get("pitch").and_then(Json::as_f64).unwrap_or(0.0);
            let r = data.get("roll").and_then(Json::as_f64).unwrap_or(0.0);
            let y = data.get("yaw").and_then(Json::as_f64).unwrap_or(0.0);
            let pan = data
                .get("pan_mode")
                .and_then(Json::as_str)
                .unwrap_or("(unknown)");
            let standby = data.get("standby").and_then(Json::as_bool).unwrap_or(false);
            let off = data.get("yaw_offset_deg").and_then(Json::as_f64);
            let fw = data
                .get("firmware_version")
                .and_then(Json::as_str)
                .unwrap_or("(unknown)");
            println!("pitch={p:.2}°  roll={r:.2}°  yaw={y:.2}°");
            println!("pan_mode={pan}  standby={standby}");
            if let Some(off) = off {
                println!("yaw_offset_deg={off:.3}");
            }
            println!("firmware: {fw}");
        }
        "version" => {
            if let Some(v) = data.get("version").and_then(Json::as_str) {
                println!("{v}");
            } else {
                println!("{data}");
            }
        }
        _ => {
            // Last-resort fallback. `serde_json::to_string_pretty` on an
            // already-decoded `Value` is infallible in practice; if it
            // somehow fails, fall back to the Display impl.
            match serde_json::to_string_pretty(data) {
                Ok(s) => println!("{s}"),
                Err(_) => println!("{data}"),
            }
        }
    }
}

/// Run a CLI subcommand: try IPC first, fall back to a direct-device
/// closure on `IpcOutcome::NotReachable`. Other IPC errors (malformed
/// response, socket failure mid-exchange) propagate as `Error` so the
/// operator sees what actually went wrong instead of a fallback that
/// might silently mask a real bug.
async fn ipc_or_direct<F>(socket_path: &Path, cmd: &str, payload: Json, direct: F) -> Result<()>
where
    F: FnOnce() -> Result<()>,
{
    match try_ipc(socket_path, &payload).await? {
        IpcOutcome::Reached(response) => {
            debug!("Routed `{cmd}` through IPC at {}", socket_path.display());
            print_ipc_response(cmd, &response)
        }
        IpcOutcome::NotReachable => {
            debug!(
                "No daemon at {}; running `{cmd}` against the device directly",
                socket_path.display()
            );
            direct()
        }
    }
}

/// Resolve which serial device to open in direct mode. Honors `--device`
/// if given; otherwise falls back to the auto-detect scanner. Same
/// behavior as before — extracted into a helper so each direct-path
/// branch stays focused.
fn resolve_device_path(matches: &ArgMatches) -> Result<String> {
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

// Single-threaded tokio runtime. The daemon's actual concurrency need
// is tiny — IPC accept loop, MAVLink RX, three broadcast loops, attitude
// poll, reconnect supervisor — all I/O-bound. Multi-thread costs ~3 MB
// RSS in per-thread arenas + worker stacks for no measurable throughput
// win on this workload. Sync serial I/O still goes through
// `tokio::task::spawn_blocking` (see `daemon::gimbal_handle`), which on
// `current_thread` runs on the same dedicated blocking pool.
#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<()> {
    let matches = Command::new("turret")
        .version(env!("CARGO_PKG_VERSION"))
        .about("STorM32 Gimbal Control using RC Commands")
        .arg(Arg::new("device").short('d').long("device").help("Serial device path (auto-detects if not specified). Used only when the IPC socket is not reachable.").value_name("DEVICE"))
        .arg(Arg::new("socket")
            .short('s')
            .long("socket")
            .help("Path to the daemon's IPC socket (default: /tmp/turret.sock). When reachable, subcommands route through the daemon; otherwise they run direct-to-device.")
            .value_name("PATH"))
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
        .subcommand(Command::new("version-str").about("Get gimbal firmware version strings (direct-device only)"))
        .subcommand(
            Command::new("pitch")
                .about("Set pitch axis (RC value 700-2300 or 0 to recenter; direct-device only)")
                .arg(Arg::new("value").help("Pitch value").required(true).value_parser(clap::value_parser!(u16)))
        )
        .subcommand(
            Command::new("roll")
                .about("Set roll axis (RC value 700-2300 or 0 to recenter; direct-device only)")
                .arg(Arg::new("value").help("Roll value").required(true).value_parser(clap::value_parser!(u16)))
        )
        .subcommand(
            Command::new("yaw")
                .about("Set yaw axis (RC value 700-2300 or 0 to recenter; direct-device only)")
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

    let socket_path = matches
        .get_one::<String>("socket")
        .cloned()
        .unwrap_or_else(|| DEFAULT_SOCKET_PATH.to_string());
    let socket_path = std::path::PathBuf::from(socket_path);

    match matches.subcommand() {
        Some(("set", sub)) => {
            let angle_args: Vec<String> = sub
                .get_many::<String>("angles")
                .ok_or_else(|| Error::InvalidInput("missing 'angles' argument".to_string()))?
                .cloned()
                .collect();
            let (pitch, roll, yaw) = parse_angles(&angle_args)?;
            let payload = serde_json::json!({
                "cmd": "set", "pitch": pitch, "roll": roll, "yaw": yaw
            });
            ipc_or_direct(&socket_path, "set", payload, || {
                direct_set(&matches, pitch, roll, yaw)
            })
            .await
        }
        Some(("center", _)) => {
            ipc_or_direct(
                &socket_path,
                "center",
                serde_json::json!({"cmd": "center"}),
                || direct_set(&matches, 0.0, 0.0, 0.0),
            )
            .await
        }
        Some(("status", _)) => {
            ipc_or_direct(
                &socket_path,
                "status",
                serde_json::json!({"cmd": "status"}),
                || direct_status(&matches),
            )
            .await
        }
        Some(("version", _)) => {
            ipc_or_direct(
                &socket_path,
                "version",
                serde_json::json!({"cmd": "version"}),
                || direct_version(&matches),
            )
            .await
        }
        Some(("pan-mode", sub)) => {
            let mode = *required::<u8>(sub, "mode")?;
            ipc_or_direct(
                &socket_path,
                "pan_mode",
                serde_json::json!({"cmd": "pan_mode", "mode": mode}),
                || direct_pan_mode(&matches, mode),
            )
            .await
        }
        Some(("standby", sub)) => {
            let enabled = parse_standby(required::<String>(sub, "enable")?)?;
            ipc_or_direct(
                &socket_path,
                "standby",
                serde_json::json!({"cmd": "standby", "enabled": enabled}),
                || direct_standby(&matches, enabled),
            )
            .await
        }
        // Direct-only commands. The daemon doesn't expose protocol-specific
        // single-axis RC writes or the long version-string read on its IPC
        // schema (see PR #28). When the daemon is running, these will bail
        // with `Error::DeviceBusy` after the EBUSY-detection improvement
        // — the message tells the operator to stop the daemon to use them.
        Some(("version-str", _)) => direct_version_str(&matches),
        Some((axis @ ("pitch" | "roll" | "yaw"), sub)) => {
            let value = *required::<u16>(sub, "value")?;
            direct_single_axis(&matches, axis, value)
        }
        _ => {
            info!("STorM32 Gimbal Control — use --help for available commands");
            info!("Examples: turret status; turret set 10,0,-15; turret center");
            Ok(())
        }
    }
}

/// Open the device for a one-shot direct-mode invocation. Lookup +
/// detection is the same as before; `DeviceBusy` is what surfaces here
/// when a daemon is holding the port.
fn open_direct(matches: &ArgMatches) -> Result<Box<dyn GimbalDevice>> {
    let device_path = resolve_device_path(matches)?;
    info!("Using device: {}", device_path);
    let gimbal = gimbal::detect_gimbal(&device_path)?;
    info!("Detected protocol: {}", gimbal.protocol_name());
    Ok(gimbal)
}

fn direct_set(matches: &ArgMatches, pitch: f32, roll: f32, yaw: f32) -> Result<()> {
    let mut gimbal = open_direct(matches)?;
    info!("Setting angles: pitch={pitch:.1}°, roll={roll:.1}°, yaw={yaw:.1}°");
    // Calibration is read-side correction (mirrors GimbalHandle::set_attitude
    // semantics from PR #43): pass the operator-frame yaw through to the
    // device unchanged so the gimbal physically lands at the asked-for
    // angle; only `direct_status` subtracts the offset on readback.
    gimbal.set_attitude(pitch, roll, yaw)?;
    println!("ok");
    Ok(())
}

fn direct_status(matches: &ArgMatches) -> Result<()> {
    let mut gimbal = open_direct(matches)?;
    let attitude = gimbal.get_attitude()?;
    // Apply the persisted yaw zero-offset so direct-mode `status` reads
    // the same value the daemon would publish via IPC. Missing /
    // malformed calibration file silently uses zero offset.
    let cal = Calibration::load();
    let yaw = attitude.yaw - cal.yaw_offset_deg;
    println!(
        "pitch={:.2}°  roll={:.2}°  yaw={:.2}°",
        attitude.pitch, attitude.roll, yaw
    );
    if cal.yaw_offset_deg != 0.0 {
        println!("yaw_offset_deg={:.3}", cal.yaw_offset_deg);
    }
    Ok(())
}

fn direct_version(matches: &ArgMatches) -> Result<()> {
    let mut gimbal = open_direct(matches)?;
    let version = gimbal.get_version()?;
    println!("{version}");
    Ok(())
}

fn direct_version_str(matches: &ArgMatches) -> Result<()> {
    let mut gimbal = open_direct(matches)?;
    let storm32 = as_storm32(&mut *gimbal)?;
    let version = storm32.get_version_string()?;
    println!("{version}");
    Ok(())
}

fn direct_pan_mode(matches: &ArgMatches, mode: u8) -> Result<()> {
    let mut gimbal = open_direct(matches)?;
    info!("Setting pan mode to {mode}...");
    gimbal.set_pan_mode(mode)?;
    println!("ok");
    Ok(())
}

fn direct_standby(matches: &ArgMatches, enabled: bool) -> Result<()> {
    let mut gimbal = open_direct(matches)?;
    info!(
        "Setting standby to {}...",
        if enabled { "enabled" } else { "disabled" }
    );
    gimbal.set_standby(enabled)?;
    println!("ok");
    Ok(())
}

fn direct_single_axis(matches: &ArgMatches, axis: &str, value: u16) -> Result<()> {
    let mut gimbal = open_direct(matches)?;
    let storm32 = as_storm32(&mut *gimbal)?;
    if value == 0 {
        info!("Recentering {axis} axis...");
        match axis {
            "pitch" => storm32.recenter_pitch()?,
            "roll" => storm32.recenter_roll()?,
            "yaw" => storm32.recenter_yaw()?,
            _ => unreachable!("guarded by outer match"),
        }
    } else {
        info!("Setting {axis} to {value}...");
        match axis {
            "pitch" => storm32.set_pitch(value)?,
            "roll" => storm32.set_roll(value)?,
            "yaw" => storm32.set_yaw(value)?,
            _ => unreachable!("guarded by outer match"),
        }
    }
    println!("ok");
    Ok(())
}
