//! Command-line routing and output.

mod arguments;
mod device;
mod ipc;

use arguments::{parse_angles, parse_standby, required};
use clap::ArgMatches;
use device::*;
use ipc::ipc_or_direct;
use std::io::Write;
use std::path::Path;
use tracing::info;
use tracing_subscriber::filter::LevelFilter;
use turret::daemon;
use turret::error::{Error, Result};

const DEFAULT_SOCKET_PATH: &str = "/tmp/turret.sock";

pub(super) async fn run() -> Result<()> {
    let matches = arguments::command().get_matches();
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

    dispatch(&matches, &socket_path).await
}

async fn dispatch(matches: &ArgMatches, socket_path: &Path) -> Result<()> {
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
            ipc_or_direct(socket_path, "set", payload, || {
                direct_set_blocking(matches, pitch, roll, yaw)
            })
            .await
        }
        Some(("center", _)) => {
            ipc_or_direct(
                socket_path,
                "center",
                serde_json::json!({"cmd": "center"}),
                || direct_center_blocking(matches),
            )
            .await
        }
        Some(("status", _)) => {
            ipc_or_direct(
                socket_path,
                "status",
                serde_json::json!({"cmd": "status"}),
                || direct_status_blocking(matches),
            )
            .await
        }
        Some(("version", _)) => {
            ipc_or_direct(
                socket_path,
                "version",
                serde_json::json!({"cmd": "version"}),
                || direct_version_blocking(matches),
            )
            .await
        }
        Some(("pan-mode", sub)) => {
            let mode = *required::<u8>(sub, "mode")?;
            ipc_or_direct(
                socket_path,
                "pan_mode",
                serde_json::json!({"cmd": "pan_mode", "mode": mode}),
                || direct_pan_mode_blocking(matches, mode),
            )
            .await
        }
        Some(("standby", sub)) => {
            let enabled = parse_standby(required::<String>(sub, "enable")?)?;
            ipc_or_direct(
                socket_path,
                "standby",
                serde_json::json!({"cmd": "standby", "enabled": enabled}),
                || direct_standby_blocking(matches, enabled),
            )
            .await
        }
        Some(("version-str", _)) => direct_version_str_blocking(matches),
        Some((axis @ ("pitch" | "roll" | "yaw"), sub)) => {
            let value = *required::<u16>(sub, "value")?;
            direct_single_axis_blocking(matches, axis, value)
        }
        _ => {
            info!("STorM32 Gimbal Control — use --help for available commands");
            info!("Examples: turret status; turret set 10,0,-15; turret center");
            Ok(())
        }
    }
}

fn output(args: std::fmt::Arguments<'_>) -> Result<()> {
    let mut stdout = std::io::stdout().lock();
    stdout.write_fmt(args)?;
    stdout.write_all(b"\n")?;
    Ok(())
}
