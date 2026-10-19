use clap::{Arg, ArgMatches, Command};
use turret::error::{Error, Result};

pub(super) fn required<'a, T: Send + Sync + Clone + 'static>(
    matches: &'a ArgMatches,
    name: &'static str,
) -> Result<&'a T> {
    matches
        .get_one::<T>(name)
        .ok_or_else(|| Error::InvalidInput(format!("missing required argument '{name}'")))
}

pub(super) fn parse_angles(args: &[String]) -> Result<(f32, f32, f32)> {
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

pub(super) fn parse_standby(value: &str) -> Result<bool> {
    match value.to_lowercase().as_str() {
        "true" | "1" | "on" | "enabled" => Ok(true),
        "false" | "0" | "off" | "disabled" => Ok(false),
        other => Err(Error::InvalidInput(format!(
            "invalid standby value '{other}'; use true/false, 1/0, on/off, or enabled/disabled"
        ))),
    }
}

pub(super) fn command() -> Command {
    Command::new("turret")
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
        .subcommand(Command::new("center").about("Move to the device's declared recenter pose"))
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
}

#[cfg(test)]
mod tests;
