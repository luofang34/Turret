use anyhow::{anyhow, Result};
use clap::{Arg, Command};
use log::{info, debug, error, LevelFilter};

mod device_scanner;
mod storm32;

use device_scanner::DeviceScanner;
use storm32::Storm32Gimbal;

fn parse_angles(args: &[String]) -> Result<(f32, f32, f32)> {
    if args.len() == 1 {
        let input = &args[0];
        
        // Try comma-separated format: x,y,z
        if input.contains(',') {
            let parts: Vec<&str> = input.split(',').collect();
            if parts.len() != 3 {
                return Err(anyhow!("Comma format must contain exactly 3 values: pitch,roll,yaw (got {})", parts.len()));
            }
            
            let pitch = parts[0].trim().parse::<f32>()
                .map_err(|_| anyhow!("Invalid pitch value: '{}'", parts[0]))?;
            let roll = parts[1].trim().parse::<f32>()
                .map_err(|_| anyhow!("Invalid roll value: '{}'", parts[1]))?;
            let yaw = parts[2].trim().parse::<f32>()
                .map_err(|_| anyhow!("Invalid yaw value: '{}'", parts[2]))?;
            
            return Ok((pitch, roll, yaw));
        }
        
        // Try colon-separated format: x:y:z
        if input.contains(':') {
            let parts: Vec<&str> = input.split(':').collect();
            if parts.len() != 3 {
                return Err(anyhow!("Colon format must contain exactly 3 values: pitch:roll:yaw (got {})", parts.len()));
            }
            
            let pitch = parts[0].trim().parse::<f32>()
                .map_err(|_| anyhow!("Invalid pitch value: '{}'", parts[0]))?;
            let roll = parts[1].trim().parse::<f32>()
                .map_err(|_| anyhow!("Invalid roll value: '{}'", parts[1]))?;
            let yaw = parts[2].trim().parse::<f32>()
                .map_err(|_| anyhow!("Invalid yaw value: '{}'", parts[2]))?;
            
            return Ok((pitch, roll, yaw));
        }
        
        Err(anyhow!("Invalid single argument format. Use: pitch,roll,yaw or pitch:roll:yaw"))
    } else if args.len() == 3 {
        // Three separate arguments
        let pitch = args[0].parse::<f32>()
            .map_err(|_| anyhow!("Invalid pitch value: '{}'", args[0]))?;
        let roll = args[1].parse::<f32>()
            .map_err(|_| anyhow!("Invalid roll value: '{}'", args[1]))?;
        let yaw = args[2].parse::<f32>()
            .map_err(|_| anyhow!("Invalid yaw value: '{}'", args[2]))?;
        
        Ok((pitch, roll, yaw))
    } else {
        Err(anyhow!("Invalid arguments. Use: 'pitch roll yaw', 'pitch,roll,yaw', or 'pitch:roll:yaw'"))
    }
}

fn main() -> Result<()> {
    let matches = Command::new("turret")
        .version("0.1.0")
        .about("STorM32 Gimbal Control using RC Commands")
        .arg(Arg::new("device").short('d').long("device").help("Serial device path (auto-detects if not specified)").value_name("DEVICE"))
        .arg(Arg::new("verbose").short('v').long("verbose")
            .help("Increase logging verbosity (-v=info, -vv=debug, -vvv=trace)")
            .action(clap::ArgAction::Count))
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
        .subcommand(
            Command::new("center")
                .about("Center gimbal (all axes to 0°)")
        )
        .subcommand(
            Command::new("status")
                .about("Get current gimbal status")
        )
        .subcommand(
            Command::new("version")
                .about("Get gimbal firmware version")
        )
        .subcommand(
            Command::new("version-str")
                .about("Get gimbal firmware version strings")
        )
        .subcommand(
            Command::new("pitch")
                .about("Set pitch axis (700-2300 or 0 to recenter)")
                .arg(Arg::new("value")
                    .help("Pitch value")
                    .required(true)
                    .value_parser(clap::value_parser!(u16)))
        )
        .subcommand(
            Command::new("roll")
                .about("Set roll axis (700-2300 or 0 to recenter)")
                .arg(Arg::new("value")
                    .help("Roll value")
                    .required(true)
                    .value_parser(clap::value_parser!(u16)))
        )
        .subcommand(
            Command::new("yaw")
                .about("Set yaw axis (700-2300 or 0 to recenter)")
                .arg(Arg::new("value")
                    .help("Yaw value")
                    .required(true)
                    .value_parser(clap::value_parser!(u16)))
        )
        .subcommand(
            Command::new("pan-mode")
                .about("Set pan mode (0=off, 1=HOLDHOLDPAN, 2=HOLDHOLDHOLD, 3=PANPANPAN, 4=PANHOLDHOLD, 5=PANHOLDPAN)")
                .arg(Arg::new("mode")
                    .help("Pan mode")
                    .required(true)
                    .value_parser(clap::value_parser!(u8)))
        )
        .subcommand(
            Command::new("standby")
                .about("Set standby mode")
                .arg(Arg::new("enable").help("Enable standby (true/false)").required(true))
        )
        .get_matches();

    // Initialize logger based on verbosity
    let verbose_count = matches.get_count("verbose");
    let log_level = match verbose_count {
        0 => LevelFilter::Warn,
        1 => LevelFilter::Info,
        2 => LevelFilter::Debug,
        _ => LevelFilter::Trace,
    };
    
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn"))
        .filter_level(log_level)
        .init();

    // Determine device path
    let device_path = if let Some(device) = matches.get_one::<String>("device") {
        // Manual device specification
        info!("Using manually specified device: {}", device);
        device.clone()
    } else {
        // Auto-detect STorM32 device
        let scanner = DeviceScanner::new();
        match scanner.find_storm32_device() {
            Some(device) => {
                info!("Auto-detected STorM32 device: {}", device);
                device
            }
            None => {
                // Show available devices to help user
                let devices = scanner.list_all_devices();
                if !devices.is_empty() {
                    error!("No STorM32 device auto-detected. Available devices:");
                    for device in &devices {
                        error!("  {} - {:?}", device.port_name, device.port_type);
                    }
                    error!("Use --device <path> to specify manually.");
                } else {
                    error!("No serial devices found.");
                }
                return Err(anyhow::anyhow!("No STorM32 device found"));
            }
        }
    };

    info!("Using device: {}", device_path);
    let mut gimbal = Storm32Gimbal::new(&device_path)
        .map_err(|e| anyhow::anyhow!("{}", e))?;

    match matches.subcommand() {
        Some(("set", sub_matches)) => {
            let angle_args: Vec<String> = sub_matches.get_many::<String>("angles")
                .unwrap()
                .cloned()
                .collect();
            
            let (pitch, roll, yaw) = parse_angles(&angle_args)?;
            
            info!("Setting angles: pitch={:.1}°, roll={:.1}°, yaw={:.1}°", pitch, roll, yaw);
            gimbal.set_angles(pitch, roll, yaw).map_err(|e| anyhow::anyhow!("{}", e))?;
            info!("Angles set successfully");
        }
        Some(("center", _)) => {
            info!("Centering gimbal...");
            gimbal.set_angles(0.0, 0.0, 0.0).map_err(|e| anyhow::anyhow!("{}", e))?;
            info!("Gimbal centered");
        }
        Some(("status", _)) => {
            debug!("Getting gimbal status...");
            gimbal.update_status().map_err(|e| anyhow::anyhow!("{}", e))?;
            let status = gimbal.get_status();
            println!("Current angles (PV): pitch={:.1}°, roll={:.1}°, yaw={:.1}°",
                     status.pv.pitch, status.pv.roll, status.pv.yaw);
            println!("Target angles (SP):  pitch={:.1}°, roll={:.1}°, yaw={:.1}°",
                     status.sp.pitch, status.sp.roll, status.sp.yaw);
        }
        Some(("version", _)) => {
            debug!("Getting firmware version...");
            match gimbal.get_version() {
                Ok(version) => println!("Firmware version: {}", version),
                Err(e) => error!("Failed to get version: {}", e),
            }
        }
        Some(("version-str", _)) => {
            debug!("Getting firmware version strings...");
            match gimbal.get_version_string() {
                Ok(version) => println!("{}", version),
                Err(e) => error!("Failed to get version string: {}", e),
            }
        }
        Some(("pitch", sub_matches)) => {
            let value = *sub_matches.get_one::<u16>("value").unwrap();

            if value == 0 {
                info!("Recentering pitch axis...");
                match gimbal.recenter_pitch() {
                    Ok(_) => info!("Pitch axis recentered"),
                    Err(e) => error!("Failed to recenter pitch: {}", e),
                }
            } else {
                info!("Setting pitch to {}...", value);
                match gimbal.set_pitch(value) {
                    Ok(_) => info!("Pitch set successfully"),
                    Err(e) => error!("Failed to set pitch: {}", e),
                }
            }
        }
        Some(("roll", sub_matches)) => {
            let value = *sub_matches.get_one::<u16>("value").unwrap();

            if value == 0 {
                info!("Recentering roll axis...");
                match gimbal.recenter_roll() {
                    Ok(_) => info!("Roll axis recentered"),
                    Err(e) => error!("Failed to recenter roll: {}", e),
                }
            } else {
                info!("Setting roll to {}...", value);
                match gimbal.set_roll(value) {
                    Ok(_) => info!("Roll set successfully"),
                    Err(e) => error!("Failed to set roll: {}", e),
                }
            }
        }
        Some(("yaw", sub_matches)) => {
            let value = *sub_matches.get_one::<u16>("value").unwrap();

            if value == 0 {
                info!("Recentering yaw axis...");
                match gimbal.recenter_yaw() {
                    Ok(_) => info!("Yaw axis recentered"),
                    Err(e) => error!("Failed to recenter yaw: {}", e),
                }
            } else {
                info!("Setting yaw to {}...", value);
                match gimbal.set_yaw(value) {
                    Ok(_) => info!("Yaw set successfully"),
                    Err(e) => error!("Failed to set yaw: {}", e),
                }
            }
        }
        Some(("pan-mode", sub_matches)) => {
            let mode = *sub_matches.get_one::<u8>("mode").unwrap();
            
            let mode_name = match mode {
                0 => "OFF",
                1 => "HOLDHOLDPAN", 
                2 => "HOLDHOLDHOLD",
                3 => "PANPANPAN",
                4 => "PANHOLDHOLD",
                5 => "PANHOLDPAN",
                _ => "UNKNOWN",
            };
            
            info!("Setting pan mode to {} ({})...", mode, mode_name);
            match gimbal.set_pan_mode(mode) {
                Ok(_) => info!("Pan mode set successfully"),
                Err(e) => error!("Failed to set pan mode: {}", e),
            }
        }
        Some(("standby", sub_matches)) => {
            let enable_str = sub_matches.get_one::<String>("enable").unwrap();
            let enable = match enable_str.to_lowercase().as_str() {
                "true" | "1" | "on" | "enabled" => true,
                "false" | "0" | "off" | "disabled" => false,
                _ => {
                    error!("Invalid standby value: '{}'. Use true/false, 1/0, on/off, or enabled/disabled", enable_str);
                    return Err(anyhow::anyhow!("Invalid standby value"));
                }
            };
            
            info!("Setting standby mode to {}...", if enable { "enabled" } else { "disabled" });
            match gimbal.set_standby(enable) {
                Ok(_) => info!("Standby mode set successfully"),
                Err(e) => error!("Failed to set standby mode: {}", e),
            }
        }
        _ => {
            println!("STorM32 Gimbal Control - Use --help for available commands");
            println!("Example usage:");
            println!("  turret status");
            println!("  turret set 10.0 0.0 -15.0");
            println!("  turret set -10,0,15");
            println!("  turret set 1.5:-2.3:0");
            println!("  turret center");
        }
    }

    Ok(())
}
