use anyhow::{anyhow, Result};
use clap::{Arg, Command};

mod device_scanner;
mod storm32;

use device_scanner::DeviceScanner;
use storm32::Storm32Gimbal;

fn parse_angles(args: &[String]) -> Result<(f32, f32, f32)> {
    // Check if first argument looks like an array format [x,y,z]
    if args.len() == 1 && args[0].starts_with('[') && args[0].ends_with(']') {
        let content = &args[0][1..args[0].len()-1]; // Remove [ and ]
        let parts: Vec<&str> = content.split(',').collect();
        
        if parts.len() != 3 {
            return Err(anyhow!("Array format must contain exactly 3 values: [pitch,roll,yaw]"));
        }
        
        let pitch = parts[0].trim().parse::<f32>()
            .map_err(|_| anyhow!("Invalid pitch value: {}", parts[0]))?;
        let roll = parts[1].trim().parse::<f32>()
            .map_err(|_| anyhow!("Invalid roll value: {}", parts[1]))?;
        let yaw = parts[2].trim().parse::<f32>()
            .map_err(|_| anyhow!("Invalid yaw value: {}", parts[2]))?;
        
        Ok((pitch, roll, yaw))
    } else if args.len() == 3 {
        // Traditional three separate arguments
        let pitch = args[0].parse::<f32>()
            .map_err(|_| anyhow!("Invalid pitch value: {}", args[0]))?;
        let roll = args[1].parse::<f32>()
            .map_err(|_| anyhow!("Invalid roll value: {}", args[1]))?;
        let yaw = args[2].parse::<f32>()
            .map_err(|_| anyhow!("Invalid yaw value: {}", args[2]))?;
        
        Ok((pitch, roll, yaw))
    } else {
        Err(anyhow!("Invalid arguments. Use either: 'set <pitch> <roll> <yaw>' or 'set \"[pitch,roll,yaw]\"'"))
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    let matches = Command::new("gimbal")
        .version("0.1.0")
        .about("STorM32 Gimbal Control using RC Commands")
        .arg(Arg::new("device").short('d').long("device").help("Serial device path").value_name("DEVICE"))
        .arg(Arg::new("auto").short('a').long("auto").help("Auto-detect device").action(clap::ArgAction::SetTrue))
        .subcommand(
            Command::new("set")
                .about("Set gimbal angles")
                .arg(Arg::new("angles")
                    .help("Angles: either 'pitch roll yaw' or '[pitch,roll,yaw]'")
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

    // Determine device path
    let device_path = if let Some(device) = matches.get_one::<String>("device") {
        device.clone()
    } else if matches.get_flag("auto") {
        let scanner = DeviceScanner::new();
        scanner.find_storm32_device()
            .ok_or_else(|| anyhow::anyhow!("No STorM32 device found"))?
    } else {
        let scanner = DeviceScanner::new();
        let devices = scanner.list_all_devices();
        println!("Available devices:");
        for device in &devices {
            println!("  {} - {:?}", device.port_name, device.port_type);
        }
        
        if let Some(storm32_device) = scanner.find_storm32_device() {
            println!("\nAuto-detected STorM32 device: {}", storm32_device);
            storm32_device
        } else {
            return Err(anyhow::anyhow!("No STorM32 device found. Use --device to specify manually or --auto for auto-detection."));
        }
    };

    println!("Using device: {}", device_path);
    let mut gimbal = Storm32Gimbal::new(&device_path)?;

    match matches.subcommand() {
        Some(("set", sub_matches)) => {
            let angle_args: Vec<String> = sub_matches.get_many::<String>("angles")
                .unwrap()
                .cloned()
                .collect();
            
            let (pitch, roll, yaw) = parse_angles(&angle_args)?;
            
            println!("Setting angles: pitch={:.1}°, roll={:.1}°, yaw={:.1}°", pitch, roll, yaw);
            gimbal.set_angles(pitch, roll, yaw).await?;
            println!("Angles set successfully");
        }
        Some(("center", _)) => {
            println!("Centering gimbal...");
            gimbal.set_angles(0.0, 0.0, 0.0).await?;
            println!("Gimbal centered");
        }
        Some(("status", _)) => {
            println!("Getting gimbal status...");
            gimbal.update_status().await?;
            let status = gimbal.get_status();
            println!("Current angles (PV): pitch={:.1}°, roll={:.1}°, yaw={:.1}°", 
                     status.pv.pitch, status.pv.roll, status.pv.yaw);
            println!("Target angles (SP):  pitch={:.1}°, roll={:.1}°, yaw={:.1}°", 
                     status.sp.pitch, status.sp.roll, status.sp.yaw);
        }
        Some(("version", _)) => {
            println!("Getting firmware version...");
            match gimbal.get_version().await {
                Ok(version) => println!("Firmware version: {}", version),
                Err(e) => println!("Failed to get version: {}", e),
            }
        }
        Some(("version-str", _)) => {
            println!("Getting firmware version strings...");
            match gimbal.get_version_string().await {
                Ok(version) => println!("{}", version),
                Err(e) => println!("Failed to get version string: {}", e),
            }
        }
        Some(("pitch", sub_matches)) => {
            let value = *sub_matches.get_one::<u16>("value").unwrap();
            
            if value == 0 {
                println!("Recentering pitch axis...");
                match gimbal.recenter_pitch().await {
                    Ok(_) => println!("Pitch axis recentered"),
                    Err(e) => println!("Failed to recenter pitch: {}", e),
                }
            } else {
                println!("Setting pitch to {}...", value);
                match gimbal.set_pitch(value).await {
                    Ok(_) => println!("Pitch set successfully"),
                    Err(e) => println!("Failed to set pitch: {}", e),
                }
            }
        }
        Some(("roll", sub_matches)) => {
            let value = *sub_matches.get_one::<u16>("value").unwrap();
            
            if value == 0 {
                println!("Recentering roll axis...");
                match gimbal.recenter_roll().await {
                    Ok(_) => println!("Roll axis recentered"),
                    Err(e) => println!("Failed to recenter roll: {}", e),
                }
            } else {
                println!("Setting roll to {}...", value);
                match gimbal.set_roll(value).await {
                    Ok(_) => println!("Roll set successfully"),
                    Err(e) => println!("Failed to set roll: {}", e),
                }
            }
        }
        Some(("yaw", sub_matches)) => {
            let value = *sub_matches.get_one::<u16>("value").unwrap();
            
            if value == 0 {
                println!("Recentering yaw axis...");
                match gimbal.recenter_yaw().await {
                    Ok(_) => println!("Yaw axis recentered"),
                    Err(e) => println!("Failed to recenter yaw: {}", e),
                }
            } else {
                println!("Setting yaw to {}...", value);
                match gimbal.set_yaw(value).await {
                    Ok(_) => println!("Yaw set successfully"),
                    Err(e) => println!("Failed to set yaw: {}", e),
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
            
            println!("Setting pan mode to {} ({})...", mode, mode_name);
            match gimbal.set_pan_mode(mode).await {
                Ok(_) => println!("Pan mode set successfully"),
                Err(e) => println!("Failed to set pan mode: {}", e),
            }
        }
        Some(("standby", sub_matches)) => {
            let enable_str = sub_matches.get_one::<String>("enable").unwrap();
            let enable = match enable_str.to_lowercase().as_str() {
                "true" | "1" | "on" | "enabled" => true,
                "false" | "0" | "off" | "disabled" => false,
                _ => {
                    println!("Invalid standby value. Use true/false, 1/0, on/off, or enabled/disabled");
                    return Ok(());
                }
            };
            
            println!("Setting standby mode to {}...", if enable { "enabled" } else { "disabled" });
            match gimbal.set_standby(enable).await {
                Ok(_) => println!("Standby mode set successfully"),
                Err(e) => println!("Failed to set standby mode: {}", e),
            }
        }
        _ => {
            println!("STorM32 Gimbal Control - Use --help for available commands");
            println!("Example usage:");
            println!("  gimbal --auto status");
            println!("  gimbal set 10.0 0.0 -15.0");
            println!("  gimbal set \"[-10,0,15]\"");
            println!("  gimbal set \"[1.5, -2.3, 0]\"");
            println!("  gimbal center");
        }
    }

    Ok(())
}
