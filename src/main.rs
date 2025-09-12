use anyhow::Result;
use clap::{Arg, Command};

mod device_scanner;
mod storm32;

use device_scanner::DeviceScanner;
use storm32::Storm32Gimbal;

#[tokio::main]
async fn main() -> Result<()> {
    let matches = Command::new("gimbal")
        .version("0.1.0")
        .about("STorM32 Gimbal Control using RC Commands")
        .subcommand(
            Command::new("set")
                .about("Set gimbal angles")
                .arg(Arg::new("pitch").help("Pitch angle in degrees").required(true))
                .arg(Arg::new("roll").help("Roll angle in degrees").required(true))
                .arg(Arg::new("yaw").help("Yaw angle in degrees").required(true))
                .arg(Arg::new("device").short('d').long("device").help("Serial device path"))
        )
        .subcommand(
            Command::new("center")
                .about("Center gimbal (all axes to 0°)")
                .arg(Arg::new("device").short('d').long("device").help("Serial device path"))
        )
        .subcommand(
            Command::new("status")
                .about("Get current gimbal status")
                .arg(Arg::new("device").short('d').long("device").help("Serial device path"))
        )
        .subcommand(
            Command::new("version")
                .about("Get gimbal firmware version")
                .arg(Arg::new("device").short('d').long("device").help("Serial device path"))
        )
        .subcommand(
            Command::new("version-str")
                .about("Get gimbal firmware version strings")
                .arg(Arg::new("device").short('d').long("device").help("Serial device path"))
        )
        .subcommand(
            Command::new("pitch")
                .about("Set pitch axis (700-2300 or 0 to recenter)")
                .arg(Arg::new("value").help("Pitch value").required(true))
                .arg(Arg::new("device").short('d').long("device").help("Serial device path"))
        )
        .subcommand(
            Command::new("roll")
                .about("Set roll axis (700-2300 or 0 to recenter)")
                .arg(Arg::new("value").help("Roll value").required(true))
                .arg(Arg::new("device").short('d').long("device").help("Serial device path"))
        )
        .subcommand(
            Command::new("yaw")
                .about("Set yaw axis (700-2300 or 0 to recenter)")
                .arg(Arg::new("value").help("Yaw value").required(true))
                .arg(Arg::new("device").short('d').long("device").help("Serial device path"))
        )
        .subcommand(
            Command::new("pan-mode")
                .about("Set pan mode (0=off, 1=HOLDHOLDPAN, 2=HOLDHOLDHOLD, 3=PANPANPAN, 4=PANHOLDHOLD, 5=PANHOLDPAN)")
                .arg(Arg::new("mode").help("Pan mode").required(true))
                .arg(Arg::new("device").short('d').long("device").help("Serial device path"))
        )
        .subcommand(
            Command::new("standby")
                .about("Set standby mode")
                .arg(Arg::new("enable").help("Enable standby (true/false)").required(true))
                .arg(Arg::new("device").short('d').long("device").help("Serial device path"))
        )
        .arg(Arg::new("auto").short('a').long("auto").help("Auto-detect device").action(clap::ArgAction::SetTrue))
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
            let pitch: f32 = sub_matches.get_one::<String>("pitch")
                .unwrap().parse().expect("Invalid pitch angle");
            let roll: f32 = sub_matches.get_one::<String>("roll")
                .unwrap().parse().expect("Invalid roll angle");
            let yaw: f32 = sub_matches.get_one::<String>("yaw")
                .unwrap().parse().expect("Invalid yaw angle");
            
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
            let value: u16 = sub_matches.get_one::<String>("value")
                .unwrap().parse().expect("Invalid pitch value");
            
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
            let value: u16 = sub_matches.get_one::<String>("value")
                .unwrap().parse().expect("Invalid roll value");
            
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
            let value: u16 = sub_matches.get_one::<String>("value")
                .unwrap().parse().expect("Invalid yaw value");
            
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
            let mode: u8 = sub_matches.get_one::<String>("mode")
                .unwrap().parse().expect("Invalid pan mode");
            
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
            println!("  gimbal center");
        }
    }

    Ok(())
}