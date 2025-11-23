use anyhow::{anyhow, Result};
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};

use crate::protocols::ProtocolType;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GimbalConfig {
    /// Protocol to use (auto, mavlink, storm32-rc)
    #[serde(default = "default_protocol")]
    pub protocol: ProtocolType,

    /// Serial device path (if empty, auto-detect)
    #[serde(default)]
    pub device: String,

    /// Baud rate for serial communication
    #[serde(default = "default_baud_rate")]
    pub baud_rate: u32,

    /// MAVLink system ID
    #[serde(default = "default_mavlink_sysid")]
    pub mavlink_system_id: u8,

    /// MAVLink component ID
    #[serde(default = "default_mavlink_compid")]
    pub mavlink_component_id: u8,

    /// Connection timeout in milliseconds
    #[serde(default = "default_timeout")]
    pub timeout_ms: u64,

}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DaemonConfig {
    /// Enable daemon mode
    #[serde(default)]
    pub enabled: bool,

    /// Socket path for Unix socket (Linux/macOS) or named pipe name (Windows)
    #[serde(default = "default_socket_path")]
    pub socket_path: String,

    /// TCP port to listen on (if 0, disable TCP)
    #[serde(default)]
    pub tcp_port: u16,

    /// Heartbeat interval in milliseconds
    #[serde(default = "default_heartbeat_interval")]
    pub heartbeat_interval_ms: u64,

    /// Maximum clients allowed to connect simultaneously
    #[serde(default = "default_max_clients")]
    pub max_clients: usize,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LoggingConfig {
    /// Log level (error, warn, info, debug, trace)
    #[serde(default = "default_log_level")]
    pub level: String,

    /// Enable logging to file
    #[serde(default)]
    pub file_enabled: bool,

    /// Log file path
    #[serde(default = "default_log_file")]
    pub file_path: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Config {
    #[serde(default)]
    pub gimbal: GimbalConfig,

    #[serde(default)]
    pub daemon: DaemonConfig,

    #[serde(default)]
    pub logging: LoggingConfig,
}

// Default value functions
fn default_protocol() -> ProtocolType {
    ProtocolType::Auto
}

fn default_baud_rate() -> u32 {
    115200
}

fn default_mavlink_sysid() -> u8 {
    1
}

fn default_mavlink_compid() -> u8 {
    154 // MAV_COMP_ID_GIMBAL
}

fn default_timeout() -> u64 {
    1000
}


fn default_socket_path() -> String {
    if cfg!(windows) {
        r"\\.\pipe\turret".to_string()
    } else {
        "/tmp/turret.sock".to_string()
    }
}

fn default_heartbeat_interval() -> u64 {
    1000 // 1 second
}

fn default_max_clients() -> usize {
    10
}

fn default_log_level() -> String {
    "info".to_string()
}

fn default_log_file() -> String {
    if cfg!(windows) {
        "turret.log".to_string()
    } else {
        "/tmp/turret.log".to_string()
    }
}

impl Default for GimbalConfig {
    fn default() -> Self {
        Self {
            protocol: default_protocol(),
            device: String::new(),
            baud_rate: default_baud_rate(),
            mavlink_system_id: default_mavlink_sysid(),
            mavlink_component_id: default_mavlink_compid(),
            timeout_ms: default_timeout(),
        }
    }
}

impl Default for DaemonConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            socket_path: default_socket_path(),
            tcp_port: 0,
            heartbeat_interval_ms: default_heartbeat_interval(),
            max_clients: default_max_clients(),
        }
    }
}

impl Default for LoggingConfig {
    fn default() -> Self {
        Self {
            level: default_log_level(),
            file_enabled: false,
            file_path: default_log_file(),
        }
    }
}

impl Default for Config {
    fn default() -> Self {
        Self {
            gimbal: GimbalConfig::default(),
            daemon: DaemonConfig::default(),
            logging: LoggingConfig::default(),
        }
    }
}

impl Config {
    /// Load configuration from a TOML file
    pub fn load_from_file<P: AsRef<Path>>(path: P) -> Result<Self> {
        let content = fs::read_to_string(path.as_ref())
            .map_err(|e| anyhow!("Failed to read config file {}: {}", path.as_ref().display(), e))?;

        let config: Config = toml::from_str(&content)
            .map_err(|e| anyhow!("Failed to parse config file: {}", e))?;

        Ok(config)
    }

    /// Save configuration to a TOML file
    pub fn save_to_file<P: AsRef<Path>>(&self, path: P) -> Result<()> {
        let content = toml::to_string_pretty(self)
            .map_err(|e| anyhow!("Failed to serialize config: {}", e))?;

        // Create parent directory if it doesn't exist
        if let Some(parent) = path.as_ref().parent() {
            fs::create_dir_all(parent)
                .map_err(|e| anyhow!("Failed to create config directory: {}", e))?;
        }

        fs::write(path.as_ref(), content)
            .map_err(|e| anyhow!("Failed to write config file {}: {}", path.as_ref().display(), e))?;

        Ok(())
    }

    /// Load configuration with fallback to defaults
    /// Tries the following locations in order:
    /// 1. ./turret.toml (current directory)
    /// 2. ~/.config/turret/config.toml (Linux/macOS)
    /// 3. %APPDATA%/turret/config.toml (Windows)
    /// 4. Default configuration if none found
    pub fn load() -> Self {
        let config_paths = [
            // Current directory
            PathBuf::from("turret.toml"),
            // User config directory
            Self::get_user_config_path(),
        ];

        for path in &config_paths {
            if path.exists() {
                match Self::load_from_file(path) {
                    Ok(config) => {
                        log::info!("Loaded configuration from: {}", path.display());
                        return config;
                    }
                    Err(e) => {
                        log::warn!("Failed to load config from {}: {}", path.display(), e);
                    }
                }
            }
        }

        log::info!("Using default configuration");
        Self::default()
    }

    /// Save configuration to user config directory
    pub fn save(&self) -> Result<()> {
        let config_path = Self::get_user_config_path();
        self.save_to_file(config_path)
    }

    /// Get the user configuration file path
    pub fn get_user_config_path() -> PathBuf {
        if cfg!(windows) {
            // Windows: %APPDATA%/turret/config.toml
            if let Ok(appdata) = std::env::var("APPDATA") {
                PathBuf::from(appdata).join("turret").join("config.toml")
            } else {
                PathBuf::from("turret.toml")
            }
        } else {
            // Linux/macOS: ~/.config/turret/config.toml
            if let Ok(home) = std::env::var("HOME") {
                PathBuf::from(home).join(".config").join("turret").join("config.toml")
            } else {
                PathBuf::from("turret.toml")
            }
        }
    }

    /// Create a sample configuration file
    pub fn create_sample<P: AsRef<Path>>(path: P) -> Result<()> {
        let sample_config = Self::default();
        sample_config.save_to_file(path)
    }

    /// Merge configuration with command line overrides
    pub fn merge_with_overrides(
        mut self,
        protocol: Option<ProtocolType>,
        device: Option<String>,
        verbose: u8,
    ) -> Self {
        // Override protocol if specified
        if let Some(protocol) = protocol {
            self.gimbal.protocol = protocol;
        }

        // Override device if specified
        if let Some(device) = device {
            self.gimbal.device = device;
        }

        // Override log level based on verbosity
        self.logging.level = match verbose {
            0 => "warn".to_string(),
            1 => "info".to_string(),
            2 => "debug".to_string(),
            _ => "trace".to_string(),
        };

        self
    }

    /// Convert to connection parameters for protocols
    pub fn to_connection_params(&self) -> crate::protocols::ConnectionParams {
        crate::protocols::ConnectionParams {
            device_path: self.gimbal.device.clone(),
            baud_rate: self.gimbal.baud_rate,
            mavlink_system_id: self.gimbal.mavlink_system_id,
            mavlink_component_id: self.gimbal.mavlink_component_id,
            timeout_ms: self.gimbal.timeout_ms,
        }
    }
}