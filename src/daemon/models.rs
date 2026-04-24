use serde::{Deserialize, Serialize};
use std::time::SystemTime;

/// Pan modes supported by the gimbal
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PanMode {
    Off = 0,
    HoldHoldPan = 1,
    HoldHoldHold = 2,
    PanPanPan = 3,
    PanHoldHold = 4,
    PanHoldPan = 5,
}

impl PanMode {
    pub fn from_u8(value: u8) -> Option<Self> {
        match value {
            0 => Some(PanMode::Off),
            1 => Some(PanMode::HoldHoldPan),
            2 => Some(PanMode::HoldHoldHold),
            3 => Some(PanMode::PanPanPan),
            4 => Some(PanMode::PanHoldHold),
            5 => Some(PanMode::PanHoldPan),
            _ => None,
        }
    }

    pub fn to_name(&self) -> &'static str {
        match self {
            PanMode::Off => "OFF",
            PanMode::HoldHoldPan => "HOLDHOLDPAN",
            PanMode::HoldHoldHold => "HOLDHOLDHOLD",
            PanMode::PanPanPan => "PANPANPAN",
            PanMode::PanHoldHold => "PANHOLDHOLD",
            PanMode::PanHoldPan => "PANHOLDPAN",
        }
    }
}

/// Control source for gimbal commands
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ControlSource {
    UnixSocket(u32), // Client ID
    Mavlink(u8),     // System ID
    Cli,             // CLI test commands
}

impl ControlSource {
    /// Get priority level (higher = more authority)
    pub fn priority(&self) -> u8 {
        match self {
            ControlSource::Mavlink(1) => 100,   // Autopilot (sysid 1)
            ControlSource::Mavlink(_) => 50,    // Other MAVLink systems (GCS)
            ControlSource::UnixSocket(_) => 20, // Local clients
            ControlSource::Cli => 10,           // CLI test commands (lowest)
        }
    }
}

/// Command mode for gimbal control
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum CommandMode {
    Position, // Absolute angle in degrees
    Rate,     // Angular velocity in degrees/second
    Manual,   // Direct stick-like inputs (-1.0 to +1.0)
}

/// Gimbal command with source and timestamp
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GimbalCommand {
    pub source: ControlSource,
    pub mode: CommandMode,
    pub yaw: Option<f32>,
    pub pitch: Option<f32>,
    pub roll: Option<f32>,
    pub timestamp: SystemTime,
}

impl GimbalCommand {
    pub fn new(
        source: ControlSource,
        mode: CommandMode,
        yaw: Option<f32>,
        pitch: Option<f32>,
        roll: Option<f32>,
    ) -> Self {
        Self {
            source,
            mode,
            yaw,
            pitch,
            roll,
            timestamp: SystemTime::now(),
        }
    }

    /// Check if command is stale (older than 5 seconds)
    pub fn is_stale(&self) -> bool {
        if let Ok(elapsed) = self.timestamp.elapsed() {
            elapsed.as_secs() > 5
        } else {
            true
        }
    }
}

/// Current gimbal state
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GimbalState {
    pub yaw: f32,
    pub pitch: f32,
    pub roll: f32,
    pub pan_mode: PanMode,
    pub standby: bool,
    pub firmware_version: Option<String>,
    pub last_update: SystemTime,
}

impl Default for GimbalState {
    fn default() -> Self {
        Self {
            yaw: 0.0,
            pitch: 0.0,
            roll: 0.0,
            pan_mode: PanMode::Off,
            standby: false,
            firmware_version: None,
            last_update: SystemTime::now(),
        }
    }
}

impl GimbalState {
    pub fn update_angles(&mut self, yaw: f32, pitch: f32, roll: f32) {
        self.yaw = yaw;
        self.pitch = pitch;
        self.roll = roll;
        self.last_update = SystemTime::now();
    }

    pub fn update_pan_mode(&mut self, mode: PanMode) {
        self.pan_mode = mode;
        self.last_update = SystemTime::now();
    }

    pub fn update_standby(&mut self, standby: bool) {
        self.standby = standby;
        self.last_update = SystemTime::now();
    }
}

/// IPC command from Unix socket clients
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "cmd", rename_all = "lowercase")]
pub enum IpcCommand {
    Set {
        pitch: f32,
        roll: f32,
        yaw: f32,
    },
    Center,
    Status,
    Version,
    #[serde(rename = "version-str")]
    VersionStr,
    Pitch {
        value: u16,
    },
    Roll {
        value: u16,
    },
    Yaw {
        value: u16,
    },
    #[serde(rename = "pan-mode")]
    PanMode {
        mode: u8,
    },
    Standby {
        enable: bool,
    },
    Help,
}

/// IPC response to Unix socket clients
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "status", rename_all = "lowercase")]
pub enum IpcResponse {
    Ok {
        #[serde(skip_serializing_if = "Option::is_none")]
        data: Option<serde_json::Value>,
    },
    Error {
        message: String,
    },
}

impl IpcResponse {
    pub fn ok() -> Self {
        IpcResponse::Ok { data: None }
    }

    pub fn ok_with_data(data: serde_json::Value) -> Self {
        IpcResponse::Ok { data: Some(data) }
    }

    pub fn error(message: impl Into<String>) -> Self {
        IpcResponse::Error {
            message: message.into(),
        }
    }
}
