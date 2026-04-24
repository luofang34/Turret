use anyhow::Result;
use async_trait::async_trait;
use serde::{Deserialize, Serialize};
use std::fmt;

// Rust 2018+ style: module file is src/protocols.rs, submodules in src/protocols/
#[path = "protocols/storm32_rc.rs"]
pub mod storm32_rc;

/// Protocol type enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProtocolType {
    Storm32Rc,
    Mavlink,
    Auto, // Auto-detect
}

impl fmt::Display for ProtocolType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ProtocolType::Storm32Rc => write!(f, "STorM32 RC Commands"),
            ProtocolType::Mavlink => write!(f, "MAVLink"),
            ProtocolType::Auto => write!(f, "Auto-detect"),
        }
    }
}

/// Connection parameters for protocol initialization
#[derive(Debug, Clone)]
pub struct ConnectionParams {
    pub device_path: String,
    pub baud_rate: u32,
    pub timeout_ms: u64,
    pub mavlink_system_id: u8,
    pub mavlink_component_id: u8,
}

impl Default for ConnectionParams {
    fn default() -> Self {
        Self {
            device_path: String::new(),
            baud_rate: 115200,
            timeout_ms: 1000,
            mavlink_system_id: 255,
            mavlink_component_id: 0,
        }
    }
}

/// Gimbal angles (Euler angles in degrees)
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct GimbalAngles {
    pub pitch: f32,
    pub roll: f32,
    pub yaw: f32,
}

/// Gimbal status information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GimbalStatus {
    pub current_angles: GimbalAngles,
    pub target_angles: GimbalAngles,
    pub is_connected: bool,
    pub gimbal_id: Option<u8>,
}

/// Gimbal capabilities
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GimbalCapabilities {
    pub name: String,
    pub vendor: Option<String>,
    pub model: Option<String>,
    pub firmware_version: Option<String>,
    pub supports_angle_control: bool,
    pub supports_rate_control: bool,
    pub supports_manual_control: bool,
    pub pitch_min: Option<f32>,
    pub pitch_max: Option<f32>,
    pub roll_min: Option<f32>,
    pub roll_max: Option<f32>,
    pub yaw_min: Option<f32>,
    pub yaw_max: Option<f32>,
}

/// Common trait for all gimbal protocols
/// Note: Async methods do not require Send futures due to SerialPort limitations
#[async_trait(?Send)]
pub trait GimbalProtocol: Send {
    /// Get the protocol type
    fn protocol_type(&self) -> ProtocolType;

    /// Connect to the gimbal device
    async fn connect(&mut self) -> Result<()>;

    /// Disconnect from the gimbal device
    async fn disconnect(&mut self) -> Result<()>;

    /// Get gimbal capabilities
    async fn get_capabilities(&self) -> Result<GimbalCapabilities>;

    /// Get current gimbal status
    async fn get_status(&mut self) -> Result<GimbalStatus>;

    /// Set gimbal attitude (Euler angles in degrees)
    async fn set_attitude(&mut self, pitch: f32, roll: f32, yaw: f32) -> Result<()>;

    /// Center gimbal (convenience method)
    async fn center(&mut self) -> Result<()> {
        self.set_attitude(0.0, 0.0, 0.0).await
    }

    /// Set standby mode (optional, may not be supported)
    async fn set_standby(&mut self, _enabled: bool) -> Result<()> {
        Ok(())
    }

    /// Send heartbeat (for protocols that require it)
    async fn heartbeat(&mut self) -> Result<()> {
        Ok(())
    }

    /// Downcast to Any for type-specific access
    fn as_any(&self) -> Option<&dyn std::any::Any> {
        None
    }

    /// Downcast to Any mutable for type-specific access
    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        None
    }
}

/// Auto-detect protocol on a given device
pub async fn detect_protocol(device_path: &str) -> Result<ProtocolType> {
    use log::debug;

    // Try Storm32 RC protocol
    debug!(
        "Attempting STorM32 RC protocol detection on {}",
        device_path
    );
    if storm32_rc::Storm32RC::try_detect(device_path).is_ok() {
        return Ok(ProtocolType::Storm32Rc);
    }

    Err(anyhow::anyhow!(
        "No supported protocol detected on {}",
        device_path
    ))
}

/// Create a protocol instance based on type
/// Note: This function is intended for use by the daemon module
/// For CLI usage, use the gimbal::detect_gimbal function instead
pub async fn create_protocol(
    _protocol_type: ProtocolType,
    _params: ConnectionParams,
) -> Result<Box<dyn GimbalProtocol>> {
    // TODO: Implement this when daemon module is fully integrated
    // For now, this is a placeholder
    Err(anyhow::anyhow!(
        "Protocol creation not yet implemented. Use gimbal::detect_gimbal() for CLI mode"
    ))
}

// Note: Storm32RcAsync wrapper is not included here to avoid Sync issues
// For async gimbal protocol implementations, see the daemon module
// which properly wraps synchronous protocols with spawn_blocking
