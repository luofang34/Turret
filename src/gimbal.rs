use anyhow::Result;

/// Gimbal attitude (Euler angles in degrees)
#[derive(Debug, Clone, Copy)]
pub struct Attitude {
    pub pitch: f32,
    pub roll: f32,
    pub yaw: f32,
}

/// Common interface for all gimbal devices
pub trait GimbalDevice: Send {
    /// Get the protocol name
    fn protocol_name(&self) -> &'static str;

    /// Set gimbal attitude (Euler angles in degrees)
    fn set_attitude(&mut self, pitch: f32, roll: f32, yaw: f32) -> Result<()>;

    /// Get current gimbal attitude
    fn get_attitude(&mut self) -> Result<Attitude>;

    /// Center gimbal (convenience method)
    fn center(&mut self) -> Result<()> {
        self.set_attitude(0.0, 0.0, 0.0)
    }

    /// Set pan mode (optional, may not be supported)
    fn set_pan_mode(&mut self, _mode: u8) -> Result<()> {
        Err(anyhow::anyhow!("Pan mode not supported by {} protocol", self.protocol_name()))
    }

    /// Set standby mode (optional, may not be supported)
    fn set_standby(&mut self, _enabled: bool) -> Result<()> {
        Err(anyhow::anyhow!("Standby mode not supported by {} protocol", self.protocol_name()))
    }

    /// Get firmware version (optional, may not be supported)
    fn get_version(&mut self) -> Result<String> {
        Err(anyhow::anyhow!("Version query not supported by {} protocol", self.protocol_name()))
    }

    /// Downcast to Any for type-specific access
    fn as_any(&self) -> &dyn std::any::Any {
        panic!("as_any not implemented for {}", self.protocol_name())
    }

    /// Downcast to Any mutable for type-specific access
    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        panic!("as_any_mut not implemented for {}", self.protocol_name())
    }
}

/// Detect and initialize gimbal device on given serial port
pub fn detect_gimbal(device_path: &str) -> Result<Box<dyn GimbalDevice>> {
    use crate::protocols::storm32_rc::Storm32RC;

    // Try protocols in order of likelihood

    // 1. STorM32 RC Commands (most common for this project)
    if let Ok(gimbal) = Storm32RC::try_detect(device_path) {
        return Ok(Box::new(gimbal));
    }

    // 2. Add more protocols here in future:
    // if let Ok(gimbal) = MAVLinkGimbal::try_detect(device_path) {
    //     return Ok(Box::new(gimbal));
    // }

    Err(anyhow::anyhow!(
        "No supported gimbal protocol detected on {}. Tried: STorM32 RC",
        device_path
    ))
}