use anyhow::{anyhow, Result};
use async_trait::async_trait;
use log::{debug, info, warn, trace};
use mavlink::{
    common::{
        MavMessage,
        GIMBAL_DEVICE_SET_ATTITUDE_DATA,
        HEARTBEAT_DATA,
        MavAutopilot,
        MavType,
        MavState,
        MavModeFlag,
        GimbalDeviceFlags,
        MavCmd,
        COMMAND_LONG_DATA,
    },
    MavConnection, MavHeader,
};
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::sync::Mutex;
use tokio::time::timeout;

use crate::protocols::{
    ConnectionParams, GimbalAngles, GimbalCapabilities, GimbalProtocol, GimbalStatus, ProtocolType,
};
use super::mavlink_common::{angles_to_quaternion, quaternion_to_angles};

/// MAVLink Gimbal Protocol v2 Implementation
/// Uses GIMBAL_DEVICE messages for direct gimbal device control
pub struct MavlinkV2Gimbal {
    connection: Arc<Mutex<Box<dyn MavConnection<MavMessage> + Send + Sync>>>,
    system_id: u8,
    component_id: u8,
    gimbal_device_id: Option<u8>, // Component ID of the gimbal device we're controlling
    is_connected: bool,

    // Gimbal info
    gimbal_capabilities: Option<GimbalCapabilities>,
    last_gimbal_status: Option<GimbalStatus>,

    // Boot time for timestamps
    boot_time: Instant,
}

impl MavlinkV2Gimbal {
    pub async fn new(connection: Box<dyn MavConnection<MavMessage> + Send + Sync>, params: ConnectionParams) -> Result<Self> {
        info!("Creating MAVLink Gimbal Device Controller v2");

        Ok(Self {
            connection: Arc::new(Mutex::new(connection)),
            system_id: params.mavlink_system_id,
            component_id: params.mavlink_component_id,
            gimbal_device_id: None,
            is_connected: false,
            gimbal_capabilities: None,
            last_gimbal_status: None,
            boot_time: Instant::now(),
        })
    }

    /// Get boot time in milliseconds for MAVLink timestamps
    fn get_boot_time_ms(&self) -> u32 {
        self.boot_time.elapsed().as_millis() as u32
    }

    /// Send a MAVLink message to the gimbal device
    async fn send_message(&self, message: MavMessage) -> Result<()> {
        let header = MavHeader {
            system_id: self.system_id,
            component_id: self.component_id,
            sequence: 0,
        };

        let connection = self.connection.lock().await;
        connection.send(&header, &message)
            .map_err(|e| anyhow!("Failed to send MAVLink v2 message: {}", e))?;

        trace!("Sent MAVLink v2 message to gimbal device: {:?}", message);
        Ok(())
    }

    /// Receive MAVLink messages from gimbal device
    async fn receive_message(&self, timeout_duration: Duration) -> Result<(MavHeader, MavMessage)> {
        let connection = self.connection.lock().await;

        timeout(timeout_duration, async move {
            connection.recv()
                .map_err(|e| anyhow!("Failed to receive MAVLink v2 message: {}", e))
        }).await
        .map_err(|_| anyhow!("Timeout waiting for MAVLink v2 message"))?
    }

    /// Send heartbeat to gimbal device
    async fn send_heartbeat(&self) -> Result<()> {
        let heartbeat = HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: MavType::MAV_TYPE_ONBOARD_CONTROLLER, // We are an onboard controller
            autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
            base_mode: MavModeFlag::empty(),
            system_status: MavState::MAV_STATE_ACTIVE,
            mavlink_version: 3,
        };

        self.send_message(MavMessage::HEARTBEAT(heartbeat)).await
    }

    /// Discover gimbal device by requesting information
    async fn discover_gimbal_device(&mut self) -> Result<()> {
        info!("Discovering MAVLink v2 gimbal device...");

        // Send heartbeat first
        self.send_heartbeat().await?;

        // Request gimbal device information
        let request = COMMAND_LONG_DATA {
            target_system: 0, // Broadcast
            target_component: 0, // Broadcast
            command: MavCmd::MAV_CMD_REQUEST_MESSAGE,
            confirmation: 0,
            param1: 283.0, // GIMBAL_DEVICE_INFORMATION message ID
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
        };

        self.send_message(MavMessage::COMMAND_LONG(request)).await?;

        // Wait for response
        let start = Instant::now();
        while start.elapsed() < Duration::from_secs(3) {
            match self.receive_message(Duration::from_millis(100)).await {
                Ok((header, MavMessage::GIMBAL_DEVICE_INFORMATION(info))) => {
                    info!("Found MAVLink v2 gimbal device at component {}", header.component_id);
                    self.gimbal_device_id = Some(header.component_id);

                    // Convert to our capabilities format
                    let vendor_str = String::from_utf8_lossy(&info.vendor_name).trim_end_matches('\0').to_string();
                    let model_str = String::from_utf8_lossy(&info.model_name).trim_end_matches('\0').to_string();

                    let capabilities = GimbalCapabilities {
                        name: format!("{} {}", vendor_str, model_str),
                        vendor: Some(vendor_str),
                        model: Some(model_str),
                        firmware_version: Some(format!("v{}.{}.{}.{}",
                            (info.firmware_version >> 24) & 0xFF,
                            (info.firmware_version >> 16) & 0xFF,
                            (info.firmware_version >> 8) & 0xFF,
                            info.firmware_version & 0xFF)),
                        supports_angle_control: true,
                        supports_rate_control: true,
                        supports_manual_control: false,
                        pitch_min: Some(info.pitch_min),
                        pitch_max: Some(info.pitch_max),
                        roll_min: Some(info.roll_min),
                        roll_max: Some(info.roll_max),
                        yaw_min: Some(info.yaw_min),
                        yaw_max: Some(info.yaw_max),
                    };

                    self.gimbal_capabilities = Some(capabilities);
                    return Ok(());
                }
                Ok((header, MavMessage::HEARTBEAT(hb))) => {
                    trace!("Received heartbeat from {}.{}: {:?}", header.system_id, header.component_id, hb.mavtype);

                    // Check if this looks like a gimbal
                    if hb.mavtype == MavType::MAV_TYPE_GIMBAL {
                        self.gimbal_device_id = Some(header.component_id);
                        info!("Found potential v2 gimbal at component {}", header.component_id);
                    }
                }
                Ok((_, msg)) => {
                    trace!("Received other message during v2 discovery: {:?}", msg);
                }
                Err(_) => {
                    // Continue waiting
                }
            }
        }

        // If we found a gimbal but no device info, create basic capabilities
        if self.gimbal_device_id.is_some() && self.gimbal_capabilities.is_none() {
            let capabilities = GimbalCapabilities {
                name: "MAVLink v2 Gimbal".to_string(),
                vendor: Some("Unknown".to_string()),
                model: None,
                firmware_version: None,
                supports_angle_control: true,
                supports_rate_control: true,
                supports_manual_control: false,
                pitch_min: Some(-90.0),
                pitch_max: Some(90.0),
                roll_min: Some(-45.0),
                roll_max: Some(45.0),
                yaw_min: Some(-180.0),
                yaw_max: Some(180.0),
            };
            self.gimbal_capabilities = Some(capabilities);
            info!("Using basic v2 gimbal capabilities");
        }

        if self.gimbal_device_id.is_some() {
            Ok(())
        } else {
            Err(anyhow!("No MAVLink v2 gimbal device found"))
        }
    }

    /// Set gimbal attitude by sending GIMBAL_DEVICE_SET_ATTITUDE to the device
    async fn set_gimbal_attitude(&self, pitch: f32, roll: f32, yaw: f32) -> Result<()> {
        let gimbal_id = self.gimbal_device_id
            .ok_or_else(|| anyhow!("No gimbal device connected"))?;

        info!("Setting v2 gimbal attitude: pitch={:.1}°, roll={:.1}°, yaw={:.1}°", pitch, roll, yaw);

        // Convert angles to quaternion
        let q = angles_to_quaternion(pitch, roll, yaw);

        let set_attitude = GIMBAL_DEVICE_SET_ATTITUDE_DATA {
            target_system: self.system_id, // Target the same system
            target_component: gimbal_id,   // Target the gimbal device
            flags: GimbalDeviceFlags::GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME,
            q,
            angular_velocity_x: f32::NAN, // Not setting rates
            angular_velocity_y: f32::NAN,
            angular_velocity_z: f32::NAN,
        };

        self.send_message(MavMessage::GIMBAL_DEVICE_SET_ATTITUDE(set_attitude)).await
    }

    /// Update status by reading from gimbal device
    async fn update_gimbal_status(&mut self) -> Result<()> {
        // Try to receive attitude status from gimbal device
        match self.receive_message(Duration::from_millis(100)).await {
            Ok((header, MavMessage::GIMBAL_DEVICE_ATTITUDE_STATUS(status))) => {
                if Some(header.component_id) == self.gimbal_device_id {
                    let angles = quaternion_to_angles(status.q);

                    let gimbal_status = GimbalStatus {
                        current_angles: angles,
                        target_angles: angles, // Use current as target for now
                        is_connected: true,
                        gimbal_id: Some(header.component_id),
                    };

                    self.last_gimbal_status = Some(gimbal_status);
                    debug!("Updated v2 gimbal status from device");
                }
            }
            Ok((_, _)) => {
                // Ignore other messages
            }
            Err(_) => {
                // No message available, that's ok
            }
        }

        // If we don't have recent status, return cached or default
        if self.last_gimbal_status.is_none() {
            let default_status = GimbalStatus {
                current_angles: GimbalAngles { pitch: 0.0, roll: 0.0, yaw: 0.0 },
                target_angles: GimbalAngles { pitch: 0.0, roll: 0.0, yaw: 0.0 },
                is_connected: self.is_connected,
                gimbal_id: self.gimbal_device_id,
            };
            self.last_gimbal_status = Some(default_status);
        }

        Ok(())
    }
}

#[async_trait]
impl GimbalProtocol for MavlinkV2Gimbal {
    fn protocol_type(&self) -> ProtocolType {
        ProtocolType::Mavlink
    }

    async fn connect(&mut self) -> Result<()> {
        info!("Connecting to MAVLink v2 gimbal...");

        self.discover_gimbal_device().await?;
        self.is_connected = true;

        info!("Successfully connected to MAVLink v2 gimbal");
        Ok(())
    }

    async fn disconnect(&mut self) -> Result<()> {
        self.is_connected = false;
        info!("Disconnected from MAVLink v2 gimbal");
        Ok(())
    }

    async fn get_capabilities(&self) -> Result<GimbalCapabilities> {
        self.gimbal_capabilities.clone()
            .ok_or_else(|| anyhow!("MAVLink v2 gimbal capabilities not available"))
    }

    async fn get_status(&mut self) -> Result<GimbalStatus> {
        self.update_gimbal_status().await?;

        self.last_gimbal_status.clone()
            .ok_or_else(|| anyhow!("MAVLink v2 gimbal status not available"))
    }

    async fn set_attitude(&mut self, pitch: f32, roll: f32, yaw: f32) -> Result<()> {
        if !self.is_connected {
            return Err(anyhow!("Not connected to gimbal"));
        }

        self.set_gimbal_attitude(pitch, roll, yaw).await?;

        // Update our cached status
        if let Some(ref mut status) = self.last_gimbal_status {
            status.target_angles = GimbalAngles { pitch, roll, yaw };
        }

        Ok(())
    }

    async fn center(&mut self) -> Result<()> {
        self.set_attitude(0.0, 0.0, 0.0).await
    }

    async fn set_standby(&mut self, enable: bool) -> Result<()> {
        info!("Setting v2 gimbal standby mode to {}", enable);
        // v2 protocol supports standby via gimbal device flags
        // This would need to be implemented based on the specific gimbal
        warn!("MAVLink v2 standby mode not yet fully implemented");
        Ok(())
    }

    async fn heartbeat(&mut self) -> Result<()> {
        self.send_heartbeat().await
    }

    fn as_any(&self) -> Option<&dyn std::any::Any> {
        Some(self)
    }

    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        Some(self)
    }
}