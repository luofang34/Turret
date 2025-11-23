use anyhow::{anyhow, Result};
use async_trait::async_trait;
use log::{debug, info, warn};
use mavlink::{
    common::{
        MavMessage,
        HEARTBEAT_DATA,
        MavAutopilot,
        MavType,
        MavState,
        MavModeFlag,
        MavCmd,
        COMMAND_LONG_DATA,
        ATTITUDE_DATA,
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

/// MAVLink Gimbal Protocol v1 Implementation using COMMAND_LONG and DO_MOUNT commands
pub struct MavlinkV1Gimbal {
    connection: Arc<Mutex<Box<dyn MavConnection<MavMessage> + Send + Sync>>>,
    system_id: u8,
    component_id: u8,
    gimbal_device_id: Option<u8>,
    is_connected: bool,

    gimbal_capabilities: Option<GimbalCapabilities>,
    last_gimbal_status: Option<GimbalStatus>,
    boot_time: Instant,
}

impl MavlinkV1Gimbal {
    pub async fn new(connection: Box<dyn MavConnection<MavMessage> + Send + Sync>, params: ConnectionParams) -> Result<Self> {
        info!("Creating MAVLink Gimbal v1 controller");

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

    async fn send_message(&self, message: MavMessage) -> Result<()> {
        let header = MavHeader {
            system_id: self.system_id,
            component_id: self.component_id,
            sequence: 0,
        };

        let connection = self.connection.lock().await;
        connection.send(&header, &message)
            .map_err(|e| anyhow!("Failed to send MAVLink v1 message: {}", e))?;

        debug!("Sent MAVLink v1 message: {:?}", message);
        Ok(())
    }

    async fn receive_message(&self, timeout_duration: Duration) -> Result<(MavHeader, MavMessage)> {
        let connection = self.connection.lock().await;

        timeout(timeout_duration, async move {
            connection.recv()
                .map_err(|e| anyhow!("Failed to receive MAVLink v1 message: {}", e))
        }).await
        .map_err(|_| anyhow!("Timeout waiting for MAVLink v1 message"))?
    }

    async fn send_heartbeat(&self) -> Result<()> {
        let heartbeat = HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: MavType::MAV_TYPE_ONBOARD_CONTROLLER,
            autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
            base_mode: MavModeFlag::empty(),
            system_status: MavState::MAV_STATE_ACTIVE,
            mavlink_version: 3,
        };

        self.send_message(MavMessage::HEARTBEAT(heartbeat)).await
    }

    async fn discover_gimbal_device(&mut self) -> Result<()> {
        info!("Discovering MAVLink v1 gimbal device...");

        // Send heartbeat first
        self.send_heartbeat().await?;

        // Listen for responses
        let start = Instant::now();
        while start.elapsed() < Duration::from_secs(3) {
            match self.receive_message(Duration::from_millis(100)).await {
                Ok((header, MavMessage::HEARTBEAT(hb))) => {
                    debug!("Received heartbeat from {}.{}: {:?}", header.system_id, header.component_id, hb.mavtype);

                    // Check if this looks like a gimbal
                    if hb.mavtype == MavType::MAV_TYPE_GIMBAL {
                        self.gimbal_device_id = Some(header.component_id);
                        info!("Found MAVLink v1 gimbal at component {}", header.component_id);

                        // Create basic capabilities for v1 gimbals
                        let capabilities = GimbalCapabilities {
                            name: "MAVLink v1 Gimbal".to_string(),
                            vendor: Some("Unknown".to_string()),
                            model: None,
                            firmware_version: None,
                            supports_angle_control: true,
                            supports_rate_control: false, // v1 typically doesn't support rates
                            supports_manual_control: false,
                            pitch_min: Some(-90.0),
                            pitch_max: Some(90.0),
                            roll_min: Some(-45.0),
                            roll_max: Some(45.0),
                            yaw_min: Some(-180.0),
                            yaw_max: Some(180.0),
                        };
                        self.gimbal_capabilities = Some(capabilities);

                        return Ok(());
                    }
                }
                Ok((_, _)) => {
                    // Ignore other messages
                }
                Err(_) => {
                    // Continue waiting
                }
            }
        }

        // If no specific gimbal found, assume there might be one anyway
        if self.gimbal_device_id.is_none() {
            warn!("No MAVLink v1 gimbal heartbeat received, proceeding anyway");
            self.gimbal_device_id = Some(1); // Default component ID

            let capabilities = GimbalCapabilities {
                name: "MAVLink v1 Gimbal (Assumed)".to_string(),
                vendor: Some("Unknown".to_string()),
                model: None,
                firmware_version: None,
                supports_angle_control: true,
                supports_rate_control: false,
                supports_manual_control: false,
                pitch_min: Some(-90.0),
                pitch_max: Some(90.0),
                roll_min: Some(-45.0),
                roll_max: Some(45.0),
                yaw_min: Some(-180.0),
                yaw_max: Some(180.0),
            };
            self.gimbal_capabilities = Some(capabilities);
        }

        Ok(())
    }

    /// Set gimbal attitude using DO_MOUNT_CONTROL command
    async fn set_gimbal_attitude(&self, pitch: f32, roll: f32, yaw: f32) -> Result<()> {
        let gimbal_id = self.gimbal_device_id
            .ok_or_else(|| anyhow!("No gimbal device connected"))?;

        info!("Setting gimbal attitude (v1): pitch={:.1}°, roll={:.1}°, yaw={:.1}°", pitch, roll, yaw);

        // Use DO_MOUNT_CONTROL command for v1 gimbals
        let mount_control = COMMAND_LONG_DATA {
            target_system: 0, // Broadcast or specific system
            target_component: gimbal_id,
            command: MavCmd::MAV_CMD_DO_MOUNT_CONTROL,
            confirmation: 0,
            param1: pitch,   // Pitch angle in degrees
            param2: roll,    // Roll angle in degrees
            param3: yaw,     // Yaw angle in degrees
            param4: 0.0,     // Altitude (not used)
            param5: 0.0,     // Latitude (not used)
            param6: 0.0,     // Longitude (not used)
            param7: 2.0,     // Mount mode: 2 = MAV_MOUNT_MODE_MAVLINK_TARGETING
        };

        self.send_message(MavMessage::COMMAND_LONG(mount_control)).await
    }

    async fn update_gimbal_status(&mut self) -> Result<()> {
        // For v1, we don't have direct status messages, so we'll maintain our own state
        // In a real implementation, we might listen for ATTITUDE messages or other telemetry

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
impl GimbalProtocol for MavlinkV1Gimbal {
    fn protocol_type(&self) -> ProtocolType {
        ProtocolType::Mavlink
    }

    async fn connect(&mut self) -> Result<()> {
        info!("Connecting to MAVLink v1 gimbal...");

        self.discover_gimbal_device().await?;
        self.is_connected = true;

        info!("Successfully connected to MAVLink v1 gimbal");
        Ok(())
    }

    async fn disconnect(&mut self) -> Result<()> {
        self.is_connected = false;
        info!("Disconnected from MAVLink v1 gimbal");
        Ok(())
    }

    async fn get_capabilities(&self) -> Result<GimbalCapabilities> {
        self.gimbal_capabilities.clone()
            .ok_or_else(|| anyhow!("Gimbal capabilities not available"))
    }

    async fn get_status(&mut self) -> Result<GimbalStatus> {
        self.update_gimbal_status().await?;

        self.last_gimbal_status.clone()
            .ok_or_else(|| anyhow!("Gimbal status not available"))
    }

    async fn set_attitude(&mut self, pitch: f32, roll: f32, yaw: f32) -> Result<()> {
        if !self.is_connected {
            return Err(anyhow!("Not connected to gimbal"));
        }

        self.set_gimbal_attitude(pitch, roll, yaw).await?;

        // Update our cached status
        if let Some(ref mut status) = self.last_gimbal_status {
            status.target_angles = GimbalAngles { pitch, roll, yaw };
            // For v1, assume the gimbal reaches the target immediately
            status.current_angles = GimbalAngles { pitch, roll, yaw };
        }

        Ok(())
    }

    async fn center(&mut self) -> Result<()> {
        self.set_attitude(0.0, 0.0, 0.0).await
    }

    async fn set_standby(&mut self, _enable: bool) -> Result<()> {
        // v1 protocol doesn't typically support standby mode
        warn!("Standby mode not supported in MAVLink v1 protocol");
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