use anyhow::{anyhow, Result};
use async_trait::async_trait;
use log::{info, warn, debug};
use mavlink::{MavConnection, common::MavMessage};
use std::time::Duration;
use tokio::time::timeout;

use super::{
    ConnectionParams, GimbalAngles, GimbalCapabilities, GimbalProtocol, GimbalStatus, ProtocolType,
};

mod common;
mod detector;
mod mavlink_v1;
mod mavlink_v2;

use detector::{MavlinkProtocolDetector, MavlinkProtocolVersion};
use mavlink_v1::MavlinkV1Gimbal;
use mavlink_v2::MavlinkV2Gimbal;

/// Unified MAVLink Protocol Handler with Auto-Detection
/// Automatically detects and uses the appropriate MAVLink variant (v1 or v2)
pub struct MavlinkGimbalManager {
    inner: Option<Box<dyn GimbalProtocol>>,
    detected_version: Option<MavlinkProtocolVersion>,
    params: ConnectionParams,
}

impl MavlinkGimbalManager {
    pub async fn new(params: ConnectionParams) -> Result<Self> {
        info!("Creating unified MAVLink Gimbal Manager on {}", params.device_path);

        Ok(Self {
            inner: None,
            detected_version: None,
            params,
        })
    }

    async fn create_connection(&self) -> Result<Box<dyn MavConnection<MavMessage> + Send + Sync>> {
        let connection_string = format!("serial:{}:{}", self.params.device_path, self.params.baud_rate);

        // Create connection with timeout
        let connection_result = tokio::time::timeout(
            Duration::from_millis(self.params.timeout_ms),
            tokio::task::spawn_blocking(move || {
                mavlink::connect(&connection_string)
            })
        )
        .await;

        let connection = match connection_result {
            Ok(conn_result) => conn_result??,
            Err(_) => {
                return Err(anyhow!("Timeout creating MAVLink connection"));
            }
        };

        Ok(connection)
    }

    async fn detect_and_create_protocol(&mut self) -> Result<()> {
        info!("Auto-detecting MAVLink protocol version...");

        let connection = self.create_connection().await?;

        // Create detector
        let mut detector = MavlinkProtocolDetector::new(
            connection,
            self.params.mavlink_system_id,
            self.params.mavlink_component_id,
        );

        // Request gimbal info to trigger v2 responses
        if let Err(e) = detector.request_gimbal_info().await {
            debug!("Failed to request gimbal info: {}", e);
        }

        // Detect protocol with timeout (shorter for testing)
        let detected_version = detector.detect_protocol(Duration::from_secs(2)).await?;
        info!("Detected MAVLink protocol: {}", detected_version);

        // Reuse the connection from the detector to avoid "device busy" error
        let connection = detector.take_connection();

        // Create appropriate protocol handler
        let protocol_handler: Box<dyn GimbalProtocol> = match detected_version {
            MavlinkProtocolVersion::V1 => {
                info!("Using MAVLink Gimbal v1 implementation (includes STorM32 MAVLink mode)");
                Box::new(MavlinkV1Gimbal::new(connection, self.params.clone()).await?)
            }
            MavlinkProtocolVersion::V2 => {
                info!("Using MAVLink Gimbal v2 implementation");
                Box::new(MavlinkV2Gimbal::new(connection, self.params.clone()).await?)
            }
        };

        self.inner = Some(protocol_handler);
        self.detected_version = Some(detected_version);

        Ok(())
    }

    fn get_inner(&self) -> Result<&Box<dyn GimbalProtocol>> {
        self.inner.as_ref()
            .ok_or_else(|| anyhow!("MAVLink protocol not initialized. Call connect() first."))
    }

    fn get_inner_mut(&mut self) -> Result<&mut Box<dyn GimbalProtocol>> {
        self.inner.as_mut()
            .ok_or_else(|| anyhow!("MAVLink protocol not initialized. Call connect() first."))
    }
}

#[async_trait]
impl GimbalProtocol for MavlinkGimbalManager {
    fn protocol_type(&self) -> ProtocolType {
        ProtocolType::Mavlink
    }

    async fn connect(&mut self) -> Result<()> {
        info!("Connecting to MAVLink gimbal with auto-detection...");

        // Detect protocol and create appropriate handler
        self.detect_and_create_protocol().await?;

        // Connect using the detected protocol
        let inner = self.get_inner_mut()?;
        inner.connect().await?;

        if let Some(version) = &self.detected_version {
            info!("Successfully connected using {}", version);
        }

        Ok(())
    }

    async fn disconnect(&mut self) -> Result<()> {
        if let Some(inner) = &mut self.inner {
            inner.disconnect().await?;
        }
        self.inner = None;
        self.detected_version = None;
        info!("Disconnected from MAVLink gimbal");
        Ok(())
    }

    async fn get_capabilities(&self) -> Result<GimbalCapabilities> {
        let inner = self.get_inner()?;
        inner.get_capabilities().await
    }

    async fn get_status(&mut self) -> Result<GimbalStatus> {
        let inner = self.get_inner_mut()?;
        inner.get_status().await
    }

    async fn set_attitude(&mut self, pitch: f32, roll: f32, yaw: f32) -> Result<()> {
        let inner = self.get_inner_mut()?;
        inner.set_attitude(pitch, roll, yaw).await
    }

    async fn center(&mut self) -> Result<()> {
        let inner = self.get_inner_mut()?;
        inner.center().await
    }

    async fn set_standby(&mut self, enable: bool) -> Result<()> {
        let inner = self.get_inner_mut()?;
        inner.set_standby(enable).await
    }

    async fn heartbeat(&mut self) -> Result<()> {
        let inner = self.get_inner_mut()?;
        inner.heartbeat().await
    }

    fn as_any(&self) -> Option<&dyn std::any::Any> {
        Some(self)
    }

    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        Some(self)
    }
}

/// Type alias for backward compatibility
pub type MavlinkGimbal = MavlinkGimbalManager;