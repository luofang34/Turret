//! Gimbal device abstraction and runtime auto-detection.

use crate::error::{Error, Result};

/// Gimbal attitude (Euler angles in degrees).
#[derive(Debug, Clone, Copy)]
pub struct Attitude {
    /// Pitch angle in degrees.
    pub pitch: f32,
    /// Roll angle in degrees.
    pub roll: f32,
    /// Yaw angle in degrees.
    pub yaw: f32,
}

/// Result of a complete transport write, separate from physical completion.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeviceReceipt {
    /// Local time the entire payload operation finished writing to the transport.
    pub sent_at: std::time::Instant,
    /// Local time of an explicit device acknowledgement, if one arrived.
    pub acknowledged_at: Option<std::time::Instant>,
    /// Whether the device explicitly accepted this operation.
    /// `None` means this transport or operation provides no acknowledgement.
    pub acknowledged: Option<bool>,
}

/// One coherent device measurement; absent axes are not filled with zero.
#[derive(Debug, Clone, Copy)]
pub struct DeviceObservation {
    /// Connection epoch assigned by the device owner.
    pub epoch: u64,
    /// Independently known gimbal angles.
    pub angles: turret_core::Axes<Option<turret_core::Angle>>,
    /// Coordinate reference of the reported angles.
    pub frame: turret_core::ReferenceFrame,
    /// Actual zoom when the device reports it.
    pub zoom: Option<turret_core::ZoomAbsolute>,
    /// Provenance of these values.
    pub source: turret_core::ObservationSource,
    /// Known acquisition time in the host monotonic domain.
    pub sampled_at: Option<std::time::Instant>,
    /// Local receipt time, independent of acquisition time.
    pub received_at: std::time::Instant,
}

impl DeviceObservation {
    /// Extract a complete attitude for formats that require all three axes.
    pub fn complete_attitude(self) -> Result<Attitude> {
        let angle = |value: Option<turret_core::Angle>, field| {
            value
                .map(|v| v.as_radians().to_degrees())
                .ok_or(Error::MissingMeasurement { field })
        };
        Ok(Attitude {
            pitch: angle(self.angles.pitch, "pitch")?,
            roll: angle(self.angles.roll, "roll")?,
            yaw: angle(self.angles.yaw, "yaw")?,
        })
    }
}

/// Independently controlled operations sharing one physical payload transport.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PayloadLane {
    /// Mount position, rate, and recenter commands.
    Gimbal,
    /// Camera zoom commands.
    Zoom,
}

impl PayloadLane {
    /// Both execution lanes, for an explicitly device-wide operation.
    pub const ALL: [Self; 2] = [Self::Gimbal, Self::Zoom];

    /// The execution lane whose command may replace or stop this operation.
    pub const fn for_command(command: turret_core::PayloadCommand) -> Self {
        match command {
            turret_core::PayloadCommand::Zoom(_) => Self::Zoom,
            _ => Self::Gimbal,
        }
    }
}

/// Synchronous host binding for a payload's portable command contract.
///
/// Backends must check `send_before` at the actual write boundary, including
/// after any prerequisite reads. A successful write does not imply movement.
pub trait GimbalDevice: Send {
    /// Human-readable protocol name.
    fn protocol_name(&self) -> &'static str;
    /// The actual operations and feedback provided by this backend.
    fn capabilities(&self) -> Result<turret_core::PayloadCapabilities>;
    /// Execute a validated payload operation before the exclusive write deadline.
    fn execute_blocking(
        &mut self,
        command: turret_core::PayloadCommand,
        send_before: std::time::Instant,
    ) -> Result<DeviceReceipt>;
    /// Acquire a physical measurement without substituting commanded values.
    fn observe_blocking(&mut self) -> Result<DeviceObservation> {
        Err(Error::Unsupported {
            protocol: self.protocol_name(),
            op: "observe",
        })
    }
    /// Attempt a stop or cancel only on the selected lane; unsupported is explicit.
    fn stop_blocking(&mut self, _lane: PayloadLane, _effect: turret_core::Effect) -> Result<()> {
        Err(Error::Unsupported {
            protocol: self.protocol_name(),
            op: "stop",
        })
    }
    /// Query device firmware identification.
    fn firmware_version_blocking(&mut self) -> Result<String> {
        Err(Error::Unsupported {
            protocol: self.protocol_name(),
            op: "firmware_version",
        })
    }
    /// Optional access to explicitly protocol-specific operations.
    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        None
    }
}

/// Detect and initialize a gimbal on `device_path`, trying each supported
/// protocol in turn.
///
/// On failure, distinguishes "device path is held by another process"
/// (typically a `turret --daemon` running locally — returns
/// [`Error::DeviceBusy`]) from "no supported protocol responded"
/// (returns [`Error::NotDetected`]). The two need different fixes from
/// the operator, so collapsing them was the visible UX bug that drove
/// this distinction.
pub fn detect_gimbal_blocking(device_path: &str) -> Result<Box<dyn GimbalDevice>> {
    use crate::protocols::storm32_rc::Storm32RC;

    match Storm32RC::try_detect_blocking(device_path) {
        Ok(gimbal) => Ok(Box::new(gimbal)),
        Err(e) => {
            if is_resource_busy(&e) {
                Err(Error::DeviceBusy {
                    device: device_path.to_string(),
                })
            } else {
                Err(Error::NotDetected {
                    device: device_path.to_string(),
                    tried: "STorM32 RC".to_string(),
                })
            }
        }
    }
}

/// Walk the error chain looking for an `io::Error` with kind
/// [`std::io::ErrorKind::ResourceBusy`] — the EBUSY that
/// `serialport-rs` raises when the device is held exclusively by
/// another process. Used by [`detect_gimbal_blocking`] to surface
/// [`Error::DeviceBusy`] instead of the generic
/// [`Error::NotDetected`] in that specific case.
fn is_resource_busy(err: &Error) -> bool {
    // `dyn std::error::Error` source chain. The actual error is a few
    // layers down: Error::Storm32(Storm32Error::SerialError(serialport::Error{ kind, .. }))
    // → wraps io::Error in some versions of serialport. Walk until we
    // find an io::Error and check its kind.
    let mut current: Option<&(dyn std::error::Error + 'static)> = Some(err);
    while let Some(e) = current {
        if let Some(io_err) = e.downcast_ref::<std::io::Error>() {
            if io_err.kind() == std::io::ErrorKind::ResourceBusy {
                return true;
            }
        }
        // serialport::Error has its own Kind enum that signals
        // "InvalidInput" / "NoDevice" / "Unknown"; the EBUSY case
        // surfaces as Unknown but with the wrapped io::Error
        // distinguishable. Conservative belt-and-braces: also peek at
        // the Display string for the common "Device or resource busy"
        // text — `serialport-rs` includes it on Linux even when the
        // ErrorKind has been remapped.
        let display = e.to_string();
        if display.contains("Device or resource busy") || display.contains("Resource busy") {
            return true;
        }
        current = e.source();
    }
    false
}
