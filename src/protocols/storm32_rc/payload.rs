//! Host payload binding with device-side deadlines and measured observations.

use super::{storm32_capabilities, GimbalAngles, Storm32Gimbal};
use crate::error::{Error, Result};
use crate::gimbal::{DeviceObservation, DeviceReceipt, GimbalDevice};
use std::time::{Duration, Instant};
use turret_core::*;

impl GimbalDevice for Storm32Gimbal {
    fn protocol_name(&self) -> &'static str {
        "STorM32 RC Commands"
    }
    fn capabilities(&self) -> Result<PayloadCapabilities> {
        Ok(storm32_capabilities()?)
    }

    fn execute_blocking(
        &mut self,
        command: PayloadCommand,
        send_before: Instant,
    ) -> Result<DeviceReceipt> {
        if Instant::now() >= send_before {
            return Err(Error::Payload(turret_core::Error::Expired));
        }
        self.write_deadline = Some(send_before);
        self.last_motion_sent_at = None;
        let result = self.execute_payload_blocking(command);
        self.write_deadline = None;
        if let Err(error) = result {
            self.current_status.sp = None;
            self.last_rate_at = None;
            return Err(error);
        }
        Ok(DeviceReceipt {
            sent_at: self.last_motion_sent_at.ok_or(Error::MissingMeasurement {
                field: "completed motion write",
            })?,
            acknowledged_at: self.last_acknowledged_at,
            acknowledged: self.last_acknowledged.then_some(true),
        })
    }

    fn observe_blocking(&mut self) -> Result<DeviceObservation> {
        let sampled_at = Instant::now();
        self.update_status_blocking()?;
        let measured = self.current_status.pv.ok_or(Error::MissingMeasurement {
            field: "gimbal attitude",
        })?;
        Ok(DeviceObservation {
            epoch: 0,
            angles: Axes {
                pitch: Some(Angle::radians(measured.pitch.to_radians())?),
                roll: Some(Angle::radians(measured.roll.to_radians())?),
                yaw: Some(Angle::radians(measured.yaw.to_radians())?),
            },
            frame: ReferenceFrame::Vehicle,
            zoom: None,
            source: ObservationSource::DeviceMeasured,
            sampled_at: Some(sampled_at),
            received_at: Instant::now(),
        })
    }

    fn stop_blocking(&mut self, lane: crate::gimbal::PayloadLane, effect: Effect) -> Result<()> {
        if effect == Effect::None {
            return Ok(());
        }
        if lane != crate::gimbal::PayloadLane::Gimbal {
            return Err(Error::Unsupported {
                protocol: self.protocol_name(),
                op: "zoom stop",
            });
        }
        self.last_rate_at = None;
        let deadline = Instant::now() + Duration::from_millis(100);
        let current = self.observe_blocking()?;
        let receipt = self.execute_blocking(
            PayloadCommand::GimbalPosition(GimbalPosition {
                angles: current.angles,
                frame: current.frame,
            }),
            deadline,
        )?;
        if receipt.acknowledged == Some(false) {
            return Err(Error::InvalidInput(
                "device rejected stop target".to_string(),
            ));
        }
        Ok(())
    }

    fn firmware_version_blocking(&mut self) -> Result<String> {
        Ok(self.get_version_blocking()?)
    }
    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        Some(self)
    }
}

impl Storm32Gimbal {
    fn execute_payload_blocking(&mut self, command: PayloadCommand) -> Result<()> {
        match command {
            PayloadCommand::GimbalPosition(target) => {
                self.last_rate_at = None;
                self.position_blocking(target)
            }
            PayloadCommand::GimbalRate(rate) => self.rate_blocking(rate),
            PayloadCommand::Recenter => {
                self.last_rate_at = None;
                self.recenter_pitch_blocking()?;
                let pitch_ack = self.last_acknowledged;
                self.recenter_roll_blocking()?;
                let roll_ack = self.last_acknowledged;
                self.recenter_yaw_blocking()?;
                self.last_acknowledged &= pitch_ack && roll_ack;
                if !self.last_acknowledged {
                    self.last_acknowledged_at = None;
                }
                self.current_status.sp = None;
                Ok(())
            }
            PayloadCommand::Zoom(_) => Err(Error::Unsupported {
                protocol: "STorM32 RC Commands",
                op: "zoom",
            }),
        }
    }

    fn position_blocking(&mut self, target: GimbalPosition) -> Result<()> {
        if !matches!(
            target.frame,
            ReferenceFrame::Mount | ReferenceFrame::Vehicle
        ) {
            return Err(turret_core::Error::UnsupportedFrame.into());
        }
        let complete = target.angles.pitch.is_some()
            && target.angles.roll.is_some()
            && target.angles.yaw.is_some();
        let baseline = if complete {
            None
        } else {
            Some(self.baseline_blocking()?)
        };
        let select = |value: Option<Angle>, fallback: Option<f32>| {
            value
                .map(|v| v.as_radians().to_degrees())
                .or(fallback)
                .ok_or(Error::MissingMeasurement {
                    field: "untouched axis baseline",
                })
        };
        let pitch = select(target.angles.pitch, baseline.map(|v| v.pitch))?;
        let roll = select(target.angles.roll, baseline.map(|v| v.roll))?;
        let yaw = select(target.angles.yaw, baseline.map(|v| v.yaw))?;
        self.set_angles_blocking(pitch, roll, yaw)?;
        Ok(())
    }

    fn baseline_blocking(&mut self) -> Result<GimbalAngles> {
        if let Some(sent) = self.current_status.sp {
            return Ok(sent);
        }
        self.update_status_blocking()?;
        self.current_status.pv.ok_or(Error::MissingMeasurement {
            field: "rate/partial-position baseline",
        })
    }

    fn rate_blocking(&mut self, rate: GimbalRate) -> Result<()> {
        let baseline = self.baseline_blocking()?;
        let now = Instant::now();
        let dt = self
            .last_rate_at
            .map(|last| {
                now.saturating_duration_since(last)
                    .min(Duration::from_millis(100))
            })
            .unwrap_or(Duration::ZERO)
            .as_secs_f32();
        let axis = |rate: Option<AngularRate>, angle: f32| {
            rate.map(|rate| {
                Angle::radians(
                    (angle.to_radians() + rate.as_radians_per_second() * dt)
                        .clamp(-core::f32::consts::PI, core::f32::consts::PI),
                )
            })
            .transpose()
        };
        let target = GimbalPosition {
            angles: Axes {
                pitch: axis(rate.rates.pitch, baseline.pitch)?,
                roll: axis(rate.rates.roll, baseline.roll)?,
                yaw: axis(rate.rates.yaw, baseline.yaw)?,
            },
            frame: rate.frame,
        };
        self.position_blocking(target)?;
        self.last_rate_at = Some(now);
        Ok(())
    }
}
