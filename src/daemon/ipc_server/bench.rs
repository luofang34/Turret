//! Explicit operator calibration and physical verification workflows.

use super::*;
use crate::daemon::calibration::Calibration;
use crate::daemon::gimbal_handle::HostCommandTiming;
use std::time::{Duration, Instant};
use turret_core::{PayloadCommand, ReferenceFrame};

const SETTLE: Duration = Duration::from_millis(2500);
const TOLERANCE_DEG: f32 = 1.0;

pub(super) async fn calibrate_yaw(gimbal: &GimbalHandle) -> IpcResponse {
    let target = match gimbal.capabilities().await.and_then(|caps| {
        caps.gimbal
            .recenter
            .available()?
            .target
            .and_then(|target| {
                (target.frame == ReferenceFrame::Vehicle)
                    .then_some(target.angles.yaw)
                    .flatten()
            })
            .ok_or(crate::Error::MissingMeasurement {
                field: "known recenter yaw target",
            })
    }) {
        Ok(target) => target,
        Err(error) => return IpcResponse::error(error.to_string()),
    };
    if let Err(error) = gimbal
        .execute(
            PayloadCommand::Recenter,
            HostCommandTiming::local_ingress(PayloadCommand::Recenter, Instant::now()),
        )
        .await
    {
        return IpcResponse::error(error.to_string());
    }
    tokio::time::sleep(SETTLE).await;
    let yaw = match gimbal.observe_raw().await.and_then(|sample| {
        sample.angles.yaw.ok_or(crate::Error::MissingMeasurement {
            field: "measured yaw",
        })
    }) {
        Ok(yaw) => yaw,
        Err(error) => return IpcResponse::error(error.to_string()),
    };
    set_yaw_offset(
        gimbal,
        (yaw.as_radians() - target.as_radians()).to_degrees(),
    )
}

pub(super) fn set_yaw_offset(gimbal: &GimbalHandle, degrees: f32) -> IpcResponse {
    if !degrees.is_finite() {
        return IpcResponse::error("yaw offset must be finite");
    }
    gimbal.set_yaw_offset_deg(degrees);
    let warning = Calibration {
        yaw_offset_deg: degrees,
    }
    .save_blocking()
    .err()
    .map(|error| format!("offset applied in memory; persistence failed: {error}"));
    IpcResponse::ok_with_data(serde_json::json!({"yaw_offset_deg": degrees, "warning": warning}))
}

pub(super) async fn selftest(gimbal: &GimbalHandle, state: &StateManager) -> IpcResponse {
    if state.is_standby() {
        return IpcResponse::error("selftest requires active stabilization");
    }
    let original = match gimbal
        .observe()
        .await
        .and_then(|sample| sample.complete_attitude())
    {
        Ok(original) => original,
        Err(error) => return IpcResponse::error(error.to_string()),
    };
    let points = [
        (0.0, 0.0, 0.0),
        (15.0, 0.0, 0.0),
        (-15.0, 0.0, 0.0),
        (0.0, 0.0, 15.0),
        (0.0, 0.0, -15.0),
        (0.0, 0.0, 0.0),
    ];
    let mut samples = Vec::new();
    let mut maximum = [0_f32; 3];
    let mut aborted = None;
    for point in points {
        match verify_point(gimbal, point).await {
            Ok((sample, errors)) => {
                for (maximum, error) in maximum.iter_mut().zip(errors) {
                    *maximum = maximum.max(error);
                }
                samples.push(sample);
            }
            Err(error) => {
                aborted = Some(error.to_string());
                break;
            }
        }
    }
    let warning = execute_pose(gimbal, (original.pitch, original.roll, original.yaw))
        .await
        .err()
        .map(|error| format!("initial pose restoration failed: {error}"));
    IpcResponse::ok_with_data(serde_json::json!({
        "passed": aborted.is_none() && maximum.iter().all(|error| *error <= TOLERANCE_DEG),
        "tolerance_deg": TOLERANCE_DEG, "samples": samples, "aborted_at": aborted,
        "warning": warning, "max_error_deg": {"pitch": maximum[0], "roll": maximum[1], "yaw": maximum[2]},
    }))
}

async fn verify_point(
    gimbal: &GimbalHandle,
    point: (f32, f32, f32),
) -> crate::Result<(serde_json::Value, [f32; 3])> {
    execute_pose(gimbal, point).await?;
    tokio::time::sleep(SETTLE).await;
    let measured = gimbal.observe().await?.complete_attitude()?;
    let errors = [
        (point.0 - measured.pitch).abs(),
        (point.1 - measured.roll).abs(),
        (point.2 - measured.yaw).abs(),
    ];
    Ok((
        serde_json::json!({
            "sp": {"pitch": point.0, "roll": point.1, "yaw": point.2},
            "pv": {"pitch": measured.pitch, "roll": measured.roll, "yaw": measured.yaw},
            "error_deg": {"pitch": errors[0], "roll": errors[1], "yaw": errors[2]},
        }),
        errors,
    ))
}

async fn execute_pose(
    gimbal: &GimbalHandle,
    pose: (f32, f32, f32),
) -> crate::Result<turret_core::CommandProgress> {
    let command = GimbalCommand::new(
        ControlSource::UnixSocket(0),
        Some(pose.0),
        Some(pose.1),
        Some(pose.2),
    );
    gimbal.execute(command.payload()?, command.timing()?).await
}
