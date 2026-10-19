use super::output;
use serde_json::Value as Json;
use std::path::Path;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::UnixStream;
use tracing::debug;
use turret::error::{Error, Result};

enum IpcOutcome {
    Reached(Json),
    NotReachable,
}

async fn try_ipc(socket_path: &Path, payload: &Json) -> Result<IpcOutcome> {
    use std::io::ErrorKind;

    let stream = match UnixStream::connect(socket_path).await {
        Ok(s) => s,
        Err(e) if matches!(e.kind(), ErrorKind::NotFound | ErrorKind::ConnectionRefused) => {
            return Ok(IpcOutcome::NotReachable);
        }
        Err(e) => return Err(e.into()),
    };

    let (reader, mut writer) = stream.into_split();
    let mut bytes = serde_json::to_vec(payload)?;
    bytes.push(b'\n');
    writer.write_all(&bytes).await?;
    writer.flush().await?;

    let mut reader = BufReader::new(reader);
    let mut line = String::new();
    reader.read_line(&mut line).await?;
    let response: Json = serde_json::from_str(line.trim())?;
    Ok(IpcOutcome::Reached(response))
}

pub(super) fn print_ipc_response(cmd: &str, response: &Json) -> Result<()> {
    let status = response.get("status").and_then(Json::as_str);
    match status {
        Some("error") => {
            let msg = response
                .get("message")
                .and_then(Json::as_str)
                .unwrap_or("(no message)");
            Err(Error::Daemon(format!("ipc {cmd}: {msg}")))
        }
        Some("ok") => {
            if let Some(data) = response.get("data") {
                pretty_print_data(cmd, data)?;
            } else {
                output(format_args!("ok"))?;
            }
            Ok(())
        }
        _ => Err(Error::Daemon(format!(
            "ipc {cmd}: unexpected response: {response}"
        ))),
    }
}

pub(super) fn pretty_print_data(cmd: &str, data: &Json) -> Result<()> {
    match cmd {
        "status" => {
            let p = data.get("pitch").and_then(Json::as_f64);
            let r = data.get("roll").and_then(Json::as_f64);
            let y = data.get("yaw").and_then(Json::as_f64);
            let pan = data
                .get("pan_mode")
                .and_then(Json::as_str)
                .unwrap_or("(unknown)");
            let standby = data
                .get("standby")
                .and_then(Json::as_bool)
                .map_or_else(|| "unknown".to_owned(), |value| value.to_string());
            let off = data.get("yaw_offset_deg").and_then(Json::as_f64);
            let fw = data
                .get("firmware_version")
                .and_then(Json::as_str)
                .unwrap_or("(unknown)");
            let angle = |value: Option<f64>| {
                value.map_or_else(|| "unknown".to_owned(), |v| format!("{v:.2}°"))
            };
            output(format_args!(
                "pitch={}  roll={}  yaw={}",
                angle(p),
                angle(r),
                angle(y)
            ))?;
            output(format_args!("pan_mode={pan}  standby={standby}"))?;
            if let Some(off) = off {
                output(format_args!("yaw_offset_deg={off:.3}"))?;
            }
            output(format_args!("firmware: {fw}"))?;
        }
        "version" => {
            if let Some(v) = data.get("version").and_then(Json::as_str) {
                output(format_args!("{v}"))?;
            } else {
                output(format_args!("{data}"))?;
            }
        }
        _ => {
            // Last-resort fallback. `serde_json::to_string_pretty` on an
            // already-decoded `Value` is infallible in practice; if it
            // somehow fails, fall back to the Display impl.
            match serde_json::to_string_pretty(data) {
                Ok(s) => output(format_args!("{s}"))?,
                Err(_) => output(format_args!("{data}"))?,
            }
        }
    }
    Ok(())
}

pub(super) async fn ipc_or_direct<F>(
    socket_path: &Path,
    cmd: &str,
    payload: Json,
    direct: F,
) -> Result<()>
where
    F: FnOnce() -> Result<()>,
{
    match try_ipc(socket_path, &payload).await? {
        IpcOutcome::Reached(response) => {
            debug!("Routed `{cmd}` through IPC at {}", socket_path.display());
            print_ipc_response(cmd, &response)
        }
        IpcOutcome::NotReachable => {
            debug!(
                "No daemon at {}; running `{cmd}` against the device directly",
                socket_path.display()
            );
            direct()
        }
    }
}
