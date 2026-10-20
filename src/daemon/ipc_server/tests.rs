use super::*;
use crate::gimbal::{DeviceReceipt, GimbalDevice, PayloadLane};
use parking_lot::Mutex;
use serde_json::{json, Value};
use std::time::{Duration, Instant};
use turret_core::{Effect, PayloadCapabilities, PayloadCommand};

struct ReceiptDevice {
    acknowledgement: Option<bool>,
    effects: Arc<Mutex<Vec<(PayloadLane, Effect)>>>,
}

impl GimbalDevice for ReceiptDevice {
    fn protocol_name(&self) -> &'static str {
        "receipt-test"
    }

    fn capabilities(&self) -> crate::Result<PayloadCapabilities> {
        Ok(crate::protocols::storm32_rc::storm32_capabilities()?)
    }

    fn execute_blocking(
        &mut self,
        _command: PayloadCommand,
        send_before: Instant,
    ) -> crate::Result<DeviceReceipt> {
        let now = Instant::now();
        if now >= send_before {
            return Err(turret_core::Error::Expired.into());
        }
        Ok(DeviceReceipt {
            sent_at: now,
            acknowledged_at: self.acknowledgement.map(|_| now),
            acknowledged: self.acknowledgement,
        })
    }

    fn stop_blocking(&mut self, lane: PayloadLane, effect: Effect) -> crate::Result<()> {
        self.effects.lock().push((lane, effect));
        Ok(())
    }
}

#[cfg(test)]
fn runtime() -> tokio::runtime::Runtime {
    tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .unwrap()
}

#[cfg(test)]
async fn roundtrip(acknowledgement: Option<bool>) -> (Value, Vec<(PayloadLane, Effect)>) {
    let state = StateManager::new();
    let effects = Arc::new(Mutex::new(Vec::new()));
    let gimbal = GimbalHandle::new(Box::new(ReceiptDevice {
        acknowledgement,
        effects: effects.clone(),
    }))
    .unwrap();
    let slots = SlotPool::new(state.clone());
    let guard = slots.acquire().unwrap();
    let arbitrator = Arc::new(Arbitrator::new(state.clone()));
    let (client, server) = UnixStream::pair().unwrap();
    let mut client_task = tokio::spawn(IpcServer::handle_client(
        guard, server, state, arbitrator, gimbal,
    ));
    let (read, mut write) = client.into_split();
    let mut read = BufReader::new(read);
    let exchange = async {
        write
            .write_all(b"{\"cmd\":\"set\",\"pitch\":10,\"roll\":0,\"yaw\":0}\n")
            .await
            .unwrap();
        let mut line = String::new();
        assert!(read.read_line(&mut line).await.unwrap() > 0);
        let response: Value = serde_json::from_str(&line).unwrap();
        write.write_all(b"{\"cmd\":\"status\"}\n").await.unwrap();
        line.clear();
        assert!(read.read_line(&mut line).await.unwrap() > 0);
        let status: Value = serde_json::from_str(&line).unwrap();
        assert_eq!(status["status"], "ok");
        assert_eq!(
            response["data"],
            status["data"]["command_progress"]["gimbal"]
        );
        assert_eq!(status["data"]["command_progress"]["zoom"], Value::Null);
        response
    };
    let response = tokio::time::timeout(Duration::from_secs(2), exchange).await;
    write.shutdown().await.unwrap();
    drop(read);
    let completed = tokio::time::timeout(Duration::from_secs(2), &mut client_task).await;
    if completed.is_err() {
        client_task.abort();
        client_task.await.ok();
    }
    completed.unwrap().unwrap().unwrap();
    assert_eq!(slots.in_use_count(), 0);
    let effects = effects.lock().clone();
    (response.unwrap(), effects)
}

#[cfg(test)]
fn assert_receipt(response: &Value, acknowledgement: Option<bool>) {
    let data = &response["data"];
    for key in [
        "epoch",
        "generation",
        "id",
        "sequence",
        "accepted_at_us",
        "prepared_at_us",
        "sent_at_us",
    ] {
        assert!(data[key].as_u64().is_some(), "missing {key}: {data}");
    }
    assert!(data["prepared_at_us"].as_u64() >= data["accepted_at_us"].as_u64());
    assert!(data["sent_at_us"].as_u64() >= data["prepared_at_us"].as_u64());
    assert_eq!(data["sent"], true);
    assert_eq!(data["device_acknowledged"], acknowledgement == Some(true));
    assert_eq!(data["observed_complete"], false);
    assert_eq!(data["observed_complete_at_us"], Value::Null);
    assert_eq!(data["acknowledgement_timed_out"], false);
    assert_eq!(
        data["acknowledged_at_us"].is_u64(),
        acknowledgement == Some(true)
    );
    assert_eq!(
        data["rejected_at_us"].is_u64(),
        acknowledgement == Some(false)
    );
}

#[test]
fn no_ack_roundtrip_preserves_dispatch_without_claiming_device_acceptance() {
    runtime().block_on(async {
        let (response, effects) = roundtrip(None).await;
        assert_eq!(response["status"], "ok");
        assert_eq!(response["data"]["ended"], Value::Null);
        assert_receipt(&response, None);
        assert!(effects.is_empty());
    });
}

#[test]
fn positive_ack_roundtrip_keeps_completion_unknown() {
    runtime().block_on(async {
        let (response, effects) = roundtrip(Some(true)).await;
        assert_eq!(response["status"], "ok");
        assert_eq!(response["data"]["ended"], Value::Null);
        assert_receipt(&response, Some(true));
        assert!(effects.is_empty());
    });
}

#[test]
fn negative_ack_roundtrip_is_an_error_even_when_cancel_succeeds() {
    runtime().block_on(async {
        let (response, effects) = roundtrip(Some(false)).await;
        assert_eq!(response["status"], "error");
        assert!(response["message"].as_str().is_some());
        assert_eq!(response["data"]["ended"], "DeviceRejected");
        assert_receipt(&response, Some(false));
        assert_eq!(effects, vec![(PayloadLane::Gimbal, Effect::CancelAction)]);
    });
}

#[test]
fn errors_without_receipts_keep_the_existing_wire_shape() {
    let response = IpcResponse::error("invalid command");
    assert_eq!(
        serde_json::to_value(response).unwrap(),
        json!({"status": "error", "message": "invalid command"})
    );
    let decoded: IpcResponse =
        serde_json::from_value(json!({"status": "error", "message": "invalid command"})).unwrap();
    assert!(matches!(decoded, IpcResponse::Error { data: None, .. }));
}
