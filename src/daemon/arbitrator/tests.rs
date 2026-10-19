use super::*;
use crate::daemon::models::ControlSource;
use crate::gimbal::{DeviceReceipt, GimbalDevice};
use crate::protocols::storm32_rc::storm32_capabilities;
use std::time::Instant;
use turret_core::{PayloadCapabilities, PayloadCommand};

struct TestDevice {
    fail: bool,
}
impl GimbalDevice for TestDevice {
    fn protocol_name(&self) -> &'static str {
        "arbitration-test"
    }
    fn capabilities(&self) -> Result<PayloadCapabilities> {
        Ok(storm32_capabilities()?)
    }
    fn execute_blocking(
        &mut self,
        _payload: PayloadCommand,
        _before: Instant,
    ) -> Result<DeviceReceipt> {
        if self.fail {
            return Err(Error::MissingMeasurement {
                field: "test device baseline",
            });
        }
        Ok(DeviceReceipt {
            sent_at: Instant::now(),
            acknowledged_at: None,
            acknowledged: None,
        })
    }
    fn stop_blocking(&mut self, _effect: turret_core::Effect) -> Result<()> {
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

#[test]
fn accepted_but_failed_command_never_advances_sent_baseline() {
    runtime().block_on(async {
        let state = StateManager::new();
        let arbitrator = Arbitrator::new(state.clone());
        let handle = GimbalHandle::new(Box::new(TestDevice { fail: true })).unwrap();
        let command = GimbalCommand::new(ControlSource::Mavlink(1), Some(20.0), None, None);
        assert!(arbitrator.execute(command, &handle).await.is_err());
        assert!(state.commands.last_cached().is_some());
        assert!(state.last_command_cached().is_none());
        assert_eq!(state.axis_baseline_cached(), (None, None, None));
        assert_eq!(state.state_cached().pitch, None);
    });
}

#[test]
fn sent_partial_command_does_not_invent_other_axes_or_measurements() {
    runtime().block_on(async {
        let state = StateManager::new();
        let arbitrator = Arbitrator::new(state.clone());
        let handle = GimbalHandle::new(Box::new(TestDevice { fail: false })).unwrap();
        let command = GimbalCommand::new(ControlSource::UnixSocket(2), Some(20.0), None, None);
        let progress = arbitrator.execute(command, &handle).await.unwrap();
        assert!(progress.sent_at.is_some());
        assert_eq!(state.axis_baseline_cached(), (Some(20.0), None, None));
        assert_eq!(state.state_cached().pitch, None);
    });
}

#[test]
fn mavlink_system_one_is_not_a_privilege_escalation() {
    assert_eq!(
        ControlSource::Mavlink(1).priority(),
        ControlSource::Mavlink(42).priority()
    );
    assert_eq!(
        ControlSource::Mavlink(1).priority(),
        ControlSource::UnixSocket(2).priority()
    );
}
