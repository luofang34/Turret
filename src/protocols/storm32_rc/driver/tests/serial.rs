//! Deterministic transport outcomes for the actual STorM32 execution path.
use serialport::*;
use std::io::{Read, Write};
use std::sync::{Arc, Mutex};
use std::time::Duration;

pub(super) struct Port {
    pub(super) writes: Arc<Mutex<Vec<u8>>>,
    pub(super) acknowledge: bool,
    pub(super) fail_write: bool,
    pub(super) response: Option<Vec<u8>>,
}
impl Read for Port {
    fn read(&mut self, bytes: &mut [u8]) -> std::io::Result<usize> {
        if let Some(response) = self.response.take() {
            if response.is_empty() {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::BrokenPipe,
                    "missing first response",
                ));
            }
            let count = response.len().min(bytes.len());
            bytes[..count].copy_from_slice(&response[..count]);
            return Ok(count);
        }
        if self.acknowledge {
            bytes[0] = 0x4a;
            Ok(1)
        } else {
            Err(std::io::Error::new(
                std::io::ErrorKind::BrokenPipe,
                "response unavailable",
            ))
        }
    }
}
impl Write for Port {
    fn write(&mut self, bytes: &[u8]) -> std::io::Result<usize> {
        if self.fail_write {
            return Err(std::io::Error::new(
                std::io::ErrorKind::BrokenPipe,
                "write failed",
            ));
        }
        self.writes.lock().unwrap().extend_from_slice(bytes);
        Ok(bytes.len())
    }
    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}
impl SerialPort for Port {
    fn name(&self) -> Option<String> {
        Some("test".to_string())
    }
    fn baud_rate(&self) -> Result<u32> {
        Ok(115200)
    }
    fn data_bits(&self) -> Result<DataBits> {
        Ok(DataBits::Eight)
    }
    fn flow_control(&self) -> Result<FlowControl> {
        Ok(FlowControl::None)
    }
    fn parity(&self) -> Result<Parity> {
        Ok(Parity::None)
    }
    fn stop_bits(&self) -> Result<StopBits> {
        Ok(StopBits::One)
    }
    fn timeout(&self) -> Duration {
        Duration::from_millis(10)
    }
    fn set_baud_rate(&mut self, _: u32) -> Result<()> {
        Ok(())
    }
    fn set_data_bits(&mut self, _: DataBits) -> Result<()> {
        Ok(())
    }
    fn set_flow_control(&mut self, _: FlowControl) -> Result<()> {
        Ok(())
    }
    fn set_parity(&mut self, _: Parity) -> Result<()> {
        Ok(())
    }
    fn set_stop_bits(&mut self, _: StopBits) -> Result<()> {
        Ok(())
    }
    fn set_timeout(&mut self, _: Duration) -> Result<()> {
        Ok(())
    }
    fn write_request_to_send(&mut self, _: bool) -> Result<()> {
        Ok(())
    }
    fn write_data_terminal_ready(&mut self, _: bool) -> Result<()> {
        Ok(())
    }
    fn read_clear_to_send(&mut self) -> Result<bool> {
        Ok(false)
    }
    fn read_data_set_ready(&mut self) -> Result<bool> {
        Ok(false)
    }
    fn read_ring_indicator(&mut self) -> Result<bool> {
        Ok(false)
    }
    fn read_carrier_detect(&mut self) -> Result<bool> {
        Ok(false)
    }
    fn bytes_to_read(&self) -> Result<u32> {
        Ok(0)
    }
    fn bytes_to_write(&self) -> Result<u32> {
        Ok(0)
    }
    fn clear(&self, _: ClearBuffer) -> Result<()> {
        Ok(())
    }
    fn try_clone(&self) -> Result<Box<dyn SerialPort>> {
        Err(Error::new(ErrorKind::Unknown, "test port is exclusive"))
    }
    fn set_break(&self) -> Result<()> {
        Ok(())
    }
    fn clear_break(&self) -> Result<()> {
        Ok(())
    }
}
