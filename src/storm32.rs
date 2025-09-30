use anyhow::{anyhow, Result};
use log::{debug, trace, warn};
use serialport::SerialPort;
use std::io::{Read, Write};
use std::path::Path;
use std::time::Duration;

#[derive(Debug, Clone, Copy)]
pub struct GimbalAngles {
    pub pitch: f32,
    pub roll: f32,
    pub yaw: f32,
}

#[derive(Debug, Clone, Copy)]
pub struct GimbalStatus {
    pub pv: GimbalAngles,  // Process Variable (current angles)
    pub sp: GimbalAngles,  // Set Point (target angles)
}

// RC Command protocol constants based on STorM32 documentation
const RC_START_SIGN_IN: u8 = 0xFA;   // Incoming messages (to gimbal)
const RC_START_SIGN_OUT: u8 = 0xFB;  // Outgoing messages (from gimbal)

// Command IDs from STorM32 RC Commands protocol
#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum RCCommand {
    GetVersion = 1,           // CMD_GETVERSION
    GetVersionStr = 2,        // CMD_GETVERSIONSTR
    GetParameter = 3,         // CMD_GETPARAMETER
    SetParameter = 4,         // CMD_SETPARAMETER
    GetData = 5,              // CMD_GETDATA
    GetDataFields = 6,        // CMD_GETDATAFIELDS
    SetPitch = 10,            // CMD_SETPITCH
    SetRoll = 11,             // CMD_SETROLL
    SetYaw = 12,              // CMD_SETYAW
    SetPanMode = 13,          // CMD_SETPANMODE
    SetStandby = 14,          // CMD_SETSTANDBY
    DoCamera = 15,            // CMD_DOCAMERA
    SetScriptControl = 16,    // CMD_SETSCRIPTCONTROL
    SetAngle = 17,            // CMD_SETANGLE  
    SetPitchRollYaw = 18,     // CMD_SETPITCHROLLYAW
    SetPwmOut = 19,           // CMD_SETPWMOUT
    RestoreParameter = 20,    // CMD_RESTOREPARAMETER
    RestoreAllParameter = 21, // CMD_RESTOREALLPARAMETER
    ActivePanModeSetting = 100, // CMD_ACTIVEPANMODESETTING
    Ack = 150,                // CMD_ACK
}

#[derive(Debug)]
pub struct RCMessage {
    pub command: RCCommand,
    pub payload: Vec<u8>,
}

impl RCMessage {
    pub fn new(command: RCCommand, payload: Vec<u8>) -> Self {
        Self { command, payload }
    }
    
    // Create payload for CMD_SETANGLE command with float angles
    // Format: float1(4) float2(4) float3(4) flags(1) type(1) = 14 bytes total
    pub fn create_set_angle_payload(pitch: f32, roll: f32, yaw: f32) -> Vec<u8> {
        let mut payload = Vec::new();
        
        // Add pitch, roll, yaw as 32-bit floats (little-endian)
        payload.extend_from_slice(&pitch.to_le_bytes());
        payload.extend_from_slice(&roll.to_le_bytes());
        payload.extend_from_slice(&yaw.to_le_bytes());
        
        // Flags: 0x00 for unlimited mode (bypasses RcMin/RcMax)
        // Bit 0x01 = pitch limited, 0x02 = roll limited, 0x04 = yaw limited
        payload.push(0x00);
        
        // Type: 0x00 (not used currently)
        payload.push(0x00);
        
        payload
    }
    
    // Create payload for CMD_SETPITCHROLLYAW command with uint16 values
    // Format: pitch(2) roll(2) yaw(2) = 6 bytes total
    // Values: 700-2300 for control, 0 to recenter
    pub fn create_set_pitch_roll_yaw_payload(pitch: u16, roll: u16, yaw: u16) -> Vec<u8> {
        let mut payload = Vec::new();
        payload.extend_from_slice(&pitch.to_le_bytes());
        payload.extend_from_slice(&roll.to_le_bytes());
        payload.extend_from_slice(&yaw.to_le_bytes());
        payload
    }
    
    // Create payload for individual axis commands (CMD_SETPITCH, CMD_SETROLL, CMD_SETYAW)
    // Format: value(2) = 2 bytes total
    // Values: 700-2300 for control, 0 to recenter
    pub fn create_single_axis_payload(value: u16) -> Vec<u8> {
        value.to_le_bytes().to_vec()
    }
    
    // Create payload for CMD_GETDATAFIELDS
    // Format: bitmask(2) = 2 bytes total
    pub fn create_get_data_fields_payload(bitmask: u16) -> Vec<u8> {
        bitmask.to_le_bytes().to_vec()
    }
    
    // Create payload for CMD_SETPANMODE
    // Format: mode(1) = 1 byte total
    pub fn create_set_pan_mode_payload(mode: u8) -> Vec<u8> {
        vec![mode]
    }
    
    // Create payload for CMD_SETSTANDBY
    // Format: state(1) = 1 byte total
    pub fn create_set_standby_payload(state: u8) -> Vec<u8> {
        vec![state]
    }
    
    // Serialize message to bytes according to RC Commands protocol
    pub fn serialize(&self) -> Vec<u8> {
        let mut message = Vec::new();
        
        // Start sign (0xFA for incoming messages)
        message.push(RC_START_SIGN_IN);
        
        // Length (payload only, excluding start sign, length byte, command byte, and CRC)
        message.push(self.payload.len() as u8);
        
        // Command byte
        message.push(self.command as u8);
        
        // Payload
        message.extend_from_slice(&self.payload);
        
        // Calculate and append CRC16 (x25 algorithm)
        let crc = self.calculate_crc16(&message[1..]);
        message.extend_from_slice(&crc.to_le_bytes());
        
        message
    }
    
    // CRC16 x25 algorithm (same as Mavlink)
    fn calculate_crc16(&self, data: &[u8]) -> u16 {
        let mut crc: u16 = 0xFFFF;

        for &byte in data {
            crc ^= byte as u16;
            for _ in 0..8 {
                if (crc & 1) != 0 {
                    crc = (crc >> 1) ^ 0x8408;
                } else {
                    crc >>= 1;
                }
            }
        }

        !crc
    }

    // Static version for verification (doesn't need self)
    fn verify_crc16(data: &[u8], expected_crc: u16) -> bool {
        let mut crc: u16 = 0xFFFF;

        for &byte in data {
            crc ^= byte as u16;
            for _ in 0..8 {
                if (crc & 1) != 0 {
                    crc = (crc >> 1) ^ 0x8408;
                } else {
                    crc >>= 1;
                }
            }
        }

        !crc == expected_crc
    }
    
    // Parse incoming message from gimbal
    pub fn parse_response(data: &[u8]) -> Result<Vec<u8>> {
        // Handle single byte ACK responses (common for SET commands)
        if data.len() == 1 {
            // Single byte response - likely a simple ACK
            // Common ACK values: 0x4A (74) seems to be success
            return Ok(vec![]);  // Return empty payload for ACK
        }
        
        if data.len() < 3 {
            return Err(anyhow!("Response too short: got {} bytes, need at least 3", data.len()));
        }
        
        // Check start sign
        if data[0] != RC_START_SIGN_OUT {
            return Err(anyhow!("Invalid start sign: expected 0xFB, got 0x{:02X}", data[0]));
        }
        
        let length = data[1] as usize;
        let expected_total = length + 4;  // start + length + payload + 2-byte CRC
        
        if data.len() < expected_total {
            // For SET commands, sometimes the response is just an ACK without payload
            // Try to handle minimal responses gracefully
            if data.len() >= 3 && length <= 1 {
                // Minimal ACK response with just command byte
                return Ok(vec![]);  // Empty payload for ACK
            }
            return Err(anyhow!("Incomplete message: got {} bytes, expected {}", data.len(), expected_total));
        }
        
        // Extract payload (skip start, length bytes only - payload includes command byte)
        let payload_start = 2;
        let payload_end = payload_start + length;
        
        if payload_end > data.len() - 2 {  // -2 for CRC
            // Handle case where there's no payload (just ACK)
            if length == 1 && data.len() >= 5 {
                return Ok(vec![]);  // Empty payload
            }
            return Err(anyhow!("Invalid payload length"));
        }
        
        let payload = if payload_end > payload_start {
            data[payload_start..payload_end].to_vec()
        } else {
            vec![]  // Empty payload for ACK responses
        };

        // Verify CRC16 (last 2 bytes of message)
        if data.len() >= expected_total {
            let crc_start = payload_end;
            if crc_start + 2 <= data.len() {
                let received_crc = u16::from_le_bytes([data[crc_start], data[crc_start + 1]]);

                // CRC is calculated over: length byte + command byte + payload
                let crc_data = &data[1..payload_end];

                if !Self::verify_crc16(crc_data, received_crc) {
                    return Err(anyhow!("CRC verification failed: expected valid CRC, got 0x{:04X}", received_crc));
                }

                trace!("CRC verification passed: 0x{:04X}", received_crc);
            }
        }

        Ok(payload)
    }
}

pub struct Storm32Gimbal {
    port: Box<dyn SerialPort>,
    current_status: GimbalStatus,
}

impl Storm32Gimbal {
    // Validate device path for security
    fn validate_device_path(device_path: &str) -> Result<()> {
        // Reject path traversal attempts
        if device_path.contains("..") {
            return Err(anyhow!("Invalid device path: path traversal detected"));
        }

        // Check if path exists
        let path = Path::new(device_path);
        if !path.exists() {
            return Err(anyhow!("Device path does not exist: {}", device_path));
        }

        // On Unix systems, verify it's a character device
        #[cfg(unix)]
        {
            use std::os::unix::fs::FileTypeExt;
            let metadata = std::fs::metadata(path)
                .map_err(|e| anyhow!("Cannot access device path: {}", e))?;
            let file_type = metadata.file_type();

            if !file_type.is_char_device() {
                return Err(anyhow!("Path is not a character device: {}", device_path));
            }
        }

        Ok(())
    }

    pub fn new(device_path: &str) -> Result<Self> {
        // Validate device path
        Self::validate_device_path(device_path)?;

        let port = serialport::new(device_path, 115200)
            .timeout(Duration::from_millis(1000))
            .open()
            .map_err(|e| anyhow!("Failed to open serial port {}: {}", device_path, e))?;

        let gimbal = Self {
            port,
            current_status: GimbalStatus {
                pv: GimbalAngles { pitch: 0.0, roll: 0.0, yaw: 0.0 },
                sp: GimbalAngles { pitch: 0.0, roll: 0.0, yaw: 0.0 },
            },
        };

        Ok(gimbal)
    }

    pub fn get_status(&self) -> GimbalStatus {
        self.current_status
    }
    
    // Send RC command and read response
    fn send_rc_command(&mut self, message: &RCMessage) -> Result<Vec<u8>> {
        // Serialize and send message
        let data = message.serialize();
        trace!("Sending RC command: {:02X?}", data);
        self.port.write_all(&data)
            .map_err(|e| anyhow!("Failed to write to serial port: {}", e))?;
        self.port.flush()
            .map_err(|e| anyhow!("Failed to flush serial port: {}", e))?;
        
        // Read response with multiple attempts and accumulated buffer
        let mut buffer = vec![0u8; 256];
        let mut total_bytes = 0;
        let max_attempts = 10;
        let mut attempts = 0;
        
        while attempts < max_attempts && total_bytes < 4 {
            match self.port.read(&mut buffer[total_bytes..]) {
                Ok(bytes_read) => {
                    if bytes_read > 0 {
                        total_bytes += bytes_read;
                        // Check if we have enough for a minimal response
                        if total_bytes >= 4 {
                            break;
                        }
                    } else if total_bytes > 0 {
                        // We got some data but no more coming
                        break;
                    }
                    attempts += 1;
                    std::thread::sleep(Duration::from_millis(10));
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {
                    if total_bytes > 0 {
                        // We got some data, might be enough
                        break;
                    }
                    attempts += 1;
                    continue;
                }
                Err(e) => return Err(anyhow!("Failed to read response: {}", e)),
            }
        }
        
        if total_bytes == 0 {
            return Err(anyhow!("No response received"));
        }
        
        trace!("Received RC response: {:02X?}", &buffer[..total_bytes]);
        RCMessage::parse_response(&buffer[..total_bytes])
    }
    
    // Validate angle values (typical gimbal range is -180 to +180 degrees)
    fn validate_angle(angle: f32, name: &str) -> Result<()> {
        if !angle.is_finite() {
            return Err(anyhow!("{} must be a finite number", name));
        }
        if angle < -180.0 || angle > 180.0 {
            return Err(anyhow!("{} must be between -180° and +180° (got {:.1}°)", name, angle));
        }
        Ok(())
    }

    // Validate RC values (700-2300 or 0 for recenter)
    fn validate_rc_value(value: u16, name: &str) -> Result<()> {
        if value != 0 && (value < 700 || value > 2300) {
            return Err(anyhow!("{} must be 0 (recenter) or 700-2300 (got {})", name, value));
        }
        Ok(())
    }

    // Validate pan mode (0-5 only)
    fn validate_pan_mode(mode: u8) -> Result<()> {
        if mode > 5 {
            return Err(anyhow!("Pan mode must be 0-5 (got {})", mode));
        }
        Ok(())
    }

    // Set gimbal angles using CMD_SETANGLE (float precision)
    pub async fn set_angles(&mut self, pitch: f32, roll: f32, yaw: f32) -> Result<()> {
        // Validate inputs
        Self::validate_angle(pitch, "Pitch")?;
        Self::validate_angle(roll, "Roll")?;
        Self::validate_angle(yaw, "Yaw")?;

        let payload = RCMessage::create_set_angle_payload(pitch, roll, yaw);
        let message = RCMessage::new(RCCommand::SetAngle, payload);

        match self.send_rc_command(&message) {
            Ok(_response) => {
                // Update our setpoint immediately
                self.current_status.sp = GimbalAngles { pitch, roll, yaw };
                Ok(())
            }
            Err(e) => Err(anyhow!("Failed to set angles: {}", e)),
        }
    }
    
    // Set gimbal angles using CMD_SETPITCHROLLYAW (RC values 700-2300)
    pub async fn set_angles_rc(&mut self, pitch: u16, roll: u16, yaw: u16) -> Result<()> {
        let payload = RCMessage::create_set_pitch_roll_yaw_payload(pitch, roll, yaw);
        let message = RCMessage::new(RCCommand::SetPitchRollYaw, payload);
        
        match self.send_rc_command(&message) {
            Ok(_response) => Ok(()),
            Err(e) => Err(anyhow!("Failed to set RC angles: {}", e)),
        }
    }
    
    // Set individual pitch axis using CMD_SETPITCH
    pub async fn set_pitch(&mut self, value: u16) -> Result<()> {
        Self::validate_rc_value(value, "Pitch")?;

        let payload = RCMessage::create_single_axis_payload(value);
        let message = RCMessage::new(RCCommand::SetPitch, payload);

        match self.send_rc_command(&message) {
            Ok(_response) => Ok(()),
            Err(e) => Err(anyhow!("Failed to set pitch: {}", e)),
        }
    }

    // Set individual roll axis using CMD_SETROLL
    pub async fn set_roll(&mut self, value: u16) -> Result<()> {
        Self::validate_rc_value(value, "Roll")?;

        let payload = RCMessage::create_single_axis_payload(value);
        let message = RCMessage::new(RCCommand::SetRoll, payload);

        match self.send_rc_command(&message) {
            Ok(_response) => Ok(()),
            Err(e) => Err(anyhow!("Failed to set roll: {}", e)),
        }
    }

    // Set individual yaw axis using CMD_SETYAW
    pub async fn set_yaw(&mut self, value: u16) -> Result<()> {
        Self::validate_rc_value(value, "Yaw")?;

        let payload = RCMessage::create_single_axis_payload(value);
        let message = RCMessage::new(RCCommand::SetYaw, payload);

        match self.send_rc_command(&message) {
            Ok(_response) => Ok(()),
            Err(e) => Err(anyhow!("Failed to set yaw: {}", e)),
        }
    }
    
    // Recenter individual axes (value = 0)
    pub async fn recenter_pitch(&mut self) -> Result<()> {
        self.set_pitch(0).await
    }
    
    pub async fn recenter_roll(&mut self) -> Result<()> {
        self.set_roll(0).await
    }
    
    pub async fn recenter_yaw(&mut self) -> Result<()> {
        self.set_yaw(0).await
    }
    
    // Get current gimbal status using CMD_GETDATAFIELDS
    pub async fn update_status(&mut self) -> Result<()> {
        // Request IMU1 angles (0x0020) which contains current angles
        let bitmask = 0x0020u16; // LIVEDATA_IMU1ANGLES
        let payload = RCMessage::create_get_data_fields_payload(bitmask);
        let message = RCMessage::new(RCCommand::GetDataFields, payload);
        
        match self.send_rc_command(&message) {
            Ok(response) => {
                // Check if we got a meaningful response
                if response.is_empty() {
                    debug!("GetDataFields command returned empty response (possibly unsupported)");
                    return Ok(());
                }
                self.parse_data_fields_response(&response)?;
                Ok(())
            }
            Err(e) => {
                // If we can't get current angles, that's not necessarily fatal
                // Just log the error and continue
                warn!("Could not update gimbal status: {}", e);
                Ok(())
            }
        }
    }
    
    // Parse response from CMD_GETDATAFIELDS
    fn parse_data_fields_response(&mut self, response: &[u8]) -> Result<()> {
        if response.len() < 7 {
            return Err(anyhow!("Data fields response too short: {} bytes", response.len()));
        }
        
        // Response format: [command_byte, bitmask_low, bitmask_high, data...]
        // Skip command byte (index 0) and get bitmask from bytes 1-2
        let _bitmask = u16::from_le_bytes([response[1], response[2]]);
        
        // For IMU1ANGLES (0x0020), the data should be 3 x 2-byte signed integers
        // representing angles in 0.01 degree units
        if response.len() >= 7 {
            let pitch_raw = i16::from_le_bytes([response[3], response[4]]);
            let roll_raw = i16::from_le_bytes([response[5], response[6]]);
            // We only have 4 bytes of data, so only pitch and roll are available
            let yaw_raw = 0i16; // Default to 0 if not enough data
            
            // Convert from 0.01 degree units to degrees
            self.current_status.pv = GimbalAngles {
                pitch: pitch_raw as f32 / 100.0,
                roll: roll_raw as f32 / 100.0,
                yaw: yaw_raw as f32 / 100.0,
            };
        }
        
        Ok(())
    }
    
    // Set pan mode using CMD_SETPANMODE
    pub async fn set_pan_mode(&mut self, mode: u8) -> Result<()> {
        Self::validate_pan_mode(mode)?;

        let payload = RCMessage::create_set_pan_mode_payload(mode);
        let message = RCMessage::new(RCCommand::SetPanMode, payload);

        match self.send_rc_command(&message) {
            Ok(_response) => Ok(()),
            Err(e) => Err(anyhow!("Failed to set pan mode: {}", e)),
        }
    }
    
    // Set standby mode using CMD_SETSTANDBY
    pub async fn set_standby(&mut self, enabled: bool) -> Result<()> {
        let state = if enabled { 1u8 } else { 0u8 };
        let payload = RCMessage::create_set_standby_payload(state);
        let message = RCMessage::new(RCCommand::SetStandby, payload);
        
        match self.send_rc_command(&message) {
            Ok(_response) => Ok(()),
            Err(e) => Err(anyhow!("Failed to set standby: {}", e)),
        }
    }
    
    // Get firmware version using CMD_GETVERSION
    pub async fn get_version(&mut self) -> Result<String> {
        let message = RCMessage::new(RCCommand::GetVersion, Vec::new());
        
        match self.send_rc_command(&message) {
            Ok(response) => {
                // CMD_GETVERSION returns structured data, not a string
                if response.len() >= 6 {
                    let firmware_version = u16::from_le_bytes([response[0], response[1]]);
                    let layout_version = u16::from_le_bytes([response[2], response[3]]);
                    let capabilities = u16::from_le_bytes([response[4], response[5]]);
                    
                    Ok(format!("Firmware: {}.{}, Layout: {}.{}, Capabilities: 0x{:04X}",
                        firmware_version >> 8, firmware_version & 0xFF,
                        layout_version >> 8, layout_version & 0xFF,
                        capabilities))
                } else {
                    Err(anyhow!("Version response too short"))
                }
            }
            Err(e) => Err(anyhow!("Failed to get version: {}", e)),
        }
    }
    
    // Get firmware version string using CMD_GETVERSIONSTR  
    pub async fn get_version_string(&mut self) -> Result<String> {
        let message = RCMessage::new(RCCommand::GetVersionStr, Vec::new());
        
        match self.send_rc_command(&message) {
            Ok(response) => {
                if response.len() >= 48 {
                    // Response contains: 16-byte version string, 16-byte name string, 16-byte board string
                    let version_str = String::from_utf8_lossy(&response[0..16]).trim_end_matches('\0').to_string();
                    let name_str = String::from_utf8_lossy(&response[16..32]).trim_end_matches('\0').to_string();
                    let board_str = String::from_utf8_lossy(&response[32..48]).trim_end_matches('\0').to_string();
                    
                    Ok(format!("Version: '{}', Name: '{}', Board: '{}'", version_str, name_str, board_str))
                } else {
                    Err(anyhow!("Version string response too short"))
                }
            }
            Err(e) => Err(anyhow!("Failed to get version string: {}", e)),
        }
    }
    
    // Debug method to send raw RC command for testing
    #[allow(dead_code)]
    pub fn debug_send_raw(&mut self, command: RCCommand, payload: Vec<u8>) -> Result<Vec<u8>> {
        let message = RCMessage::new(command, payload);
        self.send_rc_command(&message)
    }
}