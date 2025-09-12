# STorM32 Gimbal Control - RC Commands Implementation

A Rust-based command-line control program for STorM32 BGC gimbals using the official RC Commands protocol.

## Features

- **RC Commands Protocol**: Implements the official STorM32 RC Commands communication protocol
- **Command-line Interface**: Simple CLI with subcommands for different operations
- **Automatic Device Detection**: Scans and auto-detects STorM32 gimbal controllers
- **Angle Control**: Set absolute gimbal angles with proper unit conversion
- **Status Monitoring**: Query current gimbal angles and firmware version
- **Cross-platform**: Supports macOS, Linux, and Windows

## Hardware Requirements

- STorM32 gimbal controller connected via USB
- Device appears as "STM32 Virtual COM Port" (VID: 0x0483, PID: 0x5740)
- Compatible with STorM32 BGC firmware supporting RC Commands

## Installation

```bash
git clone <repository>
cd gimbal
cargo build --release
```

## Usage

### Basic Commands

```bash
# Get help
cargo run -- --help

# Auto-detect device and show status
cargo run -- --auto status

# Set specific angles (pitch, roll, yaw in degrees)
cargo run -- set 10.0 0.0 -15.0

# Center gimbal (all axes to 0°)
cargo run -- center

# Get firmware version
cargo run -- version

# Specify device manually
cargo run -- --device /dev/ttyUSB0 status
```

### Using the Built Binary

```bash
./target/release/gimbal --help
./target/release/gimbal --auto status
./target/release/gimbal set 5.0 -2.5 20.0
./target/release/gimbal center
```

## RC Commands Protocol

This implementation uses the official STorM32 RC Commands protocol:

### Message Format
- **Start Sign**: 0xFA (incoming) / 0xFB (outgoing)
- **Length**: Payload length + command byte
- **Command**: 1-byte command identifier
- **Payload**: Command-specific data
- **CRC16**: x25 checksum (same as Mavlink)

### Supported Commands
- `CMD_GETVERSION` (#1): Get firmware version string
- `CMD_SETANGLE` (#17): Set absolute gimbal angles
- `CMD_GETANGLES` (#23): Get current gimbal angles

### Angle Format
- Internal units: 0.01 degrees (16-bit signed integers)
- Range: ±327.67 degrees
- Absolute positioning with configurable flags

## Architecture

```
src/
├── main.rs           # CLI application entry point
├── device_scanner.rs # USB device detection and enumeration
└── storm32.rs        # RC Commands protocol implementation
```

## Protocol Details

The RC Commands implementation provides:

1. **Structured Communication**: Well-defined message format with error checking
2. **CRC Validation**: 16-bit x25 CRC for data integrity
3. **Response Parsing**: Proper handling of gimbal responses
4. **Unit Conversion**: Automatic conversion between degrees and internal units
5. **Error Handling**: Comprehensive error reporting and recovery

## Examples

```bash
# Set gimbal to look 15° down, level roll, 30° left
gimbal set -15.0 0.0 30.0

# Check current position
gimbal --auto status
# Output:
# Current angles (PV): pitch=-15.0°, roll=0.1°, yaw=29.8°
# Target angles (SP):  pitch=-15.0°, roll=0.0°, yaw=30.0°

# Return to center position
gimbal center
```

## Troubleshooting

- **Device not found**: Ensure STorM32 controller is connected and powered
- **Permission denied**: On Linux/Mac, add user to dialout group or run with sudo
- **Communication errors**: Verify baud rate (115200) and USB cable quality
- **Invalid responses**: Check gimbal firmware supports RC Commands protocol

## Protocol Reference

Based on the official STorM32 documentation:
- [STorM32 Serial Communication](https://www.olliw.eu/storm32bgc-v1-wiki/Serial_Communication)
- RC Commands section for detailed protocol specification
- Mavlink Communication for CRC algorithm reference

## Testing

Test the implementation:
1. Connect STorM32 gimbal via USB
2. Run: `cargo run -- --auto status`
3. Verify device detection and basic communication
4. Test angle setting with small increments: `cargo run -- set 5.0 0.0 0.0`
5. Observe physical gimbal movement matching commands