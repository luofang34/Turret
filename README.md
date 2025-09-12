# Gimbal

Command-line tool for STorM32 gimbal controllers. Uses the RC Commands protocol.

## Features

- Auto-detects STorM32 devices
- Set gimbal angles, pan modes, standby
- Query status and firmware info
- Structured logging with verbosity levels

## Installation

### Download Binary

Download the latest release from [Releases](../../releases) for your platform.

### Build from Source

```bash
# Install Rust (if needed)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Clone and build
git clone https://github.com/luofang34/gimbal
cd gimbal
cargo build --release
```

## Usage

```bash
# Basic commands (auto-detects device)
gimbal status
gimbal set 10.0 0.0 -15.0
gimbal set -10,0,15          # comma-separated
gimbal set 1.5:-2.3:0        # colon-separated
gimbal center
gimbal version

# Manual device specification
gimbal --device /dev/ttyUSB0 status

# Verbose output for debugging
gimbal -v status          # info
gimbal -vv status         # debug
gimbal -vvv status        # trace (shows raw protocol)

# Individual axis control
gimbal pitch 1500         # RC values 700-2300, 0 to recenter
gimbal roll 0             # recenter roll

# Pan mode and standby
gimbal pan-mode 3         # PANPANPAN mode
gimbal standby true
```

## Protocol

Uses STorM32 RC Commands protocol over serial (115200 baud).

Message format: `[0xFA][len][cmd][payload][crc16]`

Supports: version query, angle setting, status reading, pan modes.

## Device Detection

Auto-detects STM32 Virtual COM Port (VID:0x0483, PID:0x5740).
Falls back to common patterns: `/dev/ttyACM*`, `/dev/ttyUSB*`, `COM*`

## Troubleshooting

- **No device found**: Check USB connection and power
- **Permission denied**: Add user to `dialout` group or use `sudo`
- **Communication errors**: Verify 115200 baud rate and cable quality
