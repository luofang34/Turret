# Turret

Command-line tool for STorM32 gimbal controllers. Uses the RC Commands protocol.

## Features

- Auto-detects STorM32 devices
- Set gimbal angles, pan modes, standby
- Query status and firmware info
- Structured logging with verbosity levels

## Installation

### Download Binary

Download the latest release from [Releases](../../releases) for your platform.

OR:
### From source
With a recent [Rust](https://rustup.rs) installed:
```bash
cargo install turret
```

## Usage

```bash
# Basic commands (auto-detects device)
turret status
turret set 10.0 0.0 -15.0
turret set -10,0,15          # comma-separated
turret set 1.5:-2.3:0        # colon-separated
turret center
turret version

# Manual device specification
turret --device /dev/ttyUSB0 status

# Verbose output for debugging
turret -v status          # info
turret -vv status         # debug
turret -vvv status        # trace (shows raw protocol)

# Individual axis control
turret pitch 1500         # RC values 700-2300, 0 to recenter
turret roll 0             # recenter roll

# Pan mode and standby
turret pan-mode 3         # PANPANPAN mode
turret standby true
```

## Develop
With a recent [Rust](https://rustup.rs) installed:
```bash
# Clone and build
git clone https://github.com/luofang34/turret
cd turret
cargo build --release
```

## Protocol

Uses [STorM32 RC Commands protocol](https://www.olliw.eu/storm32bgc-v1-wiki/Serial_Communication#Serial_Communication_-_RC_Commands) over serial (115200 baud).

Message format: `[0xFA][len][cmd][payload][crc16]`

Supports: version query, angle setting, status reading, pan modes.

## Device Detection

Auto-detects STM32 Virtual COM Port (VID:0x0483, PID:0x5740).
Falls back to common patterns: `/dev/ttyACM*`, `/dev/ttyUSB*`, `COM*`

## Troubleshooting

- **No device found**: Check USB connection and power
- **Permission denied**: Add user to `dialout` group or use `sudo`
- **Communication errors**: Verify 115200 baud rate and cable quality
