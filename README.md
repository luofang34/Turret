# Turret

Command-line tool for STorM32 gimbal controllers. Uses the RC Commands protocol.

## Features

- **CLI Mode**: Direct command-line control
- **Daemon Mode**: Background service with Unix socket IPC and MAVLink
- Auto-detects STorM32 devices
- Set gimbal angles, pan modes, standby
- Query status and firmware info
- Structured logging with verbosity levels
- Priority-based command arbitration

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

### CLI Mode (Direct Control)

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

### Daemon Mode (Background Service)

```bash
# Start daemon (auto-detects device)
turret --daemon

# Or with custom config
turret --daemon --config turret.yaml

# Connect via Unix socket (JSON commands)
echo '{"cmd": "status"}' | nc -U /tmp/turret.sock
echo '{"cmd": "set", "pitch": 10.0, "roll": 0.0, "yaw": -15.0}' | nc -U /tmp/turret.sock
echo '{"cmd": "center"}' | nc -U /tmp/turret.sock
echo '{"cmd": "pan_mode", "mode": 3}' | nc -U /tmp/turret.sock
echo '{"cmd": "standby", "enabled": true}' | nc -U /tmp/turret.sock
echo '{"cmd": "help"}' | nc -U /tmp/turret.sock     # full schema

# MAVLink on UDP:14550 - use pymavlink or QGroundControl
# Implements Gimbal Manager protocol
```

The IPC wire format uses `snake_case` for both the `cmd` discriminator
and field names — matches the response shape (`firmware_version`,
`pan_mode`, ...) and standard JSON conventions. Send `{"cmd":"help"}`
for the full machine-readable schema.

### Yaw calibration

If your gimbal's IMU yaw reading is biased relative to its commanded
zero (typical symptom: `set yaw=0` then `status` returns yaw≠0 by a
fixed amount), calibrate from the daemon — no GUI required:

```bash
# Auto-calibrate: daemon centers the gimbal, samples raw IMU yaw,
# stores the bias as the new offset.
echo '{"cmd": "calibrate_yaw"}' | nc -U /tmp/turret.sock
# → {"status":"ok","data":{"yaw_offset_deg": -4.48, "prior_yaw_offset_deg": 0.0}}

# Or set an explicit value (e.g. from a measurement you took elsewhere):
echo '{"cmd": "set_yaw_offset", "deg": -4.5}' | nc -U /tmp/turret.sock

# 0.0 disables the offset:
echo '{"cmd": "set_yaw_offset", "deg": 0.0}' | nc -U /tmp/turret.sock
```

The offset persists to `$XDG_STATE_HOME/turret/calibration.toml` (default
`~/.local/state/turret/calibration.toml`) and is auto-loaded on daemon
start. The current offset is included in every `status` response as
`"yaw_offset_deg"`.

Read-side correction model: commands hit the device exactly as you wrote
them (the gimbal physically goes to the asked-for angle), and the
calibration offset is subtracted from the IMU's report before it reaches
any consumer (IPC `status`, MAVLink `GIMBAL_DEVICE_ATTITUDE_STATUS`).

### Self-test

Verify SP→PV agreement across the gimbal's working range without leaving
the daemon — useful both as a post-calibration check and as a
quick-and-dirty health probe before a mission:

```bash
echo '{"cmd": "selftest"}' | nc -U /tmp/turret.sock -q 20
```

The daemon sweeps `(0,0,0) → (15,0,0) → (-15,0,0) → (0,0,15) →
(0,0,-15) → (0,0,0)`, waits 2.5 s after each step, samples
operator-frame attitude, and returns per-sample SP/PV/error plus per-axis
max error and a pass/fail against a 1° tolerance. The whole run takes
~16 s. Refuses with an error if standby is engaged. The original pose is
restored at the end.

A clean run on a calibrated gimbal looks like:

```json
{"passed": true,
 "tolerance_deg": 1.0,
 "max_error_deg": {"pitch": 0.03, "roll": 0.0, "yaw": 0.27},
 "samples": [...]}
```

If the gimbal hasn't been calibrated yet, the failing samples make the
bias obvious — that's a good cue to run `{"cmd":"calibrate_yaw"}`.

**Configuration** (`turret.yaml`):
```yaml
device:
  path: auto

ipc:
  socket_path: /tmp/turret.sock

mavlink:
  enabled: true
  udp_port: 14550
  sysid: 1
  compid: 154
```

**Priority System**: MAVLink autopilot (highest) > GCS > Unix socket > CLI (lowest)

**Daemon behavior**:
- 1 Hz `HEARTBEAT` (advertises `MAV_TYPE_GIMBAL` on `MAV_COMP_ID_GIMBAL`)
- 5 Hz `GIMBAL_MANAGER_STATUS`, broadcast to every recorded peer
- 4 Hz attitude poll → `GIMBAL_DEVICE_ATTITUDE_STATUS` broadcast.
  `COMMS_ERROR` flag is set for ~1 s after any failed poll.
- 8 consecutive poll failures → automatic reconnect (scan + reopen,
  exponential backoff to 10 s).
- `SIGTERM` / `SIGINT` start a 2 s grace shutdown; in-flight tasks are
  drained, then aborted; the IPC socket file is removed.
- A critical task exiting unexpectedly (IPC, MAVLink, reconnect) tears
  the daemon down instead of leaving it half-running.

## Use as a library

Two consumption modes:

- **Lean driver** (`default-features = false`): just the `Storm32RC`
  driver and the `GimbalDevice` trait. Pulls 4 direct deps
  (`serde / serialport / thiserror / tracing`) and no tokio.

  ```toml
  [dependencies]
  turret = { version = "0.1", default-features = false }
  ```

  ```rust,no_run
  let mut g = turret::detect_gimbal("/dev/ttyACM0")?;
  g.set_attitude(10.0, 0.0, -15.0)?;
  # Ok::<_, turret::Error>(())
  ```

- **Embedded MAVLink manager**: bring your own `dyn GimbalDevice`
  (different protocol, simulator, network-attached gimbal) and let
  Turret handle MAVLink discovery, primary-control arbitration, hot
  attitude broadcast, IPC, and bounded shutdown. The default features
  pull tokio + the daemon stack.

  See `turret::daemon` module-level docs for the full embedding example.

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
