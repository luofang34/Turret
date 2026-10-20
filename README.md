# Turret

Portable gimbal and camera control contracts, with host drivers and command-line tools.

## What this is

Turret provides a portable control contract for gimbals and camera zoom.
`turret-core` contains capability declarations, typed commands, independent
observations, and command deadlines. It requires neither an operating system
nor heap allocation. The host supplies monotonic time, scheduling, and I/O.
`turret-dji` provides bounded DUML parsing and the experimental Mini 3 pitch
encoder. It does not provide flight control or certify device completion.
`turret-mavlink` provides portable Gimbal Manager client and camera command
policies. It also requires neither an operating system nor heap allocation;
the host validates device identity and performs the final transport write.

The `turret` package supplies host drivers, a CLI, local IPC, and a MAVLink
Gimbal Manager. The STorM32 backend uses RC Commands over serial. Pilotage can
use the portable contract with its own device connection and authority service.

### Operator protocols are pluggable

The daemon's core (`daemon::state::StateManager`, `daemon::arbitrator`,
`daemon::gimbal_handle::GimbalHandle`) owns no MAVLink types. Protocol
front-ends — `daemon::mavlink_manager` for MAVLink, `daemon::ipc_server`
for the Unix socket — are peers that translate inbound frames to
`GimbalCommand`, dispatch through the arbitrator, and translate outbound
state back. Adding a new operator protocol (CCSDS, a custom mission-bus
frame, gRPC) means writing a new module alongside `mavlink_manager`,
not touching the arbitrator, state, or gimbal handle.

Today only MAVLink (+ the local IPC) is implemented. That's a deliberate
scope choice, not architectural lock-in: assume MAVLink is one of
potentially-several front-ends, but ship one well before the second.

### Hardware protocols are pluggable too

The hardware side uses `gimbal::GimbalDevice`. A backend declares its
capabilities, executes a typed payload command before its write deadline, and
reports partial observations. Unsupported axes remain absent. A successful
write does not establish device acceptance or physical completion. Recenter
uses the backend's declared operation; its target remains unknown when the device
configuration does not expose that pose.

A host must execute the stop or cancel effects returned by `turret-core`.
Write expiry, continuous-command expiry, ACK timeout, and completion timeout
have separate meanings. A host stop request does not establish a device
watchdog guarantee.

The host retains failed stop and cancel requests with their command identities.
They block new motion on the affected lane until an explicit poll or stop
operation succeeds. The other lane can remain available. Device replacement
is refused while the old owner has an unresolved stop or cancel request;
discovering another device does not prove that the old device stopped.

### Mini 3 accessory video request

`turret-dji::Mini3AccessoryKeyframe` provides one finite request for the Mini 3
upper accessory mux channel (port 22345). It emits three fixed APP `02` → `0e`
DUML `01/01` frames, flags `00`, with 11-byte payloads whose zero-based bytes 5
and 9 are `0x24`, `0x24`, then `0x04`. All other payload bytes remain zero.
This experimental sequence is scoped to the tested Mini 3 HEVC stream; H.264 and
other devices/firmware remain unverified.
It is not flight, yaw, zoom, or lower USB CDC support.

The host reserves three wrapping wire sequences, supplies a unique connection
token, readiness time, and absolute deadline, then calls `write_available` at
the final nonblocking write boundary. Report the accepted byte count and a fresh
`turret_core::Tick` sampled after the write returns. Short writes resume only the
unwritten suffix; each complete frame starts a minimum 50 ms delay. Translate
would-block into a zero-byte receipt. Serialize writes with `close`/`cancel`;
errors, session changes, cancellation, and deadlines permanently end that request.
Aborting only stops further transmission; it does not prove the request bits were
cleared on the device. If the clear frame was not fully accepted, device state is
unknown. A reconnect requires the caller to initiate a new explicit request and
token. `TransportAccepted` means only that all bytes reached the transport: the video decoder must independently
establish a complete keyframe and usable picture.

The two `keyframe/fixtures/mini3-request-*.bin` test vectors contain only the
96 transmitted command bytes from independent Mini 3 accessory requests. They
contain no received video or device identifiers.

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

### CLI

The CLI is a thin client. Each subcommand prefers the daemon's IPC
socket (`/tmp/turret.sock` by default; override with `--socket <path>`).
When the daemon is running, commands route through it — same arbitration,
calibration, and primary-control as MAVLink/IPC clients see. When no
daemon is running, the CLI opens the device directly for the call.

```bash
# Basic commands — work the same way whether or not a daemon is running.
turret status
turret set 10.0 0.0 -15.0
turret set -10,0,15          # comma-separated
turret set 1.5:-2.3:0        # colon-separated
turret center
turret version
turret pan-mode 3            # PANPANPAN
turret standby true

# Pick a specific daemon by socket path (multi-gimbal setups).
turret --socket /tmp/turret-front.sock status
turret --socket /tmp/turret-rear.sock  set 0,0,30

# Direct-only commands (no IPC equivalent — daemon must not be holding
# the port). Errors with `Error::DeviceBusy` if a daemon is running.
turret pitch 1500            # RC value 700-2300, 0 to recenter
turret roll 0                # recenter roll
turret version-str

# Direct-mode device override (only used when daemon socket isn't reachable).
turret --device /dev/ttyUSB0 status

# Verbose tracing.
turret -v status             # info
turret -vv status            # debug
turret -vvv status           # trace (raw protocol)
```

If a daemon is holding the device and you try a direct-only subcommand
(`pitch`, `roll`, `yaw`, `version-str`), you'll get
`Error::DeviceBusy` rather than a misleading "not detected" — stop the
daemon to use them, or use `turret set` (which routes via IPC) instead.

### Daemon mode (background service)

```bash
# Start daemon (auto-detects device)
turret --daemon

# Or with custom config
turret --daemon --config turret.toml
```

Once running, the CLI talks to it automatically (see above). For
machine-readable / programmatic use, the IPC socket speaks line-delimited
JSON:

```bash
echo '{"cmd": "status"}'                                      | nc -U /tmp/turret.sock
echo '{"cmd": "set", "pitch": 10.0, "roll": 0.0, "yaw": -15}' | nc -U /tmp/turret.sock
echo '{"cmd": "center"}'                                      | nc -U /tmp/turret.sock
echo '{"cmd": "pan_mode", "mode": 3}'                         | nc -U /tmp/turret.sock
echo '{"cmd": "standby", "enabled": true}'                    | nc -U /tmp/turret.sock
echo '{"cmd": "help"}'                                        | nc -U /tmp/turret.sock

# MAVLink on UDP:14550 — pymavlink, QGroundControl, etc.
# Implements Gimbal Manager protocol.
```

The IPC wire format uses `snake_case` for both the `cmd` discriminator
and field names — matches the response shape (`firmware_version`,
`pan_mode`, ...) and standard JSON conventions. Send `{"cmd":"help"}`
for the full machine-readable schema.

`set` and `center` responses include command identity
(`epoch`, `generation`, `id`, `sequence`) and separate write, device-ACK,
rejection, timeout, and measured-completion evidence in `data`. An `ok`
response means the manager dispatched the command without a known failure;
it does not imply device acknowledgement or physical arrival. A known
execution failure returns `error` with the same evidence. `status` exposes
the latest retained receipt per lane in `command_progress`, including commands
revoked by a capability change or ended by reconnection. Compare identities
before associating a status receipt with a command; this is a bounded snapshot,
not command history. Identities and monotonic microsecond timestamps apply
only within one daemon/device-owner lifetime.

### Reading attitude after a `set`

`status` returns the most recent attitude polled by the daemon's 4 Hz
attitude loop — at most ~250 ms behind the IMU, but more importantly
it doesn't wait for the gimbal to physically settle on the new SP. The
STorM32 motor traverses at roughly 10°/s, so a `set 0,0,10` followed
immediately by `status` will show pitch/yaw still en route. Allow ~1.5
to 2 seconds of motor travel for a 10° swing before reading status if
you need the settled value:

```bash
turret set 0,0,10
sleep 2          # let the gimbal physically reach the SP
turret status    # now reflects the new pose
```

For a verified round-trip across the full working range, use
`{"cmd":"selftest"}` — it sweeps, waits per-step, samples freshly, and
reports per-axis SP/PV errors.

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

#### Continuous drift correction

After the boot-time calibration captures the initial bias, the daemon
runs a slow integral controller on every 4 Hz attitude poll: once the
gimbal has been settled at the most recent SP for 2 s, the operator-frame
`PV - SP` gap is treated as accumulated IMU drift and a tiny slice of
it (capped at 0.05° per call → ~0.2°/s of correction) is added to the
calibration offset. Geometric convergence with a ~50 s time constant
keeps the offset honest over hours, so you don't need to re-run
`calibrate_yaw` mid-session as the IMU drifts.

The corrected offset is persisted back to the same XDG state file
(throttled — at most one disk write per minute, and only after the
offset has slid more than 0.05° from the last persisted value).

The corrector assumes the gimbal is at rest in the world between SETs,
which is true for static mounts, parked vehicles, and pan-mode HOLD on
moving vehicles. In actively-yawing flight with pan-mode PAN it can
mistake vehicle motion for IMU drift; if that's your deployment,
disable it with `gimbal.yaw_corrector_enabled = false` in `turret.toml`
(defaults to on).

#### Future hardware paths

- **Encoder feedback** on the gimbal motor shafts (a STorM32 firmware
  option) would give absolute axis positions independent of the IMU.
  From the daemon's perspective the data arrives through the same
  `IMU1ANGLES` channel and is drift-free for free; no daemon-side
  change required.
- **Non-orthogonal axis design** (each axis off-vertical so gravity
  contributes a yaw-axis reference) is a vendor-side choice the
  STorM32 firmware abstracts; transparent at the host level.
- **Cross-IMU comparison** with autopilot AHRS for actively-flying
  pan-mode-PAN deployments: planned. We already ingest
  `AUTOPILOT_STATE_FOR_GIMBAL_DEVICE`; a future revision will use
  vehicle yaw as the reference instead of `PV - SP`.

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

**Configuration** (`turret.toml`):
```toml
[device]
path = "auto"

[ipc]
socket_path = "/tmp/turret.sock"

[mavlink]
enabled = true
transport = "udp"
bind_addr = "127.0.0.1"
udp_port = 14550
sysid = 1
compid = 154

[gimbal]
yaw_corrector_enabled = true
```

**Priority System**: MAVLink autopilot (highest) > GCS > Unix socket (lowest)

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

Choose the package for the execution environment:

- **Embedded firmware:** link `turret-core`, and a protocol package such as
  `turret-dji` when required. Supply a monotonic clock and bounded I/O. Check
  commands at the final device write boundary. Keep authority and motor control
  in the firmware host.
- **Host driver:** use `turret` with `default-features = false`. The
  `GimbalDevice` trait preserves capabilities, partial observations, and
  transport receipts. This package uses the operating system and serial I/O.
- **Host service:** enable the default features for the CLI, local IPC, and
  MAVLink manager. One service owns each device connection. External managers
  must not compete for the same device.

```rust,no_run
use std::time::{Duration, Instant};
use turret::turret_core::PayloadCommand;

let mut device = turret::detect_gimbal_blocking("/dev/ttyACM0")?;
let receipt = device.execute_blocking(
    PayloadCommand::Recenter,
    Instant::now() + Duration::from_millis(100),
)?;
// The receipt states write and ACK evidence, not physical completion.
let observation = device.observe_blocking()?;
# Ok::<_, turret::Error>(())
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
