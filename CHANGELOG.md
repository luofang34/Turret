# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- **Earth-frame yaw + `AUTOPILOT_STATE_FOR_GIMBAL_DEVICE` ingestion** —
  the manager now consumes vehicle attitude from the autopilot and
  honors `GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME` on inbound SET_*
  commands. Earth-frame yaw is converted to vehicle frame via
  `earth_to_vehicle_yaw_deg`, with `wrap_180_deg` normalization.
  Stale (>1 s old) or missing autopilot attitude triggers an honest
  fall-back to vehicle-frame interpretation with a warn (no silent
  rejection). Pure-function `math.rs` helpers; tests cover wrap
  in-band/above/below and the earth-to-vehicle subtraction.
- **Rate / `angular_velocity` command path** — incoming
  `GIMBAL_MANAGER_SET_ATTITUDE` (NaN-quaternion + non-NaN
  angular_velocity), `GIMBAL_MANAGER_SET_PITCHYAW` (NaN angles +
  non-NaN rate), and `MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW`
  (NaN param1/2 + param3/4 rates) now route through a one-shot
  rate→position integrator. Per-source `last_rate_at` timestamp
  drives `dt`, clamped to [10 ms, 500 ms]. Each rate tick produces
  one `current + rate × dt` position write through the existing
  arbitrator pipeline. Stops the previous "silently drop NaN-q
  commands" failure mode without adding a closed-loop rate
  controller (STorM32 has no native rate command).
- **`cargo publish --dry-run` CI gate** — runs alongside fmt / clippy /
  test / doc / release-build / lib-no-default. Catches Cargo.toml
  metadata regressions (broken `package.exclude`, stale
  `description / license / repository`, missing `readme`, etc.) and
  verifies the tarball file set still builds — before a real release
  attempt. Local `cargo publish --dry-run` confirmed 34 files
  packaged, none of `test_daemon.sh / systemd/ / .github/` leaked.
- **Embedded-API documentation** — module-level rustdoc on
  `turret::daemon` now includes a worked example for users who want
  to wire `MavlinkManager` against their own `dyn GimbalDevice`
  (different protocol, simulator, network-attached gimbal). README
  gains a "Use as a library" section describing the lean-driver
  vs embedded-manager modes.
- **Configurable MAVLink bind interface** — new `mavlink.bind_addr`
  YAML field (typed `IpAddr`, default `0.0.0.0`). Loopback-only
  deployments can set `127.0.0.1`; multi-homed boxes can pin a
  specific interface. Existing configs without the field default
  to the historical `0.0.0.0` behavior.
- **Per-source command rate limit** — new `arbitrator.min_command_interval_ms`
  YAML knob (default 0 = disabled). Two commands from the same
  `ControlSource` within the window are rejected before reaching the
  gimbal; different sources are tracked independently. Defends the
  serial bus against a runaway GCS or buggy IPC client without
  affecting normal traffic.
- **GIMBAL_DEVICE_ATTITUDE_STATUS broadcast** — 4 Hz attitude poll +
  per-peer fan-out. GCSs now see real measured pose instead of nothing.
- **`COMMS_ERROR` health flag** — set on the next successful broadcast
  for ~1 s after any failed poll, so flaky links don't masquerade as
  healthy.
- **Hot-plug recovery** — daemon-level reconnect task: on 8 consecutive
  attitude-poll failures, scans (or re-opens the configured path) with
  exponential backoff and swaps the new gimbal into the shared slot
  without restart.
- **Bounded shutdown** — SIGTERM and SIGINT both caught (Unix); each
  tracked task gets a 2 s grace before `JoinHandle::abort`; IPC socket
  file removed on exit.
- **Critical-task supervision** — IPC / MAVLink / reconnect each
  notify a shared `task_died` on exit, and `run_daemon` shuts down
  the rest instead of reporting healthy.
- **`MAV_CMD_REQUEST_MESSAGE` handling** — GCS discovery for IDs 280
  (`GIMBAL_MANAGER_INFORMATION`) and 281 (`GIMBAL_MANAGER_STATUS`).
- **Per-axis `target_component` validation** — frame addressed to e.g.
  the autopilot on the same sysid is no longer acted on by the gimbal
  manager.
- **`tracing` for diagnostics** — replaces `log`, with `#[instrument]`
  on dispatch hotspots.
- **`tests/mavlink_smoke.rs`** — end-to-end SET_PITCHYAW round-trip
  via UDP, verifies inbound dispatch + arbitrator + attitude broadcast
  in ~250 ms.

### Changed
- **Single-owner gimbal task** — every gimbal call now runs through
  `daemon::gimbal_handle::GimbalHandle`, which routes the underlying
  sync I/O via `tokio::task::spawn_blocking`. Eliminates the previous
  "tokio Mutex held across `std::thread::sleep` in the storm32 driver"
  anti-pattern: the runtime worker that originated a gimbal call now
  returns to scheduling other tasks immediately while the blocking
  thread pool absorbs the serial transaction. `Arbitrator::process_command`
  splits into a pure `arbitrate(cmd) -> Outcome` (no I/O) plus the
  caller's separate `gimbal.set_attitude(...).await`. Public API
  change: `MavlinkManager::new` and `IpcServer::new` now take
  `GimbalHandle` instead of `Arc<Mutex<Box<dyn GimbalDevice>>>`.
- **MSRV bumped to 1.74** (required for `Cargo.toml [lints]` tables).
- **`MavlinkConfig::transport`** is now `enum Transport { Udp }` —
  YAML typos caught at config-load time. `serial_device` field
  (placeholder) removed.
- **Lean library trims to 4 direct deps**: `serde`, `serialport`,
  `thiserror`, `tracing`. The `cli` feature drops `dep:anyhow`
  entirely; `main.rs` uses `turret::Error` end-to-end.
- **Lints lifted to `Cargo.toml [lints]`**: `forbid` on
  `unsafe_code` / `disallowed_types` / `await_holding_lock` /
  `let_underscore_*`; `deny` on `unwrap_used` / `expect_used` /
  `panic`. `missing_docs` enforced as `deny`.
- **Storm32 yaw decode fixed** — IMU1ANGLES payload was off-by-one
  and dropped the yaw axis; now reads all three.
- **Files reorganized**: `handlers.rs` (was 870 lines) split into
  `handlers/{build,inbound,loops}.rs`; `driver.rs` (was 518) split
  into `driver.rs` + `commands.rs`. Every file under 500 lines.
- **`broadcast_to_peers`** helper consolidates the heartbeat /
  status / attitude broadcast fan-out.

### Removed
- **IPC Storm32-specific commands** (`pitch / roll / yaw / version-str`).
  These bypassed the `GimbalDevice` trait via `as_any_mut().downcast_mut::<Storm32RC>()`
  and silently failed with "requires Storm32 device" for any future
  protocol. Equivalent functionality remains in the CLI (which talks
  to the driver directly): `turret pitch <value> / roll / yaw /
  version-str`. IPC clients that need partial-axis updates should
  read `status` and resend a full `set` instead.
- Six orphan files (TOML config + client-side MAVLink driver).
- `MavlinkManager._config` dead field, `cleanup_stale_commands` /
  `clear_stale_commands` dead method pair, `serial_device` config
  field.
- `async-trait` direct dependency.

### Fixed
- `MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE` now writes primary/secondary
  control ownership and broadcasts it via `GIMBAL_MANAGER_STATUS`.
- `time_boot_ms` on `GIMBAL_MANAGER_INFORMATION` is now real, not 0.
- `RxCtx` carries `start_time` so on-demand frames have correct
  uptime.

## [0.1.1] - 2025-09-30

### Added
- Daemon mode (`--daemon` flag)
- Unix socket IPC at `/tmp/turret.sock`
- MAVLink Gimbal Manager protocol (UDP port 14550)
- Command arbitration with priority system
- YAML configuration support

### Fixed
- Serial buffer reliability
- CRC16 X.25 implementation

## [0.1.0] - 2025-09-12

### Added
- Initial release
- STorM32 gimbal control via RC Commands protocol
- Auto-detection of STorM32 devices
- Support for multiple angle input formats: `pitch roll yaw`, `pitch,roll,yaw`, `pitch:roll:yaw`
- Comprehensive gimbal control commands:
  - `set` - Set absolute gimbal angles
  - `center` - Center gimbal to 0,0,0
  - `status` - Get current gimbal status
  - `version` - Get firmware version
  - `pitch`/`roll`/`yaw` - Individual axis control
  - `pan-mode` - Set pan operation modes
  - `standby` - Enable/disable standby mode
- Structured logging with verbosity levels (`-v`, `-vv`, `-vvv`)
- Cross-platform support (macOS, Linux, Windows)
- Silent operation by default, detailed logging when requested
- Professional error handling with clear user feedback

### Technical Details
- Uses official STorM32 RC Commands protocol over serial (115200 baud)
- Auto-detects STM32 Virtual COM Port (VID:0x0483, PID:0x5740)
- CRC16-x25 checksums for data integrity
- Async/await support for non-blocking operations
- Comprehensive device discovery with fallback patterns

[0.1.0]: https://github.com/luofang34/turret/releases/tag/v0.1.0
