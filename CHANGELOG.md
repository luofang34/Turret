# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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
