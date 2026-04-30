//! Per-message MAVLink handlers, broadcast loops, and outbound builders.
//!
//! Split into three submodules:
//! - [`build`] — pure constructors for outbound frames + the single
//!   low-level UDP send + the broadcast fan-out helper.
//! - [`loops`] — periodic tasks (heartbeat, gimbal-manager-status,
//!   attitude poll + GIMBAL_DEVICE_ATTITUDE_STATUS).
//! - [`inbound`] — per-message dispatch (SET_*, COMMAND_LONG).
//!
//! The parent [`super::MavlinkManager`] only sees the re-exports below;
//! everything else is private to this module.

#[path = "handlers/build.rs"]
mod build;
#[path = "handlers/inbound.rs"]
mod inbound;
#[path = "handlers/loops.rs"]
mod loops;

pub(super) use inbound::handle_mavlink_message;
pub(super) use loops::{attitude_loop, heartbeat_loop, publish_status_loop};
