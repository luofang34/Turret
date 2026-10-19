//! Portable MAVLink manager-client and camera operations, without transport or authority ownership.
#![no_std]

mod capability;
mod demand;
mod policy;

pub use capability::{camera_capabilities, manager_capabilities};
pub use demand::{
    claim_command, encode_payload, release_command, request_manager_status, stop_command, Demand,
    HORIZON_VEHICLE_YAW_FLAGS,
};
pub use policy::ManagerPolicy;
