//! Rust client library for ThingBot Telemetrix firmware.
//!
//! This crate currently provides the core protocol model, report decoding,
//! transport abstraction, and a synchronous client API. Serial/TCP transports
//! can be added behind the [`Transport`] trait without changing the high-level
//! command API.

#[cfg(feature = "ble")]
pub mod ble;
pub mod client;
pub mod error;
pub mod protocol;
pub mod report;
pub mod transport;

#[cfg(feature = "ble")]
pub use ble::{BleConfig, BleTransport};
pub use client::{GpioClient, Telemetrix, ThingBotClient, UltrasonicClient};
pub use error::{Error, Result};
pub use protocol::{
    Command, DhtType, Led, Motor, Packet, PinMode, Servo, TelemetrixProtocol, command_id,
    pin_mode_id, report_id,
};
pub use report::{Report, decode_report};
pub use transport::{PacketAssembler, SerialTransport, Transport};
