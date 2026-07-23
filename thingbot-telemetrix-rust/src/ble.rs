//! BLE transport for boards running the BLE firmware build.
//!
//! The firmware exposes a NUS-style GATT byte pipe: the host writes
//! length-prefixed command packets to the RX characteristic and receives
//! reports as notifications on the TX characteristic. Notifications carry
//! arbitrary chunks of the byte stream, so packets are reassembled with
//! [`PacketAssembler`].
//!
//! Enabled with the `ble` cargo feature.

use std::sync::mpsc::{self, Receiver, RecvTimeoutError};
use std::time::{Duration, Instant};

use btleplug::api::{
    Central, Characteristic, Manager as _, Peripheral as _, PeripheralProperties, ScanFilter,
    WriteType,
};
use btleplug::platform::{Adapter, Manager, Peripheral};
use futures::StreamExt;
use tokio::runtime::Runtime;
use uuid::Uuid;

use crate::error::{Error, Result};
use crate::protocol::Packet;
use crate::transport::{PacketAssembler, Transport};

/// GATT contract of the ThingBot Telemetrix BLE firmware (BLETransport.h).
pub const SERVICE_UUID: Uuid = Uuid::from_u128(0xaa70_0001_8f6a_4e2c_b369_4060_e0bb_33aa);
/// Host -> device commands (write).
pub const RX_CHAR_UUID: Uuid = Uuid::from_u128(0xaa70_0002_8f6a_4e2c_b369_4060_e0bb_33aa);
/// Device -> host reports (notify).
pub const TX_CHAR_UUID: Uuid = Uuid::from_u128(0xaa70_0003_8f6a_4e2c_b369_4060_e0bb_33aa);

const DEFAULT_NAME_PREFIX: &str = "ThingBot";

#[derive(Debug, Clone)]
pub struct BleConfig {
    /// Exact BLE address to connect to (e.g. "AA:BB:CC:DD:EE:FF").
    pub address: Option<String>,
    /// Advertised name prefix to match while scanning; the firmware
    /// advertises as `ThingBot-<mac>`.
    pub name_prefix: Option<String>,
    /// How long to scan before giving up.
    pub scan_timeout: Duration,
    /// `read_packet` timeout, mirroring the serial transport default.
    pub read_timeout: Duration,
}

impl Default for BleConfig {
    fn default() -> Self {
        Self {
            address: None,
            name_prefix: None,
            scan_timeout: Duration::from_secs(10),
            read_timeout: Duration::from_millis(100),
        }
    }
}

/// BLE transport. Owns a small tokio runtime that drives the btleplug
/// session; a background task forwards TX notifications into a channel
/// drained by [`Transport::read_packet`].
pub struct BleTransport {
    runtime: Runtime,
    peripheral: Peripheral,
    rx_char: Characteristic,
    incoming: Receiver<Vec<u8>>,
    assembler: PacketAssembler,
    read_timeout: Duration,
}

impl BleTransport {
    /// Scan for a ThingBot device (by service UUID or name prefix) and
    /// connect with default settings.
    pub fn connect() -> Result<Self> {
        Self::connect_with(BleConfig::default())
    }

    pub fn connect_with(config: BleConfig) -> Result<Self> {
        let runtime = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(1)
            .enable_all()
            .build()
            .map_err(|error| Error::Transport(error.to_string()))?;
        let (peripheral, rx_char, incoming) = runtime.block_on(Self::setup(&config))?;
        Ok(Self {
            runtime,
            peripheral,
            rx_char,
            incoming,
            assembler: PacketAssembler::new(),
            read_timeout: config.read_timeout,
        })
    }

    async fn setup(config: &BleConfig) -> Result<(Peripheral, Characteristic, Receiver<Vec<u8>>)> {
        let manager = Manager::new().await.map_err(ble_error)?;
        let adapter = manager
            .adapters()
            .await
            .map_err(ble_error)?
            .into_iter()
            .next()
            .ok_or_else(|| Error::Transport("no Bluetooth adapter found".into()))?;

        adapter
            .start_scan(ScanFilter::default())
            .await
            .map_err(ble_error)?;
        let scan_result = Self::scan_for_match(&adapter, config).await;
        let _ = adapter.stop_scan().await;
        let peripheral = scan_result?;

        peripheral.connect().await.map_err(ble_error)?;
        peripheral.discover_services().await.map_err(ble_error)?;

        let characteristics = peripheral.characteristics();
        let rx_char = characteristics
            .iter()
            .find(|c| c.uuid == RX_CHAR_UUID)
            .cloned()
            .ok_or_else(|| Error::Transport("RX characteristic not found".into()))?;
        let tx_char = characteristics
            .iter()
            .find(|c| c.uuid == TX_CHAR_UUID)
            .cloned()
            .ok_or_else(|| Error::Transport("TX characteristic not found".into()))?;

        peripheral.subscribe(&tx_char).await.map_err(ble_error)?;
        let mut notifications = peripheral.notifications().await.map_err(ble_error)?;

        let (sender, receiver) = mpsc::channel();
        tokio::spawn(async move {
            while let Some(notification) = notifications.next().await {
                if notification.uuid == TX_CHAR_UUID && sender.send(notification.value).is_err() {
                    break;
                }
            }
        });

        Ok((peripheral, rx_char, receiver))
    }

    async fn scan_for_match(adapter: &Adapter, config: &BleConfig) -> Result<Peripheral> {
        let deadline = Instant::now() + config.scan_timeout;
        loop {
            for peripheral in adapter.peripherals().await.map_err(ble_error)? {
                let Some(properties) = peripheral.properties().await.map_err(ble_error)? else {
                    continue;
                };
                if Self::matches(&properties, config) {
                    return Ok(peripheral);
                }
            }
            if Instant::now() >= deadline {
                return Err(Error::Transport(
                    "no ThingBot BLE device found; is the board powered and \
                     running the BLE firmware?"
                        .into(),
                ));
            }
            tokio::time::sleep(Duration::from_millis(500)).await;
        }
    }

    fn matches(properties: &PeripheralProperties, config: &BleConfig) -> bool {
        if let Some(address) = &config.address {
            return properties.address.to_string().eq_ignore_ascii_case(address);
        }
        if let Some(prefix) = &config.name_prefix {
            return properties
                .local_name
                .as_deref()
                .is_some_and(|name| name.starts_with(prefix.as_str()));
        }
        properties.services.contains(&SERVICE_UUID)
            || properties
                .local_name
                .as_deref()
                .is_some_and(|name| name.starts_with(DEFAULT_NAME_PREFIX))
    }
}

impl Transport for BleTransport {
    fn write_packet(&mut self, packet: &Packet) -> Result<()> {
        let wire = packet.to_wire();
        // write with response: the ATT ack throttles the host against the
        // firmware's small RX ring buffer
        self.runtime
            .block_on(
                self.peripheral
                    .write(&self.rx_char, &wire, WriteType::WithResponse),
            )
            .map_err(ble_error)
    }

    fn read_packet(&mut self) -> Result<Option<Packet>> {
        let deadline = Instant::now() + self.read_timeout;
        loop {
            if let Some(packet) = self.assembler.next_packet()? {
                return Ok(Some(packet));
            }
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return Ok(None);
            }
            match self.incoming.recv_timeout(remaining) {
                Ok(chunk) => self.assembler.push(&chunk),
                Err(RecvTimeoutError::Timeout) => return Ok(None),
                Err(RecvTimeoutError::Disconnected) => {
                    return Err(Error::Transport("BLE connection lost".into()));
                }
            }
        }
    }
}

impl Drop for BleTransport {
    fn drop(&mut self) {
        let _ = self.runtime.block_on(self.peripheral.disconnect());
    }
}

fn ble_error(error: btleplug::Error) -> Error {
    Error::Transport(error.to_string())
}
