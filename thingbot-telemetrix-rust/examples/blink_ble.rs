//! Blink an LED over BLE.
//!
//! Requires the BLE firmware build on the board:
//!
//! ```bash
//! cargo run --example blink_ble --features ble
//! ```

use std::thread::sleep;
use std::time::Duration;

use thingbot_telemetrix_rust::Telemetrix;

fn main() -> thingbot_telemetrix_rust::Result<()> {
    println!("Scanning for a ThingBot BLE device...");
    let mut board = Telemetrix::connect_ble()?;

    board.gpio().set_output(7)?;

    for _ in 0..10 {
        board.gpio().digital_write(7, true)?;
        sleep(Duration::from_millis(500));
        board.gpio().digital_write(7, false)?;
        sleep(Duration::from_millis(500));
    }

    Ok(())
}
