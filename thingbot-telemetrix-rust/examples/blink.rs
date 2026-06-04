use thingbot_telemetrix_rust::{Led, Telemetrix};

fn main() -> thingbot_telemetrix_rust::Result<()> {
    let port = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "/dev/ttyUSB0".to_string());
    let mut board = Telemetrix::connect(port)?;

    board.gpio().set_output(7)?;
    board.gpio().digital_write(7, true)?;
    board.thingbot().led(Led::One, 100)?;

    Ok(())
}
