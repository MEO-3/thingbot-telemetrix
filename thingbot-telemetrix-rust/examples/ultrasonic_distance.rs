use thingbot_telemetrix_rust::{Report, ThingBot};

fn main() -> thingbot_telemetrix_rust::Result<()> {
    let port = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "/dev/ttyUSB0".to_string());
    let mut board = ThingBot::connect(port)?;

    board.ultrasonic().set_pin_mode(5, 6)?;
    board.ultrasonic().read()?;

    if let Some(Report::Ultrasonic { distance_cm, .. }) = board.poll_report()? {
        println!("Distance: {distance_cm} cm");
    }

    Ok(())
}
