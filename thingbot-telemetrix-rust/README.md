# ThingBot Telemetrix Rust Client

Rust client library for controlling ThingBot Telemetrix firmware over a serial
connection.

The public board client is `Telemetrix`. ThingBot-specific controls are exposed
through the `thingbot()` sub-client.

## Install From GitHub

Add the GitHub tag dependency to your `Cargo.toml`:

```toml
[dependencies]
thingbot-telemetrix-rust = { git = "https://github.com/MEO-3/thingbot-telemetrix.git", tag = "rs-client-v0.1", package = "thingbot-telemetrix-rust" }
```

## Basic Usage

```rust
use thingbot_telemetrix_rust::{Led, Telemetrix};

fn main() -> thingbot_telemetrix_rust::Result<()> {
    let mut board = Telemetrix::connect("/dev/ttyUSB0")?;

    board.gpio().set_output(7)?;
    board.gpio().digital_write(7, true)?;

    board.thingbot().led(Led::One, 100)?;

    Ok(())
}
```

On Windows, use a COM port such as `COM3`. On Linux, the port is commonly
`/dev/ttyUSB0` or `/dev/ttyACM0`.

## GPIO

```rust
use thingbot_telemetrix_rust::Telemetrix;

fn main() -> thingbot_telemetrix_rust::Result<()> {
    let mut board = Telemetrix::connect("/dev/ttyUSB0")?;

    board.gpio().set_output(7)?;
    board.gpio().digital_write(7, true)?;
    board.gpio().analog_write(4, 1023)?;

    Ok(())
}
```

## ThingBot Controls

```rust
use thingbot_telemetrix_rust::{Led, Motor, Servo, Telemetrix};

fn main() -> thingbot_telemetrix_rust::Result<()> {
    let mut board = Telemetrix::connect("/dev/ttyUSB0")?;

    board.thingbot().led(Led::One, 100)?;
    board.thingbot().dc(Motor::One, 80)?;
    board.thingbot().servo(Servo::One, 90)?;
    board.thingbot().buzzer(20)?;

    Ok(())
}
```

## Ultrasonic

```rust
use thingbot_telemetrix_rust::{Report, Telemetrix};

fn main() -> thingbot_telemetrix_rust::Result<()> {
    let mut board = Telemetrix::connect("/dev/ttyUSB0")?;

    board.ultrasonic().set_pin_mode(5, 6)?;
    board.ultrasonic().read()?;

    if let Some(Report::Ultrasonic { distance_cm, .. }) = board.poll_report()? {
        println!("Distance: {distance_cm} cm");
    }

    Ok(())
}
```

## API Shape

- `Telemetrix::connect(port)` opens a serial connection at `115200` baud.
- `telemetrix.gpio()` controls digital and analog pins.
- `telemetrix.thingbot()` controls motors, servos, LEDs, and buzzer.
- `telemetrix.ultrasonic()` configures and reads ultrasonic sensors.
- `telemetrix.poll_report()` reads one incoming report if available.

## Examples

Run an example with a serial port argument:

```bash
cargo run --example blink -- /dev/ttyUSB0
cargo run --example ultrasonic_distance -- /dev/ttyUSB0
```

If no port is provided, examples default to `/dev/ttyUSB0`.

## Development Checks

From this folder:

```bash
cargo fmt --check
cargo test
cargo test --examples
```

## Current Limitations

- Explicit serial port is required; auto-detect is not implemented yet.
- TCP/IP transport is not implemented yet.
- Hardware behavior depends on firmware compatibility with the ThingBot
  Telemetrix Arduino protocol.
