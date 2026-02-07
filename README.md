# ThingBot Telemetrix

ThingBot Telemetrix provides a Python API and an Arduino companion library for controlling ThingBot-compatible boards over the Telemetrix protocol. Use this repository to script sensors and actuators from Python or run Arduino firmware that speaks Telemetrix.

## Highlights

- Lightweight Python API for digital, analog/PWM, DHT, servo and DC motor control
- Example Python scripts in `examples/`
- Arduino library and example firmware in `thingbot-telemetrix-arduino/`

## Repository layout

- `thingbot_telemetrix/` — Python package: core API and handlers
- `examples/` — Example Python scripts (`blink.py`, `dht_input.py`)
- `thingbot-telemetrix-arduino/` — Arduino library and PlatformIO example

## Requirements

- Python 3.9 or newer
- A ThingBot-compatible board running Telemetrix or compatible firmware
- Serial (USB) or network access to the board

## Installation

Install the Python package for development:

```bash
pip install thingbot-telemetrix
```

## Quickstart (Python)

Import and connect to a board (API names are illustrative — check package docstrings):

```python
from thingbot_telemetrix import telemetrix

# Example: open serial port and connect
board = telemetrix.Telemetrix('/dev/ttyUSB0')  # Adjust port as needed or None for auto-detect

# Digital write
board.gpio().digital_write(13, 1)

# PWM write
board.gpio().analog().analog_write(5, 128)

# Register analog input callback
def on_analog(value):
	print('Analog:', value)

board.gpio().set_pin_mode_analog_input(0, on_analog)
```

## Actuators & Sensors

- `control_dc(channel, speed)` — control a DC motor channel (speed range depends on firmware)
- `control_servo(index, position)` — set servo position (commonly 0–180)
- `set_pin_mode_dht(pin, callback, dht_type)` — read DHT11/DHT22 sensors

## Arduino firmware

The `thingbot-telemetrix-arduino/` folder contains a PlatformIO project and an Arduino library `ThingBotTelemetrixArduino` that implements board-side handling for the Telemetrix protocol. Open the folder in PlatformIO to build and flash the firmware.

## Examples

- `examples/blink.py` — blink an onboard LED
- `examples/dht_input.py` — sample DHT sensor reader

## Contributing

Contributions are welcome. Please open issues for bugs or feature requests and send PRs for fixes or enhancements. Keep changes focused and include tests/examples when appropriate.

## License

This project is provided under the MIT License. See `LICENSE` for details.

---

For detailed API docs, view the docstrings in the `thingbot_telemetrix` package or open the examples for usage patterns.

