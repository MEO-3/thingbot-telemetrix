# ThingBot Telemetrix

ThingBot Telemetrix provides a Python API and an Arduino companion library for controlling ThingBot over the Telemetrix protocol. Use this repository to script sensors and actuators from Python or run Arduino firmware that speaks Telemetrix.

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
from thingbot_telemetrix import Telemetrix

# Example: open serial port and connect
board = Telemetrix('/dev/ttyUSB0')  # Adjust port as needed or None for auto-detect

# Digital write
board.gpio().digital_write(13, 1)

# PWM write
board.gpio().analog_write(5, 128)

# Register analog input callback
def on_analog(value):
	print('Analog:', value)

board.gpio().set_pin_mode_analog_input(0, callback=on_analog)
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
- `examples/thingbot_switch.py` — example ThingBot switch handler

## Contributing

Contributions are welcome. Please open issues for bugs or feature requests and send PRs for fixes or enhancements. Keep changes focused and include tests/examples when appropriate.

## License

This project is provided under the GNU Affero General Public License v3 (AGPL-3.0-or-later). See [LICENSE](LICENSE) for details.

## Handler API

This section documents the Python-side handler APIs available from a `Telemetrix` instance. Use these handlers to control pins, read sensors, and interact with ThingBot-specific hardware features.

- **Access handlers**: obtain handlers from a `Telemetrix` instance:

```python
# package-level import
from thingbot_telemetrix import Telemetrix

board = Telemetrix()
gpio = board.gpio()
dht = board.dht()
thingbot = board.thingbot()
```

- **GpioHandler** (`gpio`)

	- `set_pin_mode_output(pin_number)` — set a digital output.
	- `set_pin_mode_digital_input(pin_number, callback=None)` — set a digital input and optionally register a callback `callback(value)` where `value` is `0` or `1`.
	- `set_pin_mode_analog_input(pin_number, differential=0, callback=None)` — set an analog input; `differential` is a threshold and `callback(value)` receives a 0–1023 integer value.
	- `digital_write(pin_number, value)` — write digital `0`/`1`.
	- `analog_write(pin_number, value)` — write PWM `0`–`255`.
	- `digital_read(pin_number)` — request a single digital read.
	- `analog_read(pin_number)` — request a single analog read.

	Example: register an analog callback

	```python
	def on_analog(value):
			print('Analog:', value)

	gpio.set_pin_mode_analog_input(0, differential=10, callback=on_analog)
	```

- **DhtHandler** (`dht`)

	- `set_pin_mode_dht(pin_number, dht_type, callback=None)` — enable DHT on a pin. `dht_type` is `DHTTypes.DHT11` or `DHTTypes.DHT22`. If `callback` is provided it will be called as `callback(temperature, humidity)` where values are floats (temperature in °C, humidity in %).

	Example:

	```python
	from thingbot_telemetrix.private_constants import DHTTypes

	def on_dht(temp, hum):
			print(f'Temp={temp}°C Hum={hum}%')

	dht.set_pin_mode_dht(2, DHTTypes.DHT22, callback=on_dht)
	```

- **ThingBotHandler** (`thingbot`)

	- `control_buzzer(frequency)` — set buzzer frequency (0 to turn off).
	- `control_led(led_number, state)` — set LED brightness (0–100 typical).
	- `control_dc(motor_number, speed)` — control DC motor speed (signed value, e.g. -100..100).
	- `control_servo(servo_number, angle)` — set servo position (0–180).
	- `set_sw_callback(callback)` — register a switch callback `callback(pressed)` where `pressed` is `True` when pressed, `False` when released.

	Example: set switch callback

	```python
	def on_switch(pressed):
			print('Switch pressed' if pressed else 'Switch released')

	thingbot.set_sw_callback(on_switch)
	```

For more details check the handler source files in `thingbot_telemetrix/handler/`.

---

For detailed API docs, view the docstrings in the `thingbot_telemetrix` package or open the examples for usage patterns.

