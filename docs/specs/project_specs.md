
# ThingBot Telemetrix Project Specs

## Overview

ThingBot Telemetrix provides a Python client API and companion Arduino firmware to control ThingBot-compatible hardware over the Telemetrix protocol. The repository focuses on a lightweight, callback-driven Python interface for GPIO, DHT sensors, ultrasonic sensors, and ThingBot-specific actuators (motors, servos, LEDs, buzzer), plus firmware tooling and examples.

## Goals

- Provide a simple Python API for controlling ThingBot hardware.
- Support both serial (USB) and TCP/IP transports.
- Offer Arduino firmware and examples for Telemetrix-compatible boards.

## Core Components

### Python Package (`thingbot_telemetrix`)

- **Telemetrix class** (`thingbot_telemetrix/telemetrix.py`):
  - Manages connection, auto-detection, and message dispatch.
  - Spawns receiver and reporter threads.
  - Supports serial (default) and TCP/IP transports.
  - Uses a length-prefixed command packet format: `[length, command, param1, ...]`.
- **Handlers** (`thingbot_telemetrix/handler/`):
  - `GpioHandler`: digital/analog input/output with optional callbacks.
  - `DhtHandler`: DHT11/DHT22 sensor support with callbacks.
  - `ThingBotHandler`: buzzer, LEDs, DC motors, servos, switch callbacks.
  - `UltrasonicHandler`: trigger/echo ultrasonic distance reads with callbacks.
  - `I2CHandler`: currently stubbed, no public APIs beyond initialization.

### Firmware Support

- Arduino/PlatformIO project in `thingbot-telemetrix-arduino/` (external repo referenced in README).
- Firmware can be flashed via:
  - `meo-tool` CLI (`meo-tool flash thingbot-telemetrix --latest --auto-detect`).
  - PlatformIO or Arduino IDE from the companion repo.

## API Surface (Python)

### Entry Point

- `Telemetrix(...)` in `thingbot_telemetrix.telemetrix`.
- `thingbot_telemetrix.__init__` exports `Telemetrix` directly.

### Handlers (obtained from a Telemetrix instance)

- `gpio()`
  - `set_pin_mode_output(pin)`
  - `set_pin_mode_digital_input(pin, callback=None)`
  - `set_pin_mode_analog_input(pin, differential=0, callback=None)`
  - `digital_write(pin, value)` (0/1)
  - `analog_write(pin, value)` (0-255)
  - `digital_read(pin)`
  - `analog_read(pin)`
- `dht()`
  - `set_pin_mode_dht(pin, dht_type, callback=None)`
- `thingbot()`
  - `control_buzzer(frequency)`
  - `control_led(led_number, state)`
  - `control_dc(motor_number, speed)`
  - `control_servo(servo_number, angle)`
  - `set_sw_callback(callback)`
- `UltrasonicHandler` is instantiated on `Telemetrix` and used in the report dispatch table, but there is no public accessor method in the current code.

## Telemetrix Protocol Constants

Defined in `thingbot_telemetrix/private_constants.py`.

- Command IDs: `SET_PIN_MODE`, `DIGITAL_WRITE`, `ANALOG_WRITE`, `DIGITAL_READ`, `ANALOG_READ`, `ARE_U_THERE`, `READ_ULTRASONIC`, plus ThingBot-specific commands (`DC_WRITE`, `SERVO_WRITE`, `BUZZER_WRITE`, `LED_WRITE`).
- Report IDs: `DIGITAL_REPORT`, `ANALOG_REPORT`, `I_AM_HERE_REPORT`, `DHT_REPORT`, `ULTRASONIC_REPORT`, `THINGBOT_SW_REPORT`, `DEBUG_PRINT`.
- Pin modes: input, output, input pullup, analog, DHT. (Ultrasonic trigger mode is referenced by `UltrasonicHandler`, but not defined in `PinModes`.)

## Transport and Connection Behavior

- **Serial** (default): auto-detects connected boards using `ARE_U_THERE` and an instance ID.
- **TCP/IP**: uses socket transport when `ip_address` is provided.
- Uses background threads for receiving and dispatching reports.

## Requirements and Dependencies

- Python >= 3.9.
- `pyserial >= 3.5`.
- ThingBot-compatible board running Telemetrix firmware.

## Repository Layout

- `thingbot_telemetrix/` Python package implementation.
- `examples/` usage examples (`blink.py`, `dht_input.py`, `thingbot_switch.py`).
- `docs/specs/project_specs.md` project specification summary.

## Examples (Python)

- Blink an LED on pin 7 (`examples/blink.py`).
- Read DHT11 sensor with callback (`examples/dht_input.py`).
- Handle ThingBot switch events (`examples/thingbot_switch.py`).

## License

GNU Affero General Public License v3 (AGPL-3.0-or-later).
