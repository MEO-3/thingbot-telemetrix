(# ThingBot Telemetrix — Python API Reference)

This section documents the primary Python API for controlling a ThingBot board via the Telemetrix library.

**Board Initialization**
- `board`: The main board object created when the Telemetrix connection is initialized. Use this object to read from sensors and control actuators.

**Digital & Analog I/O**
- `digital_write(pin_number, value)`: Set a digital `pin_number` to `value` (`0`/`False` or `1`/`True`). Used for driving LEDs, relays, or other digital outputs.
- `analog_write(pin_number, value)`: Set an analog (PWM) `pin_number` to `value` (typically `0`..`255` or `0.0`..`1.0` depending on board). Used for PWM-controlled LEDs, motors (through driver), etc.
- `digital_read(pin_number)`: Read and return the current digital value on `pin_number` (`0` or `1`).
- `analog_read(pin_number)`: Read and return the analog value on `pin_number` (ADC reading, typically an integer or float depending on board resolution).

**Pin Mode Helpers**
- `set_pin_mode_analog_output(pin_number)`: Configure `pin_number` for analog (PWM) output.
- `set_pin_mode_digital_output(pin_number)`: Configure `pin_number` for digital output.
- `set_pin_mode_analog_input(pin_number, callback)`: Configure `pin_number` as an analog input. `callback(value)` is invoked when readings are received.
- `set_pin_mode_digital_input(pin_number, callback)`: Configure `pin_number` as a digital input. `callback(value)` is invoked on changes or reports.
- `set_pin_mode_dht(pin_number, callback, dht_type)`: Configure `pin_number` for a DHT sensor (e.g., DHT11/DHT22). `callback(data)` receives temperature/humidity payloads; `dht_type` selects sensor model.

**Actuator Controls**
- `control_dc(dc_number, speed)`: Control a DC motor driver channel `dc_number`. `speed` is typically in the range `-255..255` (negative for reverse) or `-1.0..1.0` depending on board conventions.
- `control_servo(servo_number, speed)`: Control a servo on `servo_number`. `speed` is interpreted as an angle or normalized position depending on board firmware (commonly `0..180` degrees or `-1.0..1.0`).


Examples

- Set pin 13 HIGH (digital):

	```python
	board.digital_write(13, 1)
	```

- PWM a pin (half duty):

	```python
	board.analog_write(5, 128)
	```

- Register a digital input callback:

	```python
	def on_change(value):
			print('Digital value:', value)

	board.set_pin_mode_digital_input(7, on_change)
	```

- Control a DC motor and a servo:

	```python
	board.control_dc(1, 200)      # run DC channel 1 forward
	board.control_servo(0, 90)    # move servo 0 to 90 degrees
	```

Notes
- Exact value ranges (e.g., PWM scale or servo angle units) depend on the board firmware and should be confirmed against your platform's docs. Callbacks receive raw values as delivered by the board; convert/scale as needed.

