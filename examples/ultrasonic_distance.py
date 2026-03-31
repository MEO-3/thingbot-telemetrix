import time

from thingbot_telemetrix import telemetrix
from thingbot_telemetrix.private_constants import PinModes

TRIGGER_PIN = 5
ECHO_PIN = 6

if not hasattr(PinModes, "ULTRASONIC_TRIGGER"):
    raise RuntimeError(
        "Ultrasonic trigger mode is not available in this build. "
        "Ensure PinModes.ULTRASONIC_TRIGGER is defined and firmware supports it."
    )

board = telemetrix.Telemetrix()
ultrasonic = board.ultrasonic()


def on_distance(distance, trigger_pin, echo_pin):
    print(f"Distance: {distance} (trigger={trigger_pin}, echo={echo_pin})")


ultrasonic.set_pin_mode_ultrasonic(
    TRIGGER_PIN, ECHO_PIN, callback=on_distance
)

while True:
    ultrasonic.read_ultrasonic()
    time.sleep(0.5)
