"""
Blink an LED over BLE.

Requires the BLE firmware build on the board and the ble extra installed:
    pip install thingbot-telemetrix[ble]
"""

import time

from thingbot_telemetrix import Telemetrix

# Scan for any board advertising as ThingBot-<mac>.
# To skip scanning, pass ble_address='AA:BB:CC:DD:EE:FF' instead.
board = Telemetrix(ble_name="ThingBot")

board.gpio().set_pin_mode_output(7)

try:
    while True:
        board.gpio().digital_write(7, 1)
        time.sleep(0.5)
        board.gpio().digital_write(7, 0)
        time.sleep(0.5)
except KeyboardInterrupt:
    board.shutdown()
