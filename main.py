from time import time
import sys

from thingbot_telemetrix import telemetrix

board = telemetrix.Telemetrix()

board.gpio().set_pin_mode_digital_input(7, callback=lambda value: print(f"Digital pin 7 changed to {value}"))

while True:
    pass