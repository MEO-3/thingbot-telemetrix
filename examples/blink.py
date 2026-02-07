import time
import sys

from thingbot_telemetrix import telemetrix

board = telemetrix.Telemetrix()

board.gpio().set_pin_mode_output(7)

""" Blink the LED on pin 7 """
while True:
    time.sleep(2)
    board.gpio().digital_write(7, 1)
    print("LED ON")
    time.sleep(2)
    board.gpio().digital_write(7, 0)
    print("LED OFF")
