import time
import sys

from thingbot_telemetrix import telemetrix
from thingbot_telemetrix.private_constants import ThingBot

board = telemetrix.Telemetrix()

board.thingbot().control_led(ThingBot.LED_1, 100)

while True:
    time.sleep(2)
    board.thingbot().control_led(ThingBot.LED_2, 0)
    print("LED OFF")
    time.sleep(2)
    board.thingbot().control_led(ThingBot.LED_2, 100)
    print("LED ON")