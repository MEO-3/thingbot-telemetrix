from thingbot_telemetrix import telemetrix
from thingbot_telemetrix.private_constants import ThingBot

board = telemetrix.Telemetrix()

board.thingbot().control_led(ThingBot.LED_1, 100)