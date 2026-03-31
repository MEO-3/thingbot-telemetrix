from thingbot_telemetrix import telemetrix
from thingbot_telemetrix.private_constants import ThingBot

board = telemetrix.Telemetrix()

thingbot = board.thingbot()
thingbot.control_led(ThingBot.LED_1, 100)

def switch_callback(state):
    print("Switch event received:", state)
    thingbot.control_led(ThingBot.LED_1, 100 if state else 0)
    
thingbot.set_sw_callback(switch_callback)