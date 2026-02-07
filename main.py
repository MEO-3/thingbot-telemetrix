import time
import sys

from thingbot_telemetrix import telemetrix
from thingbot_telemetrix.private_constants import ThingBotConstants, DHTTypes

board = telemetrix.Telemetrix()

# board.gpio().set_pin_mode_output(7)

# """ Blink the onboard LED on pin 7 """
# while True:
#     time.sleep(1)
#     board.gpio().digital_write(7, 1)
#     print("LED ON")
#     time.sleep(1)
#     board.gpio().digital_write(7, 0)
#     print("LED OFF")

def dht_callback(temperature, humidity):
    print(f"Temperature: {temperature} C, Humidity: {humidity} %")

board.dht().set_pin_mode_dht(7, dht_type=DHTTypes.DHT11, callback=dht_callback)
while True:
    time.sleep(10)