import time

from thingbot_telemetrix import telemetrix
from thingbot_telemetrix.private_constants import DHTTypes

board = telemetrix.Telemetrix()

def dht_callback(temperature, humidity):
    print(f"Temperature: {temperature} C, Humidity: {humidity} %")

board.dht().set_pin_mode_dht(7, dht_type=DHTTypes.DHT11, callback=dht_callback)
while True:
    time.sleep(10)