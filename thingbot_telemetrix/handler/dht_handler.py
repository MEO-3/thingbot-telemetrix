from thingbot_telemetrix.private_constants import PinModes, ThingBotConstants

class DhtHandler:
    def __init__(self, telemetrix):
        self.telemetrix = telemetrix
        
        self.dht_callbacks = {}
        
    def set_pin_mode_dht(self, pin_number, dht_type, callback=None):
        if callback is not None:
            self.dht_callbacks[pin_number] = callback
        
        command = [ThingBotConstants.SET_PIN_MODE, pin_number, PinModes.DHT, dht_type]    
        self.telemetrix._send_command(command)
        