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

    def dht_report(self, response_data = []):
        """
        This function is called when a DHT report is received from the Arduino.

        :param response_data: The response data list from the Arduino.
                              Format: [pin_number, humidity_high_byte, humidity_low_byte, temperature_high_byte, temperature_low_byte]

        """
        if len(response_data) != 5:
            print("DHT report received with invalid data length.")
            return
        
        pin_number = response_data[0]
        humidity = (response_data[1] << 8) | response_data[2]
        temperature = (response_data[3] << 8) | response_data[4]
        
        # Convert back to float
        humidity = humidity / 100.0
        temperature = temperature / 100.0
        
        if pin_number in self.dht_callbacks:
            callback = self.dht_callbacks[pin_number]
            callback(temperature, humidity)