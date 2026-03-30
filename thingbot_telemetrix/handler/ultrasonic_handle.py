from thingbot_telemetrix.private_constants import ThingBotConstants, PinModes

class UltrasonicHandler:
    def __init__(self, telemetrix):
        self.telemetrix = telemetrix
        self.ultrasonic_callbacks = {}

    def set_pin_mode_ultrasonic(self, trigger_pin, echo_pin, callback=None):
        command = [ThingBotConstants.SET_PIN_MODE, trigger_pin, PinModes.ULTRASONIC_TRIGGER, echo_pin]
        self.telemetrix._send_command(command)
        if callback:
            self.ultrasonic_callbacks[(trigger_pin, echo_pin)] = callback

    def read_ultrasonic(self):
        command = [ThingBotConstants.READ_ULTRASONIC]
        self.telemetrix._send_command(command)
    
    def ultrasonic_report(self, response_data = []):
        """
        This function is called when an ultrasonic report is received from the Arduino.

        :param response_data: The response data list from the Arduino.
                              Format: [trigger_pin, echo_pin, distance_high_byte, distance_low_byte]

        """
        if len(response_data) != 4:
            print("Ultrasonic report received with invalid data length.")
            return
        
        trigger_pin = response_data[0]
        echo_pin = response_data[1]
        distance = (response_data[2] << 8) | response_data[3]
        
        callback_key = (trigger_pin, echo_pin)
        if callback_key in self.ultrasonic_callbacks:
            callback = self.ultrasonic_callbacks[callback_key]
            callback(distance, trigger_pin, echo_pin)
    