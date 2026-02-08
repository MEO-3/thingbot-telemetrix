from thingbot_telemetrix.private_constants import ThingBotConstants

class ThingBotHandler():
    def __init__(self, telemetrix):
        self.telemetrix = telemetrix
        self.sw_callback = None
        
    def control_buzzer(self, frequency):
        """
        Control the buzzer on the ThingBot.

        :param frequency: Frequency of the buzzer sound. Set to 0 to turn off.
        """
        command = [ThingBotConstants.BUZZER_WRITE, frequency]
        self.telemetrix._send_command(command)
        
    def control_led(self, led_number, state):
        """
        Control the LED on the ThingBot.

        :param led_number: The LED number to control.
        :param state: State of the LED. Set to 100 for fully on, 0 for off.
        """
        command = [ThingBotConstants.LED_WRITE, led_number, state]
        self.telemetrix._send_command(command)
        
    def control_dc(self, motor_number, speed):
        """
        Control a DC motor on the ThingBot.

        :param motor_number: The motor number to control.
        :param speed: Speed of the motor (-255 to 255).
        """
        command = [ThingBotConstants.DC_WRITE, motor_number, speed]
        self.telemetrix._send_command(command)
        
    def control_servo(self, servo_number, angle):
        """
        Control a servo motor on the ThingBot.

        :param servo_number: The servo number to control.
        :param angle: Angle to set the servo (0 to 180).
        """
        command = [ThingBotConstants.SERVO_WRITE, servo_number, angle]
        self.telemetrix._send_command(command)
        
    def set_sw_callback(self, callback):
        """
        Set a callback function for ThingBot switch events.

        :param callback: A reference to a call back function to be
                         called when a switch event occurs.

        """
        self.sw_callback = callback
        
    def thingbot_sw_report(self, response_data = []):
        """
        Internal method to handle ThingBot specific reports.

        :param response_data: List of data bytes from the ThingBot report.

        """
        _ = response_data[0]
        value = response_data[1]
        if self.sw_callback:
            if value == 1: # switch released
                self.sw_callback(False)
            else: # switch pressed
                self.sw_callback(True)