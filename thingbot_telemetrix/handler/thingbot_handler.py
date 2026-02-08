from thingbot_telemetrix.private_constants import ThingBotConstants

class ThingBotHandler():
    def __init__(self, telemetrix):
        self.telemetrix = telemetrix
        
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
        :param state: State of the LED. Set to 1 for ON, 0 for OFF.
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