from thingbot_telemetrix.private_constants import PinModes, ThingBotConstants

class GpioHandler:
    def __init__(self, telemetrix):
        self.telemetrix = telemetrix
        self.digital_callbacks = {}
        self.analog_callbacks = {}

    def set_pin_mode_digital_input(self, pin_number, callback=None):
        """
        This function sets the specified pin as a digital input.

        :param pin_number: Arduino pin number

        :param callback: A reference to a call back function to be
                         called when pin data value changes

        """
        self._set_pin_mode(pin_number, PinModes.INPUT, callback=callback)
    
    def set_pin_mode_analog_input(self, pin_number, differential=0, callback=None):
        """
        This function sets the specified pin as an analog input.

        :param pin_number: Arduino pin number

        :param differential: for analog inputs - threshold
                             value to be achieved for report to
                             be generated

        :param callback: A reference to a call back function to be
                         called when pin data value changes

        """
        self._set_pin_mode(pin_number, PinModes.ANALOG_INPUT, differential=differential, callback=callback)
    
    def _set_pin_mode(self, pin_number, pin_mode, differential=0, callback=None):
        """
        A private method to set the various pin modes.

        :param pin_number: arduino pin number

        :param pin_state: INPUT/OUTPUT/ANALOG/PWM/PULLUP
                         For SERVO use: set_pin_mode_servo
                         For DHT   use: set_pin_mode_dht

        :param differential: for analog inputs - threshold
                             value to be achieved for report to
                             be generated

        :param callback: A reference to a call back function to be
                         called when pin data value changes

        """
        if callback:
            if pin_mode == PinModes.ANALOG_INPUT:
                self.analog_callbacks[pin_number] = callback
            elif pin_mode == PinModes.INPUT:
                self.digital_callbacks[pin_number] = callback
            elif pin_mode == PinModes.INPUT_PULLUP:
                self.digital_callbacks[pin_number] = callback
        else:
            print('{} {}'.format('set_pin_mode: callback ignored for pin state:', pin_mode))

        if pin_mode == PinModes.INPUT:
            command = [ThingBotConstants.SET_PIN_MODE, pin_number, PinModes.INPUT, 1]
        elif pin_mode == PinModes.INPUT_PULLUP:
            command = [ThingBotConstants.SET_PIN_MODE, pin_number, PinModes.INPUT_PULLUP, 1]
        elif pin_mode == PinModes.ANALOG_INPUT:
            command = [ThingBotConstants.SET_PIN_MODE, pin_number, PinModes.ANALOG_INPUT, differential, 1]
        elif pin_mode == PinModes.OUTPUT:
            command = [ThingBotConstants.SET_PIN_MODE, pin_number, PinModes.OUTPUT, 0]
        elif pin_mode == PinModes.PWM_OUTPUT:
            command = [ThingBotConstants.SET_PIN_MODE, pin_number, PinModes.PWM_OUTPUT, 0]
        elif pin_mode == PinModes.SERVO_OUTPUT:
            command = [ThingBotConstants.SET_PIN_MODE, pin_number, PinModes.SERVO_OUTPUT, 0]
        elif pin_mode == PinModes.DHT_INPUT:
            command = [ThingBotConstants.SET_PIN_MODE, pin_number, PinModes.DHT_INPUT, 0]
        else:
            if self.telemetrix.shutdown_on_exception:
                self.telemetrix.shutdown()
            raise ValueError('Invalid pin mode specified: {}'.format(pin_mode))
        
        if command:
            self.telemetrix._send_command(command)
