"""
 Copyright (c) 2026 ThingEdu. All rights reserved.

"""

class ThingBotConstants:
    """
    This class contains a set of constants for telemetrix internal use .
    """

    # commands
    # send a loop back request - for debugging communications
    LOOP_COMMAND = 0
    SET_PIN_MODE = 1  # set a pin to INPUT/OUTPUT/PWM/etc
    DIGITAL_WRITE = 2  # set a single digital pin value instead of entire port
    ANALOG_WRITE = 3
    DIGITAL_READ = 4
    ANALOG_READ = 5
    ARE_U_THERE = 6  # Arduino ID query for auto-detect of telemetrix connected boards

    # reports
    # debug data from Arduino
    DIGITAL_REPORT = DIGITAL_WRITE
    ANALOG_REPORT = ANALOG_WRITE
    I_AM_HERE_REPORT = ARE_U_THERE
    DEBUG_PRINT = 99

    TELEMETRIX_VERSION = "1.0"

class PinModes:
    """
    This class contains a set of constants for telemetrix pin modes.
    """

    INPUT = 0
    OUTPUT = 1
    INPUT_PULLUP = 2
    ANALOG_INPUT = 3
    PWM_OUTPUT = 4
    SERVO_OUTPUT = 5
    DHT_INPUT = 6