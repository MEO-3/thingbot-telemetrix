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
    DHT_REPORT = 11
    DEBUG_PRINT = 99

    TELEMETRIX_VERSION = "1.0"

class PinModes:
    """
    This class contains a set of constants for telemetrix pin modes.
    """

    INPUT = 0x01
    OUTPUT = 0x03
    INPUT_PULLUP = 0x05
    ANALOG = 0x07
    DHT = 0x11
    
class DHTTypes:
    """
    This class contains a set of constants for DHT sensor types.
    """
    DHT11 = 11
    DHT22 = 22