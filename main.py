import time
import sys
from examples.digital_input import the_callback
from thingbot_telemetrix import telemetrix

board = telemetrix.Telemetrix(com_port="/dev/ttyACM0")

def analog_in(my_board, pin):
    """
     This function establishes the pin as an
     analog input. Any changes on this pin will
     be reported through the call back function.

     :param my_board: a telemetrix instance
     :param pin: Arduino pin number
     """

    # set the pin mode
    my_board.set_pin_mode_analog_input(pin, differential=5, callback=the_callback)

    print('Enter Control-C to quit.')
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        board.shutdown()
        sys.exit(0)

try:
    analog_in(board, 2)
except KeyboardInterrupt:
    board.shutdown()
    sys.exit(0)
