from thingbot_telemetrix.private_constants import ThingBotConstraints as PrivateConstants

class I2CHandler:
    def __init__(self, board):
        self.board = board

        self.i2c_callback = None
        self.i2c_callback2 = None

        self.i2c_1_active = False
        self.i2c_2_active = False