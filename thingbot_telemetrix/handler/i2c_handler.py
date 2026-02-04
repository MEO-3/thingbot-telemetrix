from thingbot_telemetrix.private_constants import ThingBotConstraints as PrivateConstants

class I2CHandler:
    def __init__(self, telemetrix_instance):
        self.telemetrix = telemetrix_instance

    def i2c_read(self, address, register, number_of_bytes,
                 callback=None, i2c_port=0,
                 write_register=True):
        """
        Read the specified number of bytes from the
        specified register for the i2c device.


        :param address: i2c device address

        :param register: i2c register (or None if no register
                                       selection is needed)

        :param number_of_bytes: number of bytes to be read

        :param callback: Required callback function to report
                         i2c data as a result of read command

       :param i2c_port: 0 = default, 1 = secondary

       :param write_register: If True, the register is written
                                       before read
                              Else, the write is suppressed


        callback returns a data list:

        [I2C_READ_REPORT, i2c_port, number of bytes read, address, register,
        bytes read..., time-stamp]


        """

        self._i2c_read_request(address, register, number_of_bytes,
                               callback=callback, i2c_port=i2c_port,
                               write_register=write_register)

    def i2c_read_restart_transmission(self, address, register,
                                      number_of_bytes,
                                      callback=None, i2c_port=0,
                                      write_register=True):
        """
        Read the specified number of bytes from the specified
        register for the i2c device. This restarts the transmission
        after the read. It is required for some i2c devices such as the MMA8452Q
        accelerometer.


        :param address: i2c device address

        :param register: i2c register (or None if no register
                                                    selection is needed)

        :param number_of_bytes: number of bytes to be read

        :param callback: Required callback function to report i2c
                         data as a result of read command

       :param i2c_port: 0 = default 1 = secondary

       :param write_register: If True, the register is written before read
                              Else, the write is suppressed



        callback returns a data list:

        [I2C_READ_REPORT, i2c_port, number of bytes read, address, register,
        bytes read..., time-stamp]

        """

        self._i2c_read_request(address, register, number_of_bytes,
                               stop_transmission=False,
                               callback=callback, i2c_port=i2c_port,
                               write_register=write_register)

    def _i2c_read_request(self, address, register, number_of_bytes,
                          stop_transmission=True, callback=None, i2c_port=0,
                          write_register=True):
        """
        This method requests the read of an i2c device. Results are retrieved
        via callback.

        :param address: i2c device address

        :param register: register number (or None if no register selection is needed)

        :param number_of_bytes: number of bytes expected to be returned

        :param stop_transmission: stop transmission after read

        :param callback: Required callback function to report i2c data as a
                   result of read command.

       :param write_register: If True, the register is written before read
                              Else, the write is suppressed

        """
        if not i2c_port:
            if not self.i2c_1_active:
                if self.shutdown_on_exception:
                    self.shutdown()
                raise RuntimeError(
                    'I2C Read: set_pin_mode i2c never called for i2c port 1.')

        if i2c_port:
            if not self.i2c_2_active:
                if self.shutdown_on_exception:
                    self.shutdown()
                raise RuntimeError(
                    'I2C Read: set_pin_mode i2c never called for i2c port 2.')

        if not callback:
            if self.shutdown_on_exception:
                self.shutdown()
            raise RuntimeError('I2C Read: A callback function must be specified.')

        if not i2c_port:
            self.i2c_callback = callback
        else:
            self.i2c_callback2 = callback

        if not register:
            register = 0

        if write_register:
            write_register = 1
        else:
            write_register = 0

        # message contains:
        # 1. address
        # 2. register
        # 3. number of bytes
        # 4. restart_transmission - True or False
        # 5. i2c port
        # 6. suppress write flag

        command = [PrivateConstants.I2C_READ, address, register, number_of_bytes,
                   stop_transmission, i2c_port, write_register]
        self._send_command(command)

    def i2c_write(self, address, args, i2c_port=0):
        """
        Write data to an i2c device.

        :param address: i2c device address

        :param i2c_port: 0= port 1, 1 = port 2

        :param args: A variable number of bytes to be sent to the device
                     passed in as a list

        """
        if not i2c_port:
            if not self.i2c_1_active:
                if self.shutdown_on_exception:
                    self.shutdown()
                raise RuntimeError(
                    'I2C Write: set_pin_mode i2c never called for i2c port 1.')

        if i2c_port:
            if not self.i2c_2_active:
                if self.shutdown_on_exception:
                    self.shutdown()
                raise RuntimeError(
                    'I2C Write: set_pin_mode i2c never called for i2c port 2.')

        command = [PrivateConstants.I2C_WRITE, len(args), address, i2c_port]

        for item in args:
            command.append(item)

        self._send_command(command)