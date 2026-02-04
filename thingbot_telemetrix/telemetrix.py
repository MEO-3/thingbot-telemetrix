"""
 Copyright (c) 2026 ThingEdu All rights reserved.
 Copyright (c) 2021-2025 Alan Yorinks All rights reserved.

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU AFFERO GENERAL PUBLIC LICENSE
 Version 3 as published by the Free Software Foundation; either
 or (at your option) any later version.
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 General Public License for more details.

 You should have received a copy of the GNU AFFERO GENERAL PUBLIC LICENSE
 along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

"""
import socket
import sys
import threading
import time
from collections import deque

import serial
from serial.serialutil import SerialException
from serial.tools import list_ports

from thingbot_telemetrix.private_constants import ThingBotConstraints as PrivateConstants
from thingbot_telemetrix.telemetrix_port_register import TelemetrixPortRegister


class Telemetrix(threading.Thread):
    """
    This class exposes and implements the telemetrix API.
    It uses threading to accommodate concurrency.
    It includes the public API methods as well as
    a set of private methods.
    """

    def __init__(self, com_port=None, arduino_instance_id=1, arduino_wait=4, sleep_tune=0.000001, shutdown_on_exception=True, ip_address=None, ip_port=31335):
        """
        :param com_port: e.g. COM3 or /dev/ttyACM0.
                            Only use if you wish to bypass auto com port
                            detection.

        :param arduino_instance_id: Match with the value installed on the
                                    arduino-telemetrix sketch.

        :param arduino_wait: Amount of time to wait for an Arduino to
                                fully reset itself.

        :param sleep_tune: A tuning parameter (typically not changed by user)

        :param shutdown_on_exception: call shutdown before raising
                                        a RunTimeError exception, or
                                        receiving a KeyboardInterrupt exception

        :param ip_address: ip address of tcp/ip connected device.

        :param ip_port: ip port of tcp/ip connected device
        """

        self.serial_port_register = TelemetrixPortRegister()

        # initialize threading parent
        threading.Thread.__init__(self)

        # create the threads and set them as daemons so
        # that they stop when the program is closed

        # create a thread to interpret received serial data
        self.the_reporter_thread = threading.Thread(target=self._reporter)
        self.the_reporter_thread.daemon = True

        self.ip_address = ip_address
        self.ip_port = ip_port

        if not self.ip_address:
            self.the_data_receive_thread = threading.Thread(target=self._serial_receiver)
        else:
            self.the_data_receive_thread = threading.Thread(target=self._tcp_receiver)

        self.the_data_receive_thread.daemon = True

        # flag to allow the reporter and receive threads to run.
        self.run_event = threading.Event()

        # check to make sure that Python interpreter is version 3.7 or greater
        python_version = sys.version_info
        if python_version[0] >= 3:
            if python_version[1] >= 7:
                pass
            else:
                raise RuntimeError("ERROR: Python 3.7 or greater is "
                                   "required for use of this program.")

        self.com_port = com_port
        self.arduino_instance_id = arduino_instance_id
        self.arduino_wait = arduino_wait
        self.sleep_tune = sleep_tune
        self.shutdown_on_exception = shutdown_on_exception

        self.the_deque = deque()

        self.the_reporter_thread.start()
        self.the_data_receive_thread.start()

        print(f"ThingBot Telemetrix:  Version {PrivateConstants.TELEMETRIX_VERSION}\n\n"
              f"Copyright (c) 2026 ThingEdu. All Rights Reserved.\nCopyright (c) 2021-2025 Alan Yorinks All Rights Reserved.\n")

        # using the serial link
        if not self.ip_address:
            if not self.com_port:
                # user did not specify a com_port
                try:
                    self._find_arduino()
                except KeyboardInterrupt:
                    if self.shutdown_on_exception:
                        self.shutdown()
            else:
                # com_port specified - set com_port and baud rate
                try:
                    self._manual_open()
                except KeyboardInterrupt:
                    if self.shutdown_on_exception:
                        self.shutdown()

            if self.serial_port:
                print(f"Arduino compatible device found and connected to {self.serial_port.port}")
                self.serial_port.reset_input_buffer()
                self.serial_port.reset_output_buffer()
                self.serial_port_register.add(self.serial_port)

            # no com_port found - raise a runtime exception
            else:
                if self.shutdown_on_exception:
                    self.shutdown()
                raise RuntimeError('No Arduino Found or User Aborted Program')
        else:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.ip_address, self.ip_port))
            print(f'Successfully connected to: {self.ip_address}:{self.ip_port}')

        # allow the threads to run
        self._run_threads()
        print(f'Waiting for Arduino to reset')
        print(f'Reset Complete')

        # get telemetrix firmware version and print it
        print('\nRetrieving Telemetrix4Arduino firmware ID...')
        self._get_firmware_version()
        if not self.firmware_version:
            if self.shutdown_on_exception:
                self.shutdown()
            raise RuntimeError(f'Telemetrix4Arduino firmware version')

        else:
            # if self.firmware_version[0] < 5:
            #   raise RuntimeError('Please upgrade the server firmware to version '
            #                        '5.0.0 or greater')
            print(f'Telemetrix4Arduino firmware version: {self.firmware_version[0]}.'
                  f'{self.firmware_version[1]}.{self.firmware_version[2]}')
        command = [PrivateConstants.ENABLE_ALL_REPORTS]
        self._send_command(command)

        # get the features list
        command = [PrivateConstants.GET_FEATURES]
        self._send_command(command)
        time.sleep(.2)

        # Have the server reset its data structures
        command = [PrivateConstants.RESET]
        self._send_command(command)

    def _find_arduino(self):
        """
        This method will search all potential serial ports for an Arduino
        containing a sketch that has a matching arduino_instance_id as
        specified in the input parameters of this class.

        This is used explicitly with the Telemetrix4Arduino sketch.
        """

        # a list of serial ports to be checked
        serial_ports = []

        print('Opening all potential serial ports...')
        the_ports_list = list_ports.comports()

        registered_ports = list(map(lambda p: p.port, self.serial_port_register.active))
        for port in the_ports_list:
            if port.pid is None or port.device in registered_ports:
                continue
            try:
                self.serial_port = serial.Serial(port.device, 115200, timeout=1, writeTimeout=0)
            except SerialException:
                continue
            # create a list of serial ports that we opened
            serial_ports.append(self.serial_port)

            # display to the user
            print('\t' + port.device)

            # clear out any possible data in the input buffer
        # wait for arduino to reset
        print(
            f'\nWaiting {self.arduino_wait} seconds(arduino_wait) for Arduino devices to '
            'reset...')
        # temporary for testing
        time.sleep(self.arduino_wait)
        self._run_threads()

        for serial_port in serial_ports:
            self.serial_port = serial_port

            # Since opening the port, there might be e.g., boot logs in the
            # buffer. Clear them before proceeding.
            self.serial_port.reset_input_buffer()

            self._get_arduino_id()
            retries = 50
            while self.reported_arduino_id is None and retries > 0:
                time.sleep(.2)
                retries -= 1
            if self.reported_arduino_id != self.arduino_instance_id:
                continue
            else:
                print('Valid Arduino ID Found.')
                self.serial_port.reset_input_buffer()
                self.serial_port.reset_output_buffer()
                return
        if self.shutdown_on_exception:
            self.shutdown()
        raise RuntimeError(f'Incorrect Arduino ID: {self.reported_arduino_id}')

    def _manual_open(self):
        """
        Com port was specified by the user - try to open up that port

        """
        # if port is not found, a serial exception will be thrown
        try:
            print(f'Opening {self.com_port}...')
            self.serial_port = serial.Serial(self.com_port, 115200,
                                             timeout=1, writeTimeout=0)

            print(
                f'\nWaiting {self.arduino_wait} seconds(arduino_wait) for Arduino devices to '
                'reset...')
            self._run_threads()
            time.sleep(self.arduino_wait)

            self._get_arduino_id()

            if self.reported_arduino_id != self.arduino_instance_id:
                if self.shutdown_on_exception:
                    self.shutdown()
                raise RuntimeError(f'Incorrect Arduino ID: {self.reported_arduino_id}')
            print('Valid Arduino ID Found.')
            # get arduino firmware version and print it
            print('\nRetrieving Telemetrix4Arduino firmware ID...')
            self._get_firmware_version()

            if not self.firmware_version:
                if self.shutdown_on_exception:
                    self.shutdown()
                raise RuntimeError(
                    f'Telemetrix4Arduino Sketch Firmware Version Not Found')

            else:
                print(f'Telemetrix4Arduino firmware version: {self.firmware_version[0]}.'
                      f'{self.firmware_version[1]}')
        except KeyboardInterrupt:
            if self.shutdown_on_exception:
                self.shutdown()
            raise RuntimeError('User Hit Control-C')

    def analog_write(self, pin, value):
        """
        Set the specified pin to the specified value.

        :param pin: arduino pin number

        :param value: pin value (maximum 16 bits)

        """
        value_msb = value >> 8
        value_lsb = value & 0xff
        command = [PrivateConstants.ANALOG_WRITE, pin, value_msb, value_lsb]
        self._send_command(command)

    def digital_write(self, pin, value):
        """
        Set the specified pin to the specified value.

        :param pin: arduino pin number

        :param value: pin value (1 or 0)

        """

        command = [PrivateConstants.DIGITAL_WRITE, pin, value]
        self._send_command(command)

    def disable_all_reporting(self):
        """
        Disable reporting for all digital and analog input pins
        """
        command = [PrivateConstants.MODIFY_REPORTING,
                   PrivateConstants.REPORTING_DISABLE_ALL, 0]
        self._send_command(command)

    def disable_analog_reporting(self, pin):
        """
        Disables analog reporting for a single analog pin.

        :param pin: Analog pin number. For example for A0, the number is 0.

        """
        command = [PrivateConstants.MODIFY_REPORTING,
                   PrivateConstants.REPORTING_ANALOG_DISABLE, pin]
        self._send_command(command)

    def disable_digital_reporting(self, pin):
        """
        Disables digital reporting for a single digital input.

        :param pin: Pin number.

        """
        command = [PrivateConstants.MODIFY_REPORTING,
                   PrivateConstants.REPORTING_DIGITAL_DISABLE, pin]
        self._send_command(command)

    def enable_analog_reporting(self, pin):
        """
        Enables analog reporting for the specified pin.

        :param pin: Analog pin number. For example for A0, the number is 0.


        """
        command = [PrivateConstants.MODIFY_REPORTING,
                   PrivateConstants.REPORTING_ANALOG_ENABLE, pin]
        self._send_command(command)

    def enable_digital_reporting(self, pin):
        """
        Enable reporting on the specified digital pin.

        :param pin: Pin number.
        """

        command = [PrivateConstants.MODIFY_REPORTING,
                   PrivateConstants.REPORTING_DIGITAL_ENABLE, pin]
        self._send_command(command)

    def _get_arduino_id(self):
        """
        Retrieve arduino-telemetrix arduino id

        """
        command = [PrivateConstants.ARE_U_THERE]
        self._send_command(command)
        # provide time for the reply
        time.sleep(.5)

    def _get_firmware_version(self):
        """
        This method retrieves the
        arduino-telemetrix firmware version

        """
        command = [PrivateConstants.GET_FIRMWARE_VERSION]
        self._send_command(command)
        # provide time for the reply
        time.sleep(.5)

    

    def loop_back(self, start_character, callback=None):
        """
        This is a debugging method to send a character to the
        Arduino device, and have the device loop it back.

        :param start_character: The character to loop back. It should be
                                an integer.

        :param callback: Looped back character will appear in the callback method

        """
        command = [PrivateConstants.LOOP_COMMAND, ord(start_character)]
        self.loop_back_callback = callback
        self._send_command(command)

    def set_analog_scan_interval(self, interval):
        """
        Set the analog scanning interval.

        :param interval: value of 0 - 255 - milliseconds
        """

        if 0 <= interval <= 255:
            command = [PrivateConstants.SET_ANALOG_SCANNING_INTERVAL, interval]
            self._send_command(command)
        else:
            if self.shutdown_on_exception:
                self.shutdown()
            raise RuntimeError('Analog interval must be between 0 and 255')

    def set_pin_mode_analog_output(self, pin_number):
        """
        Set a pin as a pwm (analog output) pin.

        :param pin_number:arduino pin number

        """
        self._set_pin_mode(pin_number, PrivateConstants.AT_OUTPUT)

    def set_pin_mode_analog_input(self, pin_number, differential=0, callback=None):
        """
        Set a pin as an analog input.

        :param pin_number: arduino pin number

        :param differential: difference in previous to current value before
                             report will be generated

        :param callback: callback function


        callback returns a data list:

        [pin_type, pin_number, pin_value, raw_time_stamp]

        The pin_type for analog input pins = 3

        """
        self._set_pin_mode(pin_number, PrivateConstants.AT_ANALOG, differential,
                           callback)

    def set_pin_mode_digital_input(self, pin_number, callback=None):
        """
        Set a pin as a digital input.

        :param pin_number: arduino pin number

        :param callback: callback function


        callback returns a data list:

        [pin_type, pin_number, pin_value, raw_time_stamp]

        The pin_type for all digital input pins = 2

        """
        self._set_pin_mode(pin_number, PrivateConstants.AT_INPUT, callback=callback)

    def set_pin_mode_digital_input_pullup(self, pin_number, callback=None):
        """
        Set a pin as a digital input with pullup enabled.

        :param pin_number: arduino pin number

        :param callback: callback function


        callback returns a data list:

        [pin_type, pin_number, pin_value, raw_time_stamp]

        The pin_type for all digital input pins = 2
        """
        self._set_pin_mode(pin_number, PrivateConstants.AT_INPUT_PULLUP,
                           callback=callback)

    def set_pin_mode_digital_output(self, pin_number):
        """
        Set a pin as a digital output pin.

        :param pin_number: arduino pin number
        """

        self._set_pin_mode(pin_number, PrivateConstants.AT_OUTPUT)

    def set_pin_mode_i2c(self, i2c_port=0):
        """
        Establish the standard Arduino i2c pins for i2c utilization.

        :param i2c_port: 0 = i2c1, 1 = i2c2

        NOTES: 1. THIS METHOD MUST BE CALLED BEFORE ANY I2C REQUEST IS MADE
               2. Callbacks are set within the individual i2c read methods of this
              API.

              See i2c_read, or i2c_read_restart_transmission.

        """
        # test for i2c port 2
        if i2c_port:
            # if not previously activated set it to activated
            # and the send a begin message for this port
            if not self.i2c_2_active:
                self.i2c_2_active = True
            else:
                return
        # port 1
        else:
            if not self.i2c_1_active:
                self.i2c_1_active = True
            else:
                return

        command = [PrivateConstants.I2C_BEGIN, i2c_port]
        self._send_command(command)

    def _set_pin_mode(self, pin_number, pin_state, differential=0, callback=None):
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
            if pin_state == PrivateConstants.AT_INPUT:
                self.digital_callbacks[pin_number] = callback
            elif pin_state == PrivateConstants.AT_INPUT_PULLUP:
                self.digital_callbacks[pin_number] = callback
            elif pin_state == PrivateConstants.AT_ANALOG:
                self.analog_callbacks[pin_number] = callback
            else:
                print('{} {}'.format('set_pin_mode: callback ignored for '
                                     'pin state:', pin_state))

        if pin_state == PrivateConstants.AT_INPUT:
            command = [PrivateConstants.SET_PIN_MODE, pin_number,
                       PrivateConstants.AT_INPUT, 1]

        elif pin_state == PrivateConstants.AT_INPUT_PULLUP:
            command = [PrivateConstants.SET_PIN_MODE, pin_number,
                       PrivateConstants.AT_INPUT_PULLUP, 1]

        elif pin_state == PrivateConstants.AT_OUTPUT:
            command = [PrivateConstants.SET_PIN_MODE, pin_number,
                       PrivateConstants.AT_OUTPUT]

        elif pin_state == PrivateConstants.AT_ANALOG:
            command = [PrivateConstants.SET_PIN_MODE, pin_number,
                       PrivateConstants.AT_ANALOG,
                       differential >> 8, differential & 0xff, 1]
        else:
            if self.shutdown_on_exception:
                self.shutdown()
            raise RuntimeError('Unknown pin state')

        if command:
            self._send_command(command)

    def shutdown(self):
        """
        This method attempts an orderly shutdown
        If any exceptions are thrown, they are ignored.
        """
        self.shutdown_flag = True

        self._stop_threads()

        try:
            command = [PrivateConstants.STOP_ALL_REPORTS]
            self._send_command(command)
            time.sleep(.5)

            if self.ip_address:
                try:
                    self.sock.shutdown(socket.SHUT_RDWR)
                    self.sock.close()
                except Exception:
                    pass
            else:
                try:
                    self.serial_port.reset_input_buffer()
                    self.serial_port.reset_output_buffer()

                    self.serial_port.close()
                    self.serial_port_register.remove(self.serial_port)

                except (RuntimeError, SerialException, OSError):
                    # ignore error on shutdown
                    pass
        except Exception:
            # raise RuntimeError('Shutdown failed - could not send stop streaming
            # message')
            pass

    '''
    report message handlers
    '''

    def _analog_message(self, data):
        """
        This is a private message handler method.
        It is a message handler for analog messages.

        :param data: message data

        """
        pin = data[0]
        value = (data[1] << 8) + data[2]
        # set the current value in the pin structure
        time_stamp = time.time()
        # self.digital_pins[pin].event_time = time_stamp
        try:
            if self.analog_callbacks[pin]:
                message = [PrivateConstants.ANALOG_REPORT, pin, value, time_stamp]
                self.analog_callbacks[pin](message)
        except KeyError:
            pass

    def _dht_report(self, data):
        """
        This is the dht report handler method.

        :param data:            data[0] = report error return
                                    No Errors = 0

                                    Checksum Error = 1

                                    Timeout Error = 2

                                    Invalid Value = 999

                                data[1] = pin number

                                data[2] = dht type 11 or 22

                                data[3] = humidity positivity flag

                                data[4] = temperature positivity value

                                data[5] = humidity integer

                                data[6] = humidity fractional value

                                data[7] = temperature integer

                                data[8] = temperature fractional value


        """
        if data[0]:  # DHT_ERROR
            # error report
            # data[0] = report sub type, data[1] = pin, data[2] = error message
            if self.dht_callbacks[data[1]]:
                # Callback 0=DHT REPORT, DHT_ERROR, PIN, Time
                message = [PrivateConstants.DHT_REPORT, data[0], data[1], data[2],
                           time.time()]
                self.dht_callbacks[data[1]](message)
        else:
            # got valid data DHT_DATA
            f_humidity = float(data[5] + data[6] / 100)
            if data[3]:
                f_humidity *= -1.0
            f_temperature = float(data[7] + data[8] / 100)
            if data[4]:
                f_temperature *= -1.0
            message = [PrivateConstants.DHT_REPORT, data[0], data[1], data[2],
                       f_humidity, f_temperature, time.time()]

            self.dht_callbacks[data[1]](message)

    def _digital_message(self, data):
        """
        This is a private message handler method.
        It is a message handler for Digital Messages.

        :param data: digital message

        """
        try:
            pin = data[0]
            value = data[1]

            time_stamp = time.time()
            if self.digital_callbacks[pin]:
                message = [PrivateConstants.DIGITAL_REPORT, pin, value, time_stamp]
                self.digital_callbacks[pin](message)
        except:
            # print('malformed message in _digital_message')
            pass

    def _i_am_here(self, data):
        """
        Reply to are_u_there message
        :param data: arduino id
        """
        self.reported_arduino_id = data[0]

    def _report_debug_data(self, data):
        """
        Print debug data sent from Arduino
        :param data: data[0] is a byte followed by 2
                     bytes that comprise an integer
        :return:
        """
        value = (data[1] << 8) + data[2]
        print(f'DEBUG ID: {data[0]} Value: {value}')

    def _report_loop_data(self, data):
        """
        Print data that was looped back
        :param data: byte of loop back data
        :return:
        """
        if self.loop_back_callback:
            self.loop_back_callback(data)

    def _send_command(self, command):
        """
        This is a private utility method.


        :param command:  command data in the form of a list

        """
        # the length of the list is added at the head
        command.insert(0, len(command))
        send_message = bytes(command)

        print(f'Sending command: {list(send_message)}')

        if self.serial_port:
            try:
                self.serial_port.write(send_message)
            except SerialException:
                if self.shutdown_on_exception:
                    self.shutdown()
                raise RuntimeError('write fail in _send_command')
        elif self.ip_address:
            self.sock.sendall(send_message)
        else:
            raise RuntimeError('No serial port or ip address set.')

    def _features_report(self, report):
        self.reported_features = report[0]

    def _run_threads(self):
        self.run_event.set()

    def _is_running(self):
        return self.run_event.is_set()

    def _stop_threads(self):
        self.run_event.clear()

    def _reporter(self):
        """
        This is the reporter thread. It continuously pulls data from
        the deque. When a full message is detected, that message is
        processed.
        """
        self.run_event.wait()

        while self._is_running() and not self.shutdown_flag:
            if len(self.the_deque):
                # response_data will be populated with the received data for the report
                response_data = []
                packet_length = self.the_deque.popleft()
                if packet_length:
                    # get all the data for the report and place it into response_data
                    for i in range(packet_length):
                        while not len(self.the_deque):
                            time.sleep(self.sleep_tune)
                        data = self.the_deque.popleft()
                        response_data.append(data)

                    # print(response_data)

                    # get the report type and look up its dispatch method
                    # here we pop the report type off of response_data
                    report_type = response_data.pop(0)
                    # print(report_type)

                    # retrieve the report handler from the dispatch table
                    dispatch_entry = self.report_dispatch.get(report_type)

                    # if there is additional data for the report,
                    # it will be contained in response_data
                    # noinspection PyArgumentList
                    dispatch_entry(response_data)
                    continue
                else:
                    if self.shutdown_on_exception:
                        self.shutdown()
                    raise RuntimeError(
                        'A report with a packet length of zero was received.')
            else:
                time.sleep(self.sleep_tune)

    def _serial_receiver(self):
        """
        Thread to continuously check for incoming data.
        When a byte comes in, place it onto the deque.
        """
        self.run_event.wait()

        # Don't start this thread if using a tcp/ip transport
        if self.ip_address:
            return

        while self._is_running() and not self.shutdown_flag:
            # we can get an OSError: [Errno9] Bad file descriptor when shutting down
            # just ignore it
            try:
                if self.serial_port.inWaiting():
                    c = self.serial_port.read()
                    self.the_deque.append(ord(c))
                    # print(ord(c))
                else:
                    time.sleep(self.sleep_tune)
                    # continue
            except OSError:
                pass

    def _tcp_receiver(self):
        """
        Thread to continuously check for incoming data.
        When a byte comes in, place it onto the deque.
        """
        self.run_event.wait()

        # Start this thread only if ip_address is set

        if self.ip_address:

            while self._is_running() and not self.shutdown_flag:
                try:
                    payload = self.sock.recv(1)
                    self.the_deque.append(ord(payload))
                except Exception:
                    pass
        else:
            return