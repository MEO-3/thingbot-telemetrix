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

from thingbot_telemetrix.handler.dht_handler import DhtHandler
from thingbot_telemetrix.private_constants import ThingBotConstants
from thingbot_telemetrix.telemetrix_port_register import TelemetrixPortRegister
from thingbot_telemetrix.handler.gpio_handler import GpioHandler
from thingbot_telemetrix.handler.i2c_handler import I2CHandler


class Telemetrix(threading.Thread):
    """
    This class exposes and implements the telemetrix API.
    It uses threading to accommodate concurrency.
    It includes the public API methods as well as
    a set of private methods.
    """

    def __init__(self, com_port=None, 
                 arduino_instance_id=1, arduino_wait=4, 
                 sleep_tune=0.000001, shutdown_on_exception=True, 
                 ip_address=None, ip_port=31335
                 ):
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

        self.report_thread = threading.Thread(target=self._reporter)
        self.report_thread.daemon = True
        
        # tcp transport attributes
        self.ip_address = ip_address
        self.ip_port = ip_port
        
        if not self.ip_address:
            self.thread_data_receive = threading.Thread(target=self._serial_receiver)
        else:
            self.thread_data_receive = threading.Thread(target=self._tcp_receiver)
        self.thread_data_receive.daemon = True
            
        # threading event to control thread execution
        self.run_event = threading.Event()
        
        # instance attributes
        self.arduino_wait = arduino_wait
        self.sleep_tune = sleep_tune
        self.com_port = com_port
        self.arduino_instance_id = arduino_instance_id
        self.shutdown_on_exception = shutdown_on_exception
        
        # private instance attributes
        self.serial_port = None # serial port instance
        self.sock = None        # tcp/ip socket instance
        self.shutdown_flag = False
        
        self.reported_arduino_id = None
        
        # data receive deque
        self.msg_deque = deque()
        
        self.thread_data_receive.start()
        self.report_thread.start()
        
        if not self.ip_address:
            if not self.com_port:
                try:
                    self._find_arduino()
                except KeyboardInterrupt:
                    if self.shutdown_on_exception:
                        self.shutdown()
            else:
                self._manual_open()
                print(f'Using serial port: {self.serial_port.port}')
    
            if not self.serial_port:
                raise RuntimeError('No Arduino found on any serial port.')
            
        self.gpio_handler = GpioHandler(self)
        self.i2c_handler = I2CHandler(self)
        self.dht_handler = DhtHandler(self)
        
        # report dispatch table
        self.report_dispatch = {
            ThingBotConstants.I_AM_HERE_REPORT: self._i_am_here_report,
            ThingBotConstants.DEBUG_PRINT: self._debug_print_report,
        }
    
    def gpio(self):
        """
        Access to GPIO handler methods.

        :return: reference to GPIO handler instance
        """
        return self.gpio_handler
    
    def dht(self):
        """
        Access to DHT handler methods.

        :return: reference to DHT handler instance
        """
        return self.dht_handler
        

    # Thread control methods
    def _run_threads(self):
        self.run_event.set()

    def _is_running(self):
        return self.run_event.is_set()

    def _stop_threads(self):
        self.run_event.clear()
    
    # Private utility methods
    def _send_command(self, command):
        """
        This is a private utility method.
        
        :param command:  command data in the form of a list, e.g. [ThingBotConstants.PinModes.DIGITAL_WRITE, pin, value]

        """
        # the length of the list is added at the head, the format of command package is [length, command, param1, param2, ...]
        command.insert(0, len(command))
        send_message = bytes(command)

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
        
        print(f'Sent command: {list(send_message)}')
        
    # Find the Arduino connected serial port
    def _find_arduino(self):
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
            serial_ports.append(self.serial_port)

            print('\t' + port.device)

        print(f'\nWaiting {self.arduino_wait} seconds(arduino_wait) for Arduino devices to reset...')

        time.sleep(self.arduino_wait)
        self._run_threads()
        
        for sp in serial_ports:
            self.serial_port = sp
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
            self.serial_port = serial.Serial(self.com_port, 115200, timeout=1, writeTimeout=0)

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
        except KeyboardInterrupt:
            if self.shutdown_on_exception:
                self.shutdown()
            raise RuntimeError('User Hit Control-C')
    
    def _get_arduino_id(self):
        """
        Retrieve arduino-telemetrix arduino id
        """
        command = [ThingBotConstants.ARE_U_THERE]
        self._send_command(command)
        # provide time for the reply
        time.sleep(.5)
    
    def _i_am_here_report(self, data):
        """
        Handler for I_AM_HERE_REPORT
        :param data: list of data bytes for the report
        """
        if len(data) >= 1:
            self.reported_arduino_id = data[0]
            print(f'Reported Arduino ID: {self.reported_arduino_id}')

    def _debug_print_report(self, data):
        """
        Handler for DEBUG_PRINT report
        :param data: list of data bytes for the report
        """
        if len(data) >= 3:
            debug_id = data[0]
            debug_value = (data[1] << 8) | data[2]
            print(f'DEBUG_PRINT - ID: {debug_id}, Value: {debug_value}')

    def shutdown(self):
        """
        This method attempts an orderly shutdown
        If any exceptions are thrown, they are ignored.
        """
        self.shutdown_flag = True

        self._stop_threads()

        try:
            command = [ThingBotConstants.STOP_ALL_REPORTS]
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
    
    def _reporter(self):
        """
        This is the reporter thread. It continuously pulls data from
        the deque. When a full message is detected, that message is
        processed.
        """
        print('Starting reporter thread...')
        self.run_event.wait()

        while self._is_running() and not self.shutdown_flag:
            if len(self.msg_deque) > 0:
                # response_data will be populated with the received data for the report
                response_data = []
                packet_length = self.msg_deque.popleft()
                if packet_length:
                    # get all the data for the report and place it into response_data
                    for i in range(packet_length):
                        while not len(self.msg_deque):
                            time.sleep(self.sleep_tune)
                        data = self.msg_deque.popleft()
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
                    dispatch_entry(response_data)
                    continue
                else:
                    if self.shutdown_on_exception:
                        self.shutdown()
                    raise RuntimeError(
                        'A report with a packet length of zero was Received.')
            else:
                time.sleep(self.sleep_tune)
    
    # Receiver thread methods
    def _serial_receiver(self):
        """
        Thread to continuously check for incoming data.
        When a byte comes in, place it onto the deque.
        """
        print('Starting serial receiver thread...')
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
                    self.msg_deque.append(ord(c))
                    # print(f'Received byte: {ord(c)}')
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
        print('Starting tcp/ip receiver thread...')
        self.run_event.wait()

        # Start this thread only if ip_address is set
        if self.ip_address:
            while self._is_running() and not self.shutdown_flag:
                try:
                    payload = self.sock.recv(1)
                    self.msg_deque.append(ord(payload))
                except Exception:
                    pass
        else:
            return