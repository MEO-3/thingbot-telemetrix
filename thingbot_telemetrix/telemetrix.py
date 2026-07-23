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

import threading
import time
from collections import deque

import serial
from serial.serialutil import SerialException
from serial.tools import list_ports

from thingbot_telemetrix.handler.dht_handler import DhtHandler
from thingbot_telemetrix.handler.gpio_handler import GpioHandler
from thingbot_telemetrix.handler.i2c_handler import I2CHandler
from thingbot_telemetrix.handler.thingbot_handler import ThingBotHandler
from thingbot_telemetrix.handler.ultrasonic_handle import UltrasonicHandler
from thingbot_telemetrix.private_constants import ThingBotConstants
from thingbot_telemetrix.telemetrix_port_register import TelemetrixPortRegister
from thingbot_telemetrix.transport import SerialTransport, TcpTransport


class Telemetrix:
    """
    This class exposes and implements the telemetrix API.
    It uses threading to accommodate concurrency.
    It includes the public API methods as well as
    a set of private methods.
    """

    def __init__(
        self,
        com_port=None,
        arduino_instance_id=1,
        arduino_wait=4,
        sleep_tune=0.000001,
        shutdown_on_exception=True,
        ip_address=None,
        ip_port=31335,
        ble_address=None,
        ble_name=None,
        transport=None,
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

        :param ble_address: BLE address (or macOS UUID) of a board running
                            the BLE firmware.

        :param ble_name: BLE advertised name prefix; scan for a matching
                         board. Pass e.g. 'ThingBot' or a full
                         'ThingBot-<mac>' name.

        :param transport: a pre-built BaseTransport instance. Advanced use
                          (e.g. testing); bypasses port discovery and the
                          Arduino ID handshake.
        """

        self.serial_port_register = TelemetrixPortRegister()

        # instance attributes
        self.arduino_wait = arduino_wait
        self.sleep_tune = sleep_tune
        self.com_port = com_port
        self.arduino_instance_id = arduino_instance_id
        self.shutdown_on_exception = shutdown_on_exception

        self.shutdown_flag = False
        self.reported_arduino_id = None

        # data receive deque, fed by the transport, consumed by the reporter
        self.msg_deque = deque()

        # threading event to control thread execution
        self.run_event = threading.Event()

        # transport selection
        custom_transport = transport is not None
        use_ble = bool(ble_address or ble_name)
        if custom_transport:
            self.transport = transport
        elif ip_address:
            self.transport = TcpTransport(ip_address, ip_port)
        elif use_ble:
            # imported lazily; requires the optional bleak dependency
            from thingbot_telemetrix.transport.ble_transport import BleTransport

            if ble_name:
                self.transport = BleTransport(address=ble_address, name_prefix=ble_name)
            else:
                self.transport = BleTransport(address=ble_address)
        else:
            self.transport = SerialTransport(sleep_tune)
        self.transport.attach(self.msg_deque, self.run_event)

        self.report_thread = threading.Thread(target=self._reporter)
        self.report_thread.daemon = True
        self.report_thread.start()

        # handler instances
        self.gpio_handler = GpioHandler(self)
        self.i2c_handler = I2CHandler(self)
        self.dht_handler = DhtHandler(self)
        self.thingbot_handler = ThingBotHandler(self)
        self.ultrasonic_handler = UltrasonicHandler(self)

        # report dispatch table
        self.report_dispatch = {
            ThingBotConstants.I_AM_HERE_REPORT: self._i_am_here_report,
            ThingBotConstants.DEBUG_PRINT: self._debug_print_report,
            ThingBotConstants.DIGITAL_REPORT: self.gpio_handler.digital_report,
            ThingBotConstants.ANALOG_REPORT: self.gpio_handler.analog_report,
            ThingBotConstants.DHT_REPORT: self.dht_handler.dht_report,
            ThingBotConstants.THINGBOT_SW_REPORT: self.thingbot_handler.thingbot_sw_report,
            ThingBotConstants.ULTRASONIC_REPORT: self.ultrasonic_handler.ultrasonic_report,
        }

        # establish the connection
        if custom_transport:
            self.transport.open()
            self._run_threads()
        elif ip_address or use_ble:
            self.transport.open()
            self._run_threads()
            self._verify_arduino_id()
        else:
            self.transport.open()
            if not self.com_port:
                try:
                    self._find_arduino()
                except KeyboardInterrupt:
                    if self.shutdown_on_exception:
                        self.shutdown()
            else:
                self._manual_open()
                print(f"Using serial port: {self.transport.port.port}")

            if not self.transport.port:
                raise RuntimeError("No Arduino found on any serial port.")

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

    def ultrasonic(self):
        """
        Access to Ultrasonic handler methods.

        :return: reference to Ultrasonic handler instance
        """
        return self.ultrasonic_handler

    def thingbot(self):
        """
        Access to ThingBot handler methods.

        :return: reference to ThingBot handler instance
        """
        return self.thingbot_handler

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

        try:
            self.transport.write(send_message)
        except RuntimeError:
            if self.shutdown_on_exception:
                self.shutdown()
            raise

    # Find the Arduino connected serial port
    def _find_arduino(self):
        serial_ports = []
        print("Opening all potential serial ports...")
        the_ports_list = list_ports.comports()

        registered_ports = list(map(lambda p: p.port, self.serial_port_register.active))
        for port in the_ports_list:
            if port.pid is None or port.device in registered_ports:
                continue
            try:
                candidate = serial.Serial(
                    port.device, 115200, timeout=1, writeTimeout=0
                )
            except SerialException:
                continue
            serial_ports.append(candidate)

            print("\t" + port.device)

        print(
            f"\nWaiting {self.arduino_wait} seconds(arduino_wait) for Arduino devices to reset..."
        )

        time.sleep(self.arduino_wait)
        self._run_threads()

        for candidate in serial_ports:
            self.transport.port = candidate
            candidate.reset_input_buffer()

            self.reported_arduino_id = None
            self._get_arduino_id()

            retries = 50
            while self.reported_arduino_id is None and retries > 0:
                time.sleep(0.2)
                retries -= 1
            if self.reported_arduino_id != self.arduino_instance_id:
                continue
            else:
                print("Valid Arduino ID Found.")
                candidate.reset_input_buffer()
                candidate.reset_output_buffer()
                self.serial_port_register.add(candidate)
                return

        self.transport.port = None
        if self.shutdown_on_exception:
            self.shutdown()

        raise RuntimeError(f"Incorrect Arduino ID: {self.reported_arduino_id}")

    def _manual_open(self):
        """
        Com port was specified by the user - try to open up that port

        """
        # if port is not found, a serial exception will be thrown
        try:
            print(f"Opening {self.com_port}...")
            self.transport.port = serial.Serial(
                self.com_port, 115200, timeout=1, writeTimeout=0
            )

            print(
                f"\nWaiting {self.arduino_wait} seconds(arduino_wait) for Arduino devices to "
                "reset..."
            )
            self._run_threads()
            time.sleep(self.arduino_wait)

            self._verify_arduino_id()
            self.serial_port_register.add(self.transport.port)
        except KeyboardInterrupt:
            if self.shutdown_on_exception:
                self.shutdown()
            raise RuntimeError("User Hit Control-C")

    def _verify_arduino_id(self):
        """
        Query the board for its instance id and validate it against
        arduino_instance_id.
        """
        self._get_arduino_id()

        if self.reported_arduino_id != self.arduino_instance_id:
            if self.shutdown_on_exception:
                self.shutdown()
            raise RuntimeError(f"Incorrect Arduino ID: {self.reported_arduino_id}")
        print("Valid Arduino ID Found.")

    def _get_arduino_id(self):
        """
        Retrieve arduino-telemetrix arduino id
        """
        command = [ThingBotConstants.ARE_U_THERE]
        self._send_command(command)
        # provide time for the reply
        time.sleep(0.5)

    def _i_am_here_report(self, data):
        """
        Handler for I_AM_HERE_REPORT
        :param data: list of data bytes for the report
        """
        if len(data) >= 1:
            self.reported_arduino_id = data[0]
            print(f"Reported Arduino ID: {self.reported_arduino_id}")

    def _debug_print_report(self, data):
        """
        Handler for DEBUG_PRINT report
        :param data: list of data bytes for the report
        """
        if len(data) >= 3:
            debug_id = data[0]
            debug_value = (data[1] << 8) | data[2]
            print(f"DEBUG_PRINT - ID: {debug_id}, Value: {debug_value}")

    def shutdown(self):
        """
        This method attempts an orderly shutdown
        If any exceptions are thrown, they are ignored.
        """
        self.shutdown_flag = True

        self._stop_threads()

        port = getattr(self.transport, "port", None)
        self.transport.close()
        if port:
            try:
                self.serial_port_register.remove(port)
            except ValueError:
                pass

    def _reporter(self):
        """
        This is the reporter thread. It continuously pulls data from
        the deque. When a full message is detected, that message is
        processed.
        """
        print("Starting reporter thread...")
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

                    # get the report type and look up its dispatch method
                    # here we pop the report type off of response_data
                    report_type = response_data.pop(0)

                    # retrieve the report handler from the dispatch table
                    dispatch_entry = self.report_dispatch.get(report_type)
                    if dispatch_entry is None:
                        print(f"Unknown report type received: {report_type}")
                        continue

                    # if there is additional data for the report,
                    # it will be contained in response_data
                    dispatch_entry(response_data)
                    continue
                else:
                    if self.shutdown_on_exception:
                        self.shutdown()
                    raise RuntimeError(
                        "A report with a packet length of zero was Received."
                    )
            else:
                time.sleep(self.sleep_tune)
