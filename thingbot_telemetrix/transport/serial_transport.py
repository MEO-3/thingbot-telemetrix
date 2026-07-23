"""
Copyright (c) 2026 ThingEdu. All rights reserved.

"""

import threading
import time

from serial.serialutil import SerialException

from thingbot_telemetrix.transport.base import BaseTransport


class SerialTransport(BaseTransport):
    """
    Serial (USB) transport.

    The active pyserial instance is held in self.port. During auto-detection
    Telemetrix swaps self.port across candidate ports while probing with
    ARE_U_THERE; the receiver thread always reads whichever port is current.
    """

    def __init__(self, sleep_tune=0.000001):
        super().__init__()
        self.sleep_tune = sleep_tune
        self.port = None
        self._closed = False
        self._receiver_thread = threading.Thread(target=self._receiver)
        self._receiver_thread.daemon = True

    def open(self):
        self._receiver_thread.start()

    def write(self, data):
        if not self.port:
            raise RuntimeError("No serial port is open.")
        try:
            self.port.write(data)
        except SerialException:
            raise RuntimeError("write fail in serial transport")

    def close(self):
        self._closed = True
        if self.port:
            try:
                self.port.reset_input_buffer()
                self.port.reset_output_buffer()
                self.port.close()
            except (SerialException, OSError):
                pass

    def _receiver(self):
        print("Starting serial receiver thread...")
        self.run_event.wait()

        while self.run_event.is_set() and not self._closed:
            # we can get an OSError: [Errno9] Bad file descriptor when
            # shutting down - just ignore it
            try:
                if self.port and self.port.in_waiting:
                    self.msg_deque.append(ord(self.port.read()))
                else:
                    time.sleep(self.sleep_tune)
            except OSError:
                pass
