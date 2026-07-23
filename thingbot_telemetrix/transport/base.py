"""
Copyright (c) 2026 ThingEdu. All rights reserved.

"""


class BaseTransport:
    """
    Transport interface for Telemetrix.

    A transport owns the physical connection (serial, TCP, BLE, ...) and is
    responsible for feeding every received byte onto the shared msg_deque,
    where the Telemetrix reporter thread consumes it.

    Lifecycle: Telemetrix calls attach() with its shared state, then open()
    to establish the connection and start receiving, and close() on shutdown.
    """

    def __init__(self):
        self.msg_deque = None
        self.run_event = None

    def attach(self, msg_deque, run_event):
        """
        Called by Telemetrix before open() to share its receive deque and
        the run event that gates the receiver.

        :param msg_deque: deque that received bytes must be appended to
        :param run_event: threading.Event; receivers must wait for it to be
                          set before delivering bytes, and stop when cleared
        """
        self.msg_deque = msg_deque
        self.run_event = run_event

    def open(self):
        """
        Establish the connection and start feeding msg_deque.
        """
        raise NotImplementedError

    def write(self, data):
        """
        Send raw bytes to the device.

        :param data: bytes to send
        """
        raise NotImplementedError

    def close(self):
        """
        Stop receiving and release the connection. Must be safe to call
        more than once and must not raise.
        """
        raise NotImplementedError
