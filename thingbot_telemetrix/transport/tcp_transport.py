"""
Copyright (c) 2026 ThingEdu. All rights reserved.

"""

import socket
import threading

from thingbot_telemetrix.transport.base import BaseTransport


class TcpTransport(BaseTransport):
    """
    TCP/IP transport for network-connected boards.
    """

    def __init__(self, ip_address, ip_port=31335):
        super().__init__()
        self.ip_address = ip_address
        self.ip_port = ip_port
        self.sock = None
        self._closed = False
        self._receiver_thread = threading.Thread(target=self._receiver)
        self._receiver_thread.daemon = True

    def open(self):
        self.sock = socket.create_connection((self.ip_address, self.ip_port))
        print(f"Connected to {self.ip_address}:{self.ip_port}")
        self._receiver_thread.start()

    def write(self, data):
        if not self.sock:
            raise RuntimeError("TCP socket is not connected.")
        self.sock.sendall(data)

    def close(self):
        self._closed = True
        if self.sock:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                self.sock.close()
            except OSError:
                pass

    def _receiver(self):
        print("Starting tcp/ip receiver thread...")
        self.run_event.wait()

        while self.run_event.is_set() and not self._closed:
            try:
                payload = self.sock.recv(1)
            except OSError:
                break
            if not payload:
                # remote end closed the connection
                break
            self.msg_deque.append(payload[0])
