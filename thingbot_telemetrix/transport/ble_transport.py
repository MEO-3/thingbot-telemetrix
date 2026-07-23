"""
Copyright (c) 2026 ThingEdu. All rights reserved.

"""

import asyncio
import threading

from thingbot_telemetrix.transport.base import BaseTransport

# GATT contract of the ThingBot Telemetrix BLE firmware (BLETransport.h)
SERVICE_UUID = "aa700001-8f6a-4e2c-b369-4060e0bb33aa"
RX_CHAR_UUID = "aa700002-8f6a-4e2c-b369-4060e0bb33aa"  # host -> device (write)
TX_CHAR_UUID = "aa700003-8f6a-4e2c-b369-4060e0bb33aa"  # device -> host (notify)


class BleTransport(BaseTransport):
    """
    BLE transport for the BLE firmware build.

    Runs an asyncio event loop in a daemon thread hosting a bleak client.
    Notifications on the TX characteristic feed msg_deque; commands are
    written to the RX characteristic with response - the ATT ack throttles
    the host against the firmware's small (256 byte) RX ring buffer.

    Requires the optional 'ble' extra: pip install thingbot-telemetrix[ble]
    """

    def __init__(
        self, address=None, name_prefix="ThingBot", scan_timeout=10.0, write_timeout=5.0
    ):
        """
        :param address: BLE address (or macOS UUID) of the board. If None,
                        scan for the first device advertising the ThingBot
                        service UUID or a matching name prefix.

        :param name_prefix: advertised name prefix used while scanning;
                            the firmware advertises as ThingBot-<mac>.

        :param scan_timeout: seconds to scan before giving up.

        :param write_timeout: seconds to wait for a GATT write to complete.
        """
        super().__init__()
        try:
            from bleak import BleakClient, BleakScanner
        except ImportError as import_error:
            raise RuntimeError(
                "BLE support requires the bleak package. "
                "Install it with: pip install thingbot-telemetrix[ble]"
            ) from import_error
        self._bleak_client_cls = BleakClient
        self._bleak_scanner_cls = BleakScanner

        self.address = address
        self.name_prefix = name_prefix
        self.scan_timeout = scan_timeout
        self.write_timeout = write_timeout

        self.client = None
        self._loop = None
        self._loop_thread = None
        self._closing = False

    def open(self):
        self._loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(target=self._loop.run_forever)
        self._loop_thread.daemon = True
        self._loop_thread.start()

        future = asyncio.run_coroutine_threadsafe(self._connect(), self._loop)
        try:
            # scan + connect + notification subscription
            future.result(self.scan_timeout + 20)
        except Exception:
            self.close()
            raise

    def write(self, data):
        if not (self.client and self.client.is_connected):
            raise RuntimeError("BLE device is not connected.")
        future = asyncio.run_coroutine_threadsafe(
            self.client.write_gatt_char(RX_CHAR_UUID, data, response=True), self._loop
        )
        try:
            future.result(self.write_timeout)
        except Exception as write_error:
            raise RuntimeError(
                f"write fail in BLE transport: {write_error}"
            ) from write_error

    def close(self):
        self._closing = True
        if self._loop is None:
            return
        if self.client is not None:
            try:
                asyncio.run_coroutine_threadsafe(self._disconnect(), self._loop).result(
                    5
                )
            except Exception:
                pass
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._loop_thread.join(timeout=2)

    async def _connect(self):
        target = self.address
        if not target:
            print(
                f"Scanning for a ThingBot BLE device "
                f"({self.scan_timeout} second timeout)..."
            )
            target = await self._bleak_scanner_cls.find_device_by_filter(
                self._device_filter, timeout=self.scan_timeout
            )
            if target is None:
                raise RuntimeError(
                    "No ThingBot BLE device found. Is the board powered and "
                    "running the BLE firmware?"
                )

        self.client = self._bleak_client_cls(
            target, disconnected_callback=self._on_disconnect
        )
        await self.client.connect()
        await self.client.start_notify(TX_CHAR_UUID, self._on_notify)
        print(f"Connected to BLE device: {self.client.address}")

    async def _disconnect(self):
        try:
            await self.client.disconnect()
        except Exception:
            pass

    def _device_filter(self, device, advertisement_data):
        uuids = advertisement_data.service_uuids or []
        if SERVICE_UUID in (u.lower() for u in uuids):
            return True
        name = advertisement_data.local_name or device.name or ""
        return bool(self.name_prefix) and name.startswith(self.name_prefix)

    def _on_notify(self, characteristic, data):
        # BLE delivers arbitrary chunks; the reporter thread reframes them
        for byte in data:
            self.msg_deque.append(byte)

    def _on_disconnect(self, client):
        if not self._closing:
            print("BLE device disconnected.")
