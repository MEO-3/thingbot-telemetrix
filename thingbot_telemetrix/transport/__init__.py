from thingbot_telemetrix.transport.base import BaseTransport
from thingbot_telemetrix.transport.serial_transport import SerialTransport
from thingbot_telemetrix.transport.tcp_transport import TcpTransport

__all__ = ["BaseTransport", "SerialTransport", "TcpTransport", "BleTransport"]


def __getattr__(name):
    # BleTransport is loaded lazily so the core package does not require bleak
    if name == "BleTransport":
        from thingbot_telemetrix.transport.ble_transport import BleTransport

        return BleTransport
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
