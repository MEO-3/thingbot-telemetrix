"""
Unit tests for the Telemetrix client using a fake transport.
No hardware required.
"""

import time

import pytest

from thingbot_telemetrix import Telemetrix
from thingbot_telemetrix.private_constants import DHTTypes, PinModes, ThingBotConstants
from thingbot_telemetrix.transport import BaseTransport


class FakeTransport(BaseTransport):
    """Records written commands and lets tests inject report bytes."""

    def __init__(self):
        super().__init__()
        self.written = []
        self.opened = False
        self.closed = False

    def open(self):
        self.opened = True

    def write(self, data):
        self.written.append(bytes(data))

    def close(self):
        self.closed = True

    def inject(self, report):
        """Inject a device report as it would arrive on the wire."""
        wire = bytes([len(report)] + report)
        for byte in wire:
            self.msg_deque.append(byte)


def wait_for(predicate, timeout=2.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return False


@pytest.fixture
def board():
    transport = FakeTransport()
    board = Telemetrix(transport=transport)
    yield board, transport
    board.shutdown()


def test_transport_lifecycle(board):
    b, transport = board
    assert transport.opened
    assert transport.msg_deque is b.msg_deque
    b.shutdown()
    assert transport.closed


def test_digital_write_encoding(board):
    b, transport = board
    b.gpio().digital_write(13, 1)
    assert transport.written == [bytes([3, ThingBotConstants.DIGITAL_WRITE, 13, 1])]


def test_analog_input_pin_mode_encoding(board):
    b, transport = board
    b.gpio().set_pin_mode_analog_input(0, differential=300, callback=lambda v: None)
    assert transport.written == [
        bytes(
            [
                5,
                ThingBotConstants.SET_PIN_MODE,
                0,
                PinModes.ANALOG,
                300 >> 8,
                300 & 0xFF,
            ]
        )
    ]


def test_control_led_encoding(board):
    b, transport = board
    b.thingbot().control_led(1, 100)
    assert transport.written == [bytes([3, ThingBotConstants.LED_WRITE, 1, 100])]


def test_ultrasonic_pin_mode_encoding(board):
    b, transport = board
    b.ultrasonic().set_pin_mode_ultrasonic(5, 6)
    assert transport.written == [
        bytes([4, ThingBotConstants.SET_PIN_MODE, 5, PinModes.ULTRASONIC_PIN_MODE, 6])
    ]


def test_digital_report_dispatch(board):
    b, transport = board
    values = []
    b.gpio().set_pin_mode_digital_input(8, callback=values.append)

    transport.inject([ThingBotConstants.DIGITAL_REPORT, 8, 1])
    assert wait_for(lambda: values == [1])


def test_analog_report_dispatch(board):
    b, transport = board
    values = []
    b.gpio().set_pin_mode_analog_input(0, callback=values.append)

    transport.inject([ThingBotConstants.ANALOG_REPORT, 0, 0x02, 0x2B])
    assert wait_for(lambda: values == [555])


def test_dht_report_dispatch(board):
    b, transport = board
    readings = []
    b.dht().set_pin_mode_dht(
        2, DHTTypes.DHT22, callback=lambda t, h: readings.append((t, h))
    )

    # humidity 45.50%, temperature 21.25 C, transmitted as value * 100
    humidity, temperature = 4550, 2125
    transport.inject(
        [
            ThingBotConstants.DHT_REPORT,
            2,
            humidity >> 8,
            humidity & 0xFF,
            temperature >> 8,
            temperature & 0xFF,
        ]
    )
    assert wait_for(lambda: readings == [(21.25, 45.50)])


def test_ultrasonic_report_dispatch(board):
    b, transport = board
    readings = []
    b.ultrasonic().set_pin_mode_ultrasonic(
        5, 6, callback=lambda d, t, e: readings.append((d, t, e))
    )

    transport.inject([ThingBotConstants.ULTRASONIC_REPORT, 6, 5, 0, 42])
    assert wait_for(lambda: readings == [(42, 5, 6)])


def test_switch_report_dispatch(board):
    b, transport = board
    events = []
    b.thingbot().set_sw_callback(events.append)

    transport.inject([ThingBotConstants.THINGBOT_SW_REPORT, 0, 0])  # pressed
    transport.inject([ThingBotConstants.THINGBOT_SW_REPORT, 0, 1])  # released
    assert wait_for(lambda: events == [True, False])


def test_unknown_report_does_not_kill_reporter(board):
    b, transport = board
    transport.inject([200, 1, 2])  # unknown report id
    events = []
    b.thingbot().set_sw_callback(events.append)
    transport.inject([ThingBotConstants.THINGBOT_SW_REPORT, 0, 0])
    assert wait_for(lambda: events == [True])


def test_report_split_across_chunks(board):
    """A report delivered byte-by-byte (as over BLE chunks) still dispatches."""
    b, transport = board
    events = []
    b.thingbot().set_sw_callback(events.append)

    wire = bytes([3, ThingBotConstants.THINGBOT_SW_REPORT, 0, 0])
    for byte in wire:
        transport.msg_deque.append(byte)
        time.sleep(0.01)
    assert wait_for(lambda: events == [True])
