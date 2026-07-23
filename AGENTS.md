# AGENTS.md

This file provides guidance to AI Agentswhen working with code in this repository.

## What this repo is

Client libraries for controlling ThingBot hardware over the Telemetrix protocol (serial, TCP, or BLE):

- `thingbot_telemetrix/` — Python package (the primary client, published to PyPI as `thingbot-telemetrix`)
- `thingbot-telemetrix-rust/` — independent Rust crate implementing the same protocol
- The Arduino firmware lives in a **separate repo** (`MEO-3/thingbot-telemetrix-arduino`); it is not part of this one.

Hardware-free unit tests live in `tests/` (Python, fake transport) and `thingbot-telemetrix-rust/tests/` (mock transport); examples in `examples/` double as hardware integration tests. Project spec: `docs/specs/project_specs.md`. Release process: `docs/build_release.md`.

## Commands

Python (repo root):

```bash
pip install -e .[dev]        # dev install (pytest, black, isort, mypy)
pip install -e .[ble]        # optional BLE support (bleak)
python -m pytest tests/      # unit tests, no hardware needed
black thingbot_telemetrix && isort thingbot_telemetrix
python examples/blink.py     # run against a connected board
python -m build              # build sdist/wheel (release only; bump version in pyproject.toml first)
```

Rust (from `thingbot-telemetrix-rust/`):

```bash
cargo fmt --check
cargo test                   # unit + integration tests (tests/ uses a mock transport, no hardware needed)
cargo test --features ble    # include the BLE transport in the build
cargo test --examples
cargo test protocol          # run a single test target/filter
cargo run --example blink -- /dev/ttyUSB0
cargo run --example blink_ble --features ble
```

## Architecture

### Wire protocol (shared by both clients)

Both directions use length-prefixed byte packets: `[length, command_or_report_id, params...]`. All IDs live in `thingbot_telemetrix/private_constants.py` (`ThingBotConstants`) on the Python side and `src/protocol.rs` on the Rust side — **these must stay in sync with each other and with the firmware**. Connection handshake: client sends `ARE_U_THERE`, firmware replies with `I_AM_HERE_REPORT` carrying its `arduino_instance_id`; auto-detection opens every candidate serial port at 115200 baud and keeps the one whose ID matches.

### BLE (both clients)

The BLE firmware exposes a NUS-style GATT byte pipe carrying the same length-prefixed packets: service `aa700001-8f6a-4e2c-b369-4060e0bb33aa`, host writes commands to characteristic `...0002`, reports arrive as notifications on `...0003`. The board advertises as `ThingBot-<mac>`. Notifications are arbitrary chunks — clients must accumulate bytes and reframe by length prefix. Writes use write-with-response; the ATT ack is the flow control against the firmware's 256-byte RX ring (which drops silently on overflow).

### Python client (`thingbot_telemetrix/`)

`Telemetrix` (`telemetrix.py`) is the single entry point. Transports live in `transport/` (`SerialTransport`, `TcpTransport`, `BleTransport` — the latter lazy-imported so bleak stays optional); each transport owns its connection and feeds raw bytes onto the shared `msg_deque`. A **reporter thread** (`_reporter`) reassembles length-prefixed packets from the deque and dispatches them through `self.report_dispatch`, a dict mapping report IDs to handler methods. Transport is selected by constructor args: `com_port`/none → serial (with auto-detect), `ip_address` → TCP, `ble_address`/`ble_name` → BLE, `transport=` → injected (used by the tests' `FakeTransport`).

Feature areas are split into handler classes in `handler/` (`GpioHandler`, `DhtHandler`, `ThingBotHandler`, `UltrasonicHandler`; `I2CHandler` is a stub). Each handler receives the `Telemetrix` instance, sends commands via its `_send_command`, and stores user callbacks that the dispatch table invokes. Adding a new feature means: add command/report constants to `private_constants.py`, add or extend a handler, and wire the report ID into `report_dispatch` in `Telemetrix.__init__`.

`TelemetrixPortRegister` tracks serial ports already claimed by other `Telemetrix` instances in the same process so auto-detect skips them.

### Rust client (`thingbot-telemetrix-rust/`)

Same protocol, different shape: synchronous, no callbacks. `Telemetrix::connect(port)` / `Telemetrix::connect_ble()` in `client.rs` expose `gpio()` / `thingbot()` / `ultrasonic()` sub-clients; incoming reports are pulled explicitly with `poll_report()` (returns `Option<Report>` from `report.rs`). `transport.rs` abstracts the connection (what the tests mock) and provides `PacketAssembler` for reframing byte streams. `ble.rs` (cargo feature `ble`) hides a small tokio runtime driving btleplug behind the sync `Transport` trait. TCP transport and serial auto-detect are not implemented in Rust yet.

## Conventions

- Python formatting: black (line length 88) + isort with the black profile, per `pyproject.toml`.
- License is AGPL-3.0-or-later; source files carry the license header.
