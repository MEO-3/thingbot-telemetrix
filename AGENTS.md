# AGENTS.md

This file provides guidance to AI Agentswhen working with code in this repository.

## What this repo is

Client libraries for controlling ThingBot hardware over the Telemetrix serial/TCP protocol:

- `thingbot_telemetrix/` — Python package (the primary client, published to PyPI as `thingbot-telemetrix`)
- `thingbot-telemetrix-rust/` — independent Rust crate implementing the same protocol
- The Arduino firmware lives in a **separate repo** (`MEO-3/thingbot-telemetrix-arduino`); it is not part of this one.

Most functionality requires a physical board on a serial port, so there is no Python test suite — examples in `examples/` double as manual integration tests. Project spec: `docs/specs/project_specs.md`. Release process: `docs/build_release.md`.

## Commands

Python (repo root):

```bash
pip install -e .[dev]        # dev install (pytest, black, isort, mypy)
black thingbot_telemetrix && isort thingbot_telemetrix
python examples/blink.py     # run against a connected board
python -m build              # build sdist/wheel (release only; bump version in pyproject.toml first)
```

Rust (from `thingbot-telemetrix-rust/`):

```bash
cargo fmt --check
cargo test                   # unit + integration tests (tests/ uses a mock transport, no hardware needed)
cargo test --examples
cargo test protocol          # run a single test target/filter
cargo run --example blink -- /dev/ttyUSB0
```

## Architecture

### Wire protocol (shared by both clients)

Both directions use length-prefixed byte packets: `[length, command_or_report_id, params...]`. All IDs live in `thingbot_telemetrix/private_constants.py` (`ThingBotConstants`) on the Python side and `src/protocol.rs` on the Rust side — **these must stay in sync with each other and with the firmware**. Connection handshake: client sends `ARE_U_THERE`, firmware replies with `I_AM_HERE_REPORT` carrying its `arduino_instance_id`; auto-detection opens every candidate serial port at 115200 baud and keeps the one whose ID matches.

### Python client (`thingbot_telemetrix/`)

`Telemetrix` (`telemetrix.py`) is the single entry point and owns two daemon threads:

1. A **receiver thread** (`_serial_receiver` or `_tcp_receiver`, chosen by whether `ip_address` is set) reads raw bytes onto a shared `deque`.
2. A **reporter thread** (`_reporter`) reassembles length-prefixed packets from the deque and dispatches them through `self.report_dispatch`, a dict mapping report IDs to handler methods.

Feature areas are split into handler classes in `handler/` (`GpioHandler`, `DhtHandler`, `ThingBotHandler`, `UltrasonicHandler`; `I2CHandler` is a stub). Each handler receives the `Telemetrix` instance, sends commands via its `_send_command`, and stores user callbacks that the dispatch table invokes. Adding a new feature means: add command/report constants to `private_constants.py`, add or extend a handler, and wire the report ID into `report_dispatch` in `Telemetrix.__init__`.

`TelemetrixPortRegister` tracks serial ports already claimed by other `Telemetrix` instances in the same process so auto-detect skips them.

### Rust client (`thingbot-telemetrix-rust/`)

Same protocol, different shape: synchronous, no threads or callbacks. `Telemetrix::connect(port)` in `client.rs` exposes `gpio()` / `thingbot()` / `ultrasonic()` sub-clients; incoming reports are pulled explicitly with `poll_report()` (returns `Option<Report>` from `report.rs`). `transport.rs` abstracts the serial connection, which is what the tests mock. TCP transport and port auto-detect are not implemented in Rust yet.

## Conventions

- Python formatting: black (line length 88) + isort with the black profile, per `pyproject.toml`.
- License is AGPL-3.0-or-later; source files carry the license header.
