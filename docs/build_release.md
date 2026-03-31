# Build and Release

## Scope

This document covers building and uploading the Python package in this repository to a package index (TestPyPI or PyPI).

## Prerequisites

- Python 3.9+ installed.
- Build tools installed: `pip install --upgrade build twine`.
- Credentials for the target package index:
  - TestPyPI or PyPI token configured via `TWINE_USERNAME=__token__` and `TWINE_PASSWORD=<token>` or via a `.pypirc` file.

## Update Version

- Edit `pyproject.toml` and update `[project].version`.

## Build

From the repository root:

```bash
python -m build
```

Build artifacts will be placed in `dist/`:

- `dist/thingbot_telemetrix-<version>.tar.gz`
- `dist/thingbot_telemetrix-<version>-py3-none-any.whl`

## Validate Distributions

```bash
python -m twine check dist/*
```

## Upload

### TestPyPI (recommended first)

```bash
python -m twine upload --repository testpypi dist/*
```

### PyPI

```bash
python -m twine upload dist/*
```

## Post-Release Verification

- Install from the target index and run a quick import check:

```bash
pip install --index-url https://test.pypi.org/simple/ --extra-index-url https://pypi.org/simple thingbot-telemetrix
python -c "from thingbot_telemetrix import Telemetrix; print(Telemetrix)"
```

## Notes

- The package name in `pyproject.toml` is `thingbot_telemetrix`, but the published name may appear as `thingbot-telemetrix` depending on the index.
- Firmware building and flashing is handled in the separate `thingbot-telemetrix-arduino` repository per `README.md`.
