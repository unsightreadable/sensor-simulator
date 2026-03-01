# Industrial Sensor Data Simulator

Reads current (A), voltage (V), and temperature (C) from simulated industrial sensors and sends the data as a JSON payload via HTTP POST every 2 seconds.

## Requirements

- C++14 compiler
- Make

macOS:
```bash
xcode-select --install
```

Linux:
```bash
sudo apt install build-essential
```

## Setup

Create a `.env` file in the project directory:

```
SERVER_URL = https://api.cloud-server.com/ingest
AUTH_TOKEN = Bearer API_TOKEN
DEVICE_ID  = SENSOR-UNIT-001
```

## Build

```bash
# Edge release build (default)
make

# Development build with debug symbols
make dev

# Clean
make clean
```

## Run

```bash
# Run indefinitely until Ctrl+C
./sensor_simulator

# Run a fixed number of transmissions
./sensor_simulator 10
```

## Output

```
POST #1  url=https://api.cloud-server.com/ingest  latency=74ms  status=200 OK
auth: Bearer API_TOKEN
{
  "device_id"  : "SENSOR-UNIT-001",
  "sequence"   : 1,
  "timestamp"  : "2026-03-01T10:00:00Z",
  "sensors": {
    "current":     { "value": 12.483, "unit": "A" },
    "voltage":     { "value": 227.341, "unit": "V" },
    "temperature": { "value": 43.762, "unit": "C" }
  }
}

stats  sent=1  ok=1  failed=0  rate=100.0%
```

## Runtime files

| Path | Purpose |
|---|---|
| `/tmp/sensor_sim.log` | Persistent log of all events. Append-only, survives restarts. |
| `/tmp/sensor_sim.watchdog` | Timestamp updated every 10 cycles. Used by a process supervisor to detect hangs. |

## Retry behaviour

Transient failures (503) retry up to 3 times with exponential backoff starting at 250ms. Fatal errors (401) are logged and skipped immediately.
