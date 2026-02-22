# apple-silicon-accelerometer

Go port of [olvvier/apple-silicon-accelerometer](https://github.com/olvvier/apple-silicon-accelerometer) — read accelerometer, gyroscope, ambient light sensor, and lid angle from Apple Silicon MacBooks via IOKit HID.

## What it does

Reads the Bosch BMI286 IMU and other sensors exposed through Apple's `AppleSPUHIDDevice` IOKit interface on Apple Silicon MacBooks. Two binaries:

- **`sensord`** — Daemon that reads raw sensor data via IOKit HID callbacks and writes to POSIX shared memory ring buffers. Requires `sudo`.
- **`sensordash`** — Terminal dashboard that reads from shared memory and displays live vibration detection, orientation (Mahony AHRS), heartbeat estimation (BCG), lid angle, and ambient light.

## Requirements

- macOS on Apple Silicon (tested on M-series MacBooks)
- Go 1.26+
- Root privileges for `sensord` (IOKit HID access)

## Install

```bash
go install github.com/taigrr/apple-silicon-accelerometer/cmd/sensord@latest
go install github.com/taigrr/apple-silicon-accelerometer/cmd/sensordash@latest
```

## Usage

```bash
# Terminal 1: Start the sensor daemon (requires sudo)
sudo sensord

# Terminal 2: Launch the dashboard
sensordash
```

## Architecture

```
cmd/sensord/      → Sensor daemon (IOKit HID → shared memory)
cmd/sensordash/   → Terminal dashboard (shared memory → TUI)
sensor/           → IOKit/CoreFoundation bindings via purego (no CGO)
shm/              → POSIX shared memory ring buffers and snapshots
detector/         → Vibration detection, AHRS orientation, heartbeat BCG
```

### Shared Memory Layout

The shared memory format is compatible with the original Python implementation, allowing mixed Go/Python usage:

- **Ring buffers** (accel, gyro): 16-byte header + 8000 entries × 12 bytes (3× int32)
- **Snapshots** (ALS, lid): 8-byte header + raw payload

### No CGO

All macOS framework calls (IOKit, CoreFoundation) use [purego](https://github.com/ebitengine/purego) for zero-CGO dynamic linking.

## Credits

Original Python implementation by [olvvier](https://github.com/olvvier/apple-silicon-accelerometer).

## License

MIT — see [LICENSE](LICENSE).
