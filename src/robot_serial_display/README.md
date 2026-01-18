# robot_serial_display

Serial display client: consumes `ros_ui_bridge` gRPC status stream and writes JSON lines to the ESP32-S3 display over USB serial.

## Config

Default config is installed at:

- `$(ros2 pkg prefix robot_serial_display)/share/robot_serial_display/config/default.yaml`

Key options:

- `gateway.address`: gRPC server address (default `127.0.0.1:50051`)
- `serial.port` / `serial.baudrate`: serial device (e.g. `/dev/robot_display`)
- `display.selected_ids`: ordered list of status field IDs to forward to the display (supports optional per-ID `scale`, e.g. `voltage` at x10)

Override the serial port at runtime with `ROVI_DISPLAY_PORT` (useful for debugging without udev rules).

The node calls `GetStatus` once to fetch metadata (unit/min/max/target) and the latest non-stale values, then subscribes to `StreamStatus` for ongoing updates. Only selected IDs with a current value are forwarded; values disappear when stale upstream.

If the gRPC bridge isn’t up yet, the node logs a single “waiting” line and keeps retrying until it can connect.

If the serial device is missing or can’t be opened, the node logs a warning once and keeps running; it will start sending automatically when the port becomes available.

## Run

```bash
ros2 run robot_serial_display serial_display -- --config /path/to/serial_display.yaml
```

## test
```bash
python3 - <<'PY'
from serial import Serial
payload = b'[{"id":"cpu","value":25,"text":"25%"}]\n'
with Serial("/dev/robot_display", baudrate=256000, timeout=1) as ser:
    ser.write(payload)
    ser.flush()
    print(f"sent {payload!r} to /dev/robot_display @256000")
PY
```

```bash
teleop ui_bridge_config:=/home/wass/dev/rovi_ros_ws/src/ros_ui_bridge/config/default.yaml serial_display_debug:=true
```
