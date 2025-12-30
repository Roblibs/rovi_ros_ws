# robot_serial_display

Serial display client: consumes `ros_ui_bridge` gRPC status stream and writes JSON lines to the ESP32-S3 display over USB serial.

## Config

Default config is installed at:

- `$(ros2 pkg prefix robot_serial_display)/share/robot_serial_display/config/default.yaml`

Key options:

- `gateway.address`: gRPC server address (default `127.0.0.1:50051`)
- `serial.port` / `serial.baudrate`: serial device (e.g. `/dev/robot_display`)
- `display.selected_ids`: ordered list of IDs to forward to the display (e.g. `cpu`, `voltage`, `hz_driver`, `hz_slam`)

Only the selected IDs are forwarded (if the list is empty, nothing is sent). Rate metrics come from `ros_ui_bridge` (`rates[]`) and are formatted as `{id,value,text}` like `40/50Hz` when a target is provided.

If the gRPC bridge isn’t up yet, the node logs a single “waiting” line and keeps retrying until it can connect.

If the serial device is missing or can’t be opened, the node logs a warning once and keeps running; it will start sending automatically when the port becomes available.

## Run

```bash
ros2 run robot_serial_display serial_display -- --config /path/to/serial_display.yaml
```
