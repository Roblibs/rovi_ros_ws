# rovi_serial_display

Serial display client for Rovi: consumes `rovi_ui_gateway` gRPC status stream and writes JSON lines to the ESP32-S3 display over USB serial.

## Config

Default config is installed at:

- `$(ros2 pkg prefix rovi_serial_display)/share/rovi_serial_display/config/default.yaml`

Key options:

- `gateway.address`: gRPC server address (default `127.0.0.1:50051`)
- `serial.port` / `serial.baudrate`: serial device (e.g. `/dev/rovi_display`)
- `ids.cpu` / `ids.voltage`: IDs used for CPU/voltage

Any `rates[]` received from the gateway are forwarded as additional `{id,value,text}` entries in the same JSON payload.

## Run

```bash
ros2 run rovi_serial_display serial_display -- --config /path/to/serial_display.yaml
```

