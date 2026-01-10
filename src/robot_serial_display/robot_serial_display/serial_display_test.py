#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json

from serial import Serial, SerialException


def _format_percent(value: float) -> str:
    return f"{value:.0f}%"


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Send a single test payload to the serial display.")
    parser.add_argument("--port", default="/dev/robot_display", help="Serial device (default: /dev/robot_display)")
    parser.add_argument("--baudrate", type=int, default=256000, help="Serial baudrate (default: 256000)")
    parser.add_argument("--value", type=float, default=12.0, help="cpu value to send (default: 12.0)")
    args = parser.parse_args(argv)

    value = float(args.value)
    if value.is_integer():
        encoded_value = int(value)
    else:
        encoded_value = value

    payload = [
        {
            "id": "cpu",
            "value": encoded_value,
            "text": _format_percent(value),
        }
    ]
    line = json.dumps(payload, separators=(",", ":")) + "\n"

    try:
        with Serial(args.port, baudrate=args.baudrate, timeout=1) as ser:
            print(f"[serial_display_test] Sending to {args.port} @ {args.baudrate} baud: {payload}")
            print(f"[serial_display_test] Raw line: {line.encode('utf-8')!r}")
            ser.write(line.encode("utf-8"))
            ser.flush()
            print("[serial_display_test] Write complete.")
    except SerialException as exc:
        raise SystemExit(f"Failed to write to serial display on {args.port}: {exc}") from exc


if __name__ == "__main__":
    main()
