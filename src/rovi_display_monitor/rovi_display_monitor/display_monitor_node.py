#!/usr/bin/env python3
"""Monitor voltage + CPU and stream JSON lines to an ESP32-S3 display over USB serial."""

from __future__ import annotations

import json
import logging
from typing import Optional

import psutil
import rclpy
from rclpy.node import Node
from serial import Serial, SerialException
from serial.tools import list_ports
from std_msgs.msg import Float32


class DisplayMonitor(Node):
    def __init__(self) -> None:
        super().__init__('rovi_display_monitor')

        self.declare_parameter('port', '')
        self.declare_parameter('usb_vid', '0x303a')
        self.declare_parameter('usb_pid', '0x1001')
        self.declare_parameter('baudrate', 256000)
        self.declare_parameter('voltage_topic', 'voltage')
        self.declare_parameter('publish_period', 3.0)
        self.declare_parameter('cpu_id', 'cpu')
        self.declare_parameter('voltage_id', 'voltage')

        self._serial: Optional[Serial] = None
        self._last_voltage: Optional[float] = None
        self._last_no_device_warn = None

        voltage_topic = self.get_parameter('voltage_topic').get_parameter_value().string_value
        self.create_subscription(Float32, voltage_topic, self._on_voltage, 10)

        period = float(self.get_parameter('publish_period').value)
        # Prime psutil so the first value is meaningful.
        psutil.cpu_percent(None)
        self.create_timer(period, self._on_timer)

    def _parse_vid_pid(self) -> tuple[int, int]:
        def _read_int(name: str) -> int:
            raw = str(self.get_parameter(name).value)
            try:
                return int(raw, 0)
            except ValueError:
                self.get_logger().warn(f"Invalid {name} '{raw}', falling back to 0")
                return 0

        return _read_int('usb_vid'), _read_int('usb_pid')

    def _on_voltage(self, msg: Float32) -> None:
        self._last_voltage = float(msg.data)

    def _ensure_serial(self) -> Optional[Serial]:
        if self._serial and self._serial.is_open:
            return self._serial

        port_override = self.get_parameter('port').get_parameter_value().string_value
        vid_expected, pid_expected = self._parse_vid_pid()

        port_to_use = port_override
        if not port_to_use:
            matches = [
                p for p in list_ports.comports()
                if p.vid == vid_expected and p.pid == pid_expected
            ]
            if len(matches) == 1:
                port_to_use = matches[0].device
            elif len(matches) == 0:
                now = self.get_clock().now()
                if (
                    self._last_no_device_warn is None
                    or (now - self._last_no_device_warn).nanoseconds > 10e9
                ):
                    self.get_logger().warn(
                        f"No serial device with VID:PID {vid_expected:04x}:{pid_expected:04x} found"
                    )
                    self._last_no_device_warn = now
                return None
            else:
                ports = ', '.join(m.device for m in matches)
                self.get_logger().warn(
                    f"Multiple matching devices for VID:PID {vid_expected:04x}:{pid_expected:04x}: {ports} "
                    "(set 'port' param explicitly)."
                )
                return None

        try:
            baud = int(self.get_parameter('baudrate').value)
        except (TypeError, ValueError):
            baud = 256000

        try:
            self._serial = Serial(port_to_use, baudrate=baud, timeout=1)
            self.get_logger().info(f"Opened display serial on {port_to_use} @ {baud} baud")
        except SerialException as exc:
            self.get_logger().warn(f"Failed to open serial port {port_to_use}: {exc}")
            self._serial = None
        return self._serial

    def _build_payload(self) -> list[dict]:
        payload = []

        cpu_id = self.get_parameter('cpu_id').get_parameter_value().string_value
        cpu_percent = psutil.cpu_percent(interval=None)
        payload.append(
            {
                'id': cpu_id,
                'value': int(round(cpu_percent)),
                'text': f"{int(round(cpu_percent))}%",
            }
        )

        if self._last_voltage is not None:
            voltage_id = self.get_parameter('voltage_id').get_parameter_value().string_value
            payload.append(
                {
                    'id': voltage_id,
                    'value': self._last_voltage,
                    'text': f"{self._last_voltage:.1f}V",
                }
            )
        return payload

    def _on_timer(self) -> None:
        ser = self._ensure_serial()
        if ser is None:
            return

        payload = self._build_payload()
        if not payload:
            return

        try:
            line = json.dumps(payload, separators=(',', ':')) + '\n'
            ser.write(line.encode('utf-8'))
        except SerialException as exc:
            self.get_logger().warn(f"Serial write failed: {exc}")
            try:
                ser.close()
            finally:
                self._serial = None


def main(args=None) -> None:
    logging.basicConfig(level=logging.INFO)
    rclpy.init(args=args)
    node = DisplayMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
