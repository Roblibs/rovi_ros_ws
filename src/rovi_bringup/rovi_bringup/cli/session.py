from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


SESSION_CURRENT_LAUNCH_REF_TOPIC = "/rovi/session/current_launch_ref"


class _SessionProbe(Node):
    def __init__(self) -> None:
        super().__init__("rovi_session_probe")
        self.value: str | None = None

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, SESSION_CURRENT_LAUNCH_REF_TOPIC, self._on_msg, qos)

    def _on_msg(self, msg: String) -> None:
        value = str(msg.data or "").strip()
        if value:
            self.value = value


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="rovi_session")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_get = sub.add_parser("get", help="Print current launch ref (from ROS topic)")
    p_get.add_argument("--timeout-s", type=float, default=10.0, help="Wait timeout (0 = wait forever).")

    args = parser.parse_args(argv)

    if args.cmd == "get":
        timeout_s = float(args.timeout_s)
        if timeout_s < 0:
            print("[rovi_session] --timeout-s must be >= 0", file=sys.stderr)
            return 2

        rclpy.init(args=None)
        try:
            probe = _SessionProbe()
            if timeout_s == 0:
                while rclpy.ok() and probe.value is None:
                    rclpy.spin_once(probe, timeout_sec=0.1)
            else:
                deadline = time.monotonic() + timeout_s
                while rclpy.ok() and probe.value is None and time.monotonic() < deadline:
                    rclpy.spin_once(probe, timeout_sec=0.1)

            if probe.value is None:
                return 1
            print(probe.value)
            return 0
        finally:
            try:
                probe.destroy_node()
            except Exception:
                pass
            rclpy.shutdown()

    print(f"[rovi_session] Unknown command: {args.cmd}", file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
