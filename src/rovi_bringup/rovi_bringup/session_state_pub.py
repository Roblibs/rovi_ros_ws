from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String


SESSION_CURRENT_LAUNCH_REF_TOPIC = "/rovi/session/current_launch_ref"


class SessionStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__("rovi_session_state_pub")

        self.declare_parameter("launch_ref", "")
        launch_ref = str(self.get_parameter("launch_ref").value or "").strip()
        if not launch_ref:
            raise RuntimeError("launch_ref parameter is required")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(String, SESSION_CURRENT_LAUNCH_REF_TOPIC, qos)

        msg = String()
        msg.data = launch_ref
        self._pub.publish(msg)


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    try:
        node = SessionStatePublisher()
    except Exception as e:
        print(f"[rovi_session_state_pub] {e}", file=sys.stderr)
        rclpy.shutdown()
        return 2

    try:
        # Keep the publisher alive so late subscribers can still get the latched value.
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0
