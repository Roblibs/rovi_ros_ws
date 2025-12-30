#!/usr/bin/env python3

"""Local (stub) JointState publisher.

Why this exists
--------------
On ROS 2 Jazzy, the upstream `joint_state_publisher` Python process can sometimes throw an
`RCLError: context is not valid` during Ctrl-C / launch shutdown (race between executor wait-set
setup and `rclpy.shutdown()`).

This node is a small, deterministic alternative used by `robot_mode=sim` to publish a minimal
`/joint_states` stream derived from the URDF (all non-fixed joints set to 0.0). It is *not* meant
for accurate simulation joint feedback; it only keeps `robot_state_publisher` and consumers that
expect `/joint_states` happy.

Offline URDF inspection remains handled by `joint_state_publisher_gui`.
"""

import argparse
import sys
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState


def _parse_movable_joint_names_from_urdf(urdf_xml: str) -> list[str]:
    root = ET.fromstring(urdf_xml)
    names: list[str] = []
    for joint in root.findall('joint'):
        joint_type = joint.get('type', 'fixed')
        if joint_type == 'fixed':
            continue
        name = joint.get('name')
        if name:
            names.append(name)
    return names


class LocalJointStatePublisher(Node):
    def __init__(self, urdf_path: str, publish_rate_hz: float = 20.0) -> None:
        super().__init__('rovi_local_joint_states')

        self.declare_parameter('joint_states_topic', 'joint_states')
        self.declare_parameter('publish_rate_hz', publish_rate_hz)

        joint_states_topic = str(self.get_parameter('joint_states_topic').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        publish_rate_hz = max(0.1, publish_rate_hz)

        try:
            urdf_xml = open(urdf_path, 'r', encoding='utf-8').read()
        except Exception as exc:
            raise RuntimeError(f"Failed to read URDF at '{urdf_path}': {exc}") from exc

        self._joint_names = _parse_movable_joint_names_from_urdf(urdf_xml)
        if not self._joint_names:
            self.get_logger().warn('No non-fixed joints found in URDF; /joint_states will be empty')

        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self._pub = self.create_publisher(JointState, joint_states_topic, qos)

        period = 1.0 / publish_rate_hz
        self._timer = self.create_timer(period, self._tick)

    def _tick(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_names)
        msg.position = [0.0] * len(self._joint_names)
        self._pub.publish(msg)


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description='Publish a minimal /joint_states stream (zeros) for all non-fixed URDF joints.'
    )
    parser.add_argument('urdf', nargs='?', help='Path to a URDF file')
    args, ros_args = parser.parse_known_args(argv if argv is not None else sys.argv[1:])

    if not args.urdf:
        raise SystemExit('URDF path is required (pass the URDF file as the first argument)')

    rclpy.init(args=ros_args)
    node: LocalJointStatePublisher | None = None
    try:
        node = LocalJointStatePublisher(args.urdf)
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            pass
        except Exception as exc:
            # Avoid noisy tracebacks on shutdown races; exit quietly.
            if 'context is not valid' in str(exc):
                pass
            else:
                raise
    finally:
        if node is not None:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
