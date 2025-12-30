#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from tf2_ros import TransformBroadcaster


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


class RoviGzOdom(Node):
    """Bridge helper for Gazebo ground-truth odometry.

    Gazebo publishes Odometry on its transport topic (/model/<name>/odometry), which we bridge
    via ros_gz_bridge to a ROS topic (default: /odom_gz). This node:
    - republishes it as /odom_raw with a stable frame contract (odom -> base_footprint)
    - optionally publishes TF odom -> base_footprint (for stacks that don't run EKF)
    - optionally projects the pose/twist into 2D (z=0, roll/pitch=0)
    """

    def __init__(self) -> None:
        super().__init__('rovi_gz_odom')

        self.declare_parameter('input_topic', 'odom_gz')
        self.declare_parameter('output_topic', 'odom_raw')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('two_d_mode', True)

        self._input_topic = str(self.get_parameter('input_topic').value)
        self._output_topic = str(self.get_parameter('output_topic').value)
        self._odom_frame = str(self.get_parameter('odom_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._publish_tf = bool(self.get_parameter('publish_tf').value)
        self._two_d_mode = bool(self.get_parameter('two_d_mode').value)

        qos_in = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        qos_out = QoSProfile(depth=20, reliability=QoSReliabilityPolicy.RELIABLE)

        self._pub = self.create_publisher(Odometry, self._output_topic, qos_out)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._sub = self.create_subscription(Odometry, self._input_topic, self._on_odom, qos_in)

    def _on_odom(self, msg: Odometry) -> None:
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._odom_frame
        out.child_frame_id = self._base_frame

        out.pose = msg.pose
        out.twist = msg.twist

        if self._two_d_mode:
            out.pose.pose.position.z = 0.0

            q = out.pose.pose.orientation
            yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)
            qx, qy, qz, qw = _yaw_to_quaternion(yaw)
            q.x = float(qx)
            q.y = float(qy)
            q.z = float(qz)
            q.w = float(qw)

            out.twist.twist.linear.z = 0.0
            out.twist.twist.angular.x = 0.0
            out.twist.twist.angular.y = 0.0

        self._pub.publish(out)

        if not self._publish_tf:
            return

        t = TransformStamped()
        t.header.stamp = out.header.stamp
        t.header.frame_id = out.header.frame_id
        t.child_frame_id = out.child_frame_id
        t.transform.translation.x = float(out.pose.pose.position.x)
        t.transform.translation.y = float(out.pose.pose.position.y)
        t.transform.translation.z = float(out.pose.pose.position.z)
        t.transform.rotation = out.pose.pose.orientation
        self._tf_broadcaster.sendTransform(t)


def main() -> None:
    rclpy.init()
    node = RoviGzOdom()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
