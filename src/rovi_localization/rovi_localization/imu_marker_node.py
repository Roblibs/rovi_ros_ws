#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker


class ImuAccelMarkerNode(Node):
    def __init__(self) -> None:
        super().__init__('rovi_imu_marker')

        self.declare_parameter('use_sim_time', False)
        self.declare_parameter('imu_topic', 'imu/data_raw')
        self.declare_parameter('marker_topic', 'imu/accel_marker')
        self.declare_parameter('frame_id_override', '')
        self.declare_parameter('scale_m_per_ms2', 0.02)
        self.declare_parameter('max_arrow_m', 0.5)

        self._imu_topic = str(self.get_parameter('imu_topic').value)
        self._marker_topic = str(self.get_parameter('marker_topic').value)
        self._frame_id_override = str(self.get_parameter('frame_id_override').value)
        self._scale_m_per_ms2 = float(self.get_parameter('scale_m_per_ms2').value)
        self._max_arrow_m = float(self.get_parameter('max_arrow_m').value)

        self._scale_m_per_ms2 = max(self._scale_m_per_ms2, 0.0)
        self._max_arrow_m = max(self._max_arrow_m, 0.0)

        self._pub = self.create_publisher(Marker, self._marker_topic, 10)
        self._sub = self.create_subscription(Imu, self._imu_topic, self._on_imu, qos_profile_sensor_data)

    def _on_imu(self, msg: Imu) -> None:
        ax = float(msg.linear_acceleration.x)
        ay = float(msg.linear_acceleration.y)
        az = float(msg.linear_acceleration.z)

        dx = ax * self._scale_m_per_ms2
        dy = ay * self._scale_m_per_ms2
        dz = az * self._scale_m_per_ms2

        norm = math.sqrt(dx * dx + dy * dy + dz * dz)
        if norm > self._max_arrow_m and norm > 0.0:
            factor = self._max_arrow_m / norm
            dx *= factor
            dy *= factor
            dz *= factor

        frame_id = self._frame_id_override.strip()
        if not frame_id:
            frame_id = (msg.header.frame_id or '').strip() or 'imu_link'

        marker = Marker()
        marker.header = msg.header
        marker.header.frame_id = frame_id

        marker.ns = 'imu_accel'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.frame_locked = True

        marker.points = [
            Point(x=0.0, y=0.0, z=0.0),
            Point(x=dx, y=dy, z=dz),
        ]

        marker.scale.x = 0.02
        marker.scale.y = 0.04
        marker.scale.z = 0.08

        marker.color.a = 0.9
        marker.color.r = 0.2
        marker.color.g = 0.7
        marker.color.b = 1.0

        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        self._pub.publish(marker)


def main() -> None:
    rclpy.init()
    node = ImuAccelMarkerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
