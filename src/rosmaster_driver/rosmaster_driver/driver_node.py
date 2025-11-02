#!/usr/bin/env python3
# Copyright 2025
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, MagneticField, JointState


def _safe_import_rosmaster():
    try:
        from Rosmaster_Lib import Rosmaster  # type: ignore
        return Rosmaster
    except Exception:
        return None


class RosmasterAdapter:
    """Thin wrapper around Rosmaster library with graceful fallback.

    If the underlying library is missing, publishes zeros and logs a warning once.
    """

    def __init__(self, node: Node) -> None:
        self._node = node
        self._rosmaster_cls = _safe_import_rosmaster()
        self._hw = None
        self._warned = False
        if self._rosmaster_cls is not None:
            try:
                self._hw = self._rosmaster_cls()
                # Some boards require a receive thread to be started
                if hasattr(self._hw, 'create_receive_threading'):
                    self._hw.create_receive_threading()
            except Exception as exc:
                self._node.get_logger().warn(f"Failed to init Rosmaster hardware: {exc}")
                self._hw = None

    def _log_missing(self):
        if not self._warned:
            self._node.get_logger().warn(
                "Rosmaster_Lib not available or failed to initialize. "
                "Publishing zeros and ignoring cmd_vel until the library is installed."
            )
            self._warned = True

    def set_motion(self, vx: float, vy: float, wz: float) -> None:
        if self._hw is None:
            self._log_missing()
            return
        try:
            if hasattr(self._hw, 'set_car_motion'):
                self._hw.set_car_motion(vx, vy, wz)
        except Exception as exc:
            self._node.get_logger().error(f"set_car_motion failed: {exc}")

    def get_version(self) -> float:
        if self._hw is None:
            self._log_missing()
            return 0.0
        try:
            v = self._hw.get_version()
            return float(v)
        except Exception:
            return 0.0

    def get_battery_voltage(self) -> float:
        if self._hw is None:
            self._log_missing()
            return 0.0
        try:
            v = self._hw.get_battery_voltage()
            return float(v)
        except Exception:
            return 0.0

    def get_accelerometer(self) -> Tuple[float, float, float]:
        if self._hw is None:
            self._log_missing()
            return (0.0, 0.0, 0.0)
        try:
            return tuple(float(x) for x in self._hw.get_accelerometer_data())  # type: ignore
        except Exception:
            return (0.0, 0.0, 0.0)

    def get_gyroscope(self) -> Tuple[float, float, float]:
        if self._hw is None:
            self._log_missing()
            return (0.0, 0.0, 0.0)
        try:
            return tuple(float(x) for x in self._hw.get_gyroscope_data())  # type: ignore
        except Exception:
            return (0.0, 0.0, 0.0)

    def get_magnetometer(self) -> Tuple[float, float, float]:
        if self._hw is None:
            self._log_missing()
            return (0.0, 0.0, 0.0)
        try:
            return tuple(float(x) for x in self._hw.get_magnetometer_data())  # type: ignore
        except Exception:
            return (0.0, 0.0, 0.0)

    def get_motion(self) -> Tuple[float, float, float]:
        if self._hw is None:
            self._log_missing()
            return (0.0, 0.0, 0.0)
        try:
            return tuple(float(x) for x in self._hw.get_motion_data())  # type: ignore
        except Exception:
            return (0.0, 0.0, 0.0)


class RosmasterDriverNode(Node):
    """ROS 2 node that bridges cmd_vel to Rosmaster board and publishes sensors/state."""

    def __init__(self) -> None:
        super().__init__('rosmaster_driver')

        # Parameters
        self.declare_parameter('imu_link', 'imu_link')
        self.declare_parameter('prefix', '')
        self.declare_parameter('publish_rate', 10.0)  # Hz

        self.imu_link = self.get_parameter('imu_link').get_parameter_value().string_value
        self.prefix = self.get_parameter('prefix').get_parameter_value().string_value
        self.publish_rate = float(self.get_parameter('publish_rate').get_parameter_value().double_value)
        if self.publish_rate <= 0.0:
            self.publish_rate = 10.0

        # Hardware adapter
        self.hw = RosmasterAdapter(self)

        # Interfaces
        self.sub_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)

        self.pub_edition = self.create_publisher(Float32, 'edition', 10)
        self.pub_voltage = self.create_publisher(Float32, 'voltage', 10)
        self.pub_joint_states = self.create_publisher(JointState, 'joint_states', 10)
        self.pub_vel_raw = self.create_publisher(Twist, 'vel_raw', 10)
        self.pub_imu_raw = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.pub_mag = self.create_publisher(MagneticField, '/imu/mag', 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self._on_timer)

        self.get_logger().info(
            f"rosmaster_driver started (imu_link={self.imu_link}, prefix='{self.prefix}', rate={self.publish_rate} Hz)"
        )

    # --- Callbacks ---
    def _on_cmd_vel(self, msg: Twist) -> None:
        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        wz = float(msg.angular.z)
        self.hw.set_motion(vx, vy, wz)

    def _on_timer(self) -> None:
        now = Clock().now().to_msg()

        # Read hardware
        edition_val = self.hw.get_version()
        voltage_val = self.hw.get_battery_voltage()
        ax, ay, az = self.hw.get_accelerometer()
        gx, gy, gz = self.hw.get_gyroscope()
        mx, my, mz = self.hw.get_magnetometer()
        vx, vy, wz = self.hw.get_motion()

        # edition
        edition = Float32()
        edition.data = float(edition_val)
        self.pub_edition.publish(edition)

        # voltage
        voltage = Float32()
        voltage.data = float(voltage_val)
        self.pub_voltage.publish(voltage)

        # IMU
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = self.imu_link
        imu.linear_acceleration.x = float(ax)
        imu.linear_acceleration.y = float(ay)
        imu.linear_acceleration.z = float(az)
        imu.angular_velocity.x = float(gx)
        imu.angular_velocity.y = float(gy)
        imu.angular_velocity.z = float(gz)
        # Orientation is unknown; leave default zeros with covariance unset
        self.pub_imu_raw.publish(imu)

        # Magnetometer
        mag = MagneticField()
        mag.header.stamp = now
        mag.header.frame_id = self.imu_link
        mag.magnetic_field.x = float(mx)
        mag.magnetic_field.y = float(my)
        mag.magnetic_field.z = float(mz)
        self.pub_mag.publish(mag)

        # vel_raw (feedback)
        twist = Twist()
        twist.linear.x = float(vx)
        twist.linear.y = float(vy)
        twist.angular.z = float(wz)
        self.pub_vel_raw.publish(twist)

        # joint_states (names only; positions unknown -> zeros)
        js = JointState()
        js.header.stamp = now
        js.header.frame_id = 'joint_states'
        names = [
            'back_right_joint',
            'back_left_joint',
            'front_left_steer_joint',
            'front_left_wheel_joint',
            'front_right_steer_joint',
            'front_right_wheel_joint',
        ]
        if self.prefix:
            names = [self.prefix + n for n in names]
        js.name = names
        js.position = [0.0] * len(names)
        js.velocity = [0.0] * len(names)
        js.effort = [0.0] * len(names)
        self.pub_joint_states.publish(js)


def main() -> None:
    rclpy.init()
    node = RosmasterDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
