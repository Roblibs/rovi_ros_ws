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

from typing import Tuple

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from std_msgs.msg import Float32, Int32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, MagneticField, JointState
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from Rosmaster_Lib import Rosmaster  # uses your GitHub dependency


def apply_rot90_xy(x: float, y: float, *, rot90: bool, reverse: bool) -> Tuple[float, float]:
    """Apply the documented Rosmaster<->ROS 90° XY mapping using only swap/sign.

    Mapping (rot90=True):
      - HW -> ROS: (x_ros, y_ros) = (-y_hw, x_hw)
      - ROS -> HW: (x_hw, y_hw) = (y_ros, -x_ros)

    Z is unaffected and handled by `apply_rot90_xyz`.
    """
    if not rot90:
        return x, y
    if reverse:
        return y, -x
    return -y, x


def apply_rot90_xyz(x: float, y: float, z: float, *, rot90: bool, reverse: bool) -> Tuple[float, float, float]:
    x_out, y_out = apply_rot90_xy(x, y, rot90=rot90, reverse=reverse)
    return x_out, y_out, z


class RosmasterAdapter:
    """Thin wrapper around Rosmaster library using your GitHub dependency."""

    def __init__(self, node: Node, port: str = '/dev/my_ros_board', debug: bool = False) -> None:
        self._node = node
        self._node.get_logger().info(f"[init] Creating Rosmaster(com={port}, debug={debug})")
        # Instantiate hardware using provided port and debug flag
        self._hw = Rosmaster(com=port, debug=debug)
        self._node.get_logger().info("[init] Rosmaster instance created")
        # Start background receive thread when available
        if hasattr(self._hw, 'create_receive_threading'):
            self._node.get_logger().info("[init] Starting receive thread")
            self._hw.create_receive_threading()
            self._node.get_logger().info("[init] Receive thread started")

    def set_motion(self, vx: float, vy: float, wz: float) -> None:
        self._hw.set_car_motion(vx, vy, wz)

    def get_version(self) -> float:
        return float(self._hw.get_version())

    def get_battery_voltage(self) -> float:
        return float(self._hw.get_battery_voltage())

    def get_accelerometer(self) -> Tuple[float, float, float]:
        ax, ay, az = self._hw.get_accelerometer_data()
        return float(ax), float(ay), float(az)

    def get_gyroscope(self) -> Tuple[float, float, float]:
        gx, gy, gz = self._hw.get_gyroscope_data()
        return float(gx), float(gy), float(gz)

    def get_magnetometer(self) -> Tuple[float, float, float]:
        mx, my, mz = self._hw.get_magnetometer_data()
        return float(mx), float(my), float(mz)

    def get_motion(self) -> Tuple[float, float, float]:
        vx, vy, wz = self._hw.get_motion_data()
        return float(vx), float(vy), float(wz)

    def get_motor_encoder(self) -> Tuple[int, int, int, int]:
        m1, m2, m3, m4 = self._hw.get_motor_encoder()
        return int(m1), int(m2), int(m3), int(m4)


class RosmasterDriverNode(Node):
    """ROS 2 node that bridges cmd_vel to Rosmaster board and publishes sensors/state."""

    def __init__(self) -> None:
        super().__init__('rosmaster_driver')

        # Parameters
        self.declare_parameter('imu_link', 'imu_link')
        self.declare_parameter('prefix', '')
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('port', '/dev/my_ros_board')
        self.declare_parameter('debug', False)
        self.declare_parameter('encoder_ticks_per_rev', 2464.0)
        self.declare_parameter('rot90', True)

        self.imu_link = self.get_parameter('imu_link').get_parameter_value().string_value
        self.prefix = self.get_parameter('prefix').get_parameter_value().string_value
        self.publish_rate = float(self.get_parameter('publish_rate').get_parameter_value().double_value)
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.debug = bool(self.get_parameter('debug').get_parameter_value().bool_value)
        self.ticks_per_rev = float(self.get_parameter('encoder_ticks_per_rev').get_parameter_value().double_value)
        self.rot90 = bool(self.get_parameter('rot90').get_parameter_value().bool_value)
        if self.publish_rate <= 0.0:
            self.publish_rate = 10.0
        if self.ticks_per_rev <= 0.0:
            self.ticks_per_rev = 2464.0

        # Hardware adapter
        self.get_logger().info("[init] Initializing hardware adapter ...")
        try:
            self.hw = RosmasterAdapter(self, port=self.port, debug=self.debug)
        except Exception as e:
            self.get_logger().error(f"[init] Failed to init Rosmaster hardware: {e}")
            raise
        self.get_logger().info("[init] Hardware adapter ready")

        # Interfaces
        # Use explicit QoS for subscriptions/publishers

        # For cmd_vel (control): reliable with small depth
        qos_cmd = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self.sub_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, qos_cmd)

        # For IMU/Mag (high-rate sensors): best-effort with small depth to reduce latency
        qos_imu = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.pub_edition = self.create_publisher(Float32, 'edition', qos_imu)
        self.pub_voltage = self.create_publisher(Float32, 'voltage', qos_imu)
        self.pub_joint_states = self.create_publisher(JointState, 'joint_states', qos_imu)
        self.pub_encoders = self.create_publisher(Int32MultiArray, 'encoders', qos_imu)
        self.pub_vel_raw = self.create_publisher(Twist, 'vel_raw', qos_imu)
        self.pub_imu_raw = self.create_publisher(Imu, '/imu/data_raw', qos_imu)
        self.pub_mag = self.create_publisher(MagneticField, '/imu/mag', qos_imu)
        self.get_logger().info("[init] Publishers and subscriptions created")

        # Timer
        period = max(1e-3, 1.0 / self.publish_rate)
        self.timer = self.create_timer(period, self._on_timer)
        self._tick_count = 0
        self.get_logger().info(f"[init] Timer created with period {period:.3f}s")

        self.get_logger().info(
            "rosmaster_driver started (imu_link=%s, prefix='%s', rate=%.1f Hz, port=%s, debug=%s, rot90=%s)"
            % (self.imu_link, self.prefix, self.publish_rate, self.port, self.debug, self.rot90)
        )

    # --- Callbacks ---
    def _on_cmd_vel(self, msg: Twist) -> None:
        vx_ros = float(msg.linear.x)
        vy_ros = float(msg.linear.y)
        wz_ros = float(msg.angular.z)
        vx_hw, vy_hw, wz_hw = -vy_ros, -vx_ros, wz_ros
        self.hw.set_motion(vx_hw, vy_hw, wz_hw)

    def _on_timer(self) -> None:
        now = Clock().now().to_msg()
        if self._tick_count == 0:
            self.get_logger().info("[timer] First timer callback")
        self._tick_count += 1

        # Read hardware
        edition_val = self.hw.get_version()
        voltage_val = self.hw.get_battery_voltage()
        ax, ay, az = self.hw.get_accelerometer()
        gx, gy, gz = self.hw.get_gyroscope()
        mx, my, mz = self.hw.get_magnetometer()
        vx, vy, wz = self.hw.get_motion()
        m1, m2, m3, m4 = self.hw.get_motor_encoder()

        # edition
        edition = Float32()
        edition.data = float(edition_val)
        self.pub_edition.publish(edition)

        # voltage
        voltage = Float32()
        voltage.data = float(voltage_val)
        self.pub_voltage.publish(voltage)

        # The IMU sensor is flipped on the board (Z axis down), so flip everything (rot X 180°)
        ax_imu, ay_imu, az_imu = ax, -ay, -az
        gx_imu, gy_imu, gz_imu = gx, -gy, -gz
        # Apply Rosmaster<->ROS 90° XY mapping to IMU vectors (HW -> ROS)
        ax_ros, ay_ros, az_ros = apply_rot90_xyz(ax_imu, ay_imu, az_imu, rot90=self.rot90, reverse=True)
        gx_ros, gy_ros, gz_ros = apply_rot90_xyz(gx_imu, gy_imu, gz_imu, rot90=self.rot90, reverse=True)

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = self.imu_link
        imu.linear_acceleration.x = float(ax_ros)
        imu.linear_acceleration.y = float(ay_ros)
        imu.linear_acceleration.z = float(az_ros)
        imu.angular_velocity.x = float(gx_ros)
        imu.angular_velocity.y = float(gy_ros)
        imu.angular_velocity.z = float(gz_ros)
        # Orientation is unknown; leave default zeros with covariance unset
        self.pub_imu_raw.publish(imu)

        # Prepare magnetometer in ROS axes and apply same 90° XY mapping
        mx_imu, my_imu, mz_imu = mx, -my, mz # mz ends up NOT flipped, because mag had its own Z flip already
        mx_ros, my_ros, mz_ros = apply_rot90_xyz(mx_imu, my_imu, mz_imu, rot90=self.rot90, reverse=True)
        # Magnetometer Z axis inverted on datasheet
        mag = MagneticField()
        mag.header.stamp = now
        mag.header.frame_id = self.imu_link
        mag.magnetic_field.x = float(mx_ros)
        mag.magnetic_field.y = float(my_ros)
        mag.magnetic_field.z = float(mz_ros)
        self.pub_mag.publish(mag)

        # vel_raw (feedback) tuned manually
        vx_ros, vy_ros, wz_ros = -vy, -vx, wz
        twist = Twist()
        twist.linear.x = float(vx_ros)
        twist.linear.y = float(vy_ros)
        twist.angular.z = float(wz_ros)
        self.pub_vel_raw.publish(twist)

        # encoders raw (debug)
        enc = Int32MultiArray()
        enc.data = [int(m1), int(m2), int(m3), int(m4)]
        self.pub_encoders.publish(enc)

        # joint states derived from encoders (wheel rotation only)
        joint_state = JointState()
        joint_state.header.stamp = now
        joint_state.header.frame_id = 'joint_states'
        names = [
            f"{self.prefix}front_left_joint",
            f"{self.prefix}front_right_joint",
            f"{self.prefix}back_left_joint",
            f"{self.prefix}back_right_joint",
        ]
        joint_state.name = names
        rev_to_rad = 2.0 * 3.141592653589793 / self.ticks_per_rev
        joint_state.position = [
            float(m1) * rev_to_rad * (-1.0),  # invert front_left_joint
            float(m2) * rev_to_rad,
            float(m3) * rev_to_rad,
            float(m4) * rev_to_rad * (-1.0),  # invert back_right_joint
        ]
        self.pub_joint_states.publish(joint_state)

def main(args=None) -> None:
    rclpy.init(args=args)
    node = RosmasterDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
