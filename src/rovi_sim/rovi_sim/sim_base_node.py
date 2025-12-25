#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile


def _clamp(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


class RoviSimBase(Node):
    """Simple velocity smoother for simulation.

    - Subscribes: /cmd_vel (Twist)
    - Publishes:  /cmd_vel_sim (Twist) to drive Gazebo
    - Publishes:  /vel_raw (Twist) as a "measured" velocity for rovi_base
    """

    def __init__(self) -> None:
        super().__init__('rovi_sim_base')

        self.declare_parameter('cmd_vel_in', 'cmd_vel')
        self.declare_parameter('cmd_vel_out', 'cmd_vel_sim')
        self.declare_parameter('vel_raw_out', 'vel_raw')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('cmd_timeout', 0.5)

        self.declare_parameter('max_accel_x', 1.0)      # m/s^2
        self.declare_parameter('max_accel_y', 1.0)      # m/s^2
        self.declare_parameter('max_accel_yaw', 3.0)    # rad/s^2

        self.declare_parameter('max_vel_x', 0.6)        # m/s
        self.declare_parameter('max_vel_y', 0.4)        # m/s
        self.declare_parameter('max_vel_yaw', 2.5)      # rad/s

        self._cmd_vel_in = str(self.get_parameter('cmd_vel_in').value)
        self._cmd_vel_out = str(self.get_parameter('cmd_vel_out').value)
        self._vel_raw_out = str(self.get_parameter('vel_raw_out').value)

        publish_rate = float(self.get_parameter('publish_rate').value)
        self._cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        self._max_accel_x = float(self.get_parameter('max_accel_x').value)
        self._max_accel_y = float(self.get_parameter('max_accel_y').value)
        self._max_accel_yaw = float(self.get_parameter('max_accel_yaw').value)

        self._max_vel_x = float(self.get_parameter('max_vel_x').value)
        self._max_vel_y = float(self.get_parameter('max_vel_y').value)
        self._max_vel_yaw = float(self.get_parameter('max_vel_yaw').value)

        if publish_rate <= 0.0:
            publish_rate = 50.0
        self._period_sec = 1.0 / publish_rate

        qos = QoSProfile(depth=10)
        self._pub_cmd = self.create_publisher(Twist, self._cmd_vel_out, qos)
        self._pub_vel_raw = self.create_publisher(Twist, self._vel_raw_out, qos)
        self._sub_cmd = self.create_subscription(Twist, self._cmd_vel_in, self._on_cmd, qos)

        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._last_update = self._steady_clock.now()
        self._last_cmd = self._steady_clock.now()

        self._target_vx = 0.0
        self._target_vy = 0.0
        self._target_wz = 0.0

        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0

        self._timer = self.create_timer(self._period_sec, self._tick, clock=self._steady_clock)

    def _on_cmd(self, msg: Twist) -> None:
        self._last_cmd = self._steady_clock.now()

        vx = _clamp(float(msg.linear.x), -self._max_vel_x, self._max_vel_x)
        vy = _clamp(float(msg.linear.y), -self._max_vel_y, self._max_vel_y)
        wz = _clamp(float(msg.angular.z), -self._max_vel_yaw, self._max_vel_yaw)

        self._target_vx = vx
        self._target_vy = vy
        self._target_wz = wz

    def _tick(self) -> None:
        now = self._steady_clock.now()
        dt = (now - self._last_update).nanoseconds * 1e-9
        self._last_update = now

        if dt <= 0.0 or math.isinf(dt) or math.isnan(dt):
            dt = self._period_sec

        if (now - self._last_cmd).nanoseconds * 1e-9 > self._cmd_timeout:
            self._target_vx = 0.0
            self._target_vy = 0.0
            self._target_wz = 0.0

        self._vx += _clamp(self._target_vx - self._vx, -self._max_accel_x * dt, self._max_accel_x * dt)
        self._vy += _clamp(self._target_vy - self._vy, -self._max_accel_y * dt, self._max_accel_y * dt)
        self._wz += _clamp(self._target_wz - self._wz, -self._max_accel_yaw * dt, self._max_accel_yaw * dt)

        out = Twist()
        out.linear.x = float(self._vx)
        out.linear.y = float(self._vy)
        out.angular.z = float(self._wz)

        self._pub_cmd.publish(out)
        self._pub_vel_raw.publish(out)


def main() -> None:
    rclpy.init()
    node = RoviSimBase()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

