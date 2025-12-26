#!/usr/bin/env python3

import argparse
import os
import sys
import termios
import time
import tty
from select import select

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile


def _clamp(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


def _read_key(timeout_sec: float) -> str | None:
    ready, _, _ = select([sys.stdin], [], [], timeout_sec)
    if not ready:
        return None

    ch1 = sys.stdin.read(1)
    if ch1 != '\x1b':
        return ch1

    # Try to read an ANSI escape sequence (e.g. arrows: ESC [ A/B/C/D).
    if not select([sys.stdin], [], [], 0.01)[0]:
        return ch1

    ch2 = sys.stdin.read(1)
    if ch2 != '[':
        return ch1 + ch2

    if not select([sys.stdin], [], [], 0.01)[0]:
        return ch1 + ch2

    ch3 = sys.stdin.read(1)
    return f'{ch1}{ch2}{ch3}'


def _print_help(
    *,
    topic: str,
    linear_max: float,
    angular_max: float,
    scale: float,
    scale_min: float,
    scale_max: float,
    scale_step: float,
) -> None:
    print("")
    print("[rovi_keyboard] Controls (holonomic)")
    print("  i : forward (+X)")
    print("  k : backward (-X)")
    print("  u : left (+Y)")
    print("  o : right (-Y)")
    print("  j : turn left (+Yaw)")
    print("  l : turn right (-Yaw)")
    print("  . / SPACE : stop")
    print("")
    print("  ↑ / ↓ : increase / decrease max speed")
    print("  q : quit")
    print("")
    print(f"[rovi_keyboard] Publishing: {topic} (geometry_msgs/Twist)")
    print(
        f"[rovi_keyboard] Max speeds: linear={linear_max:.3f} m/s, angular={angular_max:.3f} rad/s, "
        f"scale={scale:.2f} (min={scale_min:.2f}, max={scale_max:.2f}, step={scale_step:.2f})"
    )
    print("")


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Keyboard teleop for Rovi (holonomic). Publishes Twist to cmd_vel_keyboard (twist_mux input)."
        )
    )
    parser.add_argument(
        '--topic',
        default='cmd_vel_keyboard',
        help='Output Twist topic name (default: cmd_vel_keyboard).',
    )
    parser.add_argument(
        '--linear',
        type=float,
        default=0.30,
        help='Base max linear speed in m/s for X/Y (default: 0.30).',
    )
    parser.add_argument(
        '--angular',
        type=float,
        default=1.20,
        help='Base max angular speed in rad/s for yaw (default: 1.20).',
    )
    parser.add_argument(
        '--publish-rate',
        type=float,
        default=20.0,
        help='Publish rate in Hz (default: 20).',
    )
    parser.add_argument(
        '--key-timeout',
        type=float,
        default=0.7,
        help='Seconds without keypress before sending stop (default: 0.7).',
    )
    parser.add_argument(
        '--scale',
        type=float,
        default=1.0,
        help='Initial max-speed multiplier (default: 1.0).',
    )
    parser.add_argument(
        '--scale-step',
        type=float,
        default=0.1,
        help='Scale increment per arrow key press (default: 0.1).',
    )
    parser.add_argument(
        '--scale-min',
        type=float,
        default=0.1,
        help='Minimum scale (default: 0.1).',
    )
    parser.add_argument(
        '--scale-max',
        type=float,
        default=3.0,
        help='Maximum scale (default: 3.0).',
    )
    args = parser.parse_args()

    if not sys.stdin.isatty():
        print(
            "[rovi_keyboard] ERROR: needs a real terminal (TTY). Run this from an interactive shell.",
            file=sys.stderr,
        )
        return 2

    linear_max = float(args.linear)
    angular_max = float(args.angular)
    publish_rate_hz = float(args.publish_rate)
    if publish_rate_hz <= 0.0:
        publish_rate_hz = 20.0
    period_sec = 1.0 / publish_rate_hz

    scale = _clamp(float(args.scale), float(args.scale_min), float(args.scale_max))
    scale_step = max(0.01, float(args.scale_step))
    scale_min = float(args.scale_min)
    scale_max = float(args.scale_max)
    key_timeout = max(0.0, float(args.key_timeout))

    rclpy.init()
    node = Node('rovi_keyboard')
    pub = node.create_publisher(Twist, args.topic, QoSProfile(depth=10))

    _print_help(
        topic=args.topic,
        linear_max=linear_max,
        angular_max=angular_max,
        scale=scale,
        scale_min=scale_min,
        scale_max=scale_max,
        scale_step=scale_step,
    )

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        last_key_time = time.monotonic()

        direction_x = 0.0
        direction_y = 0.0
        direction_yaw = 0.0

        while rclpy.ok():
            key = _read_key(period_sec)
            now = time.monotonic()

            if key is not None:
                last_key_time = now

                if key == 'q':
                    break

                # Movement keys (Rovi requested layout).
                if key == 'i':  # forward
                    direction_x, direction_y, direction_yaw = 1.0, 0.0, 0.0
                elif key == 'k':  # backward
                    direction_x, direction_y, direction_yaw = -1.0, 0.0, 0.0
                elif key == 'u':  # left (ROS +Y)
                    direction_x, direction_y, direction_yaw = 0.0, 1.0, 0.0
                elif key == 'o':  # right (ROS -Y)
                    direction_x, direction_y, direction_yaw = 0.0, -1.0, 0.0
                elif key == 'j':  # turn left (CCW, +Z)
                    direction_x, direction_y, direction_yaw = 0.0, 0.0, 1.0
                elif key == 'l':  # turn right (CW, -Z)
                    direction_x, direction_y, direction_yaw = 0.0, 0.0, -1.0
                elif key in ('.', ' '):  # stop
                    direction_x, direction_y, direction_yaw = 0.0, 0.0, 0.0

                # Arrow keys: adjust max-speed scale.
                elif key == '\x1b[A':  # up
                    scale = _clamp(scale + scale_step, scale_min, scale_max)
                    print(
                        f"\r[rovi_keyboard] scale={scale:.2f}  "
                        f"linear={linear_max * scale:.3f} m/s  angular={angular_max * scale:.3f} rad/s   ",
                        end='',
                        flush=True,
                    )
                elif key == '\x1b[B':  # down
                    scale = _clamp(scale - scale_step, scale_min, scale_max)
                    print(
                        f"\r[rovi_keyboard] scale={scale:.2f}  "
                        f"linear={linear_max * scale:.3f} m/s  angular={angular_max * scale:.3f} rad/s   ",
                        end='',
                        flush=True,
                    )

            # Safety: if key repeat stops, send stop after timeout.
            if key_timeout > 0.0 and (now - last_key_time) > key_timeout:
                direction_x, direction_y, direction_yaw = 0.0, 0.0, 0.0

            msg = Twist()
            msg.linear.x = float(direction_x * linear_max * scale)
            msg.linear.y = float(direction_y * linear_max * scale)
            msg.angular.z = float(direction_yaw * angular_max * scale)
            pub.publish(msg)

            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
