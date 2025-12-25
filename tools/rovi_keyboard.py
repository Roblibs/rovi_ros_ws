#!/usr/bin/env python3

import argparse
import os
import sys


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Run teleop_twist_keyboard in the current terminal and publish to cmd_vel_keyboard "
            "(twist_mux input)."
        )
    )
    parser.add_argument(
        '--topic',
        default='cmd_vel_keyboard',
        help='Output Twist topic name (default: cmd_vel_keyboard).',
    )
    args, passthrough = parser.parse_known_args()

    if not sys.stdin.isatty():
        print(
            "[rovi_keyboard] ERROR: teleop_twist_keyboard needs a real terminal (TTY). "
            "Run this from an interactive shell (not from ros2 launch).",
            file=sys.stderr,
        )
        return 2

    cmd = [
        'ros2',
        'run',
        'teleop_twist_keyboard',
        'teleop_twist_keyboard',
        *passthrough,
        '--ros-args',
        '-r',
        f'cmd_vel:={args.topic}',
    ]

    os.execvp(cmd[0], cmd)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

