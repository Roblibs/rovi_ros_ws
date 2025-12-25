#!/usr/bin/env python3
"""Keyboard teleop publisher for rovi.

This publishes Twist commands on `cmd_vel_keyboard` which is one of the inputs to `twist_mux`.
Run this in a dedicated terminal so it can capture keyboard input.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    cmd_vel_keyboard_topic = DeclareLaunchArgument(
        'cmd_vel_keyboard_topic',
        default_value='cmd_vel_keyboard',
        description='Twist topic produced by teleop_twist_keyboard (twist_mux input).',
    )

    teleop_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        emulate_tty=True,
        remappings=[('cmd_vel', LaunchConfiguration('cmd_vel_keyboard_topic'))],
    )

    return LaunchDescription([
        cmd_vel_keyboard_topic,
        teleop_keyboard,
    ])

