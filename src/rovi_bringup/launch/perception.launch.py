#!/usr/bin/env python3
"""Reusable include point for perception blocks.

This launch is intentionally empty by default. It exists so stacks can include a single
"perception slot" (mapping/localization/nav) without duplicating wiring.

Policy:
- This file should not start camera drivers; only perception nodes that consume existing topics.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument


def generate_launch_description() -> LaunchDescription:
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use /clock time.')
    return LaunchDescription([use_sim_time])

