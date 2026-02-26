#!/usr/bin/env python3
"""Reusable include point for perception blocks.

This launch is intentionally empty by default. It exists so stacks can include a single
"perception slot" (mapping/localization/nav) without duplicating wiring.

Policy:
- This file should not start camera drivers; only perception nodes that consume existing topics.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from rovi_bringup.launch_lib.includes import include_launch


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("rovi_bringup")
    floor_runtime_launch = os.path.join(bringup_share, "launch", "floor_runtime.launch.py")

    robot_mode = DeclareLaunchArgument(
        "robot_mode",
        default_value="real",
        description="Robot backend string for perception blocks (real|sim|offline).",
    )
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false", description="Use /clock time.")
    camera_enabled = DeclareLaunchArgument(
        "camera_enabled",
        default_value="true",
        description="Enable perception blocks that depend on the camera (must degrade gracefully if the camera is missing).",
    )
    camera_topology_enabled = DeclareLaunchArgument(
        "camera_topology_enabled",
        default_value="false",
        description="Enable topology visualization (/floor/topology) in camera perception blocks.",
    )

    floor_runtime = include_launch(
        floor_runtime_launch,
        condition=IfCondition(LaunchConfiguration("camera_enabled")),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "camera_topology_enabled": LaunchConfiguration("camera_topology_enabled"),
        },
    )

    return LaunchDescription([
        robot_mode,
        use_sim_time,
        camera_enabled,
        camera_topology_enabled,
        floor_runtime,
    ])
