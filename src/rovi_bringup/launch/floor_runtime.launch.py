#!/usr/bin/env python3
"""Floor runtime: depth-floor diff -> /floor/mask (+ optional /floor/topology)."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false", description="Use /clock time.")
    camera_topology_enabled = DeclareLaunchArgument(
        "camera_topology_enabled",
        default_value="false",
        description="Enable topology visualization (/floor/topology).",
    )
    lut_dir = DeclareLaunchArgument(
        "lut_dir",
        default_value=os.path.expanduser("~/.ros/rovi/floor"),
        description="Directory containing floor LUT PNGs (floor_mm.png, t_floor_mm.png, ...).",
    )

    node = Node(
        package="rovi_floor",
        executable="floor_runtime_node",
        name="rovi_floor_runtime",
        output="screen",
        parameters=[
            {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)},
            {"camera_topology_enabled": ParameterValue(LaunchConfiguration("camera_topology_enabled"), value_type=bool)},
            {"lut_dir": LaunchConfiguration("lut_dir")},
        ],
    )

    return LaunchDescription([
        use_sim_time,
        camera_topology_enabled,
        lut_dir,
        node,
    ])

