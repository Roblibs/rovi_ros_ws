#!/usr/bin/env python3
"""Floor runtime: depth-floor diff -> /floor/mask (+ optional /floor/topology)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false", description="Use /clock time.")
    robot_mode = DeclareLaunchArgument(
        "robot_mode",
        default_value="real",
        description="Robot backend string for LUT meta/signature (real|sim).",
    )
    camera_topology_enabled = DeclareLaunchArgument(
        "camera_topology_enabled",
        default_value="false",
        description="Enable topology visualization (/floor/topology).",
    )
    node = Node(
        package="rovi_floor",
        executable="floor_runtime_node",
        name="rovi_floor_runtime",
        output="screen",
        parameters=[
            {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)},
            {"robot_mode": LaunchConfiguration("robot_mode")},
            {"camera_topology_enabled": ParameterValue(LaunchConfiguration("camera_topology_enabled"), value_type=bool)},
        ],
    )

    return LaunchDescription([
        use_sim_time,
        robot_mode,
        camera_topology_enabled,
        node,
    ])
