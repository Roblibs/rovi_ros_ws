#!/usr/bin/env python3
"""Floor LUT calibration: writes ~/.ros/rovi/floor/*.png and exits."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false", description="Use /clock time.")
    lut_dir = DeclareLaunchArgument(
        "lut_dir",
        default_value=os.path.expanduser("~/.ros/rovi/floor"),
        description="Output directory for LUT PNGs.",
    )
    capture_duration_s = DeclareLaunchArgument(
        "capture_duration_s",
        default_value="10.0",
        description="Capture duration in seconds (steady-clock wall duration).",
    )
    generate_obstacle_thresholds = DeclareLaunchArgument(
        "generate_obstacle_thresholds",
        default_value="false",
        description="Generate obstacle threshold maps (t_obst1_mm.png, t_obst2_mm.png).",
    )

    node = Node(
        package="rovi_floor",
        executable="floor_calibrate_node",
        name="rovi_floor_calibrate",
        output="screen",
        parameters=[
            {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)},
            {"lut_dir": LaunchConfiguration("lut_dir")},
            {"capture_duration_s": ParameterValue(LaunchConfiguration("capture_duration_s"), value_type=float)},
            {
                "generate_obstacle_thresholds": ParameterValue(
                    LaunchConfiguration("generate_obstacle_thresholds"), value_type=bool
                )
            },
        ],
    )

    return LaunchDescription([
        use_sim_time,
        lut_dir,
        capture_duration_s,
        generate_obstacle_thresholds,
        node,
    ])

