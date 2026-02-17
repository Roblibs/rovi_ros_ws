#!/usr/bin/env python3
"""One-shot floor LUT calibration in simulation (bringup-owned composition).

Starts the sim backend (Gazebo + TF via robot_state_publisher) and runs floor calibration once,
then shuts the launch down when calibration exits.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("rovi_bringup")
    robot_bringup_launch = os.path.join(bringup_share, "launch", "robot_bringup.launch.py")
    sim_share = get_package_share_directory("rovi_sim")
    default_calib_world = os.path.join(sim_share, "worlds", "rovi_floor_only.sdf")

    gazebo_gui = DeclareLaunchArgument("gazebo_gui", default_value="false", description="Start Gazebo GUI client.")
    world = DeclareLaunchArgument(
        "world",
        default_value=default_calib_world,
        description="Full path to the Gazebo world SDF file (calibration defaults to a floor-only world).",
    )
    capture_duration_s = DeclareLaunchArgument(
        "capture_duration_s",
        default_value="2.0",
        description="Capture duration in seconds (steady-clock wall duration).",
    )
    max_frames = DeclareLaunchArgument(
        "max_frames",
        default_value="5",
        description="Stop capture once this many depth frames are collected.",
    )
    min_valid_samples = DeclareLaunchArgument(
        "min_valid_samples",
        default_value="1",
        description="Minimum non-zero samples per pixel needed to accept a depth value (sim can use 1).",
    )
    generate_obstacle_thresholds = DeclareLaunchArgument(
        "generate_obstacle_thresholds",
        default_value="false",
        description="Generate obstacle threshold maps (t_obst1_mm.png, t_obst2_mm.png).",
    )

    backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_bringup_launch),
        launch_arguments={
            "robot_mode": "sim",
            "use_sim_time": "true",
            "gazebo_gui": LaunchConfiguration("gazebo_gui"),
            "world": LaunchConfiguration("world"),
        }.items(),
    )

    calibrate = Node(
        package="rovi_floor",
        executable="floor_calibrate_node",
        name="rovi_floor_calibrate",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"robot_mode": "sim"},
            {"capture_duration_s": ParameterValue(LaunchConfiguration("capture_duration_s"), value_type=float)},
            {"max_frames": ParameterValue(LaunchConfiguration("max_frames"), value_type=int)},
            {"min_valid_samples": ParameterValue(LaunchConfiguration("min_valid_samples"), value_type=int)},
            {
                "generate_obstacle_thresholds": ParameterValue(
                    LaunchConfiguration("generate_obstacle_thresholds"), value_type=bool
                )
            },
        ],
    )

    delayed_calibrate = TimerAction(period=3.0, actions=[calibrate])

    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=calibrate,
            on_exit=[EmitEvent(event=Shutdown(reason="floor calibration finished"))],
        )
    )

    return LaunchDescription([
        gazebo_gui,
        world,
        capture_duration_s,
        max_frames,
        min_valid_samples,
        generate_obstacle_thresholds,
        backend,
        delayed_calibrate,
        shutdown_on_exit,
    ])
