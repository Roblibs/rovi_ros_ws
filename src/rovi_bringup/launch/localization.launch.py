#!/usr/bin/env python3
"""Localization stack: state estimation + slam_toolbox in localization mode.

This launch is backend-agnostic. It assumes the robot backend is already running and provides:
- /scan
- /vel_raw
- /imu/data_raw (when using odom_mode=fusion_wheels_imu)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from rovi_bringup.launch_lib.includes import include_launch


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory('rovi_bringup')
    camera_launch = os.path.join(bringup_share, 'launch', 'camera.launch.py')
    state_estimation_launch = os.path.join(bringup_share, 'launch', 'state_estimation.launch.py')
    slam_mode_launch = os.path.join(bringup_share, 'launch', 'slam_mode.launch.py')
    perception_launch = os.path.join(bringup_share, 'launch', 'perception.launch.py')

    robot_mode = DeclareLaunchArgument(
        'robot_mode',
        default_value='real',
        description="Robot backend: 'real', 'sim', or 'offline'.",
    )
    slam_enabled = DeclareLaunchArgument(
        'slam_enabled',
        default_value='true',
        description='Start slam_toolbox and publish TF map->odom.',
    )
    camera_enabled = DeclareLaunchArgument(
        'camera_enabled',
        default_value='true',
        description='Enable optional camera pipeline (drivers + floor perception).',
    )
    camera_topology_enabled = DeclareLaunchArgument(
        'camera_topology_enabled',
        default_value='false',
        description='Enable floor topology visualization (/floor/topology).',
    )
    odom_mode = DeclareLaunchArgument(
        'odom_mode',
        default_value='filtered',
        description="Odometry mode: 'raw' (rovi_odom_integrator), 'filtered' (EKF wheel-only), 'fusion_wheels_imu' (EKF + IMU).",
    )
    mag_enabled = DeclareLaunchArgument(
        'mag_enabled',
        default_value='false',
        description='Enable magnetometer usage in IMU orientation filter.',
    )
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use /clock time.',
    )
    map_file_name = DeclareLaunchArgument(
        'map_file_name',
        default_value=os.path.expanduser('~/.ros/rovi/maps/latest.posegraph'),
        description='Pose-graph file to load (slam_toolbox param map_file_name, typically .posegraph).',
    )

    camera = include_launch(
        camera_launch,
        condition=IfCondition(LaunchConfiguration('camera_enabled')),
        launch_arguments={
            'robot_mode': LaunchConfiguration('robot_mode'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        },
    )

    state_estimation = include_launch(
        state_estimation_launch,
        launch_arguments={
            'odom_mode': LaunchConfiguration('odom_mode'),
            'mag_enabled': LaunchConfiguration('mag_enabled'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        },
    )

    slam = include_launch(
        slam_mode_launch,
        launch_arguments={
            'slam_enabled': LaunchConfiguration('slam_enabled'),
            'slam_mode': TextSubstitution(text='localization'),
            'map_file_name': LaunchConfiguration('map_file_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        },
    )

    perception = include_launch(
        perception_launch,
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'camera_enabled': LaunchConfiguration('camera_enabled'),
            'camera_topology_enabled': LaunchConfiguration('camera_topology_enabled'),
        },
    )

    return LaunchDescription([
        robot_mode,
        slam_enabled,
        camera_enabled,
        camera_topology_enabled,
        odom_mode,
        mag_enabled,
        use_sim_time,
        map_file_name,
        camera,
        state_estimation,
        slam,
        perception,
    ])
