#!/usr/bin/env python3
"""Mapping stack: localization (EKF) + slam_toolbox in mapping mode.

This launch is backend-agnostic. It assumes the robot backend is already running and provides:
- /scan
- /vel_raw
- /imu/data_raw (when using odom_mode=fusion_wheels_imu)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from rovi_bringup.launch_lib.includes import include_launch


def generate_launch_description() -> LaunchDescription:
    loc_share = get_package_share_directory('rovi_localization')
    ekf_launch = os.path.join(loc_share, 'launch', 'ekf.launch.py')

    slam_share = get_package_share_directory('rovi_slam')
    slam_launch = os.path.join(slam_share, 'launch', 'slam_toolbox.launch.py')

    slam_enabled = DeclareLaunchArgument(
        'slam_enabled',
        default_value='true',
        description='Start slam_toolbox and publish TF map->odom.',
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

    localization = include_launch(
        ekf_launch,
        launch_arguments={
            'odom_mode': LaunchConfiguration('odom_mode'),
            'mag_enabled': LaunchConfiguration('mag_enabled'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        },
    )

    slam = include_launch(
        slam_launch,
        launch_arguments={
            'slam_enabled': LaunchConfiguration('slam_enabled'),
            'slam_mode': TextSubstitution(text='mapping'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        },
    )

    return LaunchDescription([
        slam_enabled,
        odom_mode,
        mag_enabled,
        use_sim_time,
        localization,
        slam,
    ])
