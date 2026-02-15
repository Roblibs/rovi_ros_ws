#!/usr/bin/env python3
"""Reusable block: state estimation for higher-level stacks.

This block owns localization-side estimation nodes (EKF + optional IMU filter) and is backend-agnostic.
It assumes the robot backend provides `/vel_raw` and optionally `/imu/data_raw`.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from rovi_bringup.launch_lib.includes import include_launch


def generate_launch_description() -> LaunchDescription:
    loc_share = get_package_share_directory('rovi_localization')
    ekf_launch = os.path.join(loc_share, 'launch', 'ekf.launch.py')

    odom_mode = DeclareLaunchArgument(
        'odom_mode',
        default_value='filtered',
        description="Odometry mode: 'raw', 'filtered', or 'fusion_wheels_imu'.",
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

    state_estimation = include_launch(
        ekf_launch,
        launch_arguments={
            'odom_mode': LaunchConfiguration('odom_mode'),
            'mag_enabled': LaunchConfiguration('mag_enabled'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        },
    )

    return LaunchDescription([
        odom_mode,
        mag_enabled,
        use_sim_time,
        state_estimation,
    ])

