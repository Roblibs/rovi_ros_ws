#!/usr/bin/env python3
"""Navigation stack: state estimation + slam_toolbox (mapping or localization) + Nav2.

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
    bringup_share = get_package_share_directory('rovi_bringup')
    state_estimation_launch = os.path.join(bringup_share, 'launch', 'state_estimation.launch.py')
    slam_mode_launch = os.path.join(bringup_share, 'launch', 'slam_mode.launch.py')
    perception_launch = os.path.join(bringup_share, 'launch', 'perception.launch.py')

    nav_share = get_package_share_directory('rovi_nav')
    nav_launch = os.path.join(nav_share, 'launch', 'nav.launch.py')

    slam_mode = DeclareLaunchArgument(
        'slam_mode',
        default_value='mapping',
        description="SLAM mode: 'mapping' (build/update map) or 'localization' (localize against a saved pose-graph).",
    )
    map_file_name = DeclareLaunchArgument(
        'map_file_name',
        default_value=os.path.expanduser('~/.ros/rovi/maps/latest.posegraph'),
        description='Pose-graph file to load when slam_mode=localization.',
    )
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
            'slam_enabled': TextSubstitution(text='true'),
            'slam_mode': LaunchConfiguration('slam_mode'),
            'map_file_name': LaunchConfiguration('map_file_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        },
    )

    nav2 = include_launch(
        nav_launch,
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        },
    )

    perception = include_launch(
        perception_launch,
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        },
    )

    return LaunchDescription([
        slam_mode,
        map_file_name,
        odom_mode,
        mag_enabled,
        use_sim_time,
        state_estimation,
        slam,
        nav2,
        perception,
    ])
