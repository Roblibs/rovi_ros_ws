#!/usr/bin/env python3
"""Bringup stack for slam_toolbox localization on an existing map."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory('rovi_bringup')
    teleop_launch = os.path.join(bringup_share, 'launch', 'teleop.launch.py')

    loc_share = get_package_share_directory('rovi_localization')
    ekf_launch = os.path.join(loc_share, 'launch', 'ekf.launch.py')

    slam_share = get_package_share_directory('rovi_slam')
    slam_launch = os.path.join(slam_share, 'launch', 'slam_toolbox.launch.py')

    slam_enabled = DeclareLaunchArgument(
        'slam_enabled',
        default_value='true',
        description='Start slam_toolbox and publish TF map->odom.',
    )
    ekf_enabled = DeclareLaunchArgument(
        'ekf_enabled',
        default_value='false',
        description='Start robot_localization EKF and publish TF odom->base_footprint.',
    )
    imu_enabled = DeclareLaunchArgument(
        'imu_enabled',
        default_value='false',
        description='Start IMU orientation filter and fuse IMU into EKF.',
    )
    mag_enable = DeclareLaunchArgument(
        'mag_enable',
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
        default_value='',
        description='Pose-graph file to load (slam_toolbox param map_file_name, typically .posegraph).',
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(teleop_launch),
        launch_arguments={
            'rovi_base_publish_tf': PythonExpression([
                "'false' if '", LaunchConfiguration('ekf_enabled'), "' == 'true' else 'true'",
            ]),
        }.items(),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ekf_launch),
        launch_arguments={
            'ekf_enabled': LaunchConfiguration('ekf_enabled'),
            'imu_enabled': LaunchConfiguration('imu_enabled'),
            'mag_enable': LaunchConfiguration('mag_enable'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        launch_arguments={
            'slam_enabled': LaunchConfiguration('slam_enabled'),
            'slam_mode': TextSubstitution(text='localization'),
            'map_file_name': LaunchConfiguration('map_file_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    return LaunchDescription([
        slam_enabled,
        ekf_enabled,
        imu_enabled,
        mag_enable,
        use_sim_time,
        map_file_name,
        teleop,
        localization,
        slam,
    ])
