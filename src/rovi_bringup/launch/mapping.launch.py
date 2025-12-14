#!/usr/bin/env python3
"""Bringup stack for online SLAM mapping with slam_toolbox."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution


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

    # If EKF publishes TF odom->base_footprint, disable TF broadcast from rovi_base to avoid duplicates.
    disable_rovi_base_tf = SetLaunchConfiguration(
        'rovi_base_publish_tf',
        'false',
        condition=IfCondition(LaunchConfiguration('ekf_enabled')),
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(teleop_launch),
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
            'slam_mode': TextSubstitution(text='mapping'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    return LaunchDescription([
        slam_enabled,
        ekf_enabled,
        imu_enabled,
        mag_enable,
        use_sim_time,
        disable_rovi_base_tf,
        teleop,
        localization,
        slam,
    ])

