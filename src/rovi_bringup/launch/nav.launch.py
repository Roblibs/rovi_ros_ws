#!/usr/bin/env python3
"""Navigation stack: slam_toolbox (mapping or localization) + Nav2.

This launch is backend-agnostic. It assumes the robot backend is already running and provides:
- /scan
- /vel_raw
- /imu/data_raw (when using odom_mode=fusion_wheels_imu)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory('rovi_bringup')
    mapping_launch = os.path.join(bringup_share, 'launch', 'mapping.launch.py')
    localization_launch = os.path.join(bringup_share, 'launch', 'localization.launch.py')

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
        default_value='fusion_wheels_imu',
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

    mapping_selected = IfCondition(PythonExpression(["'", LaunchConfiguration('slam_mode'), "' == 'mapping'"]))
    localization_selected = IfCondition(PythonExpression(["'", LaunchConfiguration('slam_mode'), "' == 'localization'"]))

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mapping_launch),
        condition=mapping_selected,
        launch_arguments={
            'odom_mode': LaunchConfiguration('odom_mode'),
            'mag_enabled': LaunchConfiguration('mag_enabled'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
        condition=localization_selected,
        launch_arguments={
            'odom_mode': LaunchConfiguration('odom_mode'),
            'mag_enabled': LaunchConfiguration('mag_enabled'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map_file_name': LaunchConfiguration('map_file_name'),
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_launch),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    return LaunchDescription([
        slam_mode,
        map_file_name,
        odom_mode,
        mag_enabled,
        use_sim_time,
        mapping,
        localization,
        nav2,
    ])
