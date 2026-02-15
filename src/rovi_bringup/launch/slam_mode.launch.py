#!/usr/bin/env python3
"""Reusable block: slam_toolbox wiring for mapping/localization/nav stacks."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from rovi_bringup.launch_lib.includes import include_launch


def generate_launch_description() -> LaunchDescription:
    slam_share = get_package_share_directory('rovi_slam')
    slam_launch = os.path.join(slam_share, 'launch', 'slam_toolbox.launch.py')

    slam_enabled = DeclareLaunchArgument(
        'slam_enabled',
        default_value='true',
        description='Start slam_toolbox and publish TF map->odom.',
    )
    slam_mode = DeclareLaunchArgument(
        'slam_mode',
        default_value='mapping',
        description="SLAM mode: 'mapping' (build/update map) or 'localization' (localize against a saved pose-graph).",
    )
    map_file_name = DeclareLaunchArgument(
        'map_file_name',
        default_value=os.path.expanduser('~/.ros/rovi/maps/latest.posegraph'),
        description='Pose-graph file to load in localization mode.',
    )
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use /clock time.',
    )

    slam = include_launch(
        slam_launch,
        launch_arguments={
            'slam_enabled': LaunchConfiguration('slam_enabled'),
            'slam_mode': LaunchConfiguration('slam_mode'),
            'map_file_name': LaunchConfiguration('map_file_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        },
    )

    return LaunchDescription([
        slam_enabled,
        slam_mode,
        map_file_name,
        use_sim_time,
        slam,
    ])

