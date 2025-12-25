#!/usr/bin/env python3
"""Bringup stack for Nav2 navigation with slam_toolbox + rovi base.

This wraps existing bringup launches:
- slam_mode=mapping: include mapping.launch.py (teleop + EKF + slam_toolbox mapping)
- slam_mode=localization: include localization.launch.py (teleop + EKF + slam_toolbox localization)

Then it starts the Nav2 navigation stack from rovi_nav.
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
    sim_share = get_package_share_directory('rovi_sim')
    default_world = os.path.join(sim_share, 'worlds', 'rovi_room.sdf')

    nav_share = get_package_share_directory('rovi_nav')
    nav_launch = os.path.join(nav_share, 'launch', 'nav.launch.py')

    robot_mode = DeclareLaunchArgument(
        'robot_mode',
        default_value='real',
        description="Robot backend: 'real', 'sim', or 'offline'.",
    )
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
        default_value=PythonExpression([
            "'true' if '",
            LaunchConfiguration('robot_mode'),
            "' == 'sim' else 'false'",
        ]),
        description='Use /clock time.',
    )
    world = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Full path to the Gazebo world SDF file (robot_mode=sim).',
    )
    gazebo_gui = DeclareLaunchArgument(
        'gazebo_gui',
        default_value='true',
        description='Start Gazebo GUI client (server always starts) (robot_mode=sim).',
    )

    mapping_selected = IfCondition(PythonExpression(["'", LaunchConfiguration('slam_mode'), "' == 'mapping'"]))
    localization_selected = IfCondition(PythonExpression(["'", LaunchConfiguration('slam_mode'), "' == 'localization'"]))

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mapping_launch),
        condition=mapping_selected,
        launch_arguments={
            'robot_mode': LaunchConfiguration('robot_mode'),
            'odom_mode': LaunchConfiguration('odom_mode'),
            'mag_enabled': LaunchConfiguration('mag_enabled'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'world': LaunchConfiguration('world'),
            'gazebo_gui': LaunchConfiguration('gazebo_gui'),
        }.items(),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
        condition=localization_selected,
        launch_arguments={
            'robot_mode': LaunchConfiguration('robot_mode'),
            'odom_mode': LaunchConfiguration('odom_mode'),
            'mag_enabled': LaunchConfiguration('mag_enabled'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map_file_name': LaunchConfiguration('map_file_name'),
            'world': LaunchConfiguration('world'),
            'gazebo_gui': LaunchConfiguration('gazebo_gui'),
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_launch),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    return LaunchDescription([
        robot_mode,
        slam_mode,
        map_file_name,
        odom_mode,
        mag_enabled,
        use_sim_time,
        world,
        gazebo_gui,
        mapping,
        localization,
        nav2,
    ])
