#!/usr/bin/env python3
"""Bringup stack for online SLAM mapping with slam_toolbox."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory('rovi_bringup')
    teleop_launch = os.path.join(bringup_share, 'launch', 'teleop.launch.py')
    sim_share = get_package_share_directory('rovi_sim')
    default_world = os.path.join(sim_share, 'worlds', 'rovi_room.sdf')
    desc_share = get_package_share_directory('rovi_description')
    default_rviz = os.path.join(desc_share, 'rviz', 'rovi_map.rviz')

    loc_share = get_package_share_directory('rovi_localization')
    ekf_launch = os.path.join(loc_share, 'launch', 'ekf.launch.py')

    slam_share = get_package_share_directory('rovi_slam')
    slam_launch = os.path.join(slam_share, 'launch', 'slam_toolbox.launch.py')

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
    odom_mode = DeclareLaunchArgument(
        'odom_mode',
        default_value='fusion_wheels_imu',
        description="Odometry mode: 'raw' (rovi_base), 'filtered' (EKF wheel-only), 'fusion_wheels_imu' (EKF + IMU).",
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
    rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz (mapping view).',
    )
    rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz,
        description='Absolute path to an RViz config file.',
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(teleop_launch),
        launch_arguments={
            'robot_mode': LaunchConfiguration('robot_mode'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'world': LaunchConfiguration('world'),
            'gazebo_gui': LaunchConfiguration('gazebo_gui'),
            'rviz': TextSubstitution(text='false'),
            'rovi_base_publish_tf': PythonExpression([
                "'true' if '", LaunchConfiguration('odom_mode'), "' == 'raw' else 'false'",
            ]),
        }.items(),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ekf_launch),
        launch_arguments={
            'odom_mode': LaunchConfiguration('odom_mode'),
            'mag_enabled': LaunchConfiguration('mag_enabled'),
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

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen',
        parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
    )

    return LaunchDescription([
        robot_mode,
        slam_enabled,
        odom_mode,
        mag_enabled,
        use_sim_time,
        world,
        gazebo_gui,
        rviz,
        rviz_config,
        teleop,
        localization,
        slam,
        rviz_node,
    ])
