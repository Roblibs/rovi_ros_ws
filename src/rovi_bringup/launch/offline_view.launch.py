#!/usr/bin/env python3
"""Offline visualization: joint_state_publisher_gui + robot_state_publisher + RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory('rovi_bringup')
    desc_share = get_package_share_directory('rovi_description')
    default_model = os.path.join(desc_share, 'urdf', 'rovi.urdf')
    default_rviz = os.path.join(desc_share, 'rviz', 'rovi_odom.rviz')

    # Use shared memory transport instead of UDP multicast for reliable local DDS discovery
    localhost_only = SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1')

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=default_model,
        description='Absolute path to the Rovi URDF file.',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz,
        description='Absolute path to an RViz config file.',
    )
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use /clock for simulation time.',
    )

    robot_bringup_launch = os.path.join(bringup_share, 'launch', 'robot_bringup.launch.py')
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_bringup_launch),
        launch_arguments={
            'robot_mode': 'offline',
            'model': LaunchConfiguration('model'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen',
    )

    return LaunchDescription([
        localhost_only,
        model_arg,
        rviz_arg,
        sim_time_arg,
        robot_bringup,
        rviz_node,
    ])
