#!/usr/bin/env python3
"""Offline visualization: joint_state_publisher_gui + robot_state_publisher + RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction, SetEnvironmentVariable
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    desc_share = get_package_share_directory('rovi_description')
    default_model = os.path.join(desc_share, 'urdf', 'rovi.urdf')
    default_rviz = os.path.join(desc_share, 'rviz', 'rovi.rviz')

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

    robot_description = ParameterValue(
        Command(['cat ', LaunchConfiguration('model')]),
        value_type=str,
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    static_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_basefootprint',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen',
    )

    # Delay jsp_gui to ensure robot_description topic is available
    delayed_jsp_gui = RegisterEventHandler(
        OnProcessStart(
            target_action=rsp_node,
            on_start=[TimerAction(period=0.5, actions=[jsp_gui_node])],
        )
    )

    return LaunchDescription([
        localhost_only,
        model_arg,
        rviz_arg,
        sim_time_arg,
        static_odom_tf,
        rsp_node,
        delayed_jsp_gui,
        rviz_node,
    ])
