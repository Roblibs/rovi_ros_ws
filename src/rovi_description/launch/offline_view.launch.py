#!/usr/bin/env python3
"""Offline visualization: joint_state_publisher(+gui) + robot_state_publisher + RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    desc_share = get_package_share_directory('rovi_description')
    default_model = os.path.join(desc_share, 'urdf', 'rovi.urdf')
    default_rviz = os.path.join(desc_share, 'rviz', 'rovi.rviz')

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
        PythonExpression(["open('", LaunchConfiguration('model'), "', 'r').read()"]),
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

    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': robot_description},
        ],
        output='screen',
    )

    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': robot_description},
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

    return LaunchDescription([
        model_arg,
        rviz_arg,
        sim_time_arg,
        jsp_node,
        jsp_gui_node,
        static_odom_tf,
        rsp_node,
        rviz_node,
    ])
