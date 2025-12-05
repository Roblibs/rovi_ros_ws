#!/usr/bin/env python3
"""Bringup joystick + teleop_twist_joy + Rosmaster driver for manual control."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('rovi_bringup')
    default_joy_params = os.path.join(pkg_share, 'config', 'joy.params.yaml')
    default_teleop_params = os.path.join(pkg_share, 'config', 'teleop_twist_joy.yaml')

    joy_params_arg = DeclareLaunchArgument(
        'joy_params_file',
        default_value=default_joy_params,
        description='YAML file with parameters for joy_node',
    )
    teleop_params_arg = DeclareLaunchArgument(
        'teleop_params_file',
        default_value=default_teleop_params,
        description='YAML file with parameters for teleop_twist_joy',
    )
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='0',
        description='SDL device index that maps to /dev/input/js* (default: js0)',
    )
    cmd_vel_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='cmd_vel',
        description='Twist topic used for manual control',
    )
    rosmaster_port_arg = DeclareLaunchArgument(
        'rosmaster_port',
        default_value='/dev/my_ros_board',
        description='Serial device exposed by the Rosmaster base board',
    )
    rosmaster_debug_arg = DeclareLaunchArgument(
        'rosmaster_debug',
        default_value='false',
        description='Enable verbose hardware driver logging',
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[
            LaunchConfiguration('joy_params_file'),
            {'device_id': ParameterValue(LaunchConfiguration('joy_dev'), value_type=int)},
        ],
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[LaunchConfiguration('teleop_params_file')],
        remappings=[('cmd_vel', LaunchConfiguration('cmd_vel_topic'))],
    )

    rosmaster_driver_node = Node(
        package='rosmaster_driver',
        executable='rosmaster_driver_node',
        name='rosmaster_driver',
        parameters=[
            {'port': LaunchConfiguration('rosmaster_port')},
            {'debug': ParameterValue(LaunchConfiguration('rosmaster_debug'), value_type=bool)},
        ],
        remappings=[('cmd_vel', LaunchConfiguration('cmd_vel_topic'))],
    )

    return LaunchDescription([
        joy_params_arg,
        teleop_params_arg,
        joy_dev_arg,
        cmd_vel_arg,
        rosmaster_port_arg,
        rosmaster_debug_arg,
        joy_node,
        teleop_node,
        rosmaster_driver_node,
    ])
