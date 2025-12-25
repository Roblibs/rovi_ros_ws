#!/usr/bin/env python3
"""Teleop input stack: joy_node + teleop_twist_joy + twist_mux.

This launch is intentionally backend-agnostic (it does NOT start hardware or simulation).
It only produces the final `/cmd_vel` command topic.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('rovi_bringup')
    default_joy_params = os.path.join(pkg_share, 'config', 'joy.params.yaml')
    default_teleop_params = os.path.join(pkg_share, 'config', 'teleop_twist_joy.yaml')
    default_twist_mux_params = os.path.join(pkg_share, 'config', 'twist_mux.yaml')

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use /clock time.')
    joy_enabled_arg = DeclareLaunchArgument(
        'joy_enabled',
        default_value='true',
        description='Start joy_node + teleop_twist_joy (joystick).',
    )
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
    twist_mux_params_arg = DeclareLaunchArgument(
        'twist_mux_params_file',
        default_value=default_twist_mux_params,
        description='YAML file with parameters for twist_mux (cmd_vel muxing).',
    )
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='0',
        description='SDL device index that maps to /dev/input/js* (default: js0)',
    )
    cmd_vel_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='cmd_vel',
        description='Final Twist topic published by twist_mux (robot backend subscribes to this).',
    )
    cmd_vel_joy_arg = DeclareLaunchArgument(
        'cmd_vel_joy_topic',
        default_value='cmd_vel_joy',
        description='Twist topic produced by teleop_twist_joy (twist_mux input).',
    )

    use_sim_time_param = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

    joy_node = Node(
        condition=IfCondition(LaunchConfiguration('joy_enabled')),
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[
            LaunchConfiguration('joy_params_file'),
            {'device_id': ParameterValue(LaunchConfiguration('joy_dev'), value_type=int)},
            {'use_sim_time': use_sim_time_param},
        ],
    )

    teleop_node = Node(
        condition=IfCondition(LaunchConfiguration('joy_enabled')),
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[LaunchConfiguration('teleop_params_file'), {'use_sim_time': use_sim_time_param}],
        remappings=[('cmd_vel', LaunchConfiguration('cmd_vel_joy_topic'))],
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[LaunchConfiguration('twist_mux_params_file'), {'use_sim_time': use_sim_time_param}],
        remappings=[('cmd_vel_out', LaunchConfiguration('cmd_vel_topic'))],
    )

    return LaunchDescription([
        use_sim_time_arg,
        joy_enabled_arg,
        joy_params_arg,
        teleop_params_arg,
        twist_mux_params_arg,
        joy_dev_arg,
        cmd_vel_arg,
        cmd_vel_joy_arg,
        joy_node,
        teleop_node,
        twist_mux_node,
    ])
