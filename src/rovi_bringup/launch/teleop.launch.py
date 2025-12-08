#!/usr/bin/env python3
"""Bringup joystick + teleop_twist_joy + Rosmaster driver for manual control."""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('rovi_bringup')
    default_joy_params = os.path.join(pkg_share, 'config', 'joy.params.yaml')
    default_teleop_params = os.path.join(pkg_share, 'config', 'teleop_twist_joy.yaml')
    default_rovi_base_params = os.path.join(pkg_share, 'config', 'rovi_base.yaml')
    default_rosmaster_params = os.path.join(pkg_share, 'config', 'rosmaster_driver.yaml')

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
    rosmaster_params_arg = DeclareLaunchArgument(
        'rosmaster_params_file',
        default_value=default_rosmaster_params,
        description='YAML file with parameters for rosmaster_driver',
    )
    rovi_base_params_arg = DeclareLaunchArgument(
        'rovi_base_params_file',
        default_value=default_rovi_base_params,
        description='YAML file with parameters for rovi_base',
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
    rovi_base_tf_arg = DeclareLaunchArgument(
        'rovi_base_publish_tf',
        default_value='true',
        description='Enable TF broadcast from rovi_base',
    )
    rovi_base_frame_arg = DeclareLaunchArgument(
        'rovi_base_frame',
        default_value='base_footprint',
        description='Child frame id for rovi_base',
    )
    rovi_base_odom_arg = DeclareLaunchArgument(
        'rovi_base_odom_frame',
        default_value='odom',
        description='Odom frame id for rovi_base',
    )
    lidar_enable_arg = DeclareLaunchArgument(
        'lidar_enabled',
        default_value='true',
        description='Start rplidar_ros alongside teleop stack',
    )
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial device for RPLIDAR (e.g., /dev/ttyUSB0)',
    )
    lidar_frame_arg = DeclareLaunchArgument(
        'lidar_frame',
        default_value='laser',
        description='Frame id published on the LaserScan',
    )
    lidar_baud_arg = DeclareLaunchArgument(
        'lidar_serial_baudrate',
        default_value='115200',
        description='Baudrate for the RPLIDAR serial connection',
    )
    lidar_angle_comp_arg = DeclareLaunchArgument(
        'lidar_angle_compensate',
        default_value='true',
        description='Enable angle compensation in rplidar_ros',
    )

    actions = []

    # If user activated a virtual env (uv, venv, etc.), prepend its site-packages so
    # the Rosmaster dependency is available even when colcon installs elsewhere.
    venv_root = os.environ.get('VIRTUAL_ENV')
    py_ver = f"python{sys.version_info.major}.{sys.version_info.minor}"
    venv_site = os.path.join(venv_root, 'lib', py_ver, 'site-packages') if venv_root else ''
    if venv_site and os.path.isdir(venv_site):
        actions.append(
            SetEnvironmentVariable(
                name='PYTHONPATH',
                value=[
                    TextSubstitution(text=venv_site),
                    TextSubstitution(text=os.pathsep),
                    EnvironmentVariable('PYTHONPATH', default_value='')
                ],
            )
        )
        actions.append(SetEnvironmentVariable(name='PYTHONUNBUFFERED', value='1'))
        actions.append(LogInfo(msg=f"Using venv site-packages: {venv_site}"))
    else:
        actions.append(LogInfo(msg='No active virtual env detected; using default PYTHONPATH'))

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
            LaunchConfiguration('rosmaster_params_file'),
            {'port': LaunchConfiguration('rosmaster_port')},
            {'debug': ParameterValue(LaunchConfiguration('rosmaster_debug'), value_type=bool)},
        ],
        remappings=[('cmd_vel', LaunchConfiguration('cmd_vel_topic'))],
    )

    rovi_base_node = Node(
        package='rovi_base',
        executable='rovi_base_node',
        name='rovi_base',
        parameters=[
            LaunchConfiguration('rovi_base_params_file'),
            {
                'publish_tf': ParameterValue(LaunchConfiguration('rovi_base_publish_tf'), value_type=bool),
                'odom_frame': LaunchConfiguration('rovi_base_odom_frame'),
                'base_frame': LaunchConfiguration('rovi_base_frame'),
            },
        ],
    )

    actions.extend([
        joy_params_arg,
        teleop_params_arg,
        rosmaster_params_arg,
        rovi_base_params_arg,
        joy_dev_arg,
        cmd_vel_arg,
        rosmaster_port_arg,
        rosmaster_debug_arg,
        rovi_base_tf_arg,
        rovi_base_frame_arg,
        rovi_base_odom_arg,
        lidar_enable_arg,
        lidar_port_arg,
        lidar_frame_arg,
        lidar_baud_arg,
        lidar_angle_comp_arg,
        joy_node,
        teleop_node,
        rosmaster_driver_node,
        rovi_base_node,
    ])

    rplidar_share = get_package_share_directory('rplidar_ros')
    rplidar_launch = os.path.join(rplidar_share, 'launch', 'rplidar.launch.py')
    lidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch),
        condition=IfCondition(LaunchConfiguration('lidar_enabled')),
        launch_arguments={
            'serial_port': LaunchConfiguration('lidar_serial_port'),
            'serial_baudrate': LaunchConfiguration('lidar_serial_baudrate'),
            'frame_id': LaunchConfiguration('lidar_frame'),
            'angle_compensate': LaunchConfiguration('lidar_angle_compensate'),
        }.items(),
    )
    actions.append(lidar_node)

    return LaunchDescription(actions)
