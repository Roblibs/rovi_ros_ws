#!/usr/bin/env python3
"""Bringup joystick + teleop_twist_joy + twist_mux + base stack."""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('rovi_bringup')
    default_joy_params = os.path.join(pkg_share, 'config', 'joy.params.yaml')
    default_teleop_params = os.path.join(pkg_share, 'config', 'teleop_twist_joy.yaml')
    default_twist_mux_params = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    default_rovi_base_params = os.path.join(pkg_share, 'config', 'rovi_base.yaml')
    default_rosmaster_params = os.path.join(pkg_share, 'config', 'rosmaster_driver.yaml')
    desc_share = get_package_share_directory('rovi_description')
    default_model = os.path.join(desc_share, 'urdf', 'rovi.urdf')
    default_rviz = os.path.join(desc_share, 'rviz', 'rovi_odom.rviz')
    sim_share = get_package_share_directory('rovi_sim')
    default_world = os.path.join(sim_share, 'worlds', 'rovi_room.sdf')

    robot_mode_arg = DeclareLaunchArgument(
        'robot_mode',
        default_value='real',
        description="Robot backend: 'real', 'sim', or 'offline'.",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=PythonExpression([
            "'true' if '",
            LaunchConfiguration('robot_mode'),
            "' == 'sim' else 'false'",
        ]),
        description='Use /clock time (auto true for robot_mode=sim).',
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Full path to the Gazebo world SDF file (robot_mode=sim).',
    )
    gazebo_gui_arg = DeclareLaunchArgument(
        'gazebo_gui',
        default_value='true',
        description='Start Gazebo GUI client (server always starts) (robot_mode=sim).',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz (teleop view).',
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz,
        description='Absolute path to an RViz config file.',
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
        description='Final Twist topic sent to the base (twist_mux output).',
    )
    cmd_vel_joy_arg = DeclareLaunchArgument(
        'cmd_vel_joy_topic',
        default_value='cmd_vel_joy',
        description='Twist topic produced by teleop_twist_joy (twist_mux input).',
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
        default_value='laser_link',
        description='Frame id published on the LaserScan (match URDF link name)',
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
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=default_model,
        description='Absolute path to the robot URDF.',
    )

    use_sim_time_param = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

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
            {'use_sim_time': use_sim_time_param},
        ],
    )

    teleop_node = Node(
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

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time_param}],
    )

    robot_bringup_launch = os.path.join(pkg_share, 'launch', 'robot_bringup.launch.py')
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_bringup_launch),
        launch_arguments={
            'robot_mode': LaunchConfiguration('robot_mode'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'model': LaunchConfiguration('model'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'rovi_base_params_file': LaunchConfiguration('rovi_base_params_file'),
            'rovi_base_publish_tf': LaunchConfiguration('rovi_base_publish_tf'),
            'rovi_base_frame': LaunchConfiguration('rovi_base_frame'),
            'rovi_base_odom_frame': LaunchConfiguration('rovi_base_odom_frame'),
            'rosmaster_params_file': LaunchConfiguration('rosmaster_params_file'),
            'rosmaster_port': LaunchConfiguration('rosmaster_port'),
            'rosmaster_debug': LaunchConfiguration('rosmaster_debug'),
            'lidar_enabled': LaunchConfiguration('lidar_enabled'),
            'lidar_serial_port': LaunchConfiguration('lidar_serial_port'),
            'lidar_frame': LaunchConfiguration('lidar_frame'),
            'lidar_serial_baudrate': LaunchConfiguration('lidar_serial_baudrate'),
            'lidar_angle_compensate': LaunchConfiguration('lidar_angle_compensate'),
            'world': LaunchConfiguration('world'),
            'gazebo_gui': LaunchConfiguration('gazebo_gui'),
        }.items(),
    )

    actions.extend([
        robot_mode_arg,
        use_sim_time_arg,
        world_arg,
        gazebo_gui_arg,
        rviz_arg,
        rviz_config_arg,
        joy_params_arg,
        teleop_params_arg,
        twist_mux_params_arg,
        rosmaster_params_arg,
        rovi_base_params_arg,
        joy_dev_arg,
        cmd_vel_arg,
        cmd_vel_joy_arg,
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
        model_arg,
        joy_node,
        teleop_node,
        twist_mux_node,
        rviz_node,
        robot_bringup,
    ])

    return LaunchDescription(actions)
