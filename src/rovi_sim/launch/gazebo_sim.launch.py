#!/usr/bin/env python3
"""Gazebo Sim backend for rovi (robot_mode=sim).

This launch is intentionally *not* a full stack bringup. It only starts:
- Gazebo Sim (server + optional GUI)
- ros_gz_bridge (parameter_bridge) to bridge /clock, /odom_gz, /cmd_vel_sim and /camera/*/camera_info
- rovi_gz_sensors_bridge_node to bridge /scan, /imu/data_raw and /camera/*/image with stable frame IDs
- Robot spawn from the shared URDF (rovi_description)

Higher-level bringup (teleop / mapping / nav) is expected to run separately and remain unchanged.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.events import Shutdown
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    sim_share = get_package_share_directory('rovi_sim')
    desc_share = get_package_share_directory('rovi_description')

    default_world = os.path.join(sim_share, 'worlds', 'rovi_room.sdf')
    default_bridge_cfg = os.path.join(sim_share, 'config', 'bridge.yaml')
    default_model = os.path.join(desc_share, 'urdf', 'rovi.urdf')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Full path to the Gazebo world SDF file.',
    )
    gazebo_gui_arg = DeclareLaunchArgument(
        'gazebo_gui',
        default_value='true',
        description='Start Gazebo GUI client (server always starts).',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use /clock time (Gazebo simulation time).',
    )
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=default_model,
        description='URDF to spawn in Gazebo (must include Gazebo sensor tags).',
    )
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='rovi',
        description='Name of the spawned model in Gazebo.',
    )
    bridge_cfg_arg = DeclareLaunchArgument(
        'bridge_config',
        default_value=default_bridge_cfg,
        description='ros_gz_bridge YAML config file.',
    )

    use_sim_time_param = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

    # Make package:// URIs resolvable in Gazebo by ensuring the parent "share" dirs are in the resource path.
    sim_share_parent = str(Path(sim_share).parent.resolve())
    desc_share_parent = str(Path(desc_share).parent.resolve())
    set_gz_paths = [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', sim_share_parent),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', desc_share_parent),
    ]

    gz_launch = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={
            'gz_args': [TextSubstitution(text='-r -s '), LaunchConfiguration('world')],
            # Do not hard-shutdown on exit; we emit a Shutdown event below.
            'on_exit_shutdown': TextSubstitution(text='false'),
        }.items(),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        condition=IfCondition(LaunchConfiguration('gazebo_gui')),
        launch_arguments={
            'gz_args': [TextSubstitution(text='-g ')],
            'on_exit_shutdown': TextSubstitution(text='false'),
        }.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[
            {
                'config_file': LaunchConfiguration('bridge_config'),
                'use_sim_time': use_sim_time_param,
            }
        ],
    )

    # Bridge Gazebo sensor topics while enforcing stable ROS frame IDs (golden rule parity).
    sensors_bridge = Node(
        package='rovi_gz_sensors_bridge',
        executable='rovi_gz_sensors_bridge_node',
        name='rovi_gz_sensors_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time_param}],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', LaunchConfiguration('robot_name'),
            '-file', LaunchConfiguration('model'),
            '-x', TextSubstitution(text='0.0'),
            '-y', TextSubstitution(text='0.0'),
            '-z', TextSubstitution(text='0.05'),
        ],
    )

    delayed_spawn = TimerAction(period=2.0, actions=[spawn_robot])

    graceful_shutdown = RegisterEventHandler(
        OnProcessExit(
            target_action=lambda action: getattr(action, 'name', '') == 'gazebo',
            on_exit=[
                TimerAction(
                    period=0.5,
                    actions=[EmitEvent(event=Shutdown(reason='Gazebo exited'))],
                )
            ],
        )
    )

    return LaunchDescription([
        world_arg,
        gazebo_gui_arg,
        use_sim_time_arg,
        model_arg,
        robot_name_arg,
        bridge_cfg_arg,
        *set_gz_paths,
        gazebo_server,
        gazebo_client,
        bridge,
        sensors_bridge,
        delayed_spawn,
        graceful_shutdown,
    ])
