#!/usr/bin/env python3
"""Full simulation bringup: Gazebo Sim + SLAM + Nav2 + RViz.

This launch is meant to complement the real robot stack by keeping the same ROS interfaces:
- /cmd_vel (twist_mux output) drives the base
- /scan (LaserScan) is provided by a Gazebo LiDAR
- /odom_raw is produced from /vel_raw by rovi_base, then /odometry/filtered by EKF
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    sim_share = get_package_share_directory('rovi_sim')
    desc_share = get_package_share_directory('rovi_description')
    bringup_share = get_package_share_directory('rovi_bringup')

    default_world = os.path.join(sim_share, 'worlds', 'rovi_room.sdf')
    default_bridge_cfg = os.path.join(sim_share, 'config', 'bridge.yaml')

    default_urdf = os.path.join(desc_share, 'urdf', 'rovi.urdf')
    default_rviz = os.path.join(desc_share, 'rviz', 'rovi_map.rviz')

    default_joy_params = os.path.join(bringup_share, 'config', 'joy.params.yaml')
    default_teleop_params = os.path.join(bringup_share, 'config', 'teleop_twist_joy.yaml')
    default_twist_mux_params = os.path.join(bringup_share, 'config', 'twist_mux.yaml')

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
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz2 with the default config.',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use /clock time (Gazebo simulation time).',
    )
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='0',
        description='SDL device index that maps to /dev/input/js* (default: js0)',
    )
    slam_mode_arg = DeclareLaunchArgument(
        'slam_mode',
        default_value='mapping',
        description="slam_toolbox mode: 'mapping' or 'localization'.",
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
            'on_exit_shutdown': TextSubstitution(text='true'),
        }.items(),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        condition=IfCondition(LaunchConfiguration('gazebo_gui')),
        launch_arguments={
            'gz_args': [TextSubstitution(text='-g ')],
            'on_exit_shutdown': TextSubstitution(text='true'),
        }.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[
            {
                'config_file': default_bridge_cfg,
                'use_sim_time': use_sim_time_param,
            }
        ],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'rovi',
            '-file', default_urdf,
            '-x', TextSubstitution(text='0.0'),
            '-y', TextSubstitution(text='0.0'),
            '-z', TextSubstitution(text='0.05'),
        ],
    )

    # Teleop + mux (same topics as the real robot stack)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[
            default_joy_params,
            {'device_id': ParameterValue(LaunchConfiguration('joy_dev'), value_type=int)},
            {'use_sim_time': use_sim_time_param},
        ],
        output='screen',
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[default_teleop_params, {'use_sim_time': use_sim_time_param}],
        remappings=[('cmd_vel', 'cmd_vel_joy')],
        output='screen',
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[default_twist_mux_params, {'use_sim_time': use_sim_time_param}],
        remappings=[('cmd_vel_out', 'cmd_vel')],
    )

    # Smoothing layer: /cmd_vel -> (/cmd_vel_sim + /vel_raw)
    sim_base = Node(
        package='rovi_sim',
        executable='rovi_sim_base',
        name='rovi_sim_base',
        output='screen',
        parameters=[
            {
                'cmd_vel_in': 'cmd_vel',
                'cmd_vel_out': 'cmd_vel_sim',
                'vel_raw_out': 'vel_raw',
            }
        ],
    )

    # Wheel-odom integrator: /vel_raw -> /odom_raw
    rovi_base = Node(
        package='rovi_base',
        executable='rovi_base_node',
        name='rovi_base',
        output='screen',
        parameters=[
            {
                'publish_tf': False,
                'publish_rate': 50.0,
                'use_sim_time': use_sim_time_param,
            }
        ],
    )

    # Publish robot_description + TF tree for RViz (reuses the real URDF)
    with open(default_urdf, 'r', encoding='utf-8') as f:
        robot_description_content = f.read()
    robot_description = ParameterValue(robot_description_content, value_type=str)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time_param},
        ],
        output='screen',
    )

    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time_param},
        ],
        output='screen',
    )

    # Core stack (unchanged)
    ekf_launch = os.path.join(get_package_share_directory('rovi_localization'), 'launch', 'ekf.launch.py')
    slam_launch = os.path.join(get_package_share_directory('rovi_slam'), 'launch', 'slam_toolbox.launch.py')
    nav_launch = os.path.join(get_package_share_directory('rovi_nav'), 'launch', 'nav.launch.py')

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ekf_launch),
        launch_arguments={
            'odom_mode': TextSubstitution(text='filtered'),
            'mag_enabled': TextSubstitution(text='false'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        launch_arguments={
            'slam_enabled': TextSubstitution(text='true'),
            'slam_mode': LaunchConfiguration('slam_mode'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_launch),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz],
        parameters=[{'use_sim_time': use_sim_time_param}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # Keep startup resilient: bring up Gazebo + bridge first, then spawn, then start the ROS stack.
    delayed_spawn = TimerAction(period=2.0, actions=[spawn_robot])
    delayed_stack = TimerAction(
        period=3.0,
        actions=[
            rsp_node,
            joint_state_pub,
            joy_node,
            teleop_node,
            twist_mux_node,
            sim_base,
            rovi_base,
            ekf,
            slam,
            nav2,
            TimerAction(period=0.5, actions=[rviz_node]),
        ],
    )

    return LaunchDescription([
        world_arg,
        gazebo_gui_arg,
        rviz_arg,
        use_sim_time_arg,
        joy_dev_arg,
        slam_mode_arg,
        *set_gz_paths,
        gazebo_server,
        gazebo_client,
        bridge,
        delayed_spawn,
        delayed_stack,
    ])
