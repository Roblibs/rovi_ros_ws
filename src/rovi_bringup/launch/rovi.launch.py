#!/usr/bin/env python3
"""Unified entrypoint for Rovi stacks.

This launch owns the high-level choices:
- which robot backend to run (robot_mode: real|sim|offline)
- which stack to run (stack: teleop|mapping|localization|nav|offline|bringup)
- whether RViz should start (rviz), defaulting to true only for sim/offline

Intended usage:
- On the robot (Pi): run stacks headless (default rviz:=false)
- On a PC: use `view` (e.g., `view teleop|mapping|nav`) to connect and visualize
- In simulation: one command starts Gazebo + stack + RViz (default rviz:=true)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory('rovi_bringup')
    desc_share = get_package_share_directory('rovi_description')
    sim_share = get_package_share_directory('rovi_sim')

    robot_bringup_launch = os.path.join(bringup_share, 'launch', 'robot_bringup.launch.py')
    teleop_stack_launch = os.path.join(bringup_share, 'launch', 'teleop.launch.py')
    mapping_stack_launch = os.path.join(bringup_share, 'launch', 'mapping.launch.py')
    localization_stack_launch = os.path.join(bringup_share, 'launch', 'localization.launch.py')
    nav_stack_launch = os.path.join(bringup_share, 'launch', 'nav.launch.py')

    default_model = os.path.join(desc_share, 'urdf', 'rovi.urdf')
    default_rviz_map = os.path.join(desc_share, 'rviz', 'rovi_map.rviz')
    default_rviz_nav = os.path.join(desc_share, 'rviz', 'rovi_nav.rviz')
    default_rviz_odom = os.path.join(desc_share, 'rviz', 'rovi_odom.rviz')
    default_world = os.path.join(sim_share, 'worlds', 'rovi_room.sdf')

    robot_mode = DeclareLaunchArgument(
        'robot_mode',
        default_value='real',
        description="Robot backend: 'real' (hardware), 'sim' (Gazebo), or 'offline' (model only).",
    )
    stack = DeclareLaunchArgument(
        'stack',
        default_value='teleop',
        description="Stack to run: 'teleop', 'mapping', 'localization', 'nav', 'offline', or 'bringup'.",
    )
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value=PythonExpression([
            "'true' if '",
            LaunchConfiguration('robot_mode'),
            "' == 'sim' else 'false'",
        ]),
        description='Use /clock time (auto true for robot_mode=sim).',
    )
    rviz = DeclareLaunchArgument(
        'rviz',
        default_value=PythonExpression([
            "'true' if '",
            LaunchConfiguration('robot_mode'),
            "' != 'real' else 'false'",
        ]),
        description='Start RViz (default true for sim/offline, false for real robot).',
    )
    rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value='',
        description="RViz config path. If empty, an appropriate default is selected based on 'stack'.",
    )

    # Control stack (twist_mux + optional joystick). Keyboard teleop is started separately in a terminal.
    joy_enabled = DeclareLaunchArgument(
        'joy_enabled',
        default_value=PythonExpression([
            "'true' if '",
            LaunchConfiguration('robot_mode'),
            "' == 'real' else 'false'",
        ]),
        description='Start joy_node + teleop_twist_joy (default true for real robot, false for sim).',
    )
    cmd_vel_topic = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='cmd_vel',
        description='Final /cmd_vel topic produced by twist_mux and consumed by the robot backend.',
    )

    # Common robot/backend args (passed to robot_bringup).
    model = DeclareLaunchArgument('model', default_value=default_model, description='Absolute path to the robot URDF.')
    world = DeclareLaunchArgument('world', default_value=default_world, description='Gazebo world SDF (robot_mode=sim).')
    gazebo_gui = DeclareLaunchArgument('gazebo_gui', default_value='true', description='Start Gazebo GUI (sim only).')

    # Stack args (passed through).
    slam_enabled = DeclareLaunchArgument(
        'slam_enabled',
        default_value='true',
        description='Start slam_toolbox and publish TF map->odom.',
    )
    slam_mode = DeclareLaunchArgument(
        'slam_mode',
        default_value='mapping',
        description="SLAM mode for nav stack: 'mapping' or 'localization'.",
    )
    map_file_name = DeclareLaunchArgument(
        'map_file_name',
        default_value=os.path.expanduser('~/.ros/rovi/maps/latest.posegraph'),
        description='Pose-graph file to load when slam_mode=localization.',
    )
    mag_enabled = DeclareLaunchArgument(
        'mag_enabled',
        default_value='false',
        description='Enable magnetometer usage in the IMU orientation filter (odom_mode=fusion_wheels_imu).',
    )

    # Odometry coordination between backend and localization stack.
    odom_mode = DeclareLaunchArgument(
        'odom_mode',
        default_value='fusion_wheels_imu',
        description="Odometry mode: 'raw', 'filtered', or 'fusion_wheels_imu'.",
    )
    viz_downsample_config = DeclareLaunchArgument(
        'viz_downsample_config',
        default_value=os.path.join(bringup_share, 'config', 'viz_downsample.yaml'),
        description='YAML config for RViz-only downsampled topics (published under /viz/*).',
    )
    rovi_base_publish_tf = PythonExpression([
        "'true' if '",
        LaunchConfiguration('stack'),
        "' == 'teleop' else ('true' if '",
        LaunchConfiguration('odom_mode'),
        "' == 'raw' else 'false')",
    ])

    # Select RViz config based on stack unless explicitly provided.
    resolved_rviz_config = PythonExpression([
        "'",
        LaunchConfiguration('rviz_config'),
        "' if '",
        LaunchConfiguration('rviz_config'),
        "' != '' else ('",
        default_rviz_nav,
        "' if '",
        LaunchConfiguration('stack'),
        "' == 'nav' else ('",
        default_rviz_map,
        "' if '",
        LaunchConfiguration('stack'),
        "' in ['mapping','localization'] else '",
        default_rviz_odom,
        "'))",
    ])

    # Backend: always start, even for offline (robot_bringup handles robot_mode=offline).
    backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_bringup_launch),
        launch_arguments={
            'robot_mode': LaunchConfiguration('robot_mode'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'model': LaunchConfiguration('model'),
            'world': LaunchConfiguration('world'),
            'gazebo_gui': LaunchConfiguration('gazebo_gui'),
            'rovi_base_publish_tf': rovi_base_publish_tf,
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
        }.items(),
    )

    # Twist muxing (needed for teleop/mapping/nav; joy can be disabled).
    control_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(teleop_stack_launch),
        condition=IfCondition(PythonExpression([
            "'",
            LaunchConfiguration('stack'),
            "' in ['teleop','mapping','localization','nav']",
        ])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'joy_enabled': LaunchConfiguration('joy_enabled'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
        }.items(),
    )

    # Stacks (headless; RViz is owned by this top-level file).
    stack_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mapping_stack_launch),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('stack'), "' == 'mapping'"])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'odom_mode': LaunchConfiguration('odom_mode'),
            'slam_enabled': LaunchConfiguration('slam_enabled'),
            'mag_enabled': LaunchConfiguration('mag_enabled'),
        }.items(),
    )
    stack_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_stack_launch),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('stack'), "' == 'localization'"])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'odom_mode': LaunchConfiguration('odom_mode'),
            'slam_enabled': LaunchConfiguration('slam_enabled'),
            'mag_enabled': LaunchConfiguration('mag_enabled'),
            'map_file_name': LaunchConfiguration('map_file_name'),
        }.items(),
    )
    stack_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_stack_launch),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('stack'), "' == 'nav'"])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'odom_mode': LaunchConfiguration('odom_mode'),
            'slam_mode': LaunchConfiguration('slam_mode'),
            'map_file_name': LaunchConfiguration('map_file_name'),
            'mag_enabled': LaunchConfiguration('mag_enabled'),
        }.items(),
    )

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', resolved_rviz_config],
        output='screen',
        parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
    )

    viz_downsample_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rovi_bringup',
        executable='viz_downsample',
        name='viz_downsample',
        output='screen',
        arguments=['--config', LaunchConfiguration('viz_downsample_config')],
        parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
    )

    return LaunchDescription([
        robot_mode,
        stack,
        use_sim_time,
        rviz,
        rviz_config,
        joy_enabled,
        cmd_vel_topic,
        model,
        world,
        gazebo_gui,
        slam_enabled,
        slam_mode,
        map_file_name,
        mag_enabled,
        odom_mode,
        viz_downsample_config,
        backend,
        control_stack,
        stack_mapping,
        stack_localization,
        stack_nav,
        viz_downsample_node,
        rviz_node,
    ])
