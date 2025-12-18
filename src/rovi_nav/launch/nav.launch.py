#!/usr/bin/env python3
"""Nav2 navigation stack bringup for rovi.

This launch assumes:
- SLAM (slam_toolbox) is publishing `/map` and TF `map -> odom`
- Odometry is publishing TF `odom -> base_footprint`
- A LiDAR is publishing `/scan`

Nav2 publishes velocity commands on `cmd_vel_nav` (to be muxed into `/cmd_vel`).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('rovi_nav')
    default_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the Nav2 nodes.',
    )
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use /clock time.',
    )
    autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically configure+activate Nav2 lifecycle nodes.',
    )
    params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to the Nav2 parameters file.',
    )

    # Use the canonical Nav2 BT XML from nav2_bt_navigator by default.
    bt_xml_file = os.path.join(
        get_package_share_directory('nav2_bt_navigator'),
        'behavior_trees',
        'navigate_to_pose_w_replanning_and_recovery.xml',
    )

    lifecycle_node_names = [
        'controller_server',
        'planner_server',
        'bt_navigator',
        'behavior_server',
    ]

    use_sim_time_param = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    autostart_param = ParameterValue(LaunchConfiguration('autostart'), value_type=bool)

    planner_server = LifecycleNode(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': use_sim_time_param},
        ],
    )

    controller_server = LifecycleNode(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': use_sim_time_param},
        ],
    )

    behavior_server = LifecycleNode(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': use_sim_time_param},
        ],
    )

    bt_navigator = LifecycleNode(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'use_sim_time': use_sim_time_param,
                'default_bt_xml_filename': bt_xml_file,
            },
        ],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time_param,
                'autostart': autostart_param,
                'node_names': lifecycle_node_names,
            }
        ],
    )

    return LaunchDescription([
        namespace,
        use_sim_time,
        autostart,
        params_file,
        planner_server,
        controller_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager,
    ])
