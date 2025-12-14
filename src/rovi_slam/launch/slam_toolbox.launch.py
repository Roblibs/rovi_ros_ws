#!/usr/bin/env python3
"""Launch slam_toolbox in mapping or localization mode."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def _autostart_events(node_action, condition):
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(node_action),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=condition,
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node_action,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] slam_toolbox is activating."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(node_action),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=condition,
    )

    return configure_event, activate_event


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('rovi_slam')
    default_mapping_params = os.path.join(pkg_share, 'config', 'slam_toolbox_mapping.yaml')
    default_localization_params = os.path.join(pkg_share, 'config', 'slam_toolbox_localization.yaml')

    slam_enabled = DeclareLaunchArgument(
        'slam_enabled',
        default_value='true',
        description='Start slam_toolbox and publish TF map->odom.',
    )
    slam_mode = DeclareLaunchArgument(
        'slam_mode',
        default_value='mapping',
        description="slam_toolbox mode: 'mapping' or 'localization'.",
    )
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use /clock time.',
    )
    autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically configure+activate the slam_toolbox lifecycle node.',
    )
    use_lifecycle_manager = DeclareLaunchArgument(
        'use_lifecycle_manager',
        default_value='false',
        description='Enable external lifecycle manager (disables internal autostart events).',
    )
    slam_params_file = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_mapping_params,
        description='Full path to the ROS2 parameters file to use for slam_toolbox.',
    )
    localization_params_file = DeclareLaunchArgument(
        'localization_params_file',
        default_value=default_localization_params,
        description='Full path to the ROS2 parameters file to use for slam_toolbox localization.',
    )
    map_file_name = DeclareLaunchArgument(
        'map_file_name',
        default_value='',
        description='Pose-graph file to load in localization mode (slam_toolbox param map_file_name, typically .posegraph).',
    )

    mapping_selected = PythonExpression(["'", LaunchConfiguration('slam_mode'), "' == 'mapping'"])
    localization_selected = PythonExpression(["'", LaunchConfiguration('slam_mode'), "' == 'localization'"])

    mapping_node = LifecycleNode(
        condition=IfCondition(
            PythonExpression([
                "('", LaunchConfiguration('slam_enabled'), "' == 'true') and (", mapping_selected, ")",
            ])
        ),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {
                'use_lifecycle_manager': ParameterValue(LaunchConfiguration('use_lifecycle_manager'), value_type=bool),
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            },
        ],
    )

    localization_node = LifecycleNode(
        condition=IfCondition(
            PythonExpression([
                "('", LaunchConfiguration('slam_enabled'), "' == 'true') and (", localization_selected, ")",
            ])
        ),
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            LaunchConfiguration('localization_params_file'),
            {
                'use_lifecycle_manager': ParameterValue(LaunchConfiguration('use_lifecycle_manager'), value_type=bool),
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                'map_file_name': LaunchConfiguration('map_file_name'),
            },
        ],
    )

    mapping_autostart_condition = IfCondition(
        PythonExpression([
            "('", LaunchConfiguration('slam_enabled'), "' == 'true') and ",
            "('", LaunchConfiguration('slam_mode'), "' == 'mapping') and ",
            "('", LaunchConfiguration('autostart'), "' == 'true') and ",
            "('", LaunchConfiguration('use_lifecycle_manager'), "' != 'true')",
        ])
    )

    localization_autostart_condition = IfCondition(
        PythonExpression([
            "('", LaunchConfiguration('slam_enabled'), "' == 'true') and ",
            "('", LaunchConfiguration('slam_mode'), "' == 'localization') and ",
            "('", LaunchConfiguration('autostart'), "' == 'true') and ",
            "('", LaunchConfiguration('use_lifecycle_manager'), "' != 'true')",
        ])
    )

    mapping_configure_event, mapping_activate_event = _autostart_events(mapping_node, mapping_autostart_condition)
    localization_configure_event, localization_activate_event = _autostart_events(localization_node, localization_autostart_condition)

    return LaunchDescription([
        slam_enabled,
        slam_mode,
        use_sim_time,
        autostart,
        use_lifecycle_manager,
        slam_params_file,
        localization_params_file,
        map_file_name,
        mapping_node,
        localization_node,
        mapping_configure_event,
        mapping_activate_event,
        localization_configure_event,
        localization_activate_event,
    ])
