#!/usr/bin/env python3
"""Robot backend selector for rovi.

This launch provides the "robot interface contract" expected by higher-level stacks (SLAM / Nav2):
- /cmd_vel (Twist) input
- /scan (LaserScan)
- /vel_raw (Twist) feedback
- /odom_raw (Odometry) + TF odom -> base_footprint (source depends on robot_mode / odom_mode)
- /imu/data_raw (Imu) when available
- /clock when robot_mode=sim (use_sim_time:=true)

robot_mode:
  - real: hardware drivers (rosmaster_driver + rplidar_ros) + rovi_odom_integrator + robot_state_publisher
  - sim:  Gazebo Sim backend (rovi_sim) + rovi_sim_base + rovi_gz_odom + robot_state_publisher
  - offline: model inspection only (robot_state_publisher + joint_state_publisher_gui)
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory('rovi_bringup')
    desc_share = get_package_share_directory('rovi_description')
    sim_share = get_package_share_directory('rovi_sim')
    ui_bridge_share = get_package_share_directory('ros_ui_bridge')
    serial_display_share = get_package_share_directory('robot_serial_display')

    default_model = os.path.join(desc_share, 'urdf', 'rovi.urdf')
    default_odom_integrator_params = os.path.join(bringup_share, 'config', 'rovi_odom_integrator.yaml')
    default_rosmaster_params = os.path.join(bringup_share, 'config', 'rosmaster_driver.yaml')
    default_ui_bridge_config = os.path.join(ui_bridge_share, 'config', 'default.yaml')
    default_serial_display_config = os.path.join(serial_display_share, 'config', 'default.yaml')

    default_world = os.path.join(sim_share, 'worlds', 'rovi_room.sdf')
    sim_backend_launch = os.path.join(sim_share, 'launch', 'gazebo_sim.launch.py')

    robot_mode_arg = DeclareLaunchArgument(
        'robot_mode',
        default_value='real',
        description="Robot backend: 'real', 'sim', or 'offline'.",
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=default_model,
        description='Absolute path to the robot URDF.',
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

    cmd_vel_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='cmd_vel',
        description='Final Twist topic sent to the base (e.g. twist_mux output).',
    )

    # Base / odom integrator
    odom_integrator_params_arg = DeclareLaunchArgument(
        'odom_integrator_params_file',
        default_value=default_odom_integrator_params,
        description='YAML file with parameters for rovi_odom_integrator',
    )
    odom_integrator_tf_arg = DeclareLaunchArgument(
        'odom_integrator_publish_tf',
        default_value='true',
        description='Enable TF broadcast from rovi_odom_integrator (raw odom).',
    )
    odom_integrator_base_frame_arg = DeclareLaunchArgument(
        'odom_integrator_base_frame',
        default_value='base_footprint',
        description='Child frame id for rovi_odom_integrator',
    )
    odom_integrator_odom_frame_arg = DeclareLaunchArgument(
        'odom_integrator_odom_frame',
        default_value='odom',
        description='Odom frame id for rovi_odom_integrator',
    )

    # Hardware driver (real)
    rosmaster_params_arg = DeclareLaunchArgument(
        'rosmaster_params_file',
        default_value=default_rosmaster_params,
        description='YAML file with parameters for rosmaster_driver',
    )
    rosmaster_port_arg = DeclareLaunchArgument(
        'rosmaster_port',
        default_value=EnvironmentVariable('ROVI_ROSMASTER_PORT', default_value='/dev/robot_control'),
        description='Serial device exposed by the Rosmaster base board',
    )
    rosmaster_debug_arg = DeclareLaunchArgument(
        'rosmaster_debug',
        default_value='false',
        description='Enable verbose hardware driver logging',
    )

    # LiDAR driver (real)
    lidar_enable_arg = DeclareLaunchArgument(
        'lidar_enabled',
        default_value='true',
        description='Start rplidar_ros (real robot only).',
    )
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_serial_port',
        default_value=EnvironmentVariable('ROVI_LIDAR_PORT', default_value='/dev/robot_lidar'),
        description='Serial device for RPLIDAR (e.g., /dev/robot_lidar)',
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

    # UI bridge + serial display client (all modes, can be disabled).
    ui_bridge_enabled_arg = DeclareLaunchArgument(
        'ui_bridge_enabled',
        default_value='true',
        description='Start ros_ui_bridge gRPC server.',
    )
    ui_bridge_config_arg = DeclareLaunchArgument(
        'ui_bridge_config',
        default_value=default_ui_bridge_config,
        description='Path to ros_ui_bridge YAML config.',
    )
    ui_bridge_log_level_arg = DeclareLaunchArgument(
        'ui_bridge_log_level',
        default_value='info',
        description='ROS log level for ros_ui_bridge (e.g., debug, info, warn, error).',
    )
    serial_display_enabled_arg = DeclareLaunchArgument(
        'serial_display_enabled',
        default_value=PythonExpression([
            "'true' if '",
            LaunchConfiguration('robot_mode'),
            "' == 'real' else 'false'",
        ]),
        description='Start robot_serial_display (gRPC client -> USB serial display).',
    )
    serial_display_config_arg = DeclareLaunchArgument(
        'serial_display_config',
        default_value=default_serial_display_config,
        description='Path to robot_serial_display YAML config.',
    )
    serial_display_log_level_arg = DeclareLaunchArgument(
        'serial_display_log_level',
        default_value='info',
        description='ROS log level for robot_serial_display (e.g., debug, info, warn, error).',
    )
    serial_display_debug_arg = DeclareLaunchArgument(
        'serial_display_debug',
        default_value='false',
        description='Enable verbose robot_serial_display payload logging.',
    )

    # Simulation backend (sim)
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

    is_real = IfCondition(PythonExpression(["'", LaunchConfiguration('robot_mode'), "' == 'real'"]))
    is_sim = IfCondition(PythonExpression(["'", LaunchConfiguration('robot_mode'), "' == 'sim'"]))
    is_offline = IfCondition(PythonExpression(["'", LaunchConfiguration('robot_mode'), "' == 'offline'"]))
    is_not_offline = IfCondition(PythonExpression(["'", LaunchConfiguration('robot_mode'), "' != 'offline'"]))

    use_sim_time_param = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

    # Prefer an active venv; otherwise try $ROVI_ROS_WS_DIR/.venv so launches work without `activate`.
    venv_root = os.environ.get('VIRTUAL_ENV')
    if not venv_root:
        rovi_ws_dir = os.environ.get('ROVI_ROS_WS_DIR')
        if rovi_ws_dir:
            venv_root = os.path.join(rovi_ws_dir, '.venv')
    py_ver = f"python{sys.version_info.major}.{sys.version_info.minor}"
    venv_site = os.path.join(venv_root, 'lib', py_ver, 'site-packages') if venv_root else ''

    if venv_site and os.path.isdir(venv_site):
        venv_env_actions = [
            SetEnvironmentVariable(
                name='PYTHONPATH',
                value=[
                    TextSubstitution(text=venv_site),
                    TextSubstitution(text=':'),
                    EnvironmentVariable('PYTHONPATH', default_value=''),
                ],
            ),
            SetEnvironmentVariable(name='PYTHONUNBUFFERED', value='1'),
            LogInfo(msg=f"Using venv site-packages: {venv_site}"),
        ]
    else:
        venv_env_actions = [
            LogInfo(msg="No venv site-packages found; using existing PYTHONPATH")
        ]

    robot_description = ParameterValue(
        Command(['cat ', LaunchConfiguration('model')]),
        value_type=str,
    )

    # Always publish the robot TF tree from the shared URDF.
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time_param},
        ],
    )

    # Provide joint_states in modes where hardware isn't publishing them.
    # NOTE: upstream joint_state_publisher sometimes exits with an RCLError on SIGINT in Jazzy.
    # Use a small local publisher to keep shutdown clean.
    joint_state_pub_sim = Node(
        condition=is_sim,
        package='rovi_sim',
        executable='rovi_local_joint_states',
        output='screen',
        arguments=[LaunchConfiguration('model')],
        parameters=[{'use_sim_time': use_sim_time_param}],
    )

    jsp_gui_node = Node(
        condition=is_offline,
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time_param}],
    )

    # Delay jsp_gui to ensure robot_description topic is available.
    delayed_jsp_gui = RegisterEventHandler(
        OnProcessStart(
            target_action=rsp_node,
            on_start=[TimerAction(period=0.5, actions=[jsp_gui_node])],
        )
    )

    static_odom_tf = Node(
        condition=is_offline,
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_basefootprint',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
    )

    odom_integrator_node = Node(
        condition=is_real,
        package='rovi_odom_integrator',
        executable='rovi_odom_integrator_node',
        name='rovi_odom_integrator',
        output='screen',
        parameters=[
            LaunchConfiguration('odom_integrator_params_file'),
            {
                'publish_tf': ParameterValue(LaunchConfiguration('odom_integrator_publish_tf'), value_type=bool),
                'odom_frame': LaunchConfiguration('odom_integrator_odom_frame'),
                'base_frame': LaunchConfiguration('odom_integrator_base_frame'),
                'use_sim_time': use_sim_time_param,
            },
        ],
    )

    rosmaster_driver_node = Node(
        condition=is_real,
        package='rosmaster_driver',
        executable='rosmaster_driver_node',
        name='rosmaster_driver',
        output='screen',
        parameters=[
            LaunchConfiguration('rosmaster_params_file'),
            {'port': LaunchConfiguration('rosmaster_port')},
            {'debug': ParameterValue(LaunchConfiguration('rosmaster_debug'), value_type=bool)},
            {'use_sim_time': use_sim_time_param},
        ],
        remappings=[('cmd_vel', LaunchConfiguration('cmd_vel_topic'))],
    )

    lidar_node = Node(
        condition=IfCondition(PythonExpression([
            "'",
            LaunchConfiguration('robot_mode'),
            "' == 'real' and '",
            LaunchConfiguration('lidar_enabled'),
            "' == 'true'",
        ])),
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('lidar_serial_port'),
            'serial_baudrate': ParameterValue(LaunchConfiguration('lidar_serial_baudrate'), value_type=int),
            'frame_id': LaunchConfiguration('lidar_frame'),
            'inverted': False,
            'angle_compensate': ParameterValue(LaunchConfiguration('lidar_angle_compensate'), value_type=bool),
            'use_sim_time': use_sim_time_param,
        }],
    )

    sim_backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_backend_launch),
        condition=is_sim,
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gazebo_gui': LaunchConfiguration('gazebo_gui'),
            'model': LaunchConfiguration('model'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    sim_base = Node(
        condition=is_sim,
        package='rovi_sim',
        executable='rovi_sim_base',
        name='rovi_sim_base',
        output='screen',
        parameters=[
            {
                'cmd_vel_in': LaunchConfiguration('cmd_vel_topic'),
                'cmd_vel_out': 'cmd_vel_sim',
                'vel_raw_out': 'vel_raw',
            }
        ],
    )

    sim_odom = Node(
        condition=is_sim,
        package='rovi_sim',
        executable='rovi_gz_odom',
        name='rovi_gz_odom',
        output='screen',
        parameters=[
            {
                'input_topic': 'odom_gz',
                'output_topic': 'odom_raw',
                'odom_frame': LaunchConfiguration('odom_integrator_odom_frame'),
                'base_frame': LaunchConfiguration('odom_integrator_base_frame'),
                'publish_tf': ParameterValue(LaunchConfiguration('odom_integrator_publish_tf'), value_type=bool),
                'two_d_mode': True,
                'use_sim_time': use_sim_time_param,
            }
        ],
    )

    ui_bridge_node = Node(
        condition=IfCondition(LaunchConfiguration('ui_bridge_enabled')),
        package='ros_ui_bridge',
        executable='ui_bridge',
        output='screen',
        arguments=['--config', LaunchConfiguration('ui_bridge_config')],
        parameters=[{'use_sim_time': use_sim_time_param}],
        ros_arguments=['--ros-args', '--log-level', LaunchConfiguration('ui_bridge_log_level')],
    )

    serial_display_node = Node(
        condition=IfCondition(PythonExpression([
            "'",
            LaunchConfiguration('serial_display_enabled'),
            "' == 'true' and '",
            LaunchConfiguration('robot_mode'),
            "' == 'real' and '",
            LaunchConfiguration('serial_display_debug'),
            "' != 'true'",
        ])),
        package='robot_serial_display',
        executable='serial_display',
        name='robot_serial_display',
        output='screen',
        arguments=['--config', LaunchConfiguration('serial_display_config')],
        ros_arguments=['--ros-args', '--log-level', LaunchConfiguration('serial_display_log_level')],
    )

    serial_display_debug_node = Node(
        condition=IfCondition(PythonExpression([
            "'",
            LaunchConfiguration('serial_display_enabled'),
            "' == 'true' and '",
            LaunchConfiguration('robot_mode'),
            "' == 'real' and '",
            LaunchConfiguration('serial_display_debug'),
            "' == 'true'",
        ])),
        package='robot_serial_display',
        executable='serial_display',
        name='robot_serial_display',
        output='screen',
        arguments=['--config', LaunchConfiguration('serial_display_config'), '--debug'],
        ros_arguments=['--ros-args', '--log-level', LaunchConfiguration('serial_display_log_level')],
    )

    return LaunchDescription([
        robot_mode_arg,
        model_arg,
        use_sim_time_arg,
        cmd_vel_arg,
        odom_integrator_params_arg,
        odom_integrator_tf_arg,
        odom_integrator_base_frame_arg,
        odom_integrator_odom_frame_arg,
        rosmaster_params_arg,
        rosmaster_port_arg,
        rosmaster_debug_arg,
        lidar_enable_arg,
        lidar_port_arg,
        lidar_frame_arg,
        lidar_baud_arg,
        lidar_angle_comp_arg,
        ui_bridge_enabled_arg,
        ui_bridge_config_arg,
        ui_bridge_log_level_arg,
        serial_display_enabled_arg,
        serial_display_config_arg,
        serial_display_log_level_arg,
        serial_display_debug_arg,
        world_arg,
        gazebo_gui_arg,
        *venv_env_actions,
        rsp_node,
        joint_state_pub_sim,
        static_odom_tf,
        delayed_jsp_gui,
        odom_integrator_node,
        rosmaster_driver_node,
        lidar_node,
        ui_bridge_node,
        serial_display_node,
        serial_display_debug_node,
        sim_backend,
        sim_base,
        sim_odom,
    ])
