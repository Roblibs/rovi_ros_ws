#!/usr/bin/env python3
"""Unified entrypoint for Rovi stacks.

This launch owns the high-level choices:
- which robot backend to run (robot_mode: real|sim|offline)
- which stack to run (stack: teleop|camera|mapping|localization|nav|offline|bringup)
- whether RViz should start (rviz), defaulting to true only for sim/offline

Intended usage:
- On the robot (Pi): run stacks headless (default rviz:=false)
- On a PC: use `view` (e.g., `view teleop|mapping|nav`) to connect and visualize
- In simulation: one command starts Gazebo + stack + RViz (default rviz:=true)
"""

import os
import shutil
import socket
import subprocess
from typing import Optional, Tuple

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from rovi_bringup.launch_lib.args import (
    CAMERA_STACK_ARG_NAMES,
    CONTROL_STACK_ARG_NAMES,
    LOCALIZATION_STACK_ARG_NAMES,
    MAPPING_STACK_ARG_NAMES,
    NAV_STACK_ARG_NAMES,
    launch_config_map,
)
from rovi_bringup.launch_lib.camera_args import declare_camera_args
from rovi_bringup.launch_lib.includes import include_launch
from rovi_bringup.launch_lib.modes import CONTROL_STACKS, SESSION_STACKS, stack_equals, stack_in


def _publish_session_state(context, *args, **kwargs):  # noqa: ANN001
    del args, kwargs
    stack = LaunchConfiguration("stack").perform(context).strip()
    if stack not in SESSION_STACKS:
        return []

    launch_ref = f"rovi_bringup/{stack}.launch.py"
    return [
        Node(
            package="rovi_bringup",
            executable="rovi_session_state_pub",
            name="rovi_session_state_pub",
            output="screen",
            parameters=[
                {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)},
                {"launch_ref": launch_ref},
            ],
        )
    ]


def _systemd_unit_active(unit: str) -> bool:
    systemctl = shutil.which("systemctl")
    if not systemctl:
        return False
    try:
        proc = subprocess.run(
            [systemctl, "is-active", "--quiet", unit],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        return proc.returncode == 0
    except Exception:
        return False


def _parse_host_port(bind: str) -> Optional[Tuple[str, int]]:
    s = str(bind or "").strip()
    if not s:
        return None
    # Common formats:
    # - "0.0.0.0:50051"
    # - ":50051" (rare)
    # - "localhost:50051"
    if ":" not in s:
        return None
    host, port_s = s.rsplit(":", 1)
    host = host.strip() or "0.0.0.0"
    try:
        port = int(port_s.strip())
    except ValueError:
        return None
    if port <= 0 or port > 65535:
        return None
    return host, port


def _ui_bridge_bind_from_config(path: str) -> Optional[str]:
    p = str(path or "").strip()
    if not p or not os.path.exists(p):
        return None
    try:
        import yaml  # type: ignore
    except ModuleNotFoundError:
        return None
    try:
        with open(p, "r", encoding="utf-8") as f:
            doc = yaml.safe_load(f) or {}
        grpc = doc.get("grpc") if isinstance(doc, dict) else None
        bind = grpc.get("bind") if isinstance(grpc, dict) else None
        bind_s = str(bind).strip() if bind is not None else ""
        return bind_s or None
    except Exception:
        return None


def _tcp_port_in_use(port: int) -> bool:
    """Best-effort check if a TCP port is already bound locally."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(("0.0.0.0", int(port)))
            return False
        finally:
            s.close()
    except OSError:
        return True


def _maybe_auto_disable_gateway(context, *args, **kwargs):  # noqa: ANN001
    del args, kwargs

    auto_disable = LaunchConfiguration("gateway_auto_disable").perform(context).strip().lower()
    if auto_disable not in {"1", "true", "yes", "on"}:
        return []

    gateway_enabled = LaunchConfiguration("gateway_enabled").perform(context).strip().lower()
    robot_mode = LaunchConfiguration("robot_mode").perform(context).strip()
    if robot_mode != "real":
        return []

    if gateway_enabled not in {"1", "true", "yes", "on"}:
        return [
            LogInfo(
                msg=(
                    "[rovi_bringup] gateway_enabled:=false; expecting the gateway plane to already be running "
                    "(systemd rovi-gateway.service or an external gateway.launch.py)."
                )
            )
        ]

    ui_bind = _ui_bridge_bind_from_config(LaunchConfiguration("ui_bridge_config").perform(context))
    host_port = _parse_host_port(ui_bind) if ui_bind else None
    grpc_port = host_port[1] if host_port else 50051

    if _tcp_port_in_use(grpc_port):
        return [
            LogInfo(
                msg=(
                    f"[rovi_bringup] Detected an existing gRPC listener on port {grpc_port}; "
                    "auto-disabling gateway_enabled to avoid starting a second gateway plane. "
                    "(Override with: gateway_auto_disable:=false)"
                )
            ),
            SetLaunchConfiguration("gateway_enabled", "false"),
        ]
    if _systemd_unit_active("rovi-gateway.service"):
        return [
            LogInfo(
                msg=(
                    "[rovi_bringup] rovi-gateway.service is active; auto-disabling "
                    "gateway_enabled to avoid starting a second gateway plane. "
                    "(Override with: gateway_auto_disable:=false)"
                )
            ),
            SetLaunchConfiguration("gateway_enabled", "false"),
        ]
    return [
        LogInfo(
            msg=(
                "[rovi_bringup] gateway_enabled:=true and no existing gateway detected; "
                "starting gateway plane in this launch."
            )
        )
    ]


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory('rovi_bringup')
    desc_share = get_package_share_directory('rovi_description')
    sim_share = get_package_share_directory('rovi_sim')
    ui_bridge_share = get_package_share_directory('ros_ui_bridge')
    serial_display_share = get_package_share_directory('robot_serial_display')

    gateway_launch = os.path.join(bringup_share, 'launch', 'gateway.launch.py')
    teleop_stack_launch = os.path.join(bringup_share, 'launch', 'teleop.launch.py')
    camera_stack_launch = os.path.join(bringup_share, 'launch', 'camera.launch.py')
    mapping_stack_launch = os.path.join(bringup_share, 'launch', 'mapping.launch.py')
    localization_stack_launch = os.path.join(bringup_share, 'launch', 'localization.launch.py')
    nav_stack_launch = os.path.join(bringup_share, 'launch', 'nav.launch.py')

    default_model = os.path.join(desc_share, 'urdf', 'rovi.urdf')
    default_rviz_map = os.path.join(desc_share, 'rviz', 'rovi_map.rviz')
    default_rviz_nav = os.path.join(desc_share, 'rviz', 'rovi_nav.rviz')
    default_rviz_odom = os.path.join(desc_share, 'rviz', 'rovi_odom.rviz')
    default_rviz_camera = os.path.join(desc_share, 'rviz', 'rovi_camera.rviz')
    default_world = os.path.join(sim_share, 'worlds', 'rovi_room.sdf')
    default_ui_bridge_config = os.path.join(ui_bridge_share, 'config', 'default.yaml')
    default_serial_display_config = os.path.join(serial_display_share, 'config', 'default.yaml')

    robot_mode = DeclareLaunchArgument(
        'robot_mode',
        default_value='real',
        description="Robot backend: 'real' (hardware), 'sim' (Gazebo), or 'offline' (model only).",
    )
    gateway_enabled = DeclareLaunchArgument(
        'gateway_enabled',
        default_value='true',
        description='Start the always-on gateway plane (backend + UI bridge + display). Set false when systemd owns the gateway.',
    )
    gateway_auto_disable = DeclareLaunchArgument(
        'gateway_auto_disable',
        default_value='true',
        description='Auto-set gateway_enabled:=false when rovi-gateway.service is active (robot_mode=real).',
    )
    stack = DeclareLaunchArgument(
        'stack',
        default_value='teleop',
        description="Stack to run: 'teleop', 'camera', 'mapping', 'localization', 'nav', 'offline', or 'bringup'.",
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
    ui_bridge_config = DeclareLaunchArgument(
        'ui_bridge_config',
        default_value=default_ui_bridge_config,
        description='Path to ros_ui_bridge YAML config.',
    )
    ui_bridge_log_level = DeclareLaunchArgument(
        'ui_bridge_log_level',
        default_value='info',
        description='ROS log level for ros_ui_bridge (debug/info/warn/error).',
    )
    serial_display_config = DeclareLaunchArgument(
        'serial_display_config',
        default_value=default_serial_display_config,
        description='Path to robot_serial_display YAML config.',
    )
    serial_display_log_level = DeclareLaunchArgument(
        'serial_display_log_level',
        default_value='info',
        description='ROS log level for robot_serial_display (debug/info/warn/error).',
    )
    serial_display_debug = DeclareLaunchArgument(
        'serial_display_debug',
        default_value='false',
        description='Enable verbose robot_serial_display payload logging.',
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
    camera_enabled = DeclareLaunchArgument(
        'camera_enabled',
        default_value='true',
        description='Enable optional camera pipeline for mapping/localization/nav stacks (must degrade gracefully if camera is missing).',
    )
    camera_topology_enabled = DeclareLaunchArgument(
        'camera_topology_enabled',
        default_value='false',
        description='Enable floor topology visualization (/floor/topology) when camera pipeline is enabled.',
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

    # Camera stack args (passed through when stack:=camera).
    camera_args = declare_camera_args()

    # Odometry coordination between backend and localization stack.
    odom_mode = DeclareLaunchArgument(
        'odom_mode',
        default_value='filtered',
        description="Odometry mode: 'raw', 'filtered', or 'fusion_wheels_imu'.",
    )

    # When this launch owns the gateway plane (gateway_enabled:=true), it must also decide
    # who publishes TF odom->base_footprint to avoid conflicts with EKF stacks.
    # In systemd mode, the gateway service is configured separately (and may require a restart to change).
    odom_integrator_publish_tf = PythonExpression([
        "'true' if '",
        LaunchConfiguration('stack'),
        "' in ['teleop','camera'] else ('true' if '",
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
        "' in ['mapping','localization'] else ('",
        default_rviz_camera,
        "' if '",
        LaunchConfiguration('stack'),
        "' == 'camera' else '",
        default_rviz_odom,
        "')))",
    ])

    # Gateway plane: backend + UI bridge + display.
    # In systemd mode, rovi-gateway.service owns this and stacks launch with gateway_enabled:=false.
    gateway = include_launch(
        gateway_launch,
        condition=IfCondition(LaunchConfiguration('gateway_enabled')),
        launch_arguments={
            'robot_mode': LaunchConfiguration('robot_mode'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'model': LaunchConfiguration('model'),
            'world': LaunchConfiguration('world'),
            'gazebo_gui': LaunchConfiguration('gazebo_gui'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'odom_integrator_publish_tf': odom_integrator_publish_tf,
            'ui_bridge_config': LaunchConfiguration('ui_bridge_config'),
            'ui_bridge_log_level': LaunchConfiguration('ui_bridge_log_level'),
            'serial_display_config': LaunchConfiguration('serial_display_config'),
            'serial_display_log_level': LaunchConfiguration('serial_display_log_level'),
            'serial_display_debug': LaunchConfiguration('serial_display_debug'),
        },
    )

    # Twist muxing (needed for teleop/mapping/nav; joy can be disabled).
    control_stack = include_launch(
        teleop_stack_launch,
        condition=stack_in(CONTROL_STACKS),
        launch_arguments=launch_config_map(CONTROL_STACK_ARG_NAMES),
    )

    stack_camera = include_launch(
        camera_stack_launch,
        condition=stack_equals('camera'),
        launch_arguments=launch_config_map(CAMERA_STACK_ARG_NAMES),
    )

    # Stacks (headless; RViz is owned by this top-level file).
    stack_mapping = include_launch(
        mapping_stack_launch,
        condition=stack_equals('mapping'),
        launch_arguments=launch_config_map(MAPPING_STACK_ARG_NAMES),
    )
    stack_localization = include_launch(
        localization_stack_launch,
        condition=stack_equals('localization'),
        launch_arguments=launch_config_map(LOCALIZATION_STACK_ARG_NAMES),
    )
    stack_nav = include_launch(
        nav_stack_launch,
        condition=stack_equals('nav'),
        launch_arguments=launch_config_map(NAV_STACK_ARG_NAMES),
    )

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', resolved_rviz_config],
        output='screen',
        parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
    )

    return LaunchDescription([
        robot_mode,
        gateway_enabled,
        gateway_auto_disable,
        stack,
        use_sim_time,
        rviz,
        rviz_config,
        OpaqueFunction(function=_maybe_auto_disable_gateway),
        OpaqueFunction(function=_publish_session_state),
        joy_enabled,
        cmd_vel_topic,
        ui_bridge_config,
        ui_bridge_log_level,
        serial_display_config,
        serial_display_log_level,
        serial_display_debug,
        model,
        world,
        gazebo_gui,
        slam_enabled,
        camera_enabled,
        camera_topology_enabled,
        slam_mode,
        map_file_name,
        mag_enabled,
        *camera_args,
        odom_mode,
        gateway,
        control_stack,
        stack_camera,
        stack_mapping,
        stack_localization,
        stack_nav,
        rviz_node,
    ])
