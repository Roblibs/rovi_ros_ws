#!/usr/bin/env python3
"""Gateway entrypoint for ROVI (always-on plane).

This launch is intended to be owned by `rovi-gateway.service` on the robot.
It starts:
- Robot backend contract (drivers + TF + /odom_raw, etc.) via `robot_bringup.launch.py`
- `ros_ui_bridge` (gRPC UI gateway + observability)
- `robot_serial_display` (robot-only display client)

Stacks (teleop/mapping/localization/nav/camera) are expected to be started/stopped separately.
"""

from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from rovi_bringup.launch_lib.args import BACKEND_ARG_NAMES, launch_config_map
from rovi_bringup.launch_lib.includes import include_launch


def _truthy(value: str | None) -> bool:
    return str(value or "").strip().lower() in {"1", "true", "yes", "on"}


def _default_odom_integrator_publish_tf() -> str:
    # Default to publishing TF from the raw odom integrator (matches current teleop-first behavior).
    # For EKF-published TF (mapping/localization/nav stacks), restart the gateway with:
    #   ROVI_ODOM_INTEGRATOR_PUBLISH_TF=0
    env = os.environ.get("ROVI_ODOM_INTEGRATOR_PUBLISH_TF")
    if env is None:
        return "true"
    return "true" if _truthy(env) else "false"


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("rovi_bringup")
    desc_share = get_package_share_directory("rovi_description")
    sim_share = get_package_share_directory("rovi_sim")
    ui_bridge_share = get_package_share_directory("ros_ui_bridge")
    serial_display_share = get_package_share_directory("robot_serial_display")

    robot_bringup_launch = str(Path(bringup_share) / "launch" / "robot_bringup.launch.py")

    default_model = str(Path(desc_share) / "urdf" / "rovi.urdf")
    default_world = str(Path(sim_share) / "worlds" / "rovi_room.sdf")
    default_ui_bridge_config = str(Path(ui_bridge_share) / "config" / "default.yaml")
    default_serial_display_config = str(Path(serial_display_share) / "config" / "default.yaml")

    robot_mode = DeclareLaunchArgument(
        "robot_mode",
        default_value="real",
        description="Robot backend: 'real' (hardware), 'sim' (Gazebo), or 'offline' (model only).",
    )
    model = DeclareLaunchArgument(
        "model",
        default_value=default_model,
        description="Absolute path to the robot URDF.",
    )
    world = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Full path to the Gazebo world SDF file (robot_mode=sim).",
    )
    gazebo_gui = DeclareLaunchArgument(
        "gazebo_gui",
        default_value="true",
        description="Start Gazebo GUI client (server always starts) (robot_mode=sim).",
    )
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value=PythonExpression([
            "'true' if '",
            LaunchConfiguration("robot_mode"),
            "' == 'sim' else 'false'",
        ]),
        description="Use /clock time (auto true for robot_mode=sim).",
    )
    cmd_vel_topic = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="cmd_vel",
        description="Final /cmd_vel topic consumed by the robot backend.",
    )
    odom_integrator_publish_tf = DeclareLaunchArgument(
        "odom_integrator_publish_tf",
        default_value=_default_odom_integrator_publish_tf(),
        description="Publish TF odom->base_footprint from the raw odom integrator (real) / sim odom bridge (sim).",
    )

    ui_bridge_enabled = DeclareLaunchArgument(
        "ui_bridge_enabled",
        default_value="true",
        description="Start ros_ui_bridge gRPC server.",
    )
    ui_bridge_config = DeclareLaunchArgument(
        "ui_bridge_config",
        default_value=default_ui_bridge_config,
        description="Path to ros_ui_bridge YAML config.",
    )
    ui_bridge_log_level = DeclareLaunchArgument(
        "ui_bridge_log_level",
        default_value="info",
        description="ROS log level for ros_ui_bridge (debug/info/warn/error).",
    )

    serial_display_enabled = DeclareLaunchArgument(
        "serial_display_enabled",
        default_value=PythonExpression([
            "'true' if '",
            LaunchConfiguration("robot_mode"),
            "' == 'real' else 'false'",
        ]),
        description="Start robot_serial_display (gRPC client -> USB serial display).",
    )
    serial_display_config = DeclareLaunchArgument(
        "serial_display_config",
        default_value=default_serial_display_config,
        description="Path to robot_serial_display YAML config.",
    )
    serial_display_log_level = DeclareLaunchArgument(
        "serial_display_log_level",
        default_value="info",
        description="ROS log level for robot_serial_display (debug/info/warn/error).",
    )
    serial_display_debug = DeclareLaunchArgument(
        "serial_display_debug",
        default_value="false",
        description="Enable verbose robot_serial_display payload logging.",
    )

    use_sim_time_param = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    # Backend contract (drivers, base, TF, /odom_raw, /scan, etc.).
    backend_args = launch_config_map(BACKEND_ARG_NAMES)
    backend_args["model"] = LaunchConfiguration("model")
    backend_args["cmd_vel_topic"] = LaunchConfiguration("cmd_vel_topic")
    backend_args["odom_integrator_publish_tf"] = LaunchConfiguration("odom_integrator_publish_tf")
    backend = include_launch(robot_bringup_launch, launch_arguments=backend_args)

    ui_bridge_node = Node(
        condition=IfCondition(LaunchConfiguration("ui_bridge_enabled")),
        package="ros_ui_bridge",
        executable="ui_bridge",
        output="screen",
        arguments=["--config", LaunchConfiguration("ui_bridge_config")],
        parameters=[{"use_sim_time": use_sim_time_param}],
        ros_arguments=["--ros-args", "--log-level", LaunchConfiguration("ui_bridge_log_level")],
    )

    serial_display_node = Node(
        condition=IfCondition(PythonExpression([
            "'",
            LaunchConfiguration("serial_display_enabled"),
            "' == 'true' and '",
            LaunchConfiguration("serial_display_debug"),
            "' != 'true'",
        ])),
        package="robot_serial_display",
        executable="serial_display",
        name="robot_serial_display",
        output="screen",
        arguments=["--config", LaunchConfiguration("serial_display_config")],
        ros_arguments=["--ros-args", "--log-level", LaunchConfiguration("serial_display_log_level")],
    )

    serial_display_debug_node = Node(
        condition=IfCondition(PythonExpression([
            "'",
            LaunchConfiguration("serial_display_enabled"),
            "' == 'true' and '",
            LaunchConfiguration("serial_display_debug"),
            "' == 'true'",
        ])),
        package="robot_serial_display",
        executable="serial_display",
        name="robot_serial_display",
        output="screen",
        arguments=["--config", LaunchConfiguration("serial_display_config"), "--debug"],
        ros_arguments=["--ros-args", "--log-level", LaunchConfiguration("serial_display_log_level")],
    )

    return LaunchDescription([
        robot_mode,
        model,
        world,
        gazebo_gui,
        use_sim_time,
        cmd_vel_topic,
        odom_integrator_publish_tf,
        ui_bridge_enabled,
        ui_bridge_config,
        ui_bridge_log_level,
        serial_display_enabled,
        serial_display_config,
        serial_display_log_level,
        serial_display_debug,
        backend,
        ui_bridge_node,
        serial_display_node,
        serial_display_debug_node,
    ])
