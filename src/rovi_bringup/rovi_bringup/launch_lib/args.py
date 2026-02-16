"""Argument groups used by top-level launch composition."""

from __future__ import annotations

from typing import Iterable

from launch.substitutions import LaunchConfiguration

# Backend argument pass-through from higher-level launches -> robot_bringup.launch.py
BACKEND_ARG_NAMES: tuple[str, ...] = (
    "robot_mode",
    "use_sim_time",
    "model",
    "world",
    "gazebo_gui",
    "cmd_vel_topic",
)

# Gateway argument pass-through from rovi.launch.py -> gateway.launch.py
GATEWAY_ARG_NAMES: tuple[str, ...] = (
    "robot_mode",
    "use_sim_time",
    "model",
    "world",
    "gazebo_gui",
    "cmd_vel_topic",
    "odom_integrator_publish_tf",
    "ui_bridge_enabled",
    "ui_bridge_config",
    "ui_bridge_log_level",
    "serial_display_enabled",
    "serial_display_config",
    "serial_display_log_level",
    "serial_display_debug",
)

# Control stack args from rovi.launch.py -> teleop.launch.py
CONTROL_STACK_ARG_NAMES: tuple[str, ...] = (
    "use_sim_time",
    "joy_enabled",
    "cmd_vel_topic",
)

# Camera stack args from rovi.launch.py -> camera.launch.py
CAMERA_STACK_ARG_NAMES: tuple[str, ...] = (
    "robot_mode",
    "use_sim_time",
    "device_id",
    "depth_mode",
    "rgb_video_device",
    "rgb_width",
    "rgb_height",
)

# Mapping stack args from rovi.launch.py -> mapping.launch.py
MAPPING_STACK_ARG_NAMES: tuple[str, ...] = (
    "robot_mode",
    "use_sim_time",
    "odom_mode",
    "slam_enabled",
    "mag_enabled",
    "camera_enabled",
    "camera_topology_enabled",
)

# Localization stack args from rovi.launch.py -> localization.launch.py
LOCALIZATION_STACK_ARG_NAMES: tuple[str, ...] = (
    "robot_mode",
    "use_sim_time",
    "odom_mode",
    "slam_enabled",
    "mag_enabled",
    "map_file_name",
    "camera_enabled",
    "camera_topology_enabled",
)

# Nav stack args from rovi.launch.py -> nav.launch.py
NAV_STACK_ARG_NAMES: tuple[str, ...] = (
    "robot_mode",
    "use_sim_time",
    "odom_mode",
    "slam_mode",
    "map_file_name",
    "mag_enabled",
    "camera_enabled",
    "camera_topology_enabled",
)


def launch_config_map(names: Iterable[str]) -> dict[str, LaunchConfiguration]:
    """Create `{arg_name: LaunchConfiguration(arg_name)}` for include launch arguments."""
    return {name: LaunchConfiguration(name) for name in names}
