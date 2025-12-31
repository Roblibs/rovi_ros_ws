#!/usr/bin/env python3
"""Launch EKF + IMU orientation filter for rovi."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _read_log_levels(path: str) -> dict[str, str]:
    """Reads a simple YAML mapping of `node_name: level` (no nesting)."""
    try:
        lines = open(path, encoding='utf-8').read().splitlines()
    except OSError:
        return {}

    out: dict[str, str] = {}
    for line in lines:
        stripped = line.strip()
        if not stripped or stripped.startswith('#'):
            continue
        if ':' not in stripped:
            continue
        key, value = stripped.split(':', 1)
        key = key.strip().strip('"').strip("'")
        value = value.strip().strip('"').strip("'")
        if key and value:
            out[key] = value
    return out


def _ros_log_level_args(level: str | None) -> list[str]:
    if not level:
        return []
    return ['--ros-args', '--log-level', level]


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('rovi_localization')
    ekf_odom_params = os.path.join(pkg_share, 'config', 'ekf_odom.yaml')
    ekf_odom_imu_params = os.path.join(pkg_share, 'config', 'ekf_odom_imu.yaml')
    imu_filter_params = os.path.join(pkg_share, 'config', 'imu_filter_madgwick.yaml')
    log_levels_path = os.path.join(pkg_share, 'config', 'log_levels.yaml')
    log_levels = _read_log_levels(log_levels_path)

    odom_mode = DeclareLaunchArgument(
        'odom_mode',
        default_value='filtered',
        description="Odometry mode: 'raw' (no nodes), 'filtered' (EKF wheel-only), 'fusion_wheels_imu' (EKF + IMU).",
    )
    mag_enabled = DeclareLaunchArgument(
        'mag_enabled',
        default_value='false',
        description='Enable magnetometer usage in IMU orientation filter.',
    )
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use /clock time.',
    )

    imu_filter_node = Node(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('odom_mode'), "' == 'fusion_wheels_imu'"])),
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        arguments=_ros_log_level_args(log_levels.get('imu_filter')),
        parameters=[
            imu_filter_params,
            {
                'use_mag': ParameterValue(LaunchConfiguration('mag_enabled'), value_type=bool),
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            },
        ],
    )

    ekf_condition_odom = IfCondition(PythonExpression(["'", LaunchConfiguration('odom_mode'), "' == 'filtered'"]))
    ekf_condition_imu = IfCondition(PythonExpression(["'", LaunchConfiguration('odom_mode'), "' == 'fusion_wheels_imu'"]))

    ekf_node_odom = Node(
        condition=ekf_condition_odom,
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_odom_params, {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
    )

    ekf_node_odom_imu = Node(
        condition=ekf_condition_imu,
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_odom_imu_params, {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
    )

    return LaunchDescription([
        odom_mode,
        mag_enabled,
        use_sim_time,
        imu_filter_node,
        ekf_node_odom,
        ekf_node_odom_imu,
    ])
