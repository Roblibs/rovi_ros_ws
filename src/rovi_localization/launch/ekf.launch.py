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


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('rovi_localization')
    ekf_odom_params = os.path.join(pkg_share, 'config', 'ekf_odom.yaml')
    ekf_odom_imu_params = os.path.join(pkg_share, 'config', 'ekf_odom_imu.yaml')
    imu_filter_params = os.path.join(pkg_share, 'config', 'imu_filter_madgwick.yaml')

    odom_mode = DeclareLaunchArgument(
        'odom_mode',
        default_value='fusion_wheels_imu',
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
