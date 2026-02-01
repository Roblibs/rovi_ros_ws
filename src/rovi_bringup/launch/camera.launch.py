#!/usr/bin/env python3
"""Camera stack: depth (OpenNI2) + RGB (UVC) + TF mount.

This launch is intended to be composed "on top of teleop" via rovi.launch.py:
- rovi.launch.py owns starting the control stack (twist_mux + optional joystick)
- this file owns starting camera drivers

Current policy:
- Publish depth and RGB as separate feeds (no registered RGB-D pointcloud).
- Publish stable frame IDs; fixed camera TF comes from the robot URDF via robot_state_publisher.
"""

import glob
import os
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _pick_default_rgb_device() -> str:
    # Prefer stable /dev/v4l/by-id. This avoids index-based /dev/video0 surprises
    # once additional cameras (e.g. stereo) are added.
    candidates: List[str] = []
    for pattern in (
        "/dev/v4l/by-id/usb-Sonix_Technology*_video-index0",
        "/dev/v4l/by-id/usb-Sonix_Technology*_video-index1",
        "/dev/v4l/by-id/*-video-index0",
    ):
        candidates.extend(sorted(glob.glob(pattern)))
    for path in candidates:
        if os.path.exists(path):
            return path
    return "/dev/video0"


def _resolve_rgb_device(context, *args, **kwargs):  # noqa: ANN001
    del args, kwargs
    requested = LaunchConfiguration("rgb_video_device").perform(context).strip()
    resolved = requested or _pick_default_rgb_device()
    context.launch_configurations["rgb_video_device_resolved"] = resolved
    return [LogInfo(msg=f"[camera] rgb_video_device: {resolved}")]


def generate_launch_description() -> LaunchDescription:
    robot_mode_arg = DeclareLaunchArgument(
        "robot_mode",
        default_value="real",
        description="Robot backend: 'real', 'sim', or 'offline'. Cameras only start in robot_mode=real.",
    )
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false", description="Use /clock time.")

    # Depth (OpenNI2)
    device_id_arg = DeclareLaunchArgument(
        "device_id",
        default_value="#1",
        description=(
            'OpenNI2 device selector (e.g. "#1" for first device, or a URI from '
            "`ros2 run openni2_camera list_devices`)."
        ),
    )
    depth_mode_arg = DeclareLaunchArgument("depth_mode", default_value="ORBBEC_640x400_30Hz")

    # RGB (UVC / V4L2)
    rgb_video_device_arg = DeclareLaunchArgument(
        "rgb_video_device",
        default_value="",
        description=(
            "RGB V4L2 device. Prefer a stable /dev/v4l/by-id/... path; if empty, "
            "a best-effort default is selected."
        ),
    )
    rgb_width_arg = DeclareLaunchArgument("rgb_width", default_value="640")
    rgb_height_arg = DeclareLaunchArgument("rgb_height", default_value="480")
    rgb_pixel_format_arg = DeclareLaunchArgument(
        "rgb_pixel_format",
        default_value="YUYV",
        description="Pixel format for v4l2_camera (e.g. YUYV, UYVY, GREY).",
    )
    rgb_output_encoding_arg = DeclareLaunchArgument(
        "rgb_output_encoding",
        default_value="rgb8",
        description="Output encoding for v4l2_camera (e.g. rgb8, mono8).",
    )

    is_real = IfCondition(PythonExpression(["'", LaunchConfiguration("robot_mode"), "' == 'real'"]))
    use_sim_time_param = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    resolve_rgb = OpaqueFunction(function=_resolve_rgb_device)

    # Depth driver (publishes /camera/depth/image_raw, /camera/depth/image, etc.)
    depth_node = Node(
        condition=is_real,
        package="openni2_camera",
        executable="openni2_camera_driver",
        namespace="camera",
        name="openni2_camera",
        output="screen",
        remappings=[
            ("depth_raw/image", "depth/image_raw"),
            ("depth_raw/camera_info", "depth/camera_info"),
        ],
        parameters=[
            {"use_sim_time": use_sim_time_param},
            {"use_device_time": True},
            {"depth_registration": False},
            # NOTE: the OpenNI2 convention '#1' would be parsed as a YAML comment
            # unless we force it to a string at launch evaluation time.
            {"device_id": ParameterValue(LaunchConfiguration("device_id"), value_type=str)},
            {"depth_mode": LaunchConfiguration("depth_mode")},
            {"ir_mode": LaunchConfiguration("depth_mode")},
            {"enable_depth": True},
            {"enable_ir": False},
            {"enable_color": False},
            {"rgb_frame_id": "camera_color_optical_frame"},
            {"depth_frame_id": "camera_depth_optical_frame"},
            {"ir_frame_id": "camera_ir_optical_frame"},
        ],
    )

    # Color driver (publishes /camera/color/image_raw + /camera/color/camera_info)
    rgb_node = Node(
        condition=is_real,
        package="v4l2_camera",
        executable="v4l2_camera_node",
        namespace="camera/color",
        name="v4l2_camera",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time_param},
            {"video_device": LaunchConfiguration("rgb_video_device_resolved")},
            {
                "image_size": ParameterValue(
                    PythonExpression([
                        "'[' + '",
                        LaunchConfiguration("rgb_width"),
                        "' + ', ' + '",
                        LaunchConfiguration("rgb_height"),
                        "' + ']'",
                    ]),
                    value_type=List[int],
                )
            },
            {"pixel_format": LaunchConfiguration("rgb_pixel_format")},
            {"output_encoding": LaunchConfiguration("rgb_output_encoding")},
            {"camera_frame_id": "camera_color_optical_frame"},
        ],
    )

    return LaunchDescription([
        robot_mode_arg,
        use_sim_time_arg,
        device_id_arg,
        depth_mode_arg,
        rgb_video_device_arg,
        rgb_width_arg,
        rgb_height_arg,
        rgb_pixel_format_arg,
        rgb_output_encoding_arg,
        resolve_rgb,
        depth_node,
        rgb_node,
    ])
