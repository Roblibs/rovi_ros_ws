#!/usr/bin/env python3
"""Camera stack: depth (OpenNI2) + RGB (UVC) + TF mount.

This launch is intended to be composed "on top of teleop" via rovi.launch.py:
- rovi.launch.py owns starting the control stack (twist_mux + optional joystick)
- this file owns starting camera drivers

Current policy:
- Publish depth and RGB as separate feeds (no registered RGB-D pointcloud).
- Publish stable frame IDs; fixed camera TF comes from the robot URDF via robot_state_publisher.
- Publish `CameraInfo` on low-rate, transient-local topics from reference YAML files.
"""

import glob
import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from rovi_bringup.launch_lib.camera_args import declare_camera_args


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
    robot_mode = LaunchConfiguration("robot_mode").perform(context).strip()
    if robot_mode != "real":
        return []
    requested = LaunchConfiguration("rgb_video_device").perform(context).strip()
    resolved = requested or _pick_default_rgb_device()
    context.launch_configurations["rgb_video_device_resolved"] = resolved
    return [LogInfo(msg=f"[camera] rgb_video_device: {resolved}")]


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("rovi_bringup")
    default_color_info = os.path.join(bringup_share, "config", "camera_info", "color.yaml")
    default_depth_info = os.path.join(bringup_share, "config", "camera_info", "depth.yaml")

    robot_mode_arg = DeclareLaunchArgument(
        "robot_mode",
        default_value="real",
        description="Robot backend: 'real', 'sim', or 'offline'. Cameras only start in robot_mode=real.",
    )
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false", description="Use /clock time.")

    color_info_arg = DeclareLaunchArgument(
        "color_camera_info_yaml",
        default_value=default_color_info,
        description="Path to camera_info_manager-style YAML for RGB camera model.",
    )
    depth_info_arg = DeclareLaunchArgument(
        "depth_camera_info_yaml",
        default_value=default_depth_info,
        description="Path to camera_info_manager-style YAML for depth camera model.",
    )
    camera_info_period_arg = DeclareLaunchArgument(
        "camera_info_publish_period_s",
        default_value="0.0",
        description="Republish camera_info every N seconds (0 = publish once, transient-local).",
    )

    camera_args = declare_camera_args()

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
            ("depth_raw/camera_info", "depth/camera_info_raw"),
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

    # Color driver (publishes /camera/color/image; driver camera_info is published as /camera/color/camera_info_raw)
    rgb_node = Node(
        condition=is_real,
        package="v4l2_camera",
        executable="v4l2_camera_node",
        namespace="camera/color",
        name="v4l2_camera",
        output="screen",
        remappings=[
            ("image_raw", "image"),
            ("camera_info", "camera_info_raw"),
        ],
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
            # v4l2_camera in Jazzy does not support MJPG (Motion-JPEG) for this camera;
            # enforce YUYV to avoid slow/failed conversion paths and driver crashes.
            {"pixel_format": "YUYV"},
            {"output_encoding": "rgb8"},
            {"camera_frame_id": "camera_color_optical_frame"},
        ],
    )

    camera_info_pub = Node(
        condition=is_real,
        package="rovi_bringup",
        executable="rovi_camera_info_pub",
        name="camera_info_pub",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time_param},
            {"color_yaml": LaunchConfiguration("color_camera_info_yaml")},
            {"depth_yaml": LaunchConfiguration("depth_camera_info_yaml")},
            {"publish_period_s": ParameterValue(LaunchConfiguration("camera_info_publish_period_s"), value_type=float)},
            {"color_frame_id": "camera_color_optical_frame"},
            {"depth_frame_id": "camera_depth_optical_frame"},
            {"color_topic": "/camera/color/camera_info"},
            {"depth_topic": "/camera/depth/camera_info"},
        ],
    )

    return LaunchDescription([
        robot_mode_arg,
        use_sim_time_arg,
        color_info_arg,
        depth_info_arg,
        camera_info_period_arg,
        *camera_args,
        resolve_rgb,
        depth_node,
        rgb_node,
        camera_info_pub,
    ])
