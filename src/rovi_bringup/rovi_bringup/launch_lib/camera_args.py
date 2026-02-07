"""Shared camera argument specs and declaration helpers."""

from __future__ import annotations

from launch.actions import DeclareLaunchArgument

CAMERA_ARG_SPECS: tuple[tuple[str, str, str], ...] = (
    (
        "device_id",
        "#1",
        'OpenNI2 device selector (e.g. "#1" for first device, or URI from list_devices).',
    ),
    ("depth_mode", "ORBBEC_640x400_30Hz", "OpenNI2 depth/IR mode preset."),
    (
        "rgb_video_device",
        "",
        "RGB V4L2 device path. Prefer stable /dev/v4l/by-id/... when available.",
    ),
    ("rgb_width", "640", "RGB image width."),
    ("rgb_height", "480", "RGB image height."),
)


def declare_camera_args() -> list[DeclareLaunchArgument]:
    """Create standard camera-stack launch arguments."""
    return [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in CAMERA_ARG_SPECS
    ]
