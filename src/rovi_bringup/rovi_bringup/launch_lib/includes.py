"""Helpers for IncludeLaunchDescription wiring."""

from __future__ import annotations

from typing import Mapping

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include_launch(
    launch_file: str,
    *,
    launch_arguments: Mapping[str, LaunchConfiguration],
    condition=None,
) -> IncludeLaunchDescription:
    """Create IncludeLaunchDescription with optional condition and argument mapping."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        condition=condition,
        launch_arguments=launch_arguments.items(),
    )
