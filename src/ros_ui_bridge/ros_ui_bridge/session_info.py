from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path


def _ros_home() -> Path:
    return Path(os.environ.get("ROS_HOME", Path.home() / ".ros")).expanduser()


def _current_launch_path() -> Path:
    return _ros_home() / "rovi" / "session" / "current_launch"


def read_current_launch_ref() -> str | None:
    path = _current_launch_path()
    if not path.is_file():
        return None
    lines = path.read_text(encoding="utf-8").splitlines()
    if not lines:
        return None
    value = lines[0].strip()
    return value or None


def stack_from_launch_ref(launch_ref: str | None) -> str | None:
    if not launch_ref:
        return None

    value = str(launch_ref).strip()
    if not value:
        return None

    if "/" in value:
        _package, launch_file = value.split("/", 1)
        base = Path(launch_file).name
        key = base.removesuffix(".launch.py").removesuffix(".py")
        return key or None

    return value


def fixed_frame_from_stack(stack: str | None) -> str:
    # Deterministic policy (no TF guessing):
    # - teleop/offline: odom
    # - mapping/localization/nav: map
    # Default: odom
    key = (stack or "").strip().lower()
    if key in {"mapping", "localization", "nav"}:
        return "map"
    if key in {"teleop", "offline", "offline_view"}:
        return "odom"
    return "odom"


@dataclass(frozen=True)
class ResolvedSession:
    current_launch_ref: str | None
    stack: str | None
    fixed_frame: str


def resolve_session() -> ResolvedSession:
    current_launch_ref = read_current_launch_ref()
    stack = stack_from_launch_ref(current_launch_ref)
    fixed_frame = fixed_frame_from_stack(stack)
    return ResolvedSession(current_launch_ref=current_launch_ref, stack=stack, fixed_frame=fixed_frame)
