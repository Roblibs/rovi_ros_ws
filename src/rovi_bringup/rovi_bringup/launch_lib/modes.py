"""Mode and stack condition helpers for launch composition."""

from __future__ import annotations

from typing import Iterable

from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

CONTROL_STACKS: tuple[str, ...] = ("teleop", "camera", "mapping", "localization", "nav")
SESSION_STACKS: tuple[str, ...] = CONTROL_STACKS


def stack_equals(name: str) -> IfCondition:
    """Condition that matches when LaunchConfiguration('stack') == `name`."""
    return IfCondition(PythonExpression(["'", LaunchConfiguration("stack"), "' == '", name, "'"]))


def stack_in(names: Iterable[str]) -> IfCondition:
    """Condition that matches when LaunchConfiguration('stack') is in `names`."""
    quoted = ", ".join(f"'{name}'" for name in names)
    return IfCondition(PythonExpression(["'", LaunchConfiguration("stack"), f"' in [{quoted}]"]))

