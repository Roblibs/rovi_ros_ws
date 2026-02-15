from __future__ import annotations

SESSION_CURRENT_LAUNCH_REF_TOPIC = "/rovi/session/current_launch_ref"


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
