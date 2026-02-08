# Refactor: Replace file-based session coupling with explicit runtime session state

## Problem
Session data is persisted via `~/.ros/rovi/session/current_launch` and consumed by multiple components.

## Rework
Publish session info on a ROS topic (latched) or a tiny service (`/rovi/session_info`) and keep file fallback for compatibility.

## Independent scope
- `src/rovi_bringup/rovi_bringup/cli/session.py`
- `src/rovi_bringup/rovi_bringup/cli/bag.py`
- `src/ros_ui_bridge/ros_ui_bridge/session_info.py`
- `src/rovi_bringup/launch/rovi.launch.py`

## Value
- Complexity reduction: medium-high (removes hidden global state).
- Technical debt reduction: high (clear interface between launch, bagging, and UI).
