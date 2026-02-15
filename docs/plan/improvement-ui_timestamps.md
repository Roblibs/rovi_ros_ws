# Improvement: Consistent timestamp semantics in UI streams

## Problem
Mixed wall-time and ROS-time timestamping across UI streams makes time alignment brittle for consumers.

## Rework
Standardize to ROS/header time for all stream payloads; keep wall-time as optional debug metadata.

## Independent scope
- `src/ros_ui_bridge/ros_ui_bridge/map_node.py`
- Proto mapping layer

## Value
- Complexity reduction: medium (single time model for consumers).
- Technical debt reduction: medium-high (fewer time-sync bugs).
