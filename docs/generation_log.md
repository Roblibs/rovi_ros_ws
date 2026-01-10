# 2026-01-10

## Venv-first build/runtime policy

Standardized the workspace to build and run Python ROS nodes with the `.venv` interpreter to keep runtime and build dependencies consistent (e.g., `grpc`, `catkin_pkg`, `colcon`), and avoid system Python drift.

**Key changes:**
- `rovi_env.sh` activates the venv for `ws`/`build` and uses venv `colcon`.
- `pyproject.toml` includes `colcon-common-extensions` and `catkin_pkg` for venv builds.
- Docs updated to reflect venv-first build/run flow.

# 2026-01-02

## Status stream unification (fields + metadata, ROS-time staleness)

Reworked `ros_ui_bridge` status into a single fields model with metadata (unit/min/max/target) and explicit `GetStatus`/`StreamStatus` RPCs. Status now uses ROS time stamps, drops stale fields in the bridge (no client-side buffering), and merges voltage/CPU/rate metrics into configurable sources (`system`, `ros.topic_value`, `ros.topic_rate`, `ros.tf_rate`).

**Key changes:**
- New proto shape: `GetStatus` returns schema + latest non-stale values; `StreamStatus` is values-only, event-driven.
- Config reshaped to `streams.status.fields` with per-field staleness/downsample; removed legacy `voltage_topic`/rates blocks.
- ROS metrics node rewritten to collect values/rates/TF rates from config, enforce staleness with ROS time, and keep TF demux.

# 2026-01-01

## Lidar gRPC + Capped Downsampling Refactor

Added lidar streaming to the gRPC interface (`StreamLidar`) and refactored the architecture to use "capped downsampling" — data is forwarded on arrival if rate cap allows, never duplicating stale data. Merged the separate `viz_downsample` node into `ui_bridge`; lidar throttle now republishes to `/viz/scan` for RViz.

**Key changes:**
- New `ThrottledForwarder` utility for rate-capped passthrough
- Queue-based gRPC streaming (`AsyncStreamBroadcaster`) — no caching, event-driven
- Single lidar node handles both gRPC and ROS `/viz/scan` republish
- Removed: `viz_downsample_node.py`, `lidar_store.py`, `robot_state_store.py`
- Config now supports both `rate_hz` and `period_s`
