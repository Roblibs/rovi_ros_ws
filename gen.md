# 2026-01-18

## USB device identity + udev mapping
A lesson learned with CH340.

We hit intermittent failures because the Rosmaster control board and RPLidar both enumerate as identical CH340 USB-serial devices (`1a86:7523`), so `/dev/serial/by-id` is not reliable (no unique serial; “last one wins”). The fix is to avoid `/dev/ttyUSB*` and instead install stable symlinks using `ENV{ID_PATH}` (physical port path) plus an active probe to decide which CH340 is the Rosmaster.

**Design guidance:**
- Prefer stable names in the stack: `/dev/robot_control`, `/dev/robot_lidar`, `/dev/robot_display`.
- Use a single installer (`tools/rovi_usb_setup.py`) that in case of duplicated ids, requires all devices attached, to identify them in install time e.g. via `get_version()` (expects `3.5`), and writes udev rules keyed by `ID_PATH`.
- Manual debug are available for override (`ROVI_ROSMASTER_PORT`, `ROVI_LIDAR_PORT`, `ROVI_DISPLAY_PORT`), but treat them as temporary; clear them to avoid confusion.

## Ctrl-C shutdown idempotency (rclpy + launch)

ROS 2 `launch` propagates SIGINT (Ctrl-C) to all node processes. For `rclpy` nodes, shutdown may be initiated by rclpy’s signal handler and/or by application code (e.g., a `finally:` block). This means calling `rclpy.shutdown()` unconditionally can raise `RCLError: rcl_shutdown already called` and cause a non-zero exit during otherwise-normal bringup teardown.

**Design guidance:**
- Treat shutdown as **idempotent**: prefer `rclpy.try_shutdown()` (or guard with `rclpy.ok()`).
- Catch `ExternalShutdownException` alongside `KeyboardInterrupt` when spinning/executing.
- Keep teardown robust: wrap `destroy_node()` in a best-effort `try/except` so signal timing doesn’t turn into “process died” noise.

**Applied fix:**
- `rosmaster_driver` now uses `rclpy.try_shutdown()` and handles external shutdown to avoid exit code 1 on Ctrl-C.

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
