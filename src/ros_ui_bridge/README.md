# ros_ui_bridge

ROS UI bridge: a small ROS 2 node that exposes robot telemetry and visualization data to UI clients over gRPC.

It is designed to be generic and reusable: it reads from ROS topics / TF and streams downsampled snapshots suitable for dashboards and web UIs.

## What it provides

| Capability | gRPC RPC | Notes |
| --- | --- | --- |
| Status fields (values + Hz) with metadata | `GetStatus` (single) / `StreamStatus` | Fields carry unit/min/max/target from config. Stream omits metadata and is event-driven (no empty spam). |
| Robot pose + wheel joint angles (for Three.js / 3D rendering) | `StreamRobotState` | Emits a single fixed-frame `pose` chosen by the bridge from the current launch session (teleop=odom, mapping/localization/nav=map). |
| Downsampled lidar scans for 3D visualization | `StreamLidar` | Low-rate (2 Hz default) LaserScan stream; optional. |
| Occupancy grid map (`/map`) as an encoded image | `StreamMap` | Grayscale PNG stream (0=occupied, 255=free, 127=unknown); optional. |
| Robot model metadata (cache key / sha256, size) | `GetRobotModelMeta` | Use this as an ETag-like key to decide whether to refresh cached assets. |
| Robot model asset (GLB / binary glTF) | `GetRobotModel` | Chunked stream to avoid gRPC message size limits. |

Proto: `proto/ui_bridge.proto` (`package roblibs.ui_bridge.v1`)

## ROS inputs (high level)

- Status fields are configurable: CPU (`system` provider), ROS topic values (`topic_value`), topic Hz (`topic_rate`), and TF Hz (`tf_rate`). Staleness is enforced in the bridge using ROS time; stale fields simply disappear from the stream.
- Status fields support float values (default) and optional text values: text fields set `StatusFieldMeta.type=TEXT` and populate `StatusFieldValue.text`.
- Robot pose is derived from `/odom_raw` (`nav_msgs/Odometry`) and optionally projected into `map` using the `map->odom` TF transform.
- Wheel angles are read from `/joint_states` (`sensor_msgs/JointState`) using the configured wheel joint names.
- Lidar scans are read from `/scan` (`sensor_msgs/LaserScan`) and downsampled to the configured rate.

### Timestamp note (important for UI rendering)

- **Status** uses ROS time: field samples prefer message header stamps; otherwise the bridge's ROS clock. The snapshot stamp is also ROS time. A `wall_time_unix_ms` helper is included for debugging.
- **RobotState / Lidar / Map** currently expose `timestamp_unix_ms` fields; treat them as a consistent monotonic-ish timebase (sim time when `use_sim_time:=true`).

UI clients should not invent staleness: the bridge drops stale fields before sending. For initial hydration, call `GetStatus` (metadata + latest non-stale values); then subscribe to `StreamStatus` (values only, event-driven).

## Run

This node is started by `rovi_bringup/robot_bringup.launch.py` by default.

To run manually:

```bash
ros2 run ros_ui_bridge ui_bridge -- --config /path/to/ui_bridge.yaml
```

To verify the gRPC streams (server reflection is not enabled):

```bash
sudo snap install --edge grpcurl

# Robot state stream
grpcurl -plaintext -import-path ${ROVI_ROS_WS_DIR}/src/ros_ui_bridge/proto -proto ui_bridge.proto localhost:50051 roblibs.ui_bridge.v1.UiBridge/StreamRobotState

# Lidar stream
grpcurl -plaintext -import-path ${ROVI_ROS_WS_DIR}/src/ros_ui_bridge/proto -proto ui_bridge.proto localhost:50051 roblibs.ui_bridge.v1.UiBridge/StreamLidar
```

## Configuration

See `config/default.yaml` (installed at `$(ros2 pkg prefix ros_ui_bridge)/share/ros_ui_bridge/config/default.yaml`).

Configuration is organized under `streams:`:

- `streams.status` — cadence, default staleness window (`stale_after_s`), optional `debug_log`/`always_publish`, and `fields` (unit/min/max/target + source). Supported sources:
  - `system` / `cpu_percent`
  - `ros` / `topic_value` (topic + msg_type + value_key, optional downsample_period_s)
  - `ros` / `topic_rate` (topic, optional msg_type)
  - `ros` / `tf_rate` (parent, child)
  - Optional per-field staleness override via `source.stale_after_s`.
  - Field value type: float by default; set field `type: text` to publish `StatusFieldValue.text` instead of a float.
- `streams.robot_state` — pose/wheel stream downsampling (optional), topics, frames, wheel joints
- `streams.lidar` — lidar stream downsampling (optional), input/output topics, frame_id (optional; omit to disable)
- `robot_model` — GLB path and chunk size

Robot state / lidar / map support optional downsampling via either:
- `downsampling_rate_hz` (preferred)
- `downsampling_period_s` (preferred)

For backward compatibility, older `rate_hz` / `period_s` keys still work.

If neither downsampling key is provided, no downsampling is applied (forward 1:1).

When downsampling is enabled, the behavior is "capped downsampling": data is forwarded on arrival if the rate cap allows, otherwise forwarded on the next timer tick. No stale data is ever duplicated.

The lidar stream also republishes throttled scans to `output_topic` (default `/viz/scan`) for RViz visualization.

In this repo, `rovi_description` generates and installs a model at `package://rovi_description/models/rovi.glb` during `colcon build`.

`GetRobotModelMeta` expects a sidecar meta file next to the GLB at `<glb_path>.meta.json` (generated during build); it reads this file (no runtime hashing).

### UI client hints

- Call `GetStatus` once on connect to get the field metadata (unit/min/max/target) plus the latest non-stale values. No client-side buffering is needed.
- Subscribe to `StreamStatus` for ongoing values (no metadata). Fields disappear from the stream when stale; do not synthesize zeros.
- Staleness is computed in the bridge using ROS time; timestamps are forwarded from message headers when available.

## TF demux topics (plot-friendly)

Some tools make it hard to plot a specific transform inside `/tf` because `/tf` is a `tf2_msgs/TFMessage` containing an array of transforms.

When `ui_bridge` is running, it republishes each dynamic `/tf` transform into its own topic:

- `/viz/tf/<parent>_<child>` (`tf2_msgs/TFMessage` with exactly one transform)

Frame ids are normalized (leading `/` removed) and sanitized for topic safety.

YAML config (in `ros.tf_demux`):

- `enabled` (bool, default: `true`)
- `prefix` (string, default: `/viz/tf`)
