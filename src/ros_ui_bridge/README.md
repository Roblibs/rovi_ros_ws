# ros_ui_bridge

ROS UI bridge: a small ROS 2 node that exposes robot telemetry and visualization data to UI clients over gRPC.

It is designed to be generic and reusable: it reads from ROS topics / TF and streams downsampled snapshots suitable for dashboards and web UIs.

## What it provides

| Capability | gRPC RPC | Notes |
| --- | --- | --- |
| Periodic status snapshots (CPU, voltage, configurable rate metrics) | `GetStatus` | Low-rate stream (~0.3 Hz default) for UI dashboards. |
| Robot pose + wheel joint angles (for Three.js / 3D rendering) | `StreamRobotState` | Always emits `pose_odom`; emits `pose_map` when `map->odom` is available. |
| Downsampled lidar scans for 3D visualization | `StreamLidar` | Low-rate (2 Hz default) LaserScan stream; optional. |
| Robot model metadata (cache key / sha256, size) | `GetRobotModelMeta` | Use this as an ETag-like key to decide whether to refresh cached assets. |
| Robot model asset (GLB / binary glTF) | `GetRobotModel` | Chunked stream to avoid gRPC message size limits. |

Proto: `proto/ui_bridge.proto` (`package roblibs.ui_bridge.v1`)

## ROS inputs (high level)

- Voltage is read from a `std_msgs/Float32` topic (configurable).
- Rate metrics can be collected from arbitrary topics and TF pairs (configurable).
- Robot pose is derived from `/odom_raw` (`nav_msgs/Odometry`) and optionally projected into `map` using the `map->odom` TF transform.
- Wheel angles are read from `/joint_states` (`sensor_msgs/JointState`) using the configured wheel joint names.
- Lidar scans are read from `/scan` (`sensor_msgs/LaserScan`) and downsampled to the configured rate.

## Run

This node is started by `rovi_bringup/robot_bringup.launch.py` by default.

To run manually:

```bash
ros2 run ros_ui_bridge ui_bridge -- --config /path/to/ui_bridge.yaml
```

To verify the gRPC streams (server reflection is not enabled):

```bash
# Robot state stream
grpcurl -plaintext -import-path /home/wassi/dev/Roblibs/rovi_ros_ws/src/ros_ui_bridge/proto -proto ui_bridge.proto localhost:50051 roblibs.ui_bridge.v1.UiBridge/StreamRobotState

# Lidar stream
grpcurl -plaintext -import-path /home/wassi/dev/Roblibs/rovi_ros_ws/src/ros_ui_bridge/proto -proto ui_bridge.proto localhost:50051 roblibs.ui_bridge.v1.UiBridge/StreamLidar
```

## Configuration

See `config/default.yaml` (installed at `$(ros2 pkg prefix ros_ui_bridge)/share/ros_ui_bridge/config/default.yaml`).

Configuration is organized under `streams:`:

- `streams.status` — status stream rate and rate metrics to track
- `streams.robot_state` — pose/wheel stream rate cap, topics, frames, wheel joints
- `streams.lidar` — lidar stream rate cap, input/output topics, frame_id (optional; omit to disable)
- `robot_model` — GLB path and chunk size

All stream rates use "capped downsampling": data is forwarded on arrival if the rate cap allows, otherwise forwarded on the next timer tick. No stale data is ever duplicated.

The lidar stream also republishes throttled scans to `output_topic` (default `/viz/scan`) for RViz visualization.

In this repo, `rovi_description` generates and installs a model at `package://rovi_description/models/rovi.glb` during `colcon build`.

`GetRobotModelMeta` expects a sidecar meta file next to the GLB at `<glb_path>.meta.json` (generated during build); it reads this file (no runtime hashing).

## TF demux topics (plot-friendly)

Some tools make it hard to plot a specific transform inside `/tf` because `/tf` is a `tf2_msgs/TFMessage` containing an array of transforms.

When `ui_bridge` is running, it republishes each dynamic `/tf` transform into its own topic:

- `/viz/tf/<parent>_<child>` (`tf2_msgs/TFMessage` with exactly one transform)

Frame ids are normalized (leading `/` removed) and sanitized for topic safety.

ROS parameters (on node `ui_bridge_metrics`):

- `tf_demux_enabled` (bool, default: `true`)
- `tf_demux_prefix` (string, default: `/viz/tf`)
