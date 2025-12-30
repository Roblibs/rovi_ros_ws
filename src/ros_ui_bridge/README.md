# ros_ui_bridge

ROS UI bridge: a small ROS 2 node that exposes robot telemetry and visualization data to UI clients over gRPC.

It is designed to be generic and reusable: it reads from ROS topics / TF and streams downsampled snapshots suitable for dashboards and web UIs.

## What it provides

| Capability | gRPC RPC | Notes |
| --- | --- | --- |
| Periodic status snapshots (CPU, voltage, configurable rate metrics) | `GetStatus` | Low-rate stream for UI dashboards. |
| Robot pose + wheel joint angles (for Three.js / 3D rendering) | `StreamRobotState` | Always emits `pose_odom`; emits `pose_map` when `map->odom` is available. |
| Robot model metadata (cache key / sha256, size) | `GetRobotModelMeta` | Use this as an ETag-like key to decide whether to refresh cached assets. |
| Robot model asset (GLB / binary glTF) | `GetRobotModel` | Chunked stream to avoid gRPC message size limits. |

Proto: `proto/ui_bridge.proto` (`package roblibs.ui_bridge.v1`)

## ROS inputs (high level)

- Voltage is read from a `std_msgs/Float32` topic (configurable).
- Rate metrics can be collected from arbitrary topics and TF pairs (configurable).
- Robot pose is derived from `/odom_raw` (`nav_msgs/Odometry`) and optionally projected into `map` using the `map->odom` TF transform.
- Wheel angles are read from `/joint_states` (`sensor_msgs/JointState`) using the configured wheel joint names.

## Run

This node is started by `rovi_bringup/robot_bringup.launch.py` by default.

To run manually:

```bash
ros2 run ros_ui_bridge ui_bridge -- --config /path/to/ui_bridge.yaml
```

## Configuration

See `config/default.yaml` (installed at `$(ros2 pkg prefix ros_ui_bridge)/share/ros_ui_bridge/config/default.yaml`) for:

- gRPC bind address
- Status update cadence
- Which topic / TF rate metrics to track
- Robot state topics, frames, wheel joint list, and map TF freshness window
- Robot model GLB path + chunk size

In this repo, `rovi_description` generates and installs a model at `package://rovi_description/models/rovi.glb` during `colcon build`.

`GetRobotModelMeta` expects a sidecar meta file next to the GLB at `<glb_path>.meta.json` (generated during build); it reads this file (no runtime hashing).

This package also ships `viz_downsample`: a small helper node to downsample high-rate topics for RViz (publishes to `/viz/*`, configured via `config/viz_downsample.yaml`).
