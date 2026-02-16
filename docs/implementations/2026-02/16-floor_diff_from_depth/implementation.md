# Implementation: Depth-floor diff from depth camera (floor mask + optional topology)

Date: 2026-02-16

## Goal

Implement `docs/plan/floor_diff_from_depth.md`:

- Runtime: depth-floor diff → `/floor/mask` (`mono8`, `255=floor`, `0=unsafe`), plus optional `/floor/topology` visualization.
- Calibration: one-shot LUT generation writing `~/.ros/rovi/floor/*.png` (uint16 mm images).
- Keep stacks LiDAR-first: camera/LUT failures must degrade gracefully (logs + skip, no stack shutdown).
- Preserve sim/real parity (golden rule): stable topics and correct `header.frame_id` at the origin.

## What shipped

### New packages

- `src/rovi_floor/`
  - `floor_runtime_node`: subscribes `/camera/depth/image`, loads LUTs (`floor_mm.png`, `t_floor_mm.png`), publishes `/floor/mask`; optional topology (`/floor/topology`) when `camera_topology_enabled:=true`.
  - `floor_calibrate_node`: captures depth for `capture_duration_s`, builds `floor_mm.png` (per-pixel median) and `t_floor_mm.png` (per-pixel threshold map from plane/ray math), writes outputs atomically.
- `src/rovi_gz_sensors_bridge/`
  - `rovi_gz_sensors_bridge_node`: Gazebo Transport → ROS 2 bridge for `/scan`, `/imu/data_raw`, `/camera/*/image`, enforcing real-robot frame IDs (`laser_link`, `imu_link`, `camera_*_optical_frame`).

### Launch wiring / args

- New stack-level args (declared in `rovi_bringup/rovi.launch.py` and passed through mapping/localization/nav):
  - `camera_enabled` (default `true`)
  - `camera_topology_enabled` (default `false`)
- `rovi_bringup/perception.launch.py` now includes `floor_runtime.launch.py` when `camera_enabled:=true`.
- New bringup launches:
  - `rovi_bringup/floor_runtime.launch.py`
  - `rovi_bringup/floor_calibrate.launch.py`
- Simulation backend (`rovi_sim`) now uses `rovi_gz_sensors_bridge_node` for sensors requiring stable `frame_id`.

## Key behaviors

- Missing depth camera and/or missing LUTs: runtime logs rate-limited errors and publishes nothing; the rest of the stack keeps running.
- Depth intrinsics are consumed “as-is” from `/camera/depth/camera_info` (no attempt to validate whether they are theoretical vs measured).
- Topology is visualization-only and is gated by `camera_topology_enabled` (requires `camera_info` + TF `camera_depth_optical_frame -> base_footprint`).

## How to run (quick)

- Build: `build`
- Calibrate LUTs (writes default `~/.ros/rovi/floor/*.png`): `calib_floor`
- Enable camera pipeline in important stacks (mapping/localization/nav):
  - `camera_enabled:=true` (default)
  - `camera_topology_enabled:=true` to enable `/floor/topology`

## Validation (local)

- `colcon build` succeeds.
- Sim smoke: `/scan.header.frame_id == laser_link`, `/imu/data_raw.header.frame_id == imu_link`, `/camera/depth/image.header.frame_id == camera_depth_optical_frame`.
- Sim end-to-end: `floor_calibrate_node` writes LUT PNGs and `floor_runtime_node` publishes `/floor/mask`.

