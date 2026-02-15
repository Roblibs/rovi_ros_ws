# Phase 0 implementation details (Camera contract)

This file records what was implemented for Phase 0 of `docs/plan/camera_preparation.md`.

## Outcomes

### Canonical camera topics

- Canonical RGB image topic is now `/camera/color/image` (sim + real).
- Canonical depth image topic remains `/camera/depth/image` (sim + real).

Updated consumers/config:
- UI rate checks now watch `/camera/color/image` and `/camera/depth/image`: `src/ros_ui_bridge/config/default.yaml`.
- RViz camera view now uses `/camera/color/image`: `src/rovi_description/rviz/rovi_camera.rviz`.

### CameraInfo topics (mandatory, low-rate, transient-local)

Added a new publisher node:
- `rovi_camera_info_pub` (`src/rovi_bringup/rovi_bringup/camera_info_pub.py`)
  - Publishes `/camera/color/camera_info` and `/camera/depth/camera_info`
  - QoS: reliable + transient-local
  - Default behavior: publish once (optional periodic republish via `camera_info_publish_period_s`)

Wiring:
- Real camera stack publishes driver camera_info to `*/camera_info_raw` and uses `rovi_camera_info_pub` for canonical `*/camera_info`: `src/rovi_bringup/launch/camera.launch.py`.
- Sim backend publishes canonical camera_info via `rovi_camera_info_pub`: `src/rovi_bringup/launch/robot_bringup.launch.py`.

### Reference calibration files (single source-of-truth)

- Existing RGB reference: `src/rovi_bringup/config/camera_info/color.yaml`
- Added placeholder depth reference (replace when real calibration is available): `src/rovi_bringup/config/camera_info/depth.yaml`
- Both YAMLs support an optional verification-only field: `device_serial` (empty means “no verification”).

Packaging:
- `config/camera_info/*.yaml` are now installed into the package share: `src/rovi_bringup/setup.py`.
- Added console entrypoint for `rovi_camera_info_pub`: `src/rovi_bringup/setup.py`.

### Simulation optics aligned to reference intrinsics

- Updated Gazebo RGB camera `horizontal_fov` to match the RGB reference calibration-derived HFOV: `src/rovi_description/urdf/rovi.urdf`.

### Developer tooling + docs

- `calib` now remaps to `/camera/color/image`: `rovi_env.sh` and `docs/reference.md`.
- Updated camera contract notes in `docs/depth_camera_astra_stereo_s_u3.md` to reflect `/camera/color/image`.

## Phase 1 implementation (Flatten stack composition)

- Introduced reusable launch blocks:
  - State estimation: `src/rovi_bringup/launch/state_estimation.launch.py`
  - SLAM wiring: `src/rovi_bringup/launch/slam_mode.launch.py`
  - Perception include slot (empty by default): `src/rovi_bringup/launch/perception.launch.py`
- Refactored stacks to compose shared blocks (no nested mapping/localization inside nav):
  - `src/rovi_bringup/launch/mapping.launch.py`
  - `src/rovi_bringup/launch/localization.launch.py`
  - `src/rovi_bringup/launch/nav.launch.py`
