# Stacks (what runs where)

This workspace is driven by a single entrypoint launch: `rovi_bringup/rovi.launch.py`.
It composes two layers:

- **Gateway plane** (always-on): `rovi_bringup/gateway.launch.py` → `robot_bringup.launch.py` (backend) + `ros_ui_bridge` (+ `robot_serial_display` on real robot).
- **Stack plane** (session): `stack:=teleop|camera|mapping|localization|nav|offline|bringup`.

In “stack-only” workflows (typical on the real robot), the gateway plane is owned by `rovi-gateway.service`
and you launch stacks with `gateway_enabled:=false` (this is auto-added by `rovi_env.sh` when the service is active).

## Stack wiring (launch composition)

Legend: `✓` = always, `opt` = optional (launch arg), `—` = not started by that stack.

| `stack:=...` | Intended use | `twist_mux` (`/cmd_vel`) | Camera drivers (real only) | Floor runtime (`/floor/*`) | SLAM (`slam_toolbox`) | Nav2 |
|---|---|---:|---:|---:|---:|---:|
| `teleop` | Manual driving / base + LiDAR bringup | ✓ | — | — | — | — |
| `camera` | Camera bringup + teleop | ✓ | ✓ | — | — | — |
| `mapping` | Build/update map | ✓ | opt (`camera_enabled`) | opt (`camera_enabled`) | opt (`slam_enabled`) | — |
| `localization` | Localize against saved pose-graph | ✓ | opt (`camera_enabled`) | opt (`camera_enabled`) | opt (`slam_enabled`) | — |
| `nav` | Navigate (mapping or localization) | ✓ | opt (`camera_enabled`) | opt (`camera_enabled`) | ✓ (`slam_mode`) | ✓ |
| `offline` | URDF/TF inspection only | — | — | — | — | — |
| `bringup` | Gateway-only placeholder (no session stack) | — | — | — | — | — |

Notes:
- “Camera drivers (real only)” are started by `rovi_bringup/camera.launch.py` and only run when `robot_mode:=real`.
- “Floor runtime” is `rovi_floor/floor_runtime_node` included via `rovi_bringup/perception.launch.py`.
- Defaults: `camera_enabled:=true` (mapping/localization/nav) and `slam_enabled:=true` (mapping/localization).

## Real robot (`robot_mode:=real`)

- **LiDAR (`/scan`)** comes from the backend (gateway plane) and is expected in all normal “real robot” sessions.
- **Depth + color** only exist if the stack starts camera drivers:
  - always for `stack:=camera`
  - optional for `stack:=mapping|localization|nav` via `camera_enabled:=true` (default true)
  - never for `stack:=teleop`
- Camera perception consumers (floor runtime) are gated by `camera_enabled` too, so you can run LiDAR-first mapping/nav by setting `camera_enabled:=false`.

Important sensor topics you should expect when cameras are enabled:
- LiDAR: `/scan`
- Depth: `/camera/depth/image` (and/or `/camera/depth/image_raw` depending on driver pipeline)
- Color: `/camera/color/image`

## Simulation (`robot_mode:=sim`)

- **LiDAR (`/scan`)** is bridged from Gazebo via `rovi_gz_sensors_bridge_node`.
- **Depth + color** are also bridged from Gazebo and are typically present regardless of which stack you run.
- `camera_enabled:=false` disables camera-based *consumers* (like floor runtime); it does not stop Gazebo from publishing sensor topics.

Important sensor topics you should expect in sim:
- LiDAR: `/scan`
- Depth: `/camera/depth/image`
- Color: `/camera/color/image`

## Offline (`robot_mode:=offline`)

No sensors are expected; this mode is for URDF/TF inspection.

## Key launch args (most commonly used)

| Arg | Used by | Default | What it changes |
|---|---|---|---|
| `robot_mode` | all | `real` | Backend: hardware (`real`), Gazebo (`sim`), or URDF-only (`offline`). |
| `stack` | `rovi.launch.py` | `teleop` | Which session stack to compose. |
| `gateway_enabled` | `rovi.launch.py` | `true` | Start/skip gateway plane (backend + UI bridge + display). |
| `rviz` | `rovi.launch.py` | `true` for `sim/offline`, else `false` | Start RViz. |
| `joy_enabled` | control stacks | `true` on real, else `false` | Start joystick teleop nodes. |
| `odom_mode` | mapping/localization/nav | `filtered` | Select raw vs EKF odom policy. |
| `slam_enabled` | mapping/localization | `true` | Enable/disable `slam_toolbox` in those stacks. |
| `slam_mode` | nav | `mapping` | Nav stack SLAM policy: mapping vs localization. |
| `map_file_name` | localization/nav | `~/.ros/rovi/maps/latest.posegraph` | Pose-graph file used when localizing. |
| `camera_enabled` | mapping/localization/nav | `true` | Start camera drivers (real) + camera-based perception (real/sim). |
| `camera_topology_enabled` | mapping/localization/nav | `false` | Publish `/floor/topology` (visualization) when camera pipeline is enabled. |
| `device_id` | camera + camera-enabled stacks | (driver default) | OpenNI2 device selector (depth). |
| `depth_mode` | camera + camera-enabled stacks | (driver default) | OpenNI2 preset name (depth resolution/FPS). |
| `rgb_video_device` | camera + camera-enabled stacks | auto-picked | V4L2 device path (color). |
| `rgb_width`,`rgb_height` | camera + camera-enabled stacks | (launch defaults) | Color resolution. |
