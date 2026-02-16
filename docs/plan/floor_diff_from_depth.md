# Plan: Depth-floor diff → floor mask (+ optional topology) (ROS 2 Jazzy)

This replaces the older “scalar floor clearance” concept. It produces a conservative “safe floor” mask from depth, without pointcloud/LiDAR emulation.

## Goal

From a depth camera, publish:

- `/floor/mask`: **safe vs unsafe** (unsafe includes obstacles and void/cliffs)
- `/floor/topology` (optional): topological cleanup + contour (“lasso”) visualization in `base_footprint`

Driving semantics:
- there is no difference between “T1 / T2 / higher” for motion decisions — anything that is *not floor* is unsafe and should be avoided.

## Dependencies / contracts (already implemented elsewhere)

- Camera contract: `docs/contract.yaml` (`stack: camera`)
  - `/camera/depth/image`
  - `/camera/depth/camera_info`
  - frame: `camera_depth_optical_frame`
- Composition slot: `src/rovi_bringup/launch/perception.launch.py` (included by `mapping|localization|nav`)

Related plan (tracked separately):
- Nav2 integration (custom layer): `docs/plan/floor_nav2_costmap_layer.md`

Non-goals:
- No RGB-D registration.
- No `PointCloud2` / `LaserScan` emulation.

## Stack wiring policy (camera is optional in “important stacks”)

Camera drivers are currently owned by the debug/test stack `stack:=camera` (`src/rovi_bringup/launch/camera.launch.py`).

For the important stacks (`mapping|localization|nav`), camera usage is added as an **option**:

- New launch arg: `camera_enabled` (bool)
  - Default: `true` (can be set `false` on robots without a depth camera).
  - When `false`: do not start camera drivers and do not start floor perception nodes.
  - When `true`: try to start camera drivers and floor nodes; failures must **degrade gracefully** (log errors, but the stack continues running).
- New launch arg: `camera_topology_enabled` (bool)
  - Controls the floor node’s topology/contour work (visualization-only).
  - Default `false` (heavier + requires TF + camera_info).

Notes:
- Nav2 costmaps exist only in `stack:=nav` (Nav2). `stack:=localization` is SLAM-toolbox localization and does not run Nav2, so it does not have a costmap.
- Even when `camera_enabled:=true`, the stack must continue working LiDAR-first when the camera is missing/unavailable.

## Calibration is explicit (own launch; not on startup)

Calibration is only needed when the camera mount changes or the LUT is known-stale.

- Normal bringup runs runtime only.
- Calibration runs via its own launch (suitable for oneshot systemd services / remote launch switching).
  - Depth intrinsics are taken from `/camera/depth/camera_info` “as-is” (no attempt to judge whether they are theoretical or hardware-derived).

## ROS interface (runtime)

### Inputs

- `/camera/depth/image` (`sensor_msgs/Image`)
  - expected `header.frame_id = camera_depth_optical_frame` (real + sim target parity)
  - use `header.stamp` (works with `use_sim_time`)
  - accept common encodings (e.g. `32FC1` meters) and convert to mm internally
- `/camera/depth/camera_info` (`sensor_msgs/CameraInfo`) (required only when `camera_topology_enabled:=true`)
- TF at the depth image timestamp (required only when `camera_topology_enabled:=true`):
  - `camera_depth_optical_frame` → `base_footprint`

### Outputs

- `/floor/mask` (`sensor_msgs/Image`, `mono8`)
  - `255 = floor (safe)`
  - `0 = not floor (unsafe)` (obstacles + void/cliff + unknown)
- `/floor/topology` (`visualization_msgs/MarkerArray`) (only when `camera_topology_enabled:=true`)
  - visualization-only: cleaned contour “lasso” in `base_footprint`

## Launch args (runtime node)

- `camera_topology_enabled` (bool, default `false`)
  - `false`: publish only `/floor/mask`
  - `true`: additionally run topology + publish `/floor/topology`

The perception block itself is enabled by the stack-level `camera_enabled` arg (see above).

## LUT artifacts (bitmaps, not YAML arrays)

Why bitmaps:
- YAML numeric arrays are large and not visually debuggable.
- Bitmaps can be opened directly and keep runtime as pure image operations.

### Location

- Directory: `~/.ros/rovi/floor/`

### File set (proposal)

Use 16-bit PNG (`uint16`, mm), same width/height as the depth stream:

- `floor_mm.png`:
  - reference floor depth per pixel (mm) from calibration
- `t_floor_mm.png`:
  - per-pixel half-width threshold for the “floor band” (mm)
- `t_obst1_mm.png` (optional, only needed when `camera_topology_enabled:=true`):
  - boundary for the “+2cm → +5cm” region (per-pixel, mm)
- `t_obst2_mm.png` (optional):
  - boundary for “+5cm → +10cm” region (per-pixel, mm)

Optional metadata sidecar is OK if it stays small (no big arrays), e.g. `meta.yaml`:
- `width`, `height`, `units: mm`, `camera_frame_id`, `created_at`

### Size estimate (640x400)

- One `uint16` image: `640*400*2` ≈ **512 KB** raw (PNG typically smaller).
- 2–4 images: on the order of **1–3 MB** total on disk.

## Threshold semantics (signed diff; void is unsafe)

Old pitfall to avoid:
- `delta = max(0, F - D)` hides the case `D > F`, which is exactly how void/cliffs show up.

Use signed depth difference in mm:
- `diff_mm = D_mm - F_mm`
  - `diff_mm ≈ 0`: floor-like
  - `diff_mm < 0`: something closer than the floor (obstacle above floor)
  - `diff_mm > 0`: something farther than the floor (void/cliff, or missing returns)

Floor definition:
- floor iff `valid_depth && (abs(diff_mm) <= t_floor_mm[pixel])`
- everything else is unsafe

Stairs-down / void behavior:
- void returns (far depth or invalid) fail the floor test → unsafe (robot will not drive into the void thinking it’s floor).

## “Only if topology enabled, compute more thresholds”

- Base runtime (no topology) uses only `floor_mm.png` + `t_floor_mm.png` to produce `/floor/mask`.
- When topology is enabled, the node may also load and use `t_obst1_mm.png` / `t_obst2_mm.png` to classify unsafe regions for visualization and more stable contours.

## Direction bias and per-threshold images

Depth is measured along rays. A constant mm delta along the ray does not correspond to a constant physical “height above floor” across the image.

Per-pixel threshold maps (`t_*_mm.png`) compensate for this perspective effect **if** they are derived from camera geometry + the calibrated floor reference (plane/ray math in calibration is fine; runtime stays cheap).

## Algorithm (runtime)

Per depth frame:

1) Convert depth to `D_mm` (mm integer; invalid=0).
2) Load LUT images in memory:
   - always: `F_mm = floor_mm.png`, `T_floor = t_floor_mm.png`
   - if topology: `T_obst1`, `T_obst2`
3) Compute signed difference: `diff_mm = D_mm - F_mm`.
4) Compute floor mask:
   - `floor = valid && (abs(diff_mm) <= T_floor)`
   - publish `/floor/mask` (`255` floor, `0` otherwise)
5) If `camera_topology_enabled:=true`:
   - derive an “unsafe” mask = inverse of floor
   - apply morphology + connected components to stabilize blobs
   - extract contours, simplify, and publish `/floor/topology` as `MarkerArray` in `base_footprint`
     - backproject contour points using `camera_info` + `D_mm` and TF-transform to `base_footprint`

## Algorithm (calibration) + launch wiring

Calibration runs explicitly and writes the LUT images under `~/.ros/rovi/floor/`.

Calibration steps (expensive OK):

1) Capture ~10s of depth on known flat floor.
2) Build `floor_mm.png` via per-pixel median (or trimmed mean) of valid samples.
3) Build `t_floor_mm.png` for the “floor band” (default corresponds to ~±2cm physical tolerance).
4) Optionally (for topology/levels), build `t_obst1_mm.png` and `t_obst2_mm.png` for:
   - +2→+5 cm and +5→+10 cm regions (per-pixel)
5) Write outputs atomically (temp + rename).

Deriving “cm above floor” into per-pixel `*_mm.png`:
- To avoid ray-angle (perspective) bias, compute these maps during calibration using camera geometry:
  - backproject the calibrated floor pixels with `camera_info`
  - fit a floor plane in a stable frame (e.g. `base_footprint`) using TF from the URDF
  - for each pixel ray, solve the expected depth at heights {2cm, 5cm, 10cm} above that plane and convert into per-pixel depth-delta thresholds

Launch separation (proposal):
- `floor_runtime.launch.py`: runs runtime node and requires `floor_mm.png` + `t_floor_mm.png`.
- `floor_calibrate.launch.py`: runs calibration node, writes LUT bitmaps, then exits.

## Graceful degradation (must not break stacks)

`stack:=nav` must remain LiDAR-first and usable when the camera/floor pipeline is absent.

Treat these cases as “camera disabled/unavailable”:
- depth camera driver not running / not connected
- `/camera/depth/image` missing/stale
- LUT files missing/unreadable under `~/.ros/rovi/floor/`

Policy:
- log clear, rate-limited errors/warnings
- **do not** bring down the stack
- floor outputs may be absent; downstream consumers (e.g. Nav2 floor layer) must treat absence as “do nothing”

## Sim/real parity (golden rule: make sim look like real)

Current issue to fix (known tech debt):
- In simulation today, `/camera/*/image.header.frame_id` does **not** match the optical frame IDs, even though `/camera/*/camera_info.header.frame_id` does.

Plan (bigger change, preferred over workarounds):
- Rework the sim camera bridging/model so `/camera/depth/image.header.frame_id == camera_depth_optical_frame` and `/camera/color/image.header.frame_id == camera_color_optical_frame`, matching real.
- If the bridge tooling cannot be configured to emit correct `frame_id`, add a dedicated sim-only rewrite step (remap + republish) as a fallback, but treat that as a temporary compatibility layer.

## Physical limitations (not algorithmic disadvantages)

Depth sensors can produce:
- flying pixels at object edges (false near)
- depth shadows / holes behind obstacles (invalid or far)
- specular/transparent/black surfaces (invalid or wrong depth)
- sunlight/IR interference and range-dependent noise
- camera pitch/roll change vs calibration (invalidates `floor_mm.png`)

Policy: treat invalid/unknown as unsafe; use topology only to clean noise for visualization.

## Implementation constraints (C++)

- Implement in C++ (rclcpp).
- Dependencies (recommended for this “bitmap + topology” approach):
  - OpenCV (PNG I/O, morphology, contours)
  - `tf2_ros`
  - `image_transport`

## Acceptance criteria

- With `camera_enabled:=true` and valid LUTs, runtime publishes `/floor/mask` in `robot_mode=real|sim` with no remaps.
- Void/cliff regions are not classified as floor (signed diff; no saturating delta).
- With `camera_topology_enabled:=true`, `/floor/topology` renders in `base_footprint` in the 3D robot model viewer.
- Calibration is only performed via the calibration launch and produces readable LUT bitmaps under `~/.ros/rovi/floor/`.
- With missing camera and/or missing LUTs, stacks continue running LiDAR-first (floor outputs may be absent).
