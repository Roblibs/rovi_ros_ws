# Plan: Depth-floor diff → 3-level bins → topo cleanup → obstacle lasso (ROS 2 Jazzy)

This replaces the older “scalar floor clearance” concept. The new approach produces a cheap “stuff above the floor” mask + stable obstacle outlines from depth, without pointcloud/LiDAR emulation.

## Goal

From a depth camera, compute:

- (a) small binned grid image (floor-diff classes)
- (b) cleaned binary obstacle mask
- (c) stable per-obstacle contour(s) (“lasso”) for visualization (and potentially downstream logic later)

## Dependencies / contracts (already implemented elsewhere)

- Camera contract (topics + frames): `docs/contract.yaml` (`stack: camera`)
  - Input depth topic (canonical): `/camera/depth/image`
  - Required camera model: `/camera/depth/camera_info`
  - Depth frame: `camera_depth_optical_frame`
- Bringup composition slot for perception nodes: `src/rovi_bringup/launch/perception.launch.py`

Non-goals (for this plan):
- No RGB-D registration.
- No publishing `PointCloud2` / `LaserScan` as “depth emulation”.
- No Nav2 integration in the initial implementation. When integrated, the intended path is a custom costmap layer subscribing to `/floor/mask`.

## Core assumptions

- Calibration happens on known flat ground and may take ~10 seconds.
- Runtime must be very cheap (small grid; integer math; small-kernel filtering).
- Ground is treated as the reference (“floor”) and is not identified online.
- Calibration artifacts are **not committed**: stored under `~/.ros/...` and rebuilt each robot startup by default.

## ROS interface (runtime)

### Inputs (subscribers)

- `/camera/depth/image` (`sensor_msgs/Image`)
  - Expected to be in `camera_depth_optical_frame` and time-stamped correctly for `use_sim_time`.
  - Implementation should accept the common encodings (`32FC1` meters is typical here) and convert to integer mm internally.
- `/camera/depth/camera_info` (`sensor_msgs/CameraInfo`) (required)
- TF: `camera_depth_optical_frame` → `base_footprint` at the image timestamp (required for `/floor/markers`)

### Outputs (publishers)

Suggested topics (names are part of the contract for downstream users once implemented):

- `/floor/bins` (`sensor_msgs/Image`, `mono8`)
  - values: `0=invalid`, `1=floor-ish`, `2=low`, `3=high` (3 levels + invalid)
- `/floor/mask` (`sensor_msgs/Image`, `mono8`)
  - values: `0` or `255` (cleaned obstacle mask; derived from bins: `mask = (bin >= 2)`)
- `/floor/markers` (`visualization_msgs/MarkerArray`) (optional)
  - visualization-only: one `LINE_STRIP` per obstacle contour
  - publish can be disabled so the core node remains “perception” rather than “UI/viz”
- `/floor/state` (`std_msgs/String`) (optional but recommended)
  - `CALIBRATING` → `RUNNING` (+ basic stats / last-write path)

## Calibration artifacts (YAML under ~/.ros)

### Location + lifecycle

- Default directory: `~/.ros/rovi/floor/`
- Default “latest” file: `~/.ros/rovi/floor/floor_lut.yaml`
- Default behavior: **auto-calibrate on startup** and overwrite `floor_lut.yaml` once complete.
  - Provide a runtime toggle to skip calibration (for debugging) and just load the existing LUT if present.
  - Provide a service to re-calibrate on demand (e.g., after moving the camera mount).

### YAML schema (proposal)

Single file contains both the floor reference and the per-cell thresholds (LUT).

Notes:
- The floor reference (`floor_mm`) is necessary to compute “height above floor” cheaply as `delta = max(0, floor_mm - depth_mm)` without per-frame plane fitting.
- Use `cell_size_px` as the *configuration knob*. `grid.w/h` may still be stored for validation/debugging.

- `version: 1`
- `cell_size_px: <int>` (e.g. 8, 10, 12)
- `grid: {w: <int>, h: <int>}` (derived from image size and `cell_size_px`)
- `floor_mm: [w*h uint16]` (row-major)
- `t1_mm: [w*h uint16]`, `t2_mm: [w*h uint16]` (row-major)

Rationale:
- Threshold values are configured via YAML (not ROS params).
- Per-cell thresholds allow compensating for perspective/quantization without runtime plane fitting.

### Node parameters (runtime policy)

Keep runtime behavior tweakable via ROS params (small and safe to tune), while the calibration YAML stays focused on the LUT:

- `close_iters` (default 1), `open_iters` (default 0)
- `min_component_area` (default: a few cells)
- `simplify_epsilon_cells` (default: ~0.5), `max_contour_points` (default: capped)
- `publish_markers` (default false on-robot; true for viewer/debug)

## Algorithm (runtime)

Per incoming depth frame:

1) Downsample depth image into a small grid `D[cell]` in **mm** (uint16), using `cell_size_px`.
   - Use stride sampling or block-reduce of valid pixels (median preferred; mean acceptable).
2) Load floor reference + thresholds from LUT:
   - `F[cell]`, `T1[cell]`, `T2[cell]`
3) Delta-from-floor:
   - `delta = max(0, F - D)`; if `D==0` or `F==0`, mark invalid.
4) Bin into `code`:
   - `0 = invalid`
   - `1 = delta < T1`
   - `2 = T1 <= delta < T2`
   - `3 = delta >= T2`
5) Create obstacle mask `M`:
   - `M = (code >= 2)`
6) Topological cleanup on `M` (small grid):
   - Closing (3x3) to fill small holes/bridges, then optional opening to remove speckles.
   - Connected components: drop blobs with area < `min_component_area`.
7) Lasso / contour extraction:
   - For each component, find boundary cells (edge where a 4-neighbor is empty).
   - Order into a loop (border-follow / Moore neighbor tracing).
   - Optional simplification in grid coordinates (RDP) with `simplify_epsilon_cells`.
8) Publish:
   - `bins` image (mono8)
   - cleaned `mask` (mono8)
   - optional contour markers as `LINE_STRIP` in `base_footprint`
     - Backprojection proposal per contour vertex:
       - Use the cell-center pixel `(u, v)` for that grid cell (derived from `cell_size_px` and the image size).
       - Use `Z = D[cell]` (mm → meters) as the representative depth.
       - Backproject to `camera_depth_optical_frame` using `camera_info` intrinsics, then TF-transform to `base_footprint`.

Implementation note:
- Keep computations in integers (mm) once the depth image is downsampled.
- Grid sizes like `80x60` or `64x48` keep this CPU-cheap on a Pi.

Marker frame guidance:
- Default: publish `/floor/markers` in `base_footprint` so it renders naturally with the robot model (independent of camera displays).
- This requires backprojection (using `camera_info`) and a TF transform at the image timestamp.
- If TF/camera_info is unavailable, the node should still publish `/floor/bins` + `/floor/mask`, and set `/floor/state` to reflect that markers are suppressed.

## Algorithm (calibration on startup)

Goal: build `floor_lut.yaml` for the current mount/scene and derive per-cell thresholds.

Calibration steps:

1) Use the same grid/downsample operator as runtime.
2) For ~10 seconds while stationary on flat floor, accumulate valid depth samples per cell.
3) Set `F[cell] = median(samples)` (or trimmed mean).
4) Derive `T1/T2` per cell from a small set of global “target” thresholds (mm) and/or heuristics.
   - Exact derivation is free to evolve as long as outputs remain stable; store the final per-cell values in YAML.
5) Write `floor_lut.yaml` under `~/.ros/rovi/floor/` and switch to `RUNNING`.
   - Default: `~/.ros/rovi/floor/floor_lut.yaml`.

Guardrails:
- Cells with insufficient samples get `F=0` and are forced to `invalid` at runtime.
- If depth is invalid on shiny floors (0/NaN), treat as invalid (not obstacle).

Calibration policy suggestion (startup behavior):
- Default: run a short “sanity sample” first (e.g., ~1s) and compare against the existing LUT if present.
  - If the median per-cell residual is small (configurable), keep the existing LUT and start `RUNNING` immediately.
  - Otherwise (validation fails), run the full ~10s calibration and overwrite `floor_lut.yaml`.
- Always provide an explicit “recalibrate now” service for when the robot is placed on a known flat floor.
- Store minimal metadata in the YAML for safety checks (e.g., `cell_size_px`, derived `grid.w/h`, `camera_frame_id`, and a timestamp) so a mismatch can trigger recalibration automatically.

## Where this plugs into the stack

- Implement as a new node (prefer C++ for Pi safety; Python acceptable at small grids if profiling says OK).
- Start it from `src/rovi_bringup/launch/perception.launch.py` so it can be composed into `teleop|camera|mapping|localization|nav` without copy/paste.
- Once stable, consider extending `docs/contract.yaml` to require the `/floor/*` outputs for a dedicated stack variant (e.g., a future `stack:=depth_perception`).

## Acceptance criteria

- Runs in `robot_mode=real` and `robot_mode=sim` with **no remaps** (same topics/frames).
- Uses only canonical depth inputs (`/camera/depth/image`, `/camera/depth/camera_info`) and the required TF frames.
- Auto-calibrates on startup and reliably writes `~/.ros/rovi/floor/floor_lut.yaml`.
- Produces stable contours (no flicker) in a static scene at the chosen grid resolution.
- When enabled, publishes `/floor/markers` in `base_footprint`.

