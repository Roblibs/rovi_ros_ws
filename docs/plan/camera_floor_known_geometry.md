# Plan: Depth camera = obstacle-only (known geometry; delete floor calibration)

## Decisions (confirmed)

- Depth camera contributes to **obstacle detection only** (no ravine/void detection).
- Environment is closed + flat enough; safety relies on other sensors/constraints.
- Camera max usable range: **2.5 m**.
- `base_footprint` is the authoritative floor plane (`z=0`) in real + sim.
- Unknown/invalid depth must cause **no costmap contribution** (do not add obstacle, do not clear/free).
- Remove floor calibration completely: **no fallback**, **no backwards compatibility** with LUTs.

## Goal

Use known camera geometry to detect obstacles above the floor and feed Nav2/local costmaps, without any calibration step.

## Model

Known floor plane in `base_footprint`: `z=0`.

Per pixel, compute expected floor depth (Z in `camera_depth_optical_frame`) by intersecting the pixel ray with that plane,
using `camera_info` intrinsics and TF `base_footprint -> camera_depth_optical_frame`.

This is computed **on startup** and cached in-memory keyed by a signature of (`camera_info`, TF).

## Obstacle levels (for visualization)

Two height thresholds above the floor (in `base_footprint`):

- `h1 = 0.01 m` (1 cm)
- `h2 = 0.07 m` (7 cm)

Compute per-pixel depth thresholds `Z(h1)` and `Z(h2)` by intersecting the pixel ray with planes `z=h1` and `z=h2`
in `base_footprint` (same intrinsics + TF math as the floor plane).

Classification per pixel (depth `D`):

- If depth is invalid/unknown: **unknown** (no contribution).
- Else:
  - if `D <= Z(h2)`: obstacle level `2`
  - else if `D <= Z(h1)`: obstacle level `1`
  - else: `0` (no mark)

Depth range guardrails:
- Clamp/ignore any thresholds beyond `2.5 m`.

## ROS outputs (required)

We need exactly two outputs:

### 1) Nav2 gating mask (`/floor/mask`)

- Type: `sensor_msgs/Image` (`mono8`)
- Size: matches the depth stream exactly
- Header: same `stamp` and `frame_id` as the depth image
- Semantics:
  - `255` = obstacle (blocker) for **`h >= 1 cm`**
  - `0` = no contribution (includes unknown/invalid depth and “no obstacle”)

Nav2 layer policy:
- **Mark-only**: only mark lethal obstacles where mask pixels are `255`.
- **No clear**: never clear/free-space from this source, so `0` remains “ignore”.

### 2) Visualization topology (`/floor/topology`)

- Type: `visualization_msgs/MarkerArray`
- Frame: `base_footprint`
- Contains level-aware visualization derived from the same computation:
  - show obstacle contours/points colored by level (`1` vs `2`)
  - exact marker conventions are free to choose (this is viz-only)

## Cleanup scope (delete calibration)

Remove:

- `rovi_floor` calibration node/binary and any LUT file assumptions.
- `rovi_bringup` calibration launches (`floor_calibrate*.launch.py`).
- Plan/doc references that assume LUT calibration (`docs/plan/floor_diff_from_depth.md` becomes obsolete or rewritten).

## Execution plan

1) Robot model: ensure camera TF matches the new mount (optical center 11.5 cm above floor).
2) `rovi_floor`: replace LUT-based runtime with geometry-based obstacle-only detector:
   - compute/cached per-pixel depth thresholds `Z(h1)` and `Z(h2)` from `camera_info` + TF
   - publish `/floor/mask` (binary, `h>=1cm` => `255`, else `0`)
   - publish `/floor/topology` (level-aware viz)
   - treat invalid depth as `0` (no contribution)
3) Nav2 layer: consume `/floor/mask` as mark-only:
   - mark lethal cost where mask is `255`
   - do not clear on mask `0`
4) Delete calibration:
   - remove calibration node + launches + docs references
   - remove any runtime codepaths reading/writing `~/.ros/rovi/floor/*`
5) Verify:
   - sim: place small objects and confirm they appear as obstacles at the expected thresholds
   - real: confirm black/unknown floor does not create false obstacles and does not clear costmap

## Open points

1) Topology marker convention: preferred visualization (contours vs point clouds) for levels `1` and `2`?
