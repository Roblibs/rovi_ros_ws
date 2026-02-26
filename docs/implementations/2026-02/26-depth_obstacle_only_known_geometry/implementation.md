# Implementation: Depth obstacle-only `/floor/mask` (known geometry, no calibration)

Date: 2026-02-26

## Summary

Replaced the old “floor LUT calibration + floor-vs-unsafe mask” pipeline with a geometry-derived, obstacle-only output:

- `/floor/mask` is now a **Nav2 gating mask** where `255` means **obstacle (>= 1cm)** and `0` means **no contribution** (includes unknown depth).
- `/floor/topology` remains a visualization-only contour output, now level-aware (>= 1cm vs >= 7cm).
- All floor calibration/LUT generation is deleted (no fallback/back-compat).

This matches the new hardware reality: the camera is mounted horizontally and the floor can be invisible/invalid in depth, while obstacles remain visible.

## Key decisions encoded

- Depth camera is **obstacle detection only** (no ravine/void).
- Unknown/invalid depth must cause **no costmap contribution** (do not add obstacle; do not clear/free).
- Known geometry reference:
  - `base_footprint` is the floor plane (`z=0`)
  - camera optical center is at **11.5 cm** above floor (model TF).
- Range clamp: `max_range_m = 2.5`.
- Two height thresholds:
  - `h1 = 1cm` (Nav2 blocker)
  - `h2 = 7cm` (visualization level)

## Robot model change (height)

The camera mount in `rovi_description` is updated so:

- `base_footprint -> base_link` has `z=0.029`
- `base_link -> camera_link` has `z=0.086`
- Therefore `base_footprint -> camera_*_optical_frame` ends up at `z=0.115` (11.5 cm).

## New runtime algorithm (no LUTs)

### Startup / cache model build

`rovi_floor` runtime builds an in-memory per-pixel model from:

- `/camera/depth/camera_info` intrinsics (`fx, fy, cx, cy`)
- TF `base_footprint -> camera_depth_optical_frame`

For each pixel ray, it computes the expected depth (Z in optical frame) for two planes in `base_footprint`:

- plane `z=h1` → per-pixel threshold image `Z(h1)` (mm)
- plane `z=h2` → per-pixel threshold image `Z(h2)` (mm)

The model is cached and rebuilt only when intrinsics/TF/threshold params change.

### Per depth frame

Inputs:
- `/camera/depth/image` (`16UC1` mm or `32FC1` m; invalid=0 after conversion)

Outputs:

1) `/floor/mask` (`mono8`)
- `255` if `valid_depth && D_mm <= Z(h1)_mm`
- `0` otherwise (includes invalid/unknown depth)

2) `/floor/topology` (`MarkerArray`, optional via `camera_topology_enabled`)
- Builds two contour overlays from:
  - `h1_only = mask(h1) & !mask(h2)`
  - `h2 = mask(h2)`
- Uses morphology + connected components cleanup before contour extraction.
- Backprojects contour points using measured `D_mm` and the cached geometry to `base_footprint`.

## Interfaces / params (runtime)

Topics:
- Inputs: `/camera/depth/image`, `/camera/depth/camera_info`, TF `base_frame -> camera_depth_optical_frame`
- Outputs: `/floor/mask`, optional `/floor/topology`

Key params:
- `base_frame` (default `base_footprint`)
- `max_range_m` (default `2.5`)
- `obst_h1_m` (default `0.01`)
- `obst_h2_m` (default `0.07`)
- `camera_topology_enabled` (default `false`)
- contour cleanup knobs: `min_contour_area_px`, `contour_stride_px`, `morph_kernel_px`

## Deleted calibration workflow

Removed:

- `floor_calibrate_node` binary and source
- `rovi_bringup` calibration launches (`floor_calibrate*.launch.py`)
- `rovi_env.sh` helpers (`calib_floor`, `sim calib_floor`)
- runtime dependencies on `~/.ros/rovi/floor/*` LUT files

## Migration notes

- `/floor/mask` semantics changed:
  - old: `255 = floor/safe`
  - new: `255 = obstacle (>= 1cm)`
- Nav2 integration must treat `/floor/mask` as **mark-only**:
  - mark lethal where `255`
  - do not clear on `0` (ignore/no contribution)
- If you previously built/install overlays containing `floor_calibrate_node`, do a clean rebuild or remove stale install artifacts so the removed executable/launches do not linger.

