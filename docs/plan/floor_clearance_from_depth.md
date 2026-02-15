# Plan: Floor clearance from depth (camera-only)

Goal: create a minimal node that computes a “floor clearance” metric from depth and publishes it for visualization and later navigation integration.

Dependencies:
- `docs/plan/camera_preparation.md` Phase 0 contract (canonical `/camera/*/image` + `/camera/*/camera_info`, required frame_ids, and time semantics).
- `docs/plan/camera_preparation.md` Phase 1 “perception block” include point (so this can be composed consistently across stacks).

## Design decisions (to resolve before implementation)

1) Output message contract
- Topic name (candidate): `/camera/floor_clearance`
- Message type options:
  - `std_msgs/Float32` (simple “meters to nearest floor” scalar)
  - `geometry_msgs/PointStamped` (point of closest detected floor in a chosen frame)
  - custom msg (only if you need richer diagnostics/percentiles/ROI info)

2) Frame + semantics
- Output frame (recommended): `base_footprint`
- Timestamp rule: use the incoming depth `Image.header.stamp` (works with `use_sim_time`)

3) Algorithm slice (start minimal)
- Start with a fixed ROI in the depth image, compute robust percentile/min depth after filtering invalids.
- Optional next: fit a plane to the ROI point cloud and compute clearance to the plane in `base_footprint`.

4) Visualization path
- RViz: publish markers or overlay debug image (later).
- UI: optional field in UI bridge (later).

## Acceptance criteria

- Runs in `robot_mode=sim` and `robot_mode=real` without stack-specific remaps.
- Only depends on the camera contract topics + TF frames.
- Can be enabled/disabled as a composable block without touching unrelated launches.

