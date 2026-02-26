# Plan: Nav2 integration for `/floor/*` (custom costmap layer)

Goal: integrate the depth-derived floor/obstacle outputs into Nav2 without any “fake LiDAR” conversion.

Baseline policy:
- `stack:=nav` remains LiDAR-first (`/scan`) and must work without any depth/floor data.
- Depth/floor is an *augmentation*: when `/floor/mask` is available it can add near-field safety; when it is missing, Nav2 should continue normally.

## Preferred approach (recommended)

Implement a custom `nav2_costmap_2d` plugin layer that:

- Subscribes to `/floor/mask` as the gating signal (`mono8`, `255=obstacle`, `0=ignore`).
- Uses `/camera/depth/image` + `/camera/depth/camera_info` + TF to project *obstacle pixels* into `base_footprint` (with stride sampling to bound CPU).
- Marks projected points (or small stamped polygons) as lethal obstacles in the local costmap (**mark-only**).

Rationale:
- Keeps the “no `PointCloud2`/`LaserScan` emulation” rule.
- Fits Nav2’s intended extension mechanism (costmap layers).
- Reuses the floor node’s cheap classification so Nav2 only processes “likely obstacles”.

## Interface choices (keep minimal; pick one when implementing)

Option A (no new topics):
- Layer subscribes to `/floor/mask`, `/camera/depth/image`, `/camera/depth/camera_info`.
- It stride-samples obstacle pixels where `/floor/mask == 255`, looks up the corresponding depth `Z`, backprojects to the camera frame, and TF-transforms to the costmap frame.
  - It does **not** clear free space when `/floor/mask == 0` (unknown/no contribution policy).

Option B (optional optimization; only if needed):
- Add a dedicated, already-decimated obstacle sampling output from the floor node (format TBD) so the costmap layer does not need to touch full-res depth at costmap tick rate.

## Graceful behavior (depth optional)

The layer should be robust when floor is missing or calibration is not present:

- `floor_enabled:=false`:
  - Force-disable the layer even if `/floor/mask` exists (A/B testing).
- `floor_enabled:=true`:
  - If `/floor/mask` is available and fresh, apply it.
  - If `/floor/mask` is missing/stale, do nothing (Nav2 continues LiDAR-only).
  - Only print warnings when `floor_warn_if_unavailable:=true` (rate-limited).

## Coordinate policy

- The layer should update obstacles in the costmap’s global frame (typically `odom`) via TF (`base_footprint` → costmap frame), using the incoming message timestamp where possible.
- If TF is unavailable for a cycle, skip updates (do not “guess” transforms).

## Acceptance criteria

- When running `stack:=nav` in `robot_mode=real|sim`, enabling this layer causes nearby obstacles detected by depth to appear in the local costmap and influence planning/controller behavior.
- CPU impact stays bounded (stride sampling + simple backprojection; no full-res per-pixel costmap updates).
