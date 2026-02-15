# Plan: Nav2 integration for `/floor/*` (custom costmap layer)

Goal: integrate the depth-derived floor/obstacle outputs into Nav2 without any “fake LiDAR” conversion.

## Preferred approach (recommended)

Implement a custom `nav2_costmap_2d` plugin layer that:

- Subscribes to `/floor/mask` (and optionally `/floor/bins`) as the gating signal.
- Uses `/camera/depth/image` + `/camera/depth/camera_info` + TF to project obstacle cells into `base_footprint`.
- Marks projected points (or small stamped polygons) as lethal obstacles in the local costmap.

Rationale:
- Keeps the “no `PointCloud2`/`LaserScan` emulation” rule.
- Fits Nav2’s intended extension mechanism (costmap layers).
- Reuses the floor node’s cheap classification so Nav2 only processes “likely obstacles”.

## Interface choices (keep minimal; pick one when implementing)

Option A (no new topics):
- Layer subscribes to `/floor/mask`, `/camera/depth/image`, `/camera/depth/camera_info`.
- It repeats the same grid downsample (based on the calibration LUT metadata, e.g. `cell_size_px`) to get a representative `Z` per occupied cell for backprojection.

Option B (add one helper output, recommended if CPU becomes tight):
- Extend the floor node to publish a small “downsampled depth grid” (e.g. `/floor/depth_grid_mm` as `sensor_msgs/Image` `mono16`, mm).
- Costmap layer subscribes to `/floor/mask` + `/floor/depth_grid_mm` + `/camera/depth/camera_info`.
- Layer backprojects occupied cells using the provided grid `Z` without re-reading/downsampling the full depth image.

## Coordinate policy

- The layer should update obstacles in the costmap’s global frame (typically `odom`) via TF (`base_footprint` → costmap frame), using the incoming message timestamp where possible.
- If TF is unavailable for a cycle, skip updates (do not “guess” transforms).

## Acceptance criteria

- When running `stack:=nav` in `robot_mode=real|sim`, enabling this layer causes nearby obstacles detected by depth to appear in the local costmap and influence planning/controller behavior.
- CPU impact stays bounded (grid-sized work, not full-res depth processing per costmap tick).
