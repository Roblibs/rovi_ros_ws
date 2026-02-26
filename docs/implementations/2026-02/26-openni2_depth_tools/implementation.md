## 2026-02-26 — OpenNI2 depth test tools

### Why
- Need a direct (non-ROS) depth camera sanity check when ROS wiring is unclear or the camera seems broken.

### What changed
- Added `tools/depth/openni2_snapshot_depth.cpp`: captures one OpenNI2 depth frame and saves:
  - `depth_<stamp>.pgm` (16-bit PGM, millimeters)
  - `depth_<stamp>_viz.pgm` (8-bit quick-look)
- Extended `rovi_env.sh` with helper commands:
  - `depth list` → `tools/depth/openni2_list_modes`
  - `depth snapshot` → builds/runs `tools/depth/openni2_snapshot_depth`
  - `depth view` → runs `NiViewer`
  - `tools depth ...` → dispatches to the same `depth` subcommands

### Usage
```bash
# source workspace helpers first
ws

depth list
depth snapshot --out-dir output/openni2_snapshot
depth view
```
