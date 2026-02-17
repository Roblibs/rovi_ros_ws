# Plan: UI bridge frame policy (minimize UI-side transforms)

## Question to resolve
We currently include `frame_id` on gRPC geometry messages (e.g., lidar and floor topology). This does **not** force the UI to do TF; it is metadata. But if the goal is “UI should not do transformations”, we should define a consistent frame policy for all streamed geometry.

## What we do today (inventory)
- `StreamRobotState`: sends a `Pose3D` with an explicit `frame_id` (bridge-chosen fixed frame, based on session).
- `StreamLidar`: sends `frame_id` + scan parameters + ranges; the implied points are in the scan frame (typically `laser_link`).
- `StreamMap`: sends `frame_id` (typically `map`) + origin pose + PNG.
- `StreamFloorTopology`: sends `FloorPolyline.frame_id` + polyline points; today points come from `/floor/topology` markers which are typically already in `base_footprint`.

## Why `frame_id` exists (even without UI TF)
It enables **attachment** and **sanity checks**:
- UI can parent the geometry to the correct object (robot-local vs world).
- If something unexpectedly switches frames (e.g., `base_link` vs `base_footprint`), `frame_id` reveals the bug early.

If we truly want “UI does zero transforms”, then the bridge must output geometry in a single agreed-upon frame and `frame_id` becomes redundant (but still useful as an assertion).

## Decide a single UI render model
Pick one of these and document it:

### Option A (robot-local overlays): geometry in `base_footprint`
- Contract: all non-map geometry (lidar + floor topology) is expressed in `base_footprint`.
- UI strategy: render robot model in world using `StreamRobotState.pose`; render overlays as children of the robot (no extra math beyond scene graph parenting).
- Bridge work: for lidar, transform scan into `base_footprint` inside the bridge (needs TF `base_footprint -> laser_link`).
- Keep `frame_id` in proto but fixed to `base_footprint` (or remove it once stable).

### Option B (world overlays): geometry in the bridge-chosen fixed frame (`map`/`odom`)
- Contract: all streamed geometry is expressed in the same fixed frame as `StreamRobotState.pose.frame_id`.
- UI strategy: render everything directly in world coordinates; no parenting needed.
- Bridge work:
  - floor topology: transform from `base_footprint` into fixed frame using the current robot pose.
  - lidar: transform scan points into fixed frame (TF + robot pose).

### Option C (status quo): keep `frame_id`, UI may or may not transform
- Lowest bridge complexity, but violates the “don’t bother UI with transforms” lesson if UI wants to render mixed-frame geometry correctly.

## Recommended path
Option A tends to be simplest for UIs that already render a robot model: overlays stay robot-local and only robot pose updates move them globally.

## Implementation plan (if we choose Option A)
1) Document the gRPC frame policy in `src/ros_ui_bridge/README.md` (1–2 lines).
2) For floor topology: enforce `base_footprint` and optionally drop per-polyline `frame_id` once stable.
3) For lidar:
   - Decide whether UI actually needs scan visualization.
   - If yes, update bridge to convert LaserScan ranges → XY points in `base_footprint` (requires TF lookup or using the URDF static transform).
   - Update proto to stream points instead of raw scan parameters (or add a parallel `points` field).
4) Add a smoke check: verify `frame_id` never changes unexpectedly across modes (sim/real).

