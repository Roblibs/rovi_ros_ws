# Plan: Make `StreamFloorTopology` lossless (multiple polylines + MarkerArray semantics)

## Problem
Current `StreamFloorTopology` is **lossy**:
- `FloorTopologyUpdate` carries only one `points[]` polyline, so multiple LINE_STRIP markers in one `/floor/topology` `MarkerArray` get dropped.
- The bridge selects a single marker (`ns=="floor_topology"` else first LINE_STRIP), which can be “wrong” if publishers ever emit more than one relevant strip or use multiple namespaces/ids.

## Goal
Forward `/floor/topology` to UI **without destructive selection**:
- If the topic publishes multiple polylines, UI receives all of them for that update.
- If the topic is absent, the stream stays idle (no errors).
- Preserve MarkerArray “clear/delete” meaning so UI doesn’t accumulate stale geometry.

## Approach (breaking change): update `FloorTopologyUpdate` to a snapshot message
Update the existing stream payload to be a **snapshot** of the current set of polylines:
- Keep `rpc StreamFloorTopology(FloorTopologyRequest) returns (stream FloorTopologyUpdate);`
- Change `FloorTopologyUpdate` to contain `repeated FloorPolyline polylines` instead of a single `points[]`.

Each `FloorPolyline` contains:
- Identity: `string ns`, `uint32 id` (from ROS Marker)
- `string frame_id` (allow mixed frames if ever present)
- Geometry: `repeated Point3 points`, `bool closed`
- (Optional, nice-to-have) style: `float width_m`, `ColorRGBA color`

Snapshot semantics:
- Each V2 message replaces the UI’s previous set of polylines entirely.
- If the publisher sends only a clear (DELETEALL) and no line strips, the UI gets `polylines=[]` and clears.

Why snapshot semantics:
- Simpler UI contract.
- Robust even if publishers mix ADD/DELETE/DELETEALL in a MarkerArray.

## Bridge implementation plan

### 1) Proto / stubs
- Update `src/ros_ui_bridge/proto/ui_bridge.proto`:
  - Add `FloorPolyline` message.
  - Change `FloorTopologyUpdate` to `repeated FloorPolyline polylines`.
- Regenerate python stubs in `src/ros_ui_bridge/ros_ui_bridge/api/`.

### 2) ROS ingestion: keep MarkerArray state, publish snapshot
Update `src/ros_ui_bridge/ros_ui_bridge/floor_topology_node.py` (or add a `floor_topology_v2_node.py`) to:
- Subscribe to `/floor/topology` as `visualization_msgs/msg/MarkerArray`.
- Maintain an internal dict keyed by `(ns, id)` for LINE_STRIP markers:
  - If any marker has `action == DELETEALL`: clear dict.
  - For each LINE_STRIP marker:
    - `action == ADD`: store/replace points + header (frame_id, stamp) + style (optional).
    - `action == DELETE`: remove `(ns,id)` if present.
- After applying the MarkerArray, publish a snapshot containing **all** stored polylines (sorted for determinism).
- Timestamp choice:
  - Prefer max(header.stamp) across included polylines; fallback to node clock.
- Optional: rate-cap with `ThrottledForwarder` as today.

### 3) gRPC server wiring
- Update the existing conversion logic in `src/ros_ui_bridge/ros_ui_bridge/grpc_gateway.py` to emit `polylines[]`.
- Keep wiring in `src/ros_ui_bridge/ros_ui_bridge/ui_bridge_node.py` unchanged (still one broadcaster + one node).

### 4) Config
- Extend `src/ros_ui_bridge/config/default.yaml`:
  - Keep using `streams.floor_topology` for topic + downsampling.
- Keep default behavior “always on” (subscribe even if not configured), consistent with current philosophy.

### 5) Docs + verification
- `src/ros_ui_bridge/README.md`: describe the polyline snapshot stream and grpcurl example.
- Smoke check on a system that publishes multiple LINE_STRIPs:
  - Verify UI receives all polylines, deleteall clears, deletes remove only targeted polyline.

## UI client expectations
- Render all `polylines[]`.
- Treat `polylines=[]` as “clear overlay”.
