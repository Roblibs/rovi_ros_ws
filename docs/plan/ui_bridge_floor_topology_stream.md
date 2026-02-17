# Plan: Forward `/floor/topology` into `ros_ui_bridge` gRPC

## Goal
Expose the floor “topology” (currently published on `/floor/topology`) over gRPC so UI clients can render it when available. The UI bridge should **always** attempt to forward it: if a publisher exists, stream updates; if not, the stream stays idle (no errors).

## Current state (Feb 17, 2026)
- `/floor/topology` is published by `rovi_floor` as `visualization_msgs/msg/MarkerArray` (visualization-only output).
- `ros_ui_bridge` gRPC surface currently streams: Status, RobotState, Lidar, Map, RobotModel (no floor stream).

## Key decision (data shape)
Pick one of:
1) **Recommended: “Polyline” message** — Convert the incoming `MarkerArray` into a small typed message: `frame_id`, `timestamp`, and `repeated Point` (the line strip). UI renders a polyline/polygon directly with no ROS/TF dependency.
2) “Raw MarkerArray-like” message — Mirror the ROS marker concepts in proto (more generic, more fields, more client complexity).
3) “Generic bytes” message — Send a serialized payload (e.g., JSON). Fast to implement but weakly typed and harder to evolve safely.

Recommendation rationale: option (1) matches today’s producer (a single LINE_STRIP marker in `base_footprint`) and keeps the UI contract stable and small.

## Proposed gRPC API (option 1)
Update `src/ros_ui_bridge/proto/ui_bridge.proto`:
- Add `rpc StreamFloorTopology(FloorTopologyRequest) returns (stream FloorTopologyUpdate);`
- Add messages:
  - `message FloorTopologyRequest {}` (or later: allow topic name override)
  - `message Point3 { float x=1; float y=2; float z=3; }`
  - `message FloorTopologyUpdate { int64 timestamp_unix_ms=1; string frame_id=2; repeated Point3 points=3; bool closed=4; }`
    - `closed=true` when the polyline forms a loop (matches current publisher behavior).
    - When the producer indicates “clear” (e.g., only DELETEALL), send `points=[]` to let UI clear its overlay.

Regenerate Python gRPC stubs (the repo already vendors generated `*_pb2.py` under `src/ros_ui_bridge/ros_ui_bridge/api/`).

## ROS ingestion & forwarding
Add a new ROS node similar to `lidar_node.py` / `map_node.py`:
- New file: `src/ros_ui_bridge/ros_ui_bridge/floor_topology_node.py`
- Behavior:
  - `create_subscription(MarkerArray, "/floor/topology", qos_best_effort)`
  - Parse the latest LINE_STRIP marker (ignore DELETEALL except as “clear” signal).
  - Normalize `frame_id` (strip leading `/`).
  - Timestamp: prefer marker header stamp; fallback to node clock.
  - Optional downsampling cap (recommended; topology may be published at camera rate):
    - Reuse `ThrottledForwarder` + `AsyncStreamBroadcaster` pattern.

## Configuration
Extend `src/ros_ui_bridge/config/default.yaml` with a new stream block (enabled by default):
```yaml
streams:
  floor_topology:
    #downsampling_rate_hz: 2.0
    topic: "/floor/topology"
```
Notes:
- Leaving it enabled by default satisfies “always forward if present”.
- If later needed, add an explicit `enabled: false` flag, but default stays “on”.

## Wiring into the server
Update `src/ros_ui_bridge/ros_ui_bridge/ui_bridge_node.py` and `src/ros_ui_bridge/ros_ui_bridge/grpc_gateway.py`:
- Instantiate the new broadcaster + ROS node when configured.
- Implement the new gRPC handler `StreamFloorTopology` that yields updates from the broadcaster (same pattern as `StreamLidar` / `StreamMap`).
- Keep behavior consistent with existing streams: if not configured, return `UNAVAILABLE`; if configured but topic is absent, stream simply produces no messages.

## Client impact
UI client changes (out of scope here) should:
- Add a subscription to `StreamFloorTopology`.
- Replace any RViz-marker assumptions with “polyline in `frame_id`”.
- Clear overlay when receiving an update with `points=[]`.

## Validation / smoke checks
On robot or sim:
1) Ensure `/floor/topology` is being published (e.g., `camera_topology_enabled:=true`).
2) Run the UI bridge and confirm the new RPC exists and streams data:
   - `grpcurl -plaintext -import-path ${ROVI_ROS_WS_DIR}/src/ros_ui_bridge/proto -proto ui_bridge.proto localhost:50051 roblibs.ui_bridge.v1.UiBridge/StreamFloorTopology`
3) Confirm “topic missing” behavior:
   - Run UI bridge without enabling topology publisher; ensure no errors and stream stays idle.

## Docs updates (keep minimal)
- `src/ros_ui_bridge/README.md`: add one row for FloorTopology stream (RPC name + what it is).
- (Optional) `docs/reference.md`: add a short note that floor topology can be consumed via gRPC (not ROS) by UIs.

