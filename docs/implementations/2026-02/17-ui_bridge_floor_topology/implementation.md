# Implementation: gRPC FloorTopology stream (clean polylines)

## Summary
Keep `/floor/topology` as RViz-friendly `visualization_msgs/msg/MarkerArray`, but expose a clean, typed UI payload over gRPC:
- gRPC stream emits a **snapshot** of polylines in meters (`polylines[]`), so UI clients do not need to implement Marker semantics.
- Bridge ingests MarkerArray and applies `ADD`/`DELETE`/`DELETEALL` to maintain the current set, then publishes the snapshot.

## gRPC surface
- Proto: `src/ros_ui_bridge/proto/ui_bridge.proto`
  - RPC: `StreamFloorTopology`
  - Message: `FloorTopologyUpdate` now contains `repeated FloorPolyline polylines`

## ROS ingestion
- Node: `src/ros_ui_bridge/ros_ui_bridge/floor_topology_node.py`
  - Subscribes to `/floor/topology`
  - Maintains `(ns,id) -> FloorPolylineData` for LINE_STRIP markers
  - Publishes a deterministic, sorted snapshot each update

## Server wiring
- gRPC service: `src/ros_ui_bridge/ros_ui_bridge/grpc_gateway.py`
- Node + broadcaster wiring: `src/ros_ui_bridge/ros_ui_bridge/ui_bridge_node.py`

## Config / docs
- Default config: `src/ros_ui_bridge/config/default.yaml` (`streams.floor_topology`)
- Bridge README: `src/ros_ui_bridge/README.md`

