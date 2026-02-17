# Plan: `/floor/topology` semantics (RViz markers + clean UI payload)

## Why this is confusing
`/floor/topology` is currently a `visualization_msgs/msg/MarkerArray`. That type is historically an **RViz display protocol**, not a “data product” message:
- RViz is stateful: it keeps markers around until they are overwritten, deleted, or expire.
- Therefore markers have `action` fields (`ADD`, `DELETE`, `DELETEALL`) so a publisher can do incremental updates without resending everything.

For *our* depth→topology pipeline, your mental model is correct: the output is a **full, unique snapshot each frame**. In that model, “keeping old markers” is unnecessary.

## Root cause (where it comes from)
This is inherited from choosing `MarkerArray` as the ROS transport because it is convenient for RViz visualization.

In `rovi_floor_runtime` today, the publisher already uses RViz semantics to *force snapshot behavior*:
- It publishes a `DELETEALL` marker every time and then (optionally) adds the current LINE_STRIP contour marker.
- That guarantees RViz never “leaves old geometry behind”.

So: the *topic* is snapshot-like in practice, even though the *message type* supports incremental updates.

## Chosen contract (recommended for ROVI)
Keep `/floor/topology` as **RViz-friendly markers**, but treat it as a visualization output, not a data API.

For UIs and non-RViz consumers, **do not consume MarkerArray directly**; consume a clean, typed payload produced by the UI bridge.

### `/floor/topology` (ROS) contract
- **Producer intent:** “snapshot per publish” (emit `DELETEALL` + current markers).
- **Consumer expectation (RViz):** stateful, applies `action` and `lifetime` as usual.
- **Consumer expectation (UI bridge):** may apply MarkerArray semantics, but always emits a *snapshot* gRPC message downstream.

### `StreamFloorTopology` (gRPC) contract
- Each gRPC event is a full snapshot: `repeated FloorPolyline polylines` in meters.
- No client-side marker bookkeeping; `polylines=[]` means “clear overlay”.

## Follow-ups (optional)
- Minimal docs note: add a small statement to `docs/reference.md` that `/floor/topology` is RViz markers, and UI consumers should use gRPC `StreamFloorTopology` instead.
- Longer-term: if multiple downstream consumers appear (not only UI bridge), consider adding a dedicated ROS “data” topic (polyline message) and keeping markers purely for RViz.

## Acceptance checks
- With topology enabled, UI receives exactly the polylines for the current frame, and clears when none are present.
- No accumulation across frames in UI when the producer outputs a changing contour.
- RViz continues to behave identically.
