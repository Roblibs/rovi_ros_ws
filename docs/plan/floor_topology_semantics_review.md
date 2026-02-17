# Plan: Review `/floor/topology` semantics (sane contract for UI + RViz)

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

## Decision to make (contract)
Pick and document **one** contract for `/floor/topology`:

### Option A (recommended for current use): Snapshot contract
- **Contract:** Every publish is a full snapshot. Consumers must not keep state between messages.
- **Producer rule:** Always include a “clear” signal (either `DELETEALL` or simply publish the full set and ensure identities are stable).
- **Bridge/UI rule:** Convert each `MarkerArray` into a `FloorTopologyUpdate` containing *all* polylines found in that message; if none, treat as clear.

### Option B (generic MarkerArray contract)
- **Contract:** `/floor/topology` is a true MarkerArray command stream. Consumers must apply ADD/DELETE/DELETEALL.
- Pros: supports future incremental publishers.
- Cons: harder to reason about; mismatched with “unique snapshot per frame”.

## Plan to align the design (recommended path)
1) **Confirm producer behavior** in `rovi_floor` (document it):
   - Exactly what markers are emitted (`ns`, `id`, `type`, `action`) and whether `DELETEALL` is always present.
2) **Declare the topic contract** (choose Option A unless you explicitly want Option B).
   - Update `docs/reference.md` (minimal note) stating `/floor/topology` is a snapshot-per-message visualization output.
3) **Align gRPC semantics**:
   - Keep `FloorTopologyUpdate` as a snapshot: `repeated FloorPolyline polylines`.
4) **Align UI bridge ingestion** with the contract:
   - If Option A: simplify the bridge to be stateless per message (do not keep `(ns,id)` state; ignore DELETE/DELETEALL except as “clear”).
   - If Option B: keep stateful application of actions.
5) **(Longer-term cleanup)** split “data” from “visualization”:
   - Publish a dedicated data message (e.g., polygon/polyline message) on a new topic (e.g. `/floor/topology_polyline`).
   - Keep `/floor/topology` as RViz-only markers.
   - UI bridge consumes the dedicated data topic instead of `MarkerArray`.

## Acceptance checks
- With topology enabled, UI receives exactly the polylines for the current frame, and clears when none are present.
- No accumulation across frames in UI when the producer outputs a changing contour.
- RViz continues to behave identically.

