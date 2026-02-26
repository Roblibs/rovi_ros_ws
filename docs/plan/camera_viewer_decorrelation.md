# Plan — Camera viewer decorrelation (Wi‑Fi-safe view topics)

Problem statement:
- When a Wi‑Fi viewer (e.g., RViz on a laptop) subscribes to raw camera topics, the camera publish rate can collapse (e.g. color ~5 Hz, depth ~0–1 Hz).
- Root causes can include: insufficient Wi‑Fi throughput, DDS serialization pressure, and (especially for depth) **RELIABLE** QoS causing backpressure when the link is lossy/slow.

Goal:
- Keep the **robot-side camera contract** stable for local consumers (30 Hz feeds, same topic names).
- Provide **separate, explicitly bandwidth-managed viewer topics** so a remote viewer can’t destabilize the primary streams.

Non-goals (for this plan):
- Changing the existing raw camera topic names.
- Implementing registered RGB‑D / pointcloud (out of scope).

Related analysis / current state:
- See `docs/depth_camera_analysis.md` for current publishers, gateway `hz_*` derivation, and QoS notes.

## Proposed interface (new “view topics”)

Pick one naming convention (recommend `_view`):
- Color: `/camera/color/image_view`
- Depth: `/camera/depth/image_view`

Optional (if using `image_transport`):
- Color: `/camera/color/image_view` with `compressed` transport enabled
- Depth: `/camera/depth/image_view` with `compressedDepth` transport enabled (if supported/available)

Constraints:
- Viewer topics must be **best-effort** and effectively **keep-last=1** semantics to avoid queue buildup.
- Viewer topics must have a **hard rate cap** (default target: **5 Hz**) to bound bandwidth.

## Architecture options

Option A (recommended): on-robot “viewer forwarder” node
- Subscribes locally to:
  - `/camera/color/image`
  - `/camera/depth/image_raw` (preferred) or `/camera/depth/image` (if needed)
- Republish to:
  - `/camera/color/image_view`
  - `/camera/depth/image_view`
- Behavior:
  - “Latest-sample” republish: store newest incoming frame; publish only at 5 Hz tick; drop intermediate frames.
  - QoS:
    - Subscriptions: sensor-data / best-effort when possible (avoid affecting publishers).
    - Publications: best-effort, small queue, drop-oldest.
  - Optional compression:
    - Publish via `image_transport` to allow `compressed` / `compressedDepth` on the viewer side.

Option B: use existing CLI tools (`image_transport` republish + `topic_tools throttle`)
- Pros: fast to try manually.
- Cons: harder to standardize (startup ordering, QoS knobs, “latest-sample” semantics, logging/metrics).

Option C: move viewer forwarding into `ros_ui_bridge`
- Pros: a single “gateway owns viz-plane” place.
- Cons: increases coupling/complexity in the gateway; may conflict with the existing UI bridge goals (gRPC telemetry vs ROS viz topics).

## RViz enforcement (prevent accidental subscription to raw topics)

Update RViz configs used by `view camera` to subscribe only to view topics:
- Update `src/rovi_description/rviz/rovi_camera.rviz` Image displays to point to:
  - `/camera/color/image_view`
  - `/camera/depth/image_view`
- If the RViz config currently subscribes by default to raw topics, treat that as a correctness issue: remote viewing must use view topics only.

## “Unreliable 30 Hz” fallback policy

Interpretation:
- If the original stream is unstable over Wi‑Fi, the system should continue to deliver a stable view stream (5 Hz best-effort).

Implementation policy for the forwarder:
- The forwarder always runs a fixed-rate publish loop (5 Hz default).
- On each tick, publish the latest cached frame if it is not stale; otherwise publish nothing (or publish a diagnostic).
- Optionally expose:
  - Staleness timeout (e.g. 1 s) to avoid publishing ancient frames.
  - Diagnostics: last-received age, publish tick rate, dropped-frame counters.

## Verification plan (future, when implementing)

Functional:
- With no remote viewer, raw topics remain ~30 Hz.
- With remote viewer on Wi‑Fi, raw topics remain ~30 Hz and view topics remain ~5 Hz.

Network:
- Measure `ros2 topic bw` for view topics and ensure they stay within expected Wi‑Fi budget.
- Run `iperf3` between robot and viewer to bound throughput; ensure view topics fit below a safe fraction.

QoS:
- Validate with `ros2 topic info -v` that:
  - view topics are best-effort on the wire
  - queues are shallow

## Rollout steps

1. Add forwarder node/package (or extend an existing viz-plane package).
2. Wire it into bringup (likely via `rovi_bringup` when `stack:=camera` and/or when a new launch arg `viewer_topics:=true` is enabled).
3. Update RViz configs to use view topics only.
4. (Optional) Add status fields to `ros_ui_bridge`:
   - `hz_color_view`, `hz_depth_view`
   - last-view-frame age

