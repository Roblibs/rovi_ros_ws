# Plan: Camera preparation for depth-aware navigation

Context: simulation camera now publishes usable RGB + depth. Next goals are:
1) a camera-only intermediate node that computes “floor clearance” from depth (visualize first),
2) a new `nav` command/stack that uses both LiDAR + depth for mapping/navigation.

This document consolidates and expands existing planning items that currently live in:
- `docs/plan/camera.md`
- `docs/plan/refactors/flatten_stack_composition.md`
- `docs/plan/refactors/backend_contract_tests.md`
- `docs/plan/refactors/session_state.md`

Golden rule reminder: real and sim must look identical to the rest of the stack (topics + TF + behavior).

---

## Decision register (update phase-by-phase)

- D0 (OP0): Canonical depth topic for consumers: **TBD** (`/camera/depth/image` vs `/camera/depth/image_raw`)
- D1 (OP1): Depth `camera_info` required in sim: **TBD** (yes/no; if yes, source = bridge vs static)
- D2 (OP2): Floor-clearance output contract (topic/type/frame): **TBD**
- D3 (OP3): Real depth device identity strategy: **TBD** (serial/URI vs bus-path vs other)
- D4 (OP4): Calibration artifact location + selection rule: **TBD**

---

## Phase 0 — Lock the “camera contract” (design decisions first)

Before writing new perception nodes, decide and document the *canonical* interfaces that downstream consumers will rely on.

### 0.1 Canonical topics (depth/RGB + camera_info)

Current mismatch to resolve (examples):
- Sim bridges depth image to `/camera/depth/image` (see `src/rovi_sim/launch/gazebo_sim.launch.py`).
- UI config expects depth rate on `/camera/depth/image_raw` (see `src/ros_ui_bridge/config/default.yaml`).
- RViz camera config displays `/camera/depth/image` (see `src/rovi_description/rviz/rovi_camera.rviz`).

Decide:
- Which depth topic is the canonical input for new nodes (pick one, then make real+sim provide it):
  - Option A: canonicalize on `/camera/depth/image` (and keep `/camera/depth/image_raw` as an optional “raw” feed).
  - Option B: canonicalize on `/camera/depth/image_raw` (and make sim match).
- Do we require `camera_info` for:
  - depth only (`/camera/depth/camera_info`) for floor clearance and depth→3D projection?
  - RGB too (`/camera/color/camera_info`)?

Acceptance criteria:
- A single “consumer contract” paragraph exists in this doc listing the canonical camera topics.
- Real and sim both publish those canonical topics (or provide a deterministic remap/relay plan).

### 0.2 Canonical frames and TF requirements

Decide:
- Required frame_ids on the image streams (do they need to match URDF optical frames exactly?).
- Fixed TF chain used by camera-derived products (e.g. floor clearance in `base_footprint`).

Acceptance criteria:
- “Required frame_ids” are enumerated.
- Any frame normalization policy is explicit (no “guess TF at runtime”).

### 0.3 Time semantics (ROS time vs wall time)

Decide:
- New depth-derived nodes must use message header stamps (recommended) and respect `use_sim_time`.

Acceptance criteria:
- A 1-line timestamp rule exists and is referenced by later phases.

### 0.4 Compute budget + QoS conventions (so RViz/Nav2 remain stable)

Decide:
- Image QoS policy for depth/RGB (best-effort vs reliable) for both real and sim.
- Rate caps/downsampling policy for UI-facing streams vs algorithm-facing streams.

Acceptance criteria:
- A short QoS + rate note exists for camera streams and for any derived products.

Open points to answer in Phase 0 (collect here, resolve before Phase 1):
- OP0: Which depth topic becomes canonical: `/camera/depth/image` or `/camera/depth/image_raw`?
- OP1: Do we require `camera_info` in sim for depth? If yes, where does it come from (bridge vs static)?
- OP2: What is the canonical output topic (and frame) for “floor clearance” so UI + future Nav2 can share it?

---

## Phase 1 — Refactor: Flatten stack composition (mapping/localization/nav)

Moved from `docs/plan/refactors/flatten_stack_composition.md`.

### Problem
`nav` includes `mapping`/`localization` which include `ekf` + `slam`, creating nested orchestration and repeated parameter wiring.

### Rework
Introduce explicit reusable blocks (for example: `state_estimation.launch.py`, `slam_mode.launch.py`) and have `mapping`, `localization`, `nav` compose the same blocks.

### Independent scope
- `src/rovi_bringup/launch/nav.launch.py`
- `src/rovi_bringup/launch/mapping.launch.py`
- `src/rovi_bringup/launch/localization.launch.py`

### Value
- Complexity reduction: high (shallower launch graph and clearer ownership).
- Technical debt reduction: medium-high (less duplicated parameter plumbing).

Phase 1 camera-specific add-on (do *after* Phase 0 decisions):
- Add a reusable “perception block” include point where camera-derived nodes can be composed consistently across stacks without copy/paste.

Acceptance criteria:
- `nav`, `mapping`, `localization` are shallow compositions of shared blocks.
- Adding a new node to “nav + depth” is a single-include change (not 3 files).

---

## Phase 2 — Refactor: Formalize backend contract as a schema plus contract tests

Moved from `docs/plan/refactors/backend_contract_tests.md`.

### Problem
Contract parity (`/cmd_vel`, key feedback topics, TF chain) is policy-driven but not enforced automatically.

### Rework
Define machine-readable contract (`docs/contract.yaml` or equivalent) and add parity tests for `real|sim|offline` launch outputs.

### Independent scope
- New contract file
- Test utilities in `tools/` or package tests

### Value
- Complexity reduction: medium (faster reasoning and onboarding).
- Technical debt reduction: very high (prevents silent real/sim divergence).

Phase 2 camera-specific extension:
- Add camera contract items (canonical depth/RGB topics, frame_ids, and required camera_info topics) once Phase 0 is resolved.

Acceptance criteria:
- Contract tests fail if sim/real diverge on the camera contract (topic names/types and required TF frames).

---

## Phase 3 — Refactor: Replace file-based session coupling with explicit runtime session state

Moved from `docs/plan/refactors/session_state.md`.

### Problem
Session data is persisted via `~/.ros/rovi/session/current_launch` and consumed by multiple components.

### Rework
Publish session info on a ROS topic (latched) or a tiny service (`/rovi/session_info`) and keep file fallback for compatibility.

### Independent scope
- `src/rovi_bringup/rovi_bringup/cli/session.py`
- `src/rovi_bringup/rovi_bringup/cli/bag.py`
- `src/ros_ui_bridge/ros_ui_bridge/session_info.py`
- `src/rovi_bringup/launch/rovi.launch.py`

### Value
- Complexity reduction: medium-high (removes hidden global state).
- Technical debt reduction: high (clear interface between launch, bagging, and UI).

Why this matters for “nav + depth”:
- You’ll likely add new stacks/variants (e.g. `nav_depth`) and want UI/bagging to follow deterministically without relying on a global file convention.

Acceptance criteria:
- UI fixed-frame and bagging selection can resolve session state without reading a file (file remains as fallback only).

---

## Phase 4 — Camera stack robustness (real hardware readiness)

Moved from `docs/plan/camera.md`.

### Calibration
- calibration file has no specific camera id, serial usage to be clarified.

### USB enumeration
- Keep RGB pinned to stable `/dev/v4l/by-id/...`.
- Pin depth to stable device identifier where supported; otherwise document bus-path risk and validate runtime selection.

### Controls & config
- on sttartup control 10092545 is failing with an error
- list all needed controls and evaluate their usage
- check if any configuration needed for performance optimization

Acceptance criteria:
- Deterministic device selection is documented and validated.
- Startup control failures are either fixed or explicitly gated/ignored with rationale.

Open points to resolve during Phase 4:
- OP3: What is the authoritative identity for depth camera selection (serial, URI, bus path)?
- OP4: Where do calibration artifacts live, and how are they selected per device?

---

## Phase 5 — First feature slice (after prep): Floor-clearance from depth (camera-only)

Goal: create a minimal new node that computes a “floor clearance” metric from depth and publishes it for visualization and future Nav2 integration.

Design decisions needed (resolve after Phase 0, before implementation):
- Output message type + topic name (recommend: a small custom message only if needed; otherwise use existing standard messages).
- Output frame semantics (likely `base_footprint`) and timestamp rule (use depth header stamp).
- Visualization path: RViz overlay vs UI bridge field vs both.

Acceptance criteria:
- Works in `robot_mode=sim` and `robot_mode=real` without stack-specific remaps.
- Can be enabled/disabled as a composable “perception block” (ties back to Phase 1).

---

## Suggested “implementation call” breakdown (phase-by-phase)

Call 1 (design + docs only):
- Complete Phase 0 decisions + update this doc with final canonical contracts.

Call 2 (launch refactor):
- Implement Phase 1 flattening, plus a named include point for “perception blocks”.

Call 3 (contract enforcement):
- Implement Phase 2 contract schema + tests, including camera contract items.

Call 4 (runtime coupling cleanup):
- Implement Phase 3 session state interface (topic/service) with file fallback.

Call 5 (hardware robustness):
- Implement Phase 4 camera determinism + controls cleanup.

Call 6 (first depth-derived product):
- Implement Phase 5 floor-clearance node + visualization.
