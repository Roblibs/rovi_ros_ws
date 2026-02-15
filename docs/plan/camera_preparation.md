# Plan: Camera preparation for depth-aware navigation

Context: simulation camera now publishes usable RGB + depth. Next goal is a new `nav` command/stack that uses both LiDAR + depth for mapping/navigation.

This document consolidates and expands existing planning items that currently live in:
- `docs/plan/camera.md`
- `docs/plan/refactors/flatten_stack_composition.md`
- `docs/plan/refactors/backend_contract_tests.md`
- `docs/plan/refactors/session_state.md`

Golden rule reminder: real and sim must look identical to the rest of the stack (topics + TF + behavior).

---

## Decision register (update phase-by-phase)

- D0 (OP0): Canonical consumer topics prefer `image` over `image_raw`: **depth = `/camera/depth/image`**, **rgb = `/camera/color/image`** (raw topics remain optional/debug)
- D1 (OP1): `camera_info` is mandatory and always-on: **depth = required** (sim+real), **rgb = required** (sim+real). Each camera stream must publish both `*/image` and `*/camera_info` (see Phase 0).
- D3 (OP3): Real depth device identity strategy: **TBD** (serial/URI vs bus-path vs other)
- D4 (OP4): Calibration artifact location + selection rule: **TBD**
- D5 (OP5): Sim intrinsics match real reference: **Yes**. Sim `CameraInfo.K` should be aligned to a declared “reference real camera model”, and Gazebo `<sensor>` params (width/height/hfov) should be derived from the same reference.
- D6 (OP1): Sim `camera_info` publication: **publish from the same reference YAML as real** (camera_info_manager-style), and validate against URDF/Gazebo sensor params via contract tests.
- D7 (OP6): One robot = one model: **Yes**. Optional device serial may be stored in the reference YAML for verification-only; if present, hard-fail on mismatch.
- D8 (OP7): Intrinsics consistency: **YAML is authoritative**. Hard-fail when `width/height` disagree with the image stream; do not hard-fail just because `fx != fy` (use them as-is and accept the implied FOV consequences).
- D9 (OP8): `camera_info` update rate: **low-rate / on-change** (not per-frame). Use durable QoS so late subscribers get the latest without high-frequency repetition.
- D10 (OP9): QoS: **images = best-effort**, **camera_info = reliable + transient-local** (latched behavior).

---

## Phase 0 — Lock the “camera contract” (design decisions first)

Before writing new perception nodes, decide and document the *canonical* interfaces that downstream consumers will rely on.

### 0.1 Canonical topics (depth/RGB + camera_info)

Current mismatch to resolve (examples):
- Sim bridges depth image to `/camera/depth/image` (see `src/rovi_sim/launch/gazebo_sim.launch.py`).
- UI config tracks camera rates on `/camera/depth/image` and `/camera/color/image` (see `src/ros_ui_bridge/config/default.yaml`).
- RViz camera config displays `/camera/depth/image` + `/camera/color/image` (see `src/rovi_description/rviz/rovi_camera.rviz`).

Decide:
- Canonical depth consumer topic (chosen): `/camera/depth/image`
- Canonical RGB consumer topic (chosen): `/camera/color/image`
- Keep raw topics as optional/debug feeds (`*/image_raw`) as long as they are not used as “the contract”.
- Mandatory camera model topics:
  - `/camera/depth/camera_info` is required (real+sim).
  - `/camera/color/camera_info` is required (real+sim).
- Canonical “always present” pairs (per camera stream):
  - Depth: `/camera/depth/image` + `/camera/depth/camera_info`
  - RGB: `/camera/color/image` + `/camera/color/camera_info`

Notes:
- If a driver only natively provides `*/image_raw`, publish an explicit alias topic `*/image` (byte-identical passthrough) so consumers can rely on `*/image` without guessing. Document clearly that `*/image` is not “rectified” unless a later phase introduces rectification.
- If a driver can publish `*/image` directly, it should do so and still publish `*/image_raw` as optional/debug.

#### 0.1.1 Sim `camera_info` strategy (OP1)

Options (ranked):
1) Bridge/generate from Gazebo sensor parameters (recommended): make Gazebo the source-of-truth for sim intrinsics and bridge/publish `sensor_msgs/CameraInfo` alongside the images.
   - Pros: “ROS-y” because the simulated sensor that publishes images also publishes its model; no duplicated specs; timestamps align with `/clock`; least surprising for downstream consumers.
   - Cons: depends on what Gazebo/`ros_gz_*` exposes; may require bridging Gazebo Transport `CameraInfo` topics (or using a bridge that can compute `CameraInfo` from the sensor).
2) Static `camera_info` publisher (YAML) on the ROS side:
   - Pros: simplest to implement; deterministic; works even if Gazebo doesn’t expose camera model topics.
   - Cons: duplicates spec in two places (URDF/SDF vs YAML); can drift; requires extra discipline/tests to keep “sim camera_info matches sim camera”.
3) Compute from robot description (derive `K` from `horizontal_fov` + `width/height` at runtime):
   - Pros: single spec location (robot description); deterministic; avoids a separate YAML for sim.
   - Cons: ideal-pinhole-only; no distortion; requires custom code; less transparent than “sensor publishes its model”.

Phase-0 recommendation:
- Default to Option 2 (simplest + closest behavior to real): publish `CameraInfo` from the same reference YAML used on real hardware.
- Option 1 is acceptable only if it’s trivially available in your current Gazebo/ros_gz pipeline; still enforce Phase-2 contract tests to prevent drift.

#### 0.1.2 “Sim should match real” policy (avoid random params)

Goal: sim camera parameters should be explicit, reviewable, and close to real by default.

Policy proposal:
- Declare sim camera parameters explicitly in `src/rovi_description/urdf/rovi.urdf` under the Gazebo `<sensor>` blocks: resolution, `horizontal_fov`, clip range, update rate, and (optional) noise.
- Treat real calibration YAML (from actual calibration) as authoritative for real `CameraInfo` (distortion + intrinsics).
- Enforce determinism and “close to real” via Phase-2 contract tests:
  - `CameraInfo.width/height` matches the image shape.
  - `header.frame_id` matches the required optical frame id.
  - `K` is deterministic and within tolerance of the expected values derived from the declared sim spec (or from a declared reference file, if you choose to align sim `K` to real).

Decision:
- OP5 resolved: Sim `K` aligns to a declared “reference real camera model”.

How to make Gazebo params match that reference (principle):
- Choose reference `{width, height, fx, fy, cx, cy}` (per stream: depth + rgb).
- Set Gazebo `<image><width/height>` to match `{width,height}`.
- Set Gazebo `<horizontal_fov>` from the reference `fx`:
  - `horizontal_fov = 2 * atan(width / (2 * fx))`
  - (Vertical FOV is implied by aspect ratio; check that `fy` is consistent. If not, decide whether to prioritize `fx` or `fy`.)

Current note:
- `src/rovi_bringup/config/camera_info/color.yaml` implies an RGB HFOV that likely differs from the current URDF value; OP7 covers how we reconcile this so sim matches real.

Decisions:
- OP6 resolved: reference camera models live in-repo as YAML and are updated/replaced when the camera changes or is recalibrated. We do not support multiple camera serials in parallel; serial is verification-only.
- OP7 resolved: width/height must match the stream (hard fail). `fx/fy` mismatch is allowed; treat the values as authoritative and accept the implied FOV.

Remaining implementation detail (Phase 4 / Phase 2 tests):
- If a reference YAML includes a serial, decide how we fetch the live serial (depth + rgb) and where we enforce the hard-fail check.

Proposed reference files (single source-of-truth):
- RGB: `src/rovi_bringup/config/camera_info/color.yaml` (exists today)
- Depth: `src/rovi_bringup/config/camera_info/depth.yaml` (placeholder now; replace with real calibration when available)

Hardcoding policy:
- It’s OK to hardcode these *default* paths in launch defaults/config, as long as they can be overridden via launch args/env for field debugging.
- Optional verification-only field in the YAML:
  - `device_serial`: if empty/missing, no verification; if set, mismatch is a hard failure.

Acceptance criteria:
- A single “consumer contract” paragraph exists in this doc listing the canonical camera topics.
- Real and sim both publish those canonical topics (or provide a deterministic remap/relay plan).

### 0.2 Canonical frames and TF requirements

Decide:
- Required `header.frame_id` on the image streams (must match URDF optical frames exactly).
- Fixed TF chain used by camera-derived products (for depth-aware navigation, typically in `base_footprint`).

Acceptance criteria:
- “Required frame_ids” are enumerated.
- Any frame normalization policy is explicit (no “guess TF at runtime”).

Required frame_ids (contract):
- Depth images + camera_info must use `camera_depth_optical_frame`
- RGB images + camera_info must use `camera_color_optical_frame`

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
- None (Phase 0 decisions are complete).

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
