# Detailed Plan: Camera Stack

## 1) Evaluation of current camera-stack items

### What is already well-scoped
- TF ownership decision is clear:
  - Camera frames come from `src/rovi_description/urdf/rovi.urdf`
  - No launch-time static TF duplication
- Optional features are practical and incremental:
  - `camera_profile`
  - `camera_health`
  - `camera verify`

### What still needs implementation decisions
- Calibration workflow is declared, but not yet operationally defined end-to-end:
  - How YAMLs are named per device/serial
  - How launch selects the right calibration file
  - How sim/offline should behave when calibration files are missing
- Warning handling needs a "tolerate vs fix" policy per warning with measurable criteria.
- Startup race (`robot_serial_display` vs `ros_ui_bridge`) needs an explicit reliability target (acceptable retries vs enforced ordering).

### Dependency summary
- Strongly coupled (must be sequential): calibration + frame correctness + downstream validation.
- Mostly independent: warning triage, startup robustness, `camera_health`.
- Partially dependent:
  - `camera_profile` depends on warning/performance measurements.
  - `camera verify` depends on finalized calibration + frame conventions.

## 2) Independent Workstreams (can run in parallel)

### Track A: Warning triage and policy locks
- Scope:
  - `v4l2_camera` permission/control warnings
  - unsupported control type warning
  - RGB conversion CPU cost
  - OpenNI2 thread priority warning
- Deliverables:
  - Per-warning decision: `ignore`, `mitigate in config`, or `fix in system setup`.
  - One short decision matrix in docs (warning -> impact -> action).
  - If needed: udev/permissions notes and launch parameter adjustments.
- Exit criteria:
  - Camera launch is stable and warnings are either removed or documented as accepted.
  - CPU impact measured under at least one representative profile.

### Track B: Startup robustness for display/UI bridge
- Scope:
  - Handle initial `Connection refused` cleanly between `robot_serial_display` and `ros_ui_bridge`.
- Deliverables (choose one implementation path and keep retries safe):
  - Keep retry model but reduce noisy logs and classify as expected transient.
  - Or enforce startup ordering/readiness before first connection attempt.
- Exit criteria:
  - Clean startup without persistent error spam.
  - No regression in reconnection behavior after bridge restart.

### Track C: `camera_health` stream (optional but independent)
- Scope:
  - Publish RGB/depth rates and dropped frame counters in `ros_ui_bridge`.
- Deliverables:
  - New health payload fields + producer logic.
  - Configurable update rate and bounded logging.
- Exit criteria:
  - Values visible and sane during idle + movement.
  - No noticeable overhead on control/navigation paths.

## 3) Single Coupled Flow (must be sequential)

### Flow S: Calibration + Frames + Consumer correctness

#### Phase S1: Calibration storage and launch wiring
- Define in-repo calibration policy under `src/rovi_bringup/config/camera_info/`.
- Implement launch-time loading strategy for RGB camera calibration.
- Define fallback behavior when calibration file is absent.
- Output:
  - Deterministic file lookup and clear runtime log on which calibration is loaded.

#### Phase S2: Generate and commit RGB calibration artifacts
- Capture calibration dataset and generate YAML for UVC RGB stream.
- Store committed YAML according to S1 naming rules.
- Point camera launch config to this file.
- Output:
  - `/camera/color/camera_info` is calibrated and stable across reboots.

#### Phase S3: Frame and CameraInfo validation in the full stack
- Verify frame IDs and TF chain compatibility with URDF-owned frames.
- Validate RViz rectification/projection behavior.
- Validate downstream consumers that rely on `CameraInfo`.
- Output:
  - No frame mismatch warnings and expected projection behavior.

#### Phase S4: Sim/offline parity validation
- Confirm sim uses same frame contracts and topic naming policy.
- Ensure offline inspection mode still launches without calibration regressions.
- Output:
  - Real/sim/offline contract parity retained for camera topics and frames.

## 4) Dependent Optional Features (after Flow S)

### Feature D1: `camera_profile` launch arg
- Depends on Track A measurements.
- Add presets: `quality|balanced|performance`.
- Include profile-specific RGB/depth resolution/fps defaults.
- Validation:
  - Profile switch works without manual param edits.
  - CPU/FPS behavior matches documented expectations.

### Feature D2: `camera verify` command
- Depends on completed Flow S artifacts.
- Checks:
  - `CameraInfo` presence for required camera topics
  - expected frame IDs
  - basic intrinsics sanity
- Validation:
  - Non-zero exit on missing/invalid camera metadata.
  - Human-readable diagnostics for field debugging.

## 5) Suggested execution order
1. Track A (warning triage baseline).
2. Flow S (S1 -> S2 -> S3 -> S4).
3. Track B (startup robustness) and Track C (`camera_health`) in parallel.
4. Feature D1 (`camera_profile`) then Feature D2 (`camera verify`).

## 6) Done criteria for this camera-stack program
- Namespaces and frame ownership are enforced in launch/runtime behavior.
- Calibrated RGB `CameraInfo` is repo-managed and reproducible.
- Warning set is intentionally resolved (fixed or explicitly accepted).
- Optional diagnostics (`camera_health`, `camera verify`) are functional if enabled.
- Real/sim/offline camera contracts remain aligned with bringup policy.
