# Camera stack (remaining focus)

## Scope
- Finalize calibration policy and make camera-to-calibration mapping deterministic.
- Close remaining camera launch warnings by classifying each as `fixed` or `accepted` and documenting the baseline.
- Resolve startup robustness for UI/display path: `robot_serial_display` initial gRPC connection failure before `ros_ui_bridge` is ready.

## Camera identifiers
- Keep RGB pinned to stable `/dev/v4l/by-id/...`.
- Pin depth to stable device identifier where supported; otherwise document bus-path risk and validate runtime selection.
- Ensure calibration file mapping is deterministic for the selected RGB camera and validated at startup.

## Exit criteria
- Replug/reboot keeps RGB and depth assignment stable.
- Startup logs clearly identify selected devices and calibration source.
- Camera launch baseline has no ambiguous unresolved warnings.
- Startup sequence no longer produces unresolved `robot_serial_display` connection issues (either fixed ordering or accepted/retry policy documented).

## Optional follow-ups
- `camera_profile` launch arg for quality/performance presets.
- `camera_health` status stream in `ros_ui_bridge`.
- `camera verify` command for calibration/frames sanity checks.

# Large refactors

## 1) Replace file-based session coupling with explicit runtime session state
- Problem: session data is persisted via `~/.ros/rovi/session/current_launch` and consumed by multiple components.
- Rework: publish session info on a ROS topic (latched) or a tiny service (`/rovi/session_info`) and keep file fallback for compatibility.
- Independent scope: `src/rovi_bringup/rovi_bringup/cli/session.py`, `src/rovi_bringup/rovi_bringup/cli/bag.py`, `src/ros_ui_bridge/ros_ui_bridge/session_info.py`, `src/rovi_bringup/launch/rovi.launch.py`.
- Value:
  - Complexity reduction: medium-high (removes hidden global state).
  - Technical debt reduction: high (clear interface between launch, bagging, and UI).

## 2) Flatten stack composition (mapping/localization/nav)
- Problem: `nav` includes `mapping`/`localization` which include `ekf` + `slam`, creating nested orchestration and repeated parameter wiring.
- Rework: introduce explicit reusable blocks (`state_estimation.launch.py`, `slam_mode.launch.py`) and have `mapping`, `localization`, `nav` compose the same blocks.
- Independent scope: `src/rovi_bringup/launch/nav.launch.py`, `src/rovi_bringup/launch/mapping.launch.py`, `src/rovi_bringup/launch/localization.launch.py`.
- Value:
  - Complexity reduction: high (shallower launch graph and clearer ownership).
  - Technical debt reduction: medium-high (less duplicated parameter plumbing).

## 3) Formalize backend contract as a schema + contract tests
- Problem: contract parity (`/cmd_vel`, feedback topics, TF chain) is policy-driven but not enforced automatically.
- Rework: define machine-readable contract (`docs/contract.yaml` or equivalent) and add parity tests for `real|sim|offline` launch outputs.
- Independent scope: new contract file + test utilities in `tools/` or package tests.
- Value:
  - Complexity reduction: medium (faster reasoning and onboarding).
  - Technical debt reduction: very high (prevents silent real/sim divergence).

## 4) Split UI bridge into explicit data-plane and transport-plane responsibilities
- Problem: single-process/single-executor bridge can accumulate mixed responsibilities (sampling, conversion, transport, session policy).
- Rework: separate data collection/normalization from transport serving and isolate heavy map/image transformations from status/state paths.
- Independent scope: `src/ros_ui_bridge/ros_ui_bridge/*`.
- Value:
  - Complexity reduction: medium-high (clearer module boundaries).
  - Technical debt reduction: high (better scalability and fault isolation).

# Improvements

## 1) Enforce consistent timestamp semantics in UI streams
- Problem: mixed wall-time and ROS-time timestamping across UI streams.
- Rework: standardize to ROS/header time for all stream payloads; keep wall-time as optional debug metadata.
- Independent scope: `src/ros_ui_bridge/ros_ui_bridge/map_node.py` and proto mapping layer.
- Value:
  - Complexity reduction: medium (single time model for consumers).
  - Technical debt reduction: medium-high (fewer time-sync bugs).

## 2) Remove launch-time PYTHONPATH mutation where possible
- Problem: bringup mutates `PYTHONPATH` for venv fallback.
- Rework: package Python deps and startup paths so runtime works without launch-time environment surgery.
- Independent scope: `src/rovi_bringup/launch/robot_bringup.launch.py` + packaging/install docs.
- Value:
  - Complexity reduction: medium.
  - Technical debt reduction: medium-high (fewer environment-specific failures).

## 3) Replace stringly mode logic with validated constants/enums
- Problem: many mode checks are string expressions in launch conditions.
- Rework: central constants + validation helper that fails fast on invalid mode values.
- Independent scope: `src/rovi_bringup/launch/*.py`.
- Value:
  - Complexity reduction: medium.
  - Technical debt reduction: medium.

## 4) Improve observability for throttled forwarding paths
- Problem: forwarder swallows callback exceptions and can hide failures.
- Rework: structured warning/error logging with bounded rate limiting; keep forwarder resilient but observable.
- Independent scope: `src/ros_ui_bridge/ros_ui_bridge/throttled_forwarder.py`.
- Value:
  - Complexity reduction: low-medium (debugging becomes linear).
  - Technical debt reduction: medium.

## 5) Harden rosbag profile management
- Problem: bag profile behavior depends on session conventions and manual YAML maintenance.
- Rework: validate that each known stack has a bag profile; warn/fail when stack/profile mismatch.
- Independent scope: `src/rovi_bringup/rovi_bringup/cli/bag.py`, `src/rovi_bringup/config/bag_topics.yaml`.
- Value:
  - Complexity reduction: medium.
  - Technical debt reduction: medium.

# Suggested execution order (lowest risk first)
1. Improvements 1, 3, and 4.
2. Improvement 2.
3. Improvement 5 (+ CI guardrail for profile coverage).
4. Large refactor 2.
5. Large refactor 1.
6. Large refactors 3 and 4 (heaviest; schedule with contract tests in place).
