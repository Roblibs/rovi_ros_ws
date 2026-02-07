# Camera stack (remaining focus)

## Calibration (required)
- Decide calibration storage policy:
  - Preferred for this robot: commit calibration YAML in-repo (restore-friendly) under `src/rovi_bringup/config/camera_info/` and load it from launch (avoid relying only on `~/.ros/camera_info/` on the Pi).
- Generate RGB calibration YAML for the UVC camera and point the RGB driver to it (the current `/camera/color/camera_info` is uncalibrated).
- Verify RViz / downstream nodes are using the intended frames and `CameraInfo` (rectification / pointcloud projection sanity check).
 
## Frames / TF (decision)
- Source of truth is `src/rovi_description/urdf/rovi.urdf` (no launch-time static TF for camera frames).
- Gazebo sim uses the same frames and publishes simulated camera sensors from the URDF.

## Warnings/errors from `camera` launch (need decisions/actions)
- `v4l2_camera` control read fails with `Permission denied (13)` for control id `10092545`: confirm whether this affects any required settings; if yes, fix via udev/device permissions or avoid querying that control.
- `v4l2_camera` “Control type not currently supported: 6” (Camera Controls): confirm if it is safe to ignore; otherwise, either patch `v4l2_camera` to support it or remove it from the control list.
- RGB format conversion warning (`yuv422_yuy2 => rgb8`): measure CPU impact; if needed, adjust requested pixel format / use MJPEG / reduce resolution or frame rate.
- OpenNI2 “USB events thread - failed to set priority”: decide whether to tolerate (most cases) or configure realtime privileges to reduce risk of dropped frames under load.

## Startup robustness (nice-to-have)
- `robot_serial_display` initially fails to connect to gRPC (`Connection refused`) until `ros_ui_bridge` starts: decide if we want to enforce start order (or keep retry behavior and classify as acceptable).

## New features (camera stack, optional)
- Add a `camera_profile` launch arg (`quality|balanced|performance`) to switch depth/RGB resolution/fps presets without editing params.
- Add a `camera_health` status stream in `ros_ui_bridge` (RGB Hz, depth Hz, dropped frame counters) for runtime diagnostics.
- Add a calibration sanity command (`camera verify`) that checks `CameraInfo` presence, frame IDs, and basic intrinsics consistency.
## Camera identifiers
- Keep RGB bound to stable `/dev/v4l/by-id/...` path (already in use) and document it as mandatory.
- Verify depth camera binding is stable across replug/reboot (OpenNI2 currently reports `2bc5/0614@3/6`, which is bus-topology sensitive).
- Make camera identity to calibration mapping deterministic:
  - Current calibration is loaded from `~/.ros/camera_info/usb_2.0_camera:_usb_camera.yaml`.
  - Confirm uniqueness if multiple similar UVC cameras are ever connected.

## Warnings/errors from `camera` launch (remaining)
- `v4l2_camera` control read fails with `Permission denied (13)` for control id `10092545`: decide if this is ignorable or fix via permissions/udev/patched control handling.
- `v4l2_camera` warning: `Control type not currently supported: 6` (`Camera Controls`): decide if safe to ignore or patch/remove from queried controls.
- RGB conversion warning: `yuv422_yuy2 => rgb8`: measure CPU impact and decide whether to switch format (e.g., MJPEG) or keep current config.
- OpenNI2 warning: `USB events thread - failed to set priority`: decide whether to tolerate or configure realtime privileges to reduce drop risk under load.

# Large refactors

## 1) Decompose launch orchestration into reusable modules
- Problem: `rovi.launch.py` and `robot_bringup.launch.py` duplicate argument declarations, mode condition wiring, and option pass-through.
- Rework: create shared launch helpers (`launch/lib/args.py`, `launch/lib/modes.py`, `launch/lib/includes.py`) and make top-level files thin composition layers.
- Independent scope: `src/rovi_bringup/launch/*`.
- Value:
  - Complexity reduction: high (single source of truth for launch args/conditions).
  - Technical debt reduction: high (removes stringly-typed duplication and drift risk).

## 2) Replace file-based session coupling with explicit runtime session state
- Problem: session data is persisted via `~/.ros/rovi/session/current_launch` and consumed by multiple components.
- Rework: publish session info on a ROS topic (latched) or a tiny service (`/rovi/session_info`) and keep file fallback for compatibility.
- Independent scope: `src/rovi_bringup/rovi_bringup/cli/session.py`, `src/rovi_bringup/rovi_bringup/cli/bag.py`, `src/ros_ui_bridge/ros_ui_bridge/session_info.py`, `src/rovi_bringup/launch/rovi.launch.py`.
- Value:
  - Complexity reduction: medium-high (removes hidden global state).
  - Technical debt reduction: high (clear interface between launch, bagging, and UI).

## 3) Flatten stack composition (mapping/localization/nav)
- Problem: `nav` includes `mapping`/`localization` which include `ekf` + `slam`, creating nested orchestration and repeated parameter wiring.
- Rework: introduce explicit reusable blocks (`state_estimation.launch.py`, `slam_mode.launch.py`) and have `mapping`, `localization`, `nav` compose the same blocks.
- Independent scope: `src/rovi_bringup/launch/nav.launch.py`, `src/rovi_bringup/launch/mapping.launch.py`, `src/rovi_bringup/launch/localization.launch.py`.
- Value:
  - Complexity reduction: high (shallower launch graph and clearer ownership).
  - Technical debt reduction: medium-high (less duplicated parameter plumbing).

## 4) Formalize backend contract as a schema + contract tests
- Problem: contract parity (`/cmd_vel`, feedback topics, TF chain) is policy-driven but not enforced automatically.
- Rework: define machine-readable contract (`docs/contract.yaml` or equivalent) and add parity tests for `real|sim|offline` launch outputs.
- Independent scope: new contract file + test utilities in `tools/` or package tests.
- Value:
  - Complexity reduction: medium (faster reasoning and onboarding).
  - Technical debt reduction: very high (prevents silent real/sim divergence).

## 5) Split UI bridge into explicit data-plane and transport-plane responsibilities
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
1. Improvements 1, 3, 4, 5.
2. Improvement 5 (CI guardrails).
3. Large refactors 1 and 3.
4. Large refactor 2.
5. Large refactor 4 (heaviest, best done with contract tests in place).
