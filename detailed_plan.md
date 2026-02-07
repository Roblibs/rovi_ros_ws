# Detailed Plan: Camera Stack Remaining Work

## 1) What is already done (removed from plan)
- Calibration is active and loaded at runtime.
- Core camera stack launches successfully.
- Transient `robot_serial_display` startup retry resolves automatically.

### What is already well-scoped
- TF ownership decision is clear:
  - Camera frames come from `src/rovi_description/urdf/rovi.urdf`
  - No launch-time static TF duplication
- Optional features are practical and incremental:
  - `camera_profile`
  - `camera_health`
  - `camera verify`
## 2) Remaining scope
- Focus area A: camera identifiers (device identity stability and calibration mapping).
- Focus area B: camera warnings/errors still visible in launch logs.

## 3) Track A: Camera identifier hardening

### Goal
- Ensure RGB and depth cameras are pinned to stable identifiers across reboot/replug and cannot silently swap identity.

### Tasks
1. RGB identifier policy:
   - Keep using `/dev/v4l/by-id/...` for RGB device selection.
   - Add a short startup assert/log that shows the resolved real device path.
2. Depth identifier policy:
   - Verify whether OpenNI2 can be pinned by serial/device URI instead of topology string like `2bc5/0614@3/6`.
   - If supported, enforce serial/device-id selection in launch params.
   - If not supported, document the bus-path risk and add a quick runtime check.
3. Calibration mapping guard:
   - Confirm that `~/.ros/camera_info/usb_2.0_camera:_usb_camera.yaml` uniquely maps to this RGB camera.
   - Add a check to fail/warn when the expected calibration file is missing or camera name changes.

### Exit criteria
- Replug/reboot test keeps same RGB + depth assignment.
- Startup logs clearly identify selected RGB/depth devices.
- Calibration file resolution is deterministic and validated.

## 4) Track B: Warning/error closure

### Goal
- Convert current warnings/errors into explicit accepted behavior or concrete fixes.

### Tasks
1. `v4l2_camera` control read permission error:
   - `Failed getting value for control 10092545: Permission denied (13)`.
   - Determine control purpose and whether it is operationally required.
   - Choose: ignore + document, or fix via permissions/udev/config/driver handling.
2. Unsupported control type warning:
   - `Control type not currently supported: 6` (`Camera Controls`).
   - Choose: ignore (if harmless) or patch/filter queried control list.
3. Pixel format conversion warning:
   - `yuv422_yuy2 => rgb8` conversion may be slow.
   - Measure CPU and frame stability under expected camera load.
   - Decide whether to keep YUYV conversion or switch RGB path (e.g., MJPEG or lower resolution/fps).
4. OpenNI2 scheduling warning:
   - `USB events thread - failed to set priority`.
   - Measure dropped frames under stress.
   - Choose: tolerate and document, or configure realtime privileges.

### Exit criteria
- Each warning/error is marked `fixed` or `accepted`.
- No unresolved ambiguous warnings remain in the camera launch baseline.

## 5) Suggested order
1. Track A (identifiers first, to prevent false debugging due to device swaps).
2. Track B.1 + B.2 (`v4l2` control issues).
3. Track B.3 (format/performance).
4. Track B.4 (OpenNI2 priority under load).

## 6) Final deliverable
- A clean "camera baseline" checklist with:
  - Stable identifier rules (RGB + depth)
  - Calibration mapping rule
  - Warning/error disposition table (`fixed` vs `accepted`)
