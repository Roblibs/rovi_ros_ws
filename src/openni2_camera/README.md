# openni2_camera (workspace overlay)

This workspace vendors `openni2_camera` (upstream version `2.2.2`) under `src/openni2_camera` to apply a small compatibility patch for the Orbbec Astra Stereo S U3 (Yahboom AI View Depth Camera).

## Why this exists

The upstream `openni2_camera` node uses a hard-coded table of named modes (e.g. `VGA_30Hz` → `640x480@30`). On this camera + Orbbec OpenNI2 driver, the supported modes are **`640x400`** / **`320x200`** / **`1280x800`** (see `tools/depth/openni2_list_modes.cpp`), so the default `VGA_30Hz` selection fails with errors like:

- `Unsupported depth video mode - Resolution: 640x480@30Hz Format: Depth 1mm`

In practice this can lead to “streaming” topics that are black/invalid.

## What we patch

- Add Orbbec mode names to the table:
  - `ORBBEC_640x400_{15,30,60}Hz`
  - `ORBBEC_320x200_{15,30,60}Hz`
  - `ORBBEC_1280x800_{15,30}Hz`
- Default `ir_mode` and `depth_mode` to `ORBBEC_640x400_30Hz` so `ros2 run openni2_camera openni2_camera_driver` works out-of-the-box on this robot.
- Skip trying to configure color video modes if the device has no OpenNI2 color sensor (this camera’s RGB is a separate UVC device, not OpenNI2).

## How it is used

When you build this workspace (`build`) and source it (`ws` / `setup`), ROS will prefer the overlay package in `install/` over the system apt package.

## Updating the vendored copy

If you want to refresh this directory from upstream:

- Upstream repo: `https://github.com/ros-drivers/openni2_camera`
- Branch/tag aligned to ROS Jazzy: `2.2.2`

Keep the patch minimal and re-run `tools/depth/openni2_list_modes` + `ros2 run openni2_camera openni2_camera_driver` as the validation.
