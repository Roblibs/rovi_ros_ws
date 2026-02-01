# Yahboom AI View Depth Camera — why OpenNI2 (and what Yahboom did)

This document captures what we learned from Yahboom’s tutorial workspace and explains why `rovi_ros_ws` uses OpenNI2 for this camera instead of chasing Orbbec’s “newer” ROS driver stacks.

Reference workspace:

- Yahboom download: `Code-ROS2/orbbec_ws` from https://www.yahboom.net/study/AIVIEW_Camera
- Relevant subfolders:
  - `orbbec_ws/src/OrbbecSDK_ROS2/orbbec_camera` (OrbbecSDK-based driver + bundled SDK)
  - `orbbec_ws/src/ros2_astra_camera` (OpenNI2 + UVC integrated driver + bundled OpenNI2)

Scope:

- Driver integration strategy (SDK choice, binaries vs non-binaries, node architecture, and the Astra Stereo S U3 “640x400” reality).
- Calibration is intentionally not the focus here.

## 1) Why we don’t “just use Orbbec” (and why we stopped chasing OrbbecSDK v1/v2 paths)

The confusion is that “Orbbec driver” can mean multiple incompatible stacks:

- **Legacy/OpenNI-era devices** (like Astra variants) are commonly supported via **OpenNI2** (and older OrbbecSDK v1-era solutions).
- **Newer Orbbec ROS2 drivers** (what you typically get via modern ROS distro packages) tend to target the newer SDK family and may not enumerate legacy/OpenNI-style devices at all (our observed symptom was “USB present but device list empty”).

Yahboom’s tutorial works with an OrbbecSDK-based node largely because:

- It bundles an older OrbbecSDK-based stack that explicitly includes an `AstraStereoSU3` profile (with depth/IR `640x400`) and bundles the matching SDK binaries.

We intentionally do not keep chasing this route in `rovi_ros_ws` because it conflicts with the repo’s integration constraints and increases maintenance risk:

- It typically implies **pinning to vendor SDK versions** and/or legacy branches, which can lag OS/toolchain support (Ubuntu 24.04 is a common pain point).
- It tends to require **bundling vendor binaries** or installing them outside of the ROS dependency model, which makes the stack harder to reproduce and review.
- Most importantly: we already have a working depth path via OpenNI2, so the “benefit” would have to be huge to justify taking on a second, higher-maintenance driver line.

So the current policy is:

- Use **OpenNI2** for depth/IR (works with this camera on the robot).
- Keep RGB as a separate **UVC** device for now.
- Patch only the smallest possible ROS-layer pieces when needed.

## 2) Yahboom OpenNI vs our OpenNI2: same underlying stack, different ROS packaging

Both approaches accept the same hardware reality:

- Depth/IR is accessible via **OpenNI2** and the camera exposes `640x400` / `320x200` (not `640x480` / `320x240`).
- RGB is a separate **UVC** camera.

### Yahboom’s OpenNI2 driver strategy (in `orbbec_ws/src/ros2_astra_camera`)

Yahboom does not use the community `openni2_camera` package. Instead they ship a custom driver (`astra_camera_node`) that:

- Directly accepts width/height stream parameters in launch (so `640x400` is just a default in their `*.launch.xml`).
- Integrates UVC RGB (via `libuvc`) into the same node.
- Ships OpenNI2 redistributables in-tree for convenience.

This is “one node does everything”, at the cost of bundling binaries and owning a larger driver.

### `rovi_ros_ws` strategy (community driver + minimal patch)

`rovi_ros_ws` uses the standard ROS 2 `openni2_camera` driver and patches only what we must:

- The upstream driver selects modes by **named presets** (e.g. VGA → `640x480`).
- This camera’s supported presets are different (`640x400` / `320x200`), so we add Orbbec-specific mode names and default to `ORBBEC_640x400_30Hz`.

Net: we get the “ROS-native” driver behavior with a tiny compatibility patch, and we keep vendor SDK installs out-of-repo.

## 3) Minimal comparison (what to keep in mind)

- Yahboom solves the U3 “640x400” quirk by setting the right defaults in their custom node (and by bundling the runtime).
- We solve the same quirk by patching `openni2_camera`’s mode table so the standard driver can select the correct OpenNI2 mode.

If we later decide we need a “single node does RGB-D + registration + pointcloud”, Yahboom’s integrated-node approach is a good reference for a future “full” camera stack — but we’d still want to keep it integration-friendly (no bundled vendor binaries) and accept the added driver/runtime surface area described above.
