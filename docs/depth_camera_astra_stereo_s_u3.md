# Depth camera — Orbbec Astra Stereo S U3 (Yahboom “AI View Depth Camera”)

This device presents as **two separate Linux devices** over one cable:

- **Depth/IR** via **OpenNI2** (Orbbec) → ROS driver: `openni2_camera`
- **Color** via **UVC / V4L2** → ROS driver: `v4l2_camera`

ROVI’s current policy is to keep depth and color as **separate feeds** (no registered RGB-D / pointcloud yet) and provide a stable TF mount and consistent ROS topic names for RViz and downstream stacks.

## Status

- Streaming works: depth + color images show in RViz via `stack:=camera`.
- Depth modes for this camera are `640x400` / `320x200` (not `640x480` / `320x240`).
- Color calibration is required for correct `CameraInfo` (see “Calibration”).

## Run

On the robot:
```bash
camera
```

Equivalent:
```bash
ros2 launch rovi_bringup rovi.launch.py robot_mode:=real stack:=camera rviz:=false
```

On a PC to visualize:
```bash
view camera
```

## ROS contract (topics + TF)

Depth (OpenNI2) under `/camera/depth`:
- `/camera/depth/image_raw` (`16UC1`, mm)
- `/camera/depth/image` (`32FC1`, m)
- `/camera/depth/camera_info`

Color (UVC) under `/camera/color`:
- `/camera/color/image_raw`
- `/camera/color/camera_info`

TF is published by `robot_state_publisher` from `src/rovi_description/urdf/rovi.urdf`:
- `base_link -> camera_link` (mount)
- `camera_link -> camera_depth_frame` (approx internal offset)
- `camera_depth_frame -> camera_depth_optical_frame`
- `camera_link -> camera_color_frame` (approx internal offset)
- `camera_color_frame -> camera_color_optical_frame`

If you move the camera mount, update the fixed joint in `src/rovi_description/urdf/rovi.urdf` (`camera_joint` origin).

## Device selection

### Color (recommended: stable `/dev/v4l/by-id/...`)

Avoid `/dev/video0` because it can renumber when other cameras are added.

Override explicitly if needed:
```bash
ros2 launch rovi_bringup rovi.launch.py stack:=camera \
  rgb_video_device:=/dev/v4l/by-id/<your-camera>-video-index0
```

### Depth (OpenNI2 `device_id`)

`device_id:="#1"` means “first OpenNI2 enumerated device”. If you ever plug multiple OpenNI2 devices, set `device_id` using:
```bash
ros2 run openni2_camera list_devices
```

## OpenNI2 mode note (`640x400`)

The upstream `openni2_camera` selects modes by named presets (e.g. VGA → `640x480`).
This workspace carries a small `openni2_camera` overlay patch in `src/openni2_camera` so this camera’s `640x400` / `320x200` modes are selectable by name, defaulting to `ORBBEC_640x400_30Hz`.

If you see errors about unsupported `640x480` modes and the images are black, re-check that:

- you rebuilt the workspace after pulling changes (`build`)
- you’re running with the Orbbec-friendly mode names (directly or via `stack:=camera`)

## Calibration

Depth and color are produced by two different drivers. The goal is a predictable “camera contract” (topics + TF + calibration file location) so downstream consumers don’t care.

### Color intrinsics (what to store + where)

- Store color intrinsics as a `camera_info_manager` YAML (ROS standard).
- Canonical (committed) location for this robot:
  - `src/rovi_bringup/config/camera_info/color.yaml`
- Runtime (ROS default) location on the robot:
  - `~/.ros/camera_info/` (after calibrating, keep a symlink here pointing at the committed file so a reinstall can be restored quickly)

### Frames (what to store + where)

- Store robot mount (`base_link -> camera_link`) as fixed URDF joints (`src/rovi_description/urdf/rovi.urdf`).
- Store the internal depth↔color offsets (`camera_link -> camera_depth_frame` / `camera_link -> camera_color_frame`) as fixed URDF joints too.
- Today’s `camera_link -> camera_depth_frame` / `camera_link -> camera_color_frame` offsets are placeholders; treat them as “good enough to visualize” until measured.

### How to generate the color calibration file

UVC cameras typically do not provide usable intrinsics automatically. Calibrate the color stream and save the resulting YAML.

Recommended tool:
```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 \
  image:=/camera/color/image_raw camera:=/camera/color
```

Notes:
- `--size` is inner corners; `--square` is meters.
- The tool typically writes via `/camera/color/set_camera_info` and stores under `~/.ros/camera_info/`.
- After generating calibration:
  1) Identify the generated YAML under `~/.ros/camera_info/` (filename depends on the camera node’s naming).
  2) Copy its content into `src/rovi_bringup/config/camera_info/color.yaml` and commit it.
  3) Replace the generated file with a symlink to the committed one (same filename):
     ```bash
     ln -sf "$(pwd)/src/rovi_bringup/config/camera_info/color.yaml" "$HOME/.ros/camera_info/<the-generated-filename>.yaml"
     ```

## Simulation (`sim camera`)

Simulation publishes camera topics from Gazebo sensors defined in `src/rovi_description/urdf/rovi.urdf` and bridges them via `src/rovi_sim/config/bridge.yaml`:

- Color: `/camera/color/image_raw`
- Depth: `/camera/depth/image`

To make simulated optics “feel closer” to the real device, tweak the Gazebo sensor settings in the URDF (`horizontal_fov`, `width/height`, `near/far`).

## Native tools (optional, for lower-level sanity checks)

The robot setup uses Orbbec’s OpenNI2 runtime. One known-good install flow is:

```bash
sudo apt update
sudo apt install -y freeglut3-dev libglu1-mesa libgl1 unrar

mkdir -p ~/OpenNI && cd ~/OpenNI
unzip OpenNI_2.3.0.86_202210111155_4c8f5aa4_beta6_arm64.zip
unrar x 066797_OpenNI_2.3.0.86_202210111155_4c8f5aa4_beta6_a311d.rar

cd ~/OpenNI/OpenNI_2.3.0/rules
sudo chmod +x install.sh
sudo ./install.sh
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Sanity checks:

- Color snapshot (UVC):
  ```bash
  ls -la /dev/v4l/by-id/
  ffmpeg -hide_banner -loglevel error -y -f v4l2 \
    -i /dev/v4l/by-id/<your-camera>-video-index0 \
    -frames:v 1 color.png
  ```
- Depth (OpenNI2):
  ```bash
  cd ~/OpenNI/OpenNI_2.3.0/tools/NiViewer
  export OPENNI2_REDIST=$PWD
  export LD_LIBRARY_PATH=$PWD:$LD_LIBRARY_PATH
  ./SimpleRead
  ```

## Viewer-only build (PC)

If your PC is only a viewer and you don’t want to build the vendored `openni2_camera` overlay:
```bash
export ROVI_SKIP_OPENNI2=1
build
```
