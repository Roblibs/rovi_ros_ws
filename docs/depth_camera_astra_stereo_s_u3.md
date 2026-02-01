# Depth camera — Orbbec Astra Stereo S U3 (Yahboom “AI View Depth Camera”)

This device presents as **two separate Linux devices** over one cable:

- **Depth/IR** via **OpenNI2** (Orbbec) → ROS driver: `openni2_camera`
- **RGB** via **UVC / V4L2** → ROS driver: `v4l2_camera`

ROVI’s current policy is to keep depth and RGB as **separate feeds** (no registered RGB-D / pointcloud yet) and provide a stable TF mount and consistent ROS topic names for RViz and downstream stacks.

## Status

- Streaming works: depth + RGB images show in RViz via `stack:=camera`.
- Depth modes for this camera are `640x400` / `320x200` (not `640x480` / `320x240`).
- RGB calibration is required for correct `CameraInfo` (see “Calibration”).

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

Depth (OpenNI2) under `/camera`:
- `/camera/depth_raw/image` (`16UC1`, mm)
- `/camera/depth/image` (`32FC1`, m)
- `/camera/depth_raw/camera_info`, `/camera/depth/camera_info`

RGB (UVC) under `/camera/rgb`:
- `/camera/rgb/image_raw`
- `/camera/rgb/camera_info`

TF published by `src/rovi_bringup/launch/camera.launch.py`:
- `base_link -> camera_link` (mount)
- `camera_link -> camera_depth_optical_frame`
- `camera_link -> camera_rgb_optical_frame`

Tune the mount if the camera is moved:
```bash
ros2 launch rovi_bringup rovi.launch.py stack:=camera camera_x:=... camera_y:=... camera_z:=... camera_yaw:=...
```

## Device selection

### RGB (recommended: stable `/dev/v4l/by-id/...`)

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

## Calibration (RGB)

UVC cameras typically do not provide usable intrinsics automatically. Calibrate the RGB stream and provide real `camera_info`.

Recommended tool:
```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 \
  image:=/camera/rgb/image_raw camera:=/camera/rgb
```

Notes:
- `--size` is inner corners; `--square` is meters.
- This typically uses `/camera/rgb/set_camera_info` to store calibration (commonly under `~/.ros/camera_info/`).

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

- RGB snapshot (UVC):
  ```bash
  ls -la /dev/v4l/by-id/
  ffmpeg -hide_banner -loglevel error -y -f v4l2 \
    -i /dev/v4l/by-id/<your-camera>-video-index0 \
    -frames:v 1 rgb.png
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

