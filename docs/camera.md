# Camera (depth + RGB)

ROVI’s “depth camera” shows up as **two separate Linux devices** over one cable:
- **Depth** via OpenNI2 (Orbbec) → ROS driver: `openni2_camera`
- **RGB** as a **UVC / V4L2** camera → ROS driver: `v4l2_camera`

This repo’s goal is to expose them in a “ROS-y” way:
- separate image feeds
- valid `sensor_msgs/CameraInfo`
- stable TF frames

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

## Topics (current contract)

Depth (OpenNI2), namespaced under `/camera`:
- `/camera/depth_raw/image` (16UC1, mm)
- `/camera/depth/image` (32FC1, m)
- `/camera/depth_raw/camera_info`, `/camera/depth/camera_info`

RGB (UVC) under `/camera/rgb`:
- `/camera/rgb/image_raw`
- `/camera/rgb/camera_info`

## TF (current contract)

`rovi_bringup/camera.launch.py` publishes:
- `base_link -> camera_link` (mount; defaults are “good enough” until measured)
- `camera_link -> camera_depth_optical_frame`
- `camera_link -> camera_rgb_optical_frame`

If the camera is moved, tune the mount with:
```bash
ros2 launch rovi_bringup rovi.launch.py stack:=camera camera_x:=... camera_y:=... camera_z:=... camera_yaw:=...
```

## RGB device selection (stable path)

Avoid `/dev/video0` because it can change when new cameras are added.

Prefer a stable `/dev/v4l/by-id/...` path (see `docs/depth.md` for a working example).

Override the device explicitly if needed:
```bash
ros2 launch rovi_bringup rovi.launch.py stack:=camera rgb_video_device:=/dev/v4l/by-id/<your-camera>-video-index0
```

## Calibration (RGB)

UVC cameras typically **do not** provide usable intrinsics automatically. Calibrate the RGB stream and publish real `camera_info`.

### ROS tool (recommended)

Install:
- `ros-jazzy-camera-calibration`

Then run the ROS calibrator while `camera` is running:
```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 \
  image:=/camera/rgb/image_raw camera:=/camera/rgb
```

Notes:
- `--size` is “inner corners” (e.g. 8x6), `--square` is square size in meters.
- This typically uses the `/camera/rgb/set_camera_info` service to store calibration.

### OpenCV DIY (fallback)

If you want full control, OpenCV’s `calibrateCamera()` works fine, but you still need to export results into a ROS `CameraInfo` YAML and ensure the driver loads it (or you publish `camera_info` yourself).

## Depth intrinsics / calibration

`openni2_camera` publishes `CameraInfo` for depth topics. That’s the intrinsics consumers expect for projection/rectification.

For now, ROVI uses **separate** depth + RGB feeds (no registered depth-to-color), so **RGB↔Depth extrinsics are not required yet**.

## Timestamps and sync (what to expect)

- The OpenNI2 driver is configured with `use_device_time:=true`, so depth timestamps come from the device when available.
- The UVC RGB path typically timestamps frames in software (driver/OS), and is not guaranteed to be phase-locked to depth.

For “two panels in RViz”, this is fine. If you later need true RGB-D alignment, plan on using approximate time sync and adding explicit RGB↔Depth extrinsics/registration.

