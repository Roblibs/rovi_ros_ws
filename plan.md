# plan.md — future notes / next steps

## Camera stack (RGB UVC + Depth OpenNI2)

### Topic conventions (decision)
- Use `/camera/color/*` for the Astra color stream, `/camera/depth/*` for the Astra depth stream, and reserve `/camera/stereo/*` for the ELP stereo camera.

### Calibration (required)
- Decide calibration storage policy:
  - Preferred for this robot: commit calibration YAML in-repo (restore-friendly) under `src/rovi_bringup/config/camera_info/` and load it from launch (avoid relying only on `~/.ros/camera_info/` on the Pi).
- Generate RGB calibration YAML for the UVC camera and point the RGB driver to it (the current `/camera/color/camera_info` is uncalibrated).
- Verify RViz / downstream nodes are using the intended frames and `CameraInfo` (rectification / pointcloud projection sanity check).
 
### Frames / TF (decision)
- Source of truth is `src/rovi_description/urdf/rovi.urdf` (no launch-time static TF for camera frames).
- Gazebo sim uses the same frames and publishes simulated camera sensors from the URDF.

### Warnings/errors from `camera` launch (need decisions/actions)
- `v4l2_camera` control read fails with `Permission denied (13)` for control id `10092545`: confirm whether this affects any required settings; if yes, fix via udev/device permissions or avoid querying that control.
- `v4l2_camera` “Control type not currently supported: 6” (Camera Controls): confirm if it is safe to ignore; otherwise, either patch `v4l2_camera` to support it or remove it from the control list.
- RGB format conversion warning (`yuv422_yuy2 => rgb8`): measure CPU impact; if needed, adjust requested pixel format / use MJPEG / reduce resolution or frame rate.
- OpenNI2 “USB events thread - failed to set priority”: decide whether to tolerate (most cases) or configure realtime privileges to reduce risk of dropped frames under load.

### Startup robustness (nice-to-have)
- `robot_serial_display` initially fails to connect to gRPC (`Connection refused`) until `ros_ui_bridge` starts: decide if we want to enforce start order (or keep retry behavior and classify as acceptable).
