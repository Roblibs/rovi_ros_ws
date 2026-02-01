# plan.md — future notes / next steps

## Camera stack (RGB UVC + Depth OpenNI2)

### Topic conventions (decision)
- Use `/camera/color/*` for the Astra color stream, `/camera/depth/*` for the Astra depth stream, and reserve `/camera/stereo/*` for the ELP stereo camera.

### Calibration (required)
- Generate and install RGB calibration YAML for the UVC camera (the current default path under `~/.ros/camera_info/` is missing, so `camera_info` is uncalibrated).
- Decide how depth intrinsics should be sourced (factory/OpenNI2 vs calibrated YAML) and ensure `depth/*/camera_info` is correct and stable across boots.
- Verify RViz / downstream nodes are using the intended frames and `CameraInfo` (rectification / pointcloud projection sanity check).

### Warnings/errors from `camera` launch (need decisions/actions)
- `v4l2_camera` control read fails with `Permission denied (13)` for control id `10092545`: confirm whether this affects any required settings; if yes, fix via udev/device permissions or avoid querying that control.
- `v4l2_camera` “Control type not currently supported: 6” (Camera Controls): confirm if it is safe to ignore; otherwise, either patch `v4l2_camera` to support it or remove it from the control list.
- RGB format conversion warning (`yuv422_yuy2 => rgb8`): measure CPU impact; if needed, adjust requested pixel format / use MJPEG / reduce resolution or frame rate.
- OpenNI2 “USB events thread - failed to set priority”: decide whether to tolerate (most cases) or configure realtime privileges to reduce risk of dropped frames under load.

### Startup robustness (nice-to-have)
- `robot_serial_display` initially fails to connect to gRPC (`Connection refused`) until `ros_ui_bridge` starts: decide if we want to enforce start order (or keep retry behavior and classify as acceptable).
