# Runtime notes (expected warnings / acceptable noise)

This doc lists runtime warnings/errors you may see during bringup that are **currently acceptable**, plus the conditions under which they stop being acceptable.

## Acceptable (can be tolerated)

### `robot_serial_display`: gRPC connection refused at startup
Typical log:
- `Failed to connect ... 127.0.0.1:50051 ... Connection refused (111); retrying ...`

Why it happens:
- `robot_serial_display` can start before `ros_ui_bridge` begins listening on `:50051`.

Why it’s acceptable:
- The display node retries and should connect once the UI gateway is up.

Not acceptable if:
- It never transitions to a successful connect (e.g. no later log like `Connected to UI gateway at 127.0.0.1:50051`), or it repeatedly disconnects during normal operation.

---

### `openni2_camera_driver`: “USB events thread - failed to set priority”
Typical log:
- `Warning: USB events thread - failed to set priority. This might cause loss of data...`

Why it happens:
- The driver attempts to raise thread scheduling priority but the process does not have the required privileges / realtime policy.

Why it’s acceptable:
- Depth streaming typically remains stable at the current resolutions, and the warning is only a risk indicator.

Not acceptable if:
- You observe frequent depth frame drops under normal load, depth stream stalls, or the sensor becomes unreliable during navigation/mapping.

---

### `v4l2_camera`: unsupported V4L2 control type
Typical log:
- `Control type not currently supported: 6, for control: Camera Controls`

Why it happens:
- Some UVC devices expose extended/compound controls that `v4l2_camera` does not currently implement.

Why it’s acceptable:
- Streaming and basic controls still work; it only affects unsupported settings.

Not acceptable if:
- A required control (exposure/white balance/etc.) cannot be set or queried and it impacts image quality enough to break downstream perception.

---

### `v4l2_camera`: on-the-fly colorspace/encoding conversion
Typical log:
- `Image encoding not the same as requested output ... yuv422_yuy2 => rgb8`

Why it happens:
- The camera outputs YUYV/YUV422 and the node converts frames to the requested `rgb8` encoding.

Why it’s acceptable:
- Functionally correct images arrive in RViz.

Not acceptable if:
- CPU usage is high enough to cause latency or missed deadlines (e.g. Nav2/SLAM responsiveness), or RGB frame rate becomes unstable.

## Not acceptable (requires action)

### Missing color camera calibration file
Typical log:
- `camera_calibration_parsers: Unable to open camera calibration file [...]`
- `Camera calibration file ... not found`

Why it matters:
- Without RGB calibration, `/camera/color/camera_info` is missing/incorrect and any rectification or pixel-to-ray projection from the color camera is unreliable.
