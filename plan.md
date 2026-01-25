# plan.md — future notes / next steps

## Depth camera (Orbbec Astra Stereo S U3 / Yahboom AI View Depth Camera)

### Current status (2026-01-25)
- OpenNI2 path works (depth is readable with OpenNI tools), so the hardware is functional.
- `ros-jazzy-orbbec-camera` (OrbbecSDK_ROS2 wrapper) is currently not usable with this camera on this robot: OpenOrbbecSDK `2.5.5` enumerates USB but returns `deviceCount=0`, so no ROS topics ever appear beyond `device_status`.
  - Repro: `tools/depth/ob_list.cpp` prints SDK version and `deviceCount=0` and logs `Failed to query USB device serial number` (missing USB serial descriptor).
  - Implication: any solution based on `ros-jazzy-orbbec-camera` will remain blocked until the underlying OpenOrbbecSDK accepts this device (or the device firmware exposes the expected descriptors).

### Options (short)
- **Preferred (pragmatic):** integrate via ROS2 `openni2_camera` (matches the proven-working OpenNI2 stack).
- **Fallback 1:** write a small ROS2 node that uses OpenNI2 directly (publish `sensor_msgs/Image` + `CameraInfo`) for depth/IR, and treat RGB as a separate UVC camera if needed.
- **Fallback 2 (investigation-heavy):** try a different OpenOrbbecSDK build/version or patch OpenOrbbecSDK device filtering so devices without a USB serial descriptor are still accepted; then re-test `ros-jazzy-orbbec-camera`.

### Next steps
- Try `openni2_camera` on Jazzy and confirm topics: depth image, IR image (if available), and camera info; document the working launch in `docs/depth.md`.
- If OpenNI2 works, define the target ROS topic/TF contract for the rest of the stack (Nav/RViz) before wiring it into `rovi_bringup`.
