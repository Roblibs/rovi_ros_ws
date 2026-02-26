# Depth camera analysis (Astra Stereo S U3) — rates, QoS, and viewer decoupling

This doc captures how the gateway/display currently derives `hz_color` / `hz_depth`, and provides an overview of camera-related topics + QoS assumptions to support future decisions (viewer over Wi‑Fi, throttling, compression, etc.).

## Where `hz_color` / `hz_depth` come from (gateway → display)

Gateway launch: `src/rovi_bringup/launch/gateway.launch.py` starts:
- `ros_ui_bridge` (`ros2 run ros_ui_bridge ui_bridge -- --config ...`)
- `robot_serial_display` (gRPC client → USB serial display)

`ros_ui_bridge` status fields are configured in `src/ros_ui_bridge/config/default.yaml`:
- `hz_color` uses `source.type: topic_rate` on topic `/camera/color/image` (`sensor_msgs/msg/Image`)
- `hz_depth` uses `source.type: topic_rate` on topic `/camera/depth/image` (`sensor_msgs/msg/Image`)

Implementation note:
- `ros_ui_bridge` collects `topic_rate` by subscribing to the configured topics with **BEST_EFFORT** QoS (`QoSProfile(depth=10, reliability=BEST_EFFORT)`) and tracking arrival timestamps (`src/ros_ui_bridge/ros_ui_bridge/ros_metrics_node.py`).
- `robot_serial_display` selects the ids `hz_color` / `hz_depth` from the gRPC `GetStatus`/`StreamStatus` responses and formats them for the display (`src/robot_serial_display/config/default.yaml`, `src/robot_serial_display/robot_serial_display/serial_display_node.py`).

Implication:
- The **display Hz is “subscriber-observed arrival rate” at the gateway**, not a driver-internal publish loop rate.
- The gateway subscriptions themselves are **best-effort**, so they should not directly backpressure the camera publishers; Wi‑Fi viewer behavior is typically a *separate* subscriber/QoS/bandwidth issue.

## Camera stack publishers (real robot)

Camera launch: `src/rovi_bringup/launch/camera.launch.py` starts:
- `openni2_camera/openni2_camera_driver` (depth)
- `v4l2_camera/v4l2_camera_node` (RGB; forced `pixel_format=YUYV`, `output_encoding=rgb8`)

ROVI topic contract (from `docs/depth_camera_astra_stereo_s_u3.md`):
- Depth under `/camera/depth/*`
- Color under `/camera/color/*`

## Overview: topics, expected QoS, and consumers

QoS note:
- Exact QoS is runtime-discoverable and may differ by distro/version/config. Treat the “expected” values below as defaults to validate with `ros2 topic info -v <topic>` on the robot and on the viewer machine.

| Topic | Msg type | Notes (encoding / payload) | Publisher | Gateway usage | QoS (expected / known) | Typical consumers |
| --- | --- | --- | --- | --- | --- | --- |
| `/camera/color/image` | `sensor_msgs/msg/Image` | RGB image (`rgb8`) after conversion from UVC YUYV | `v4l2_camera` (`camera.launch.py`) | `hz_color` (`topic_rate`) | Pub: unknown (verify at runtime); Sub (gateway): **BEST_EFFORT depth=10** | RViz/rqt viewers (local/remote), any vision nodes |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | Intrinsics from YAML (`rovi_camera_info_pub`) | `rovi_camera_info_pub` | none | Pub: **RELIABLE + TRANSIENT_LOCAL depth=1**; Sub: N/A | RViz camera model, any perception nodes |
| `/camera/color/camera_info_raw` | `sensor_msgs/msg/CameraInfo` | Driver-provided (often empty/unreliable for UVC) | `v4l2_camera` (remapped to `_raw`) | none | Pub: unknown; Sub: N/A | Debug only |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | Depth in mm (`16UC1`), “raw” depth feed | `openni2_camera_driver` (remapped) | none (currently) | Pub: **RELIABLE keep-last depth=1** (from `rmw_qos_profile_default` + `depth=1`); Sub (gateway): **not subscribed** by default | Local processing, (future) compressedDepth/view republish |
| `/camera/depth/image` | `sensor_msgs/msg/Image` | Converted depth in meters (`32FC1`), larger payload than `image_raw` | `openni2_camera_driver` | `hz_depth` (`topic_rate`) | Pub: **RELIABLE keep-last depth=1** (same as above); Sub (gateway): **BEST_EFFORT depth=10** | RViz/rqt viewers (local/remote) |
| `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | Depth model from YAML (`rovi_camera_info_pub`) | `rovi_camera_info_pub` | none | Pub: **RELIABLE + TRANSIENT_LOCAL depth=1**; Sub: N/A | RViz camera model, any perception nodes |
| `/camera/depth/camera_info_raw` | `sensor_msgs/msg/CameraInfo` | Driver-provided | `openni2_camera_driver` (remapped to `_raw`) | none | Pub: unknown; Sub: N/A | Debug only |

Observation:
- The gateway/display’s `hz_depth` is tracking `/camera/depth/image` (float depth). If the goal is “sensor health” rather than “viewer-friendly topic”, it may be better (future change) to track `/camera/depth/image_raw` to avoid counting conversion side-effects and to reduce any accidental downstream bandwidth/cpu coupling.

## Why a remote viewer can collapse Hz (most likely causes)

When *no remote consumer exists*, the camera nodes publish locally and only pay local serialization costs.
When a *remote consumer appears over Wi‑Fi*, the producer must serialize and transmit full frames over DDS/UDP.

Raw image bandwidth is usually too high for typical Wi‑Fi links:
- Color `rgb8` @ `640x480`: `640*480*3 ≈ 0.88 MB/frame` → ~`26 MB/s` at 30 Hz (~`210 Mbit/s`)
- Depth `32FC1` @ `640x400`: `640*400*4 ≈ 1.0 MB/frame` → ~`30 MB/s` at 30 Hz (~`240 Mbit/s`)

So a viewer subscribing to `/camera/depth/image` + `/camera/color/image` can easily exceed sustainable Wi‑Fi throughput, leading to drops, queue growth, or backpressure depending on QoS settings.

Additional risk for depth specifically:
- `openni2_camera_driver` publishes depth using `rmw_qos_profile_default` with `depth=1` (RELIABLE). With a slow/lossy Wi‑Fi link, a RELIABLE image stream is more likely to stall/throttle the producer compared to BEST_EFFORT.

## Measurement checklist (to pinpoint the bottleneck)

On the robot (publisher-side view):
- Who is subscribing? `ros2 topic info -v /camera/color/image` and `ros2 topic info -v /camera/depth/image`
- What QoS are they using? (look for reliability/history/depth per endpoint)
- Payload bandwidth attempt: `ros2 topic bw /camera/color/image`, `ros2 topic bw /camera/depth/image`
- Local CPU pressure: `htop` (watch `v4l2_camera_node`, `openni2_camera_driver`, and DDS threads)

Between robot ↔ viewer:
- Throughput capacity: `iperf3` (TCP + UDP tests) to bound achievable Mbit/s and loss/jitter under contention.

## Plan: camera viewer decorrelation

The detailed plan (topic naming, throttling policy, RViz enforcement) lives in:
- `docs/plan/camera_viewer_decorrelation.md`
