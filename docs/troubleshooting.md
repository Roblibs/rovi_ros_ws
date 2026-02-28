# Troubleshooting

This doc collects common “if you see X, do Y” fixes for this workspace.

## Build

### `ModuleNotFoundError: No module named 'lark'` during `colcon build`

Symptom (example):
- `rosidl_generator_type_description ... ModuleNotFoundError: No module named 'lark'`

Cause:
- Your system Python is missing a runtime dependency required by ROS interface generators.

Fix:
```bash
sudo apt install python3-lark
```

Then re-run the build (or your `build` helper).

## Camera stack (Astra Stereo S U3)

### OpenNI2: unsupported `640x480` / black depth images

Symptom (example):
- `Unsupported depth video mode - Resolution: 640x480@30Hz Format: Depth 1mm`
- Depth topics exist but the images are black/invalid.

Fix checklist:
- Rebuild the workspace after pulling changes: `build`
- Ensure you’re using Orbbec-friendly OpenNI2 mode names (e.g. the `stack:=camera` defaults)

Related docs:
- `docs/depth_camera_astra_stereo_s_u3.md`
- `src/openni2_camera/README.md`

### `openni2_camera_driver`: “USB events thread - failed to set priority”

Typical log:
- `Warning: USB events thread - failed to set priority. This might cause loss of data...`

What to do:
- Usually safe to ignore unless you observe frequent depth frame drops.
- If you do see drops under load, consider enabling appropriate realtime/scheduling privileges for the driver process.

See also: `docs/runtime.md`

### ROS depth image looks “corrupted” (striped / mostly black), but `depth snapshot` looks fine

If `depth snapshot` (direct OpenNI2) produces a good image but RViz/rqt/ROS topics look corrupted, the hardware is likely fine.
The goal is to determine whether the corruption is:
- **In the ROS message itself** (OpenNI2 → ROS conversion / encoding / step / endianness), or
- **Only in the viewer/subscriber path** (image_transport selection, RViz transport hint, decoding plugins).

#### Step 0 — Confirm OpenNI2 hardware path (non-ROS)
```bash
ws
depth snapshot --out-dir output/openni2_snapshot
depth view   # NiViewer (optional)
```

If `depth snapshot` and/or NiViewer are bad too, stop here and troubleshoot OpenNI2 first:
`docs/depth_camera_astra_stereo_s_u3.md`.

#### Step 1 — Start the ROS camera launch in isolation
```bash
ws
ros2 launch rovi_bringup camera.launch.py robot_mode:=real
```

#### Step 2 — Sanity-check the topic graph and rates
```bash
ros2 node list | rg "openni2|camera"
ros2 topic list | rg "^/camera/(depth|ir|openni2)"
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /camera/depth/image
```

If `/camera/depth/image_raw` is missing or has 0 Hz, the issue is in the driver/SDK path (not transport).

#### Step 3 — Inspect the `sensor_msgs/Image` metadata (encoding/step)
Capture one message and check fields:
```bash
ros2 topic echo --once /camera/depth/image_raw | rg "encoding:|width:|height:|step:|is_bigendian:"
ros2 topic echo --once /camera/depth/image | rg "encoding:|width:|height:|step:|is_bigendian:"
```

Expected (typical):
- `/camera/depth/image_raw.encoding` is `16UC1` (or `mono16`), and `step == width*2`
- `/camera/depth/image.encoding` is often `32FC1`, and `step == width*4`

Red flags that usually cause “striped/corrupted” displays:
- `encoding` says 8-bit (e.g. `mono8`) but `step` matches 16-bit (`width*2`)
- `step` does not match `width * bytes_per_pixel`

If the metadata is wrong, treat this as an **OpenNI2 → ROS conversion/publisher bug** (or wrong OpenNI2 library being loaded).

#### Step 4 — Snapshot ROS images to disk (bypasses RViz UI confusion)
This uses the message *as published* and saves it to disk + prints basic stats:
```bash
python3 tools/depth/ros_snapshot.py --topic /camera/depth/image_raw --out-dir output/cam_snapshot_ros
python3 tools/depth/ros_snapshot.py --topic /camera/depth/image --out-dir output/cam_snapshot_ros
ls -la output/cam_snapshot_ros
```

Interpretation:
- If the saved `_viz.pgm` files look **corrupted too** → the corruption is in the **published ROS message** (conversion/encoding/step).
- If the saved `_viz.pgm` files look **fine** but RViz/rqt looks corrupted → the issue is likely **viewer/image_transport**.

#### Step 5 — Rule out image_transport / viewer transport hints
Check if any transport plugins are being used by your viewer:
```bash
ros2 topic list | rg "/camera/depth/image(_raw)?/.+"
```

Then try forcing “raw” transport in the tool:
- RViz Image display: set **Transport Hint** to `raw`
- `rqt_image_view`: choose the `raw` transport in the transport drop-down

If “raw” is fine but a compressed transport is corrupted, focus on the corresponding transport plugin/decoder.

#### Step 6 — Confirm which OpenNI2 library the ROS driver is loading
If this started “since yesterday”, a library path change can cause subtle breakage.
```bash
ros2 pkg prefix openni2_camera
ldd "$(ros2 pkg prefix openni2_camera)"/lib/openni2_camera/openni2_camera_driver | rg "OpenNI2|orbbec|usb" || true
```

If `openni2_camera_driver` is not loading the expected `libOpenNI2.so.0` from `OPENNI2_REDIST`, re-check `rovi_env.sh` OpenNI2 env setup and rebuild (`clean && build`).

### `v4l2_camera`: control read `Permission denied (13)`

Symptom (example):
- `Permission denied (13)` while reading a V4L2 control.

What to do:
- Confirm your user has permission to access the camera device under `/dev/v4l/` (often via udev rules and/or membership in the `video` group).
- If streaming works and only a non-essential control fails to read, it may be safe to ignore.

### Missing color camera calibration file

Symptom (example):
- `camera_calibration_parsers: Unable to open camera calibration file [...]`
- `Camera calibration file ... not found`

What to do:
- Generate and provide an RGB calibration YAML (and/or explicitly set `camera_info_url`) so `/camera/color/camera_info` is deterministic and correct.
- Workspace convention: commit the calibration YAML under `src/rovi_bringup/config/camera_info/`.

See also: `docs/depth_camera_astra_stereo_s_u3.md`, `docs/runtime.md`

## LiDAR

### LiDAR missing (`/scan` not published)

Symptom:
- `/scan` topic is missing or has 0 Hz.

What to do:
- Follow the dedicated checklist in `docs/lidar.md` (USB device mapping → driver node → topic rate → gRPC stream).

## UI / display

### `robot_serial_display`: gRPC `Connection refused` at startup

Typical log:
- `Failed to connect ... 127.0.0.1:50051 ... Connection refused (111); retrying ...`

What to do:
- Usually acceptable at startup; the node should connect once `ros_ui_bridge` is listening.
- If it never connects (or keeps disconnecting), treat it as a real fault.

See also: `docs/runtime.md`

### `robot_serial_display`: values disappear when starting `mapping` (or another stack)

This usually means one of:
- The **UI gateway** (`ros_ui_bridge`) is down / restarted / moved to a different port, or
- The **serial display client** is not running (or can’t open `/dev/robot_display`), or
- You accidentally started a **second gateway plane** (two `ros_ui_bridge` / two `robot_serial_display` instances fighting for port `50051` or the serial device).

#### Step 1 — Check gateway + display processes
```bash
# gRPC server should be listening
ss -lntp | rg ":50051" || true

# ROS nodes should exist
ros2 node list | rg "ui_bridge_metrics|robot_serial_display"
```

If `:50051` is not listening, restart the gateway plane:
```bash
sudo systemctl restart rovi-gateway.service
```

#### Step 2 — Check for multiple gateways (common when launching stacks manually)
```bash
ps aux | rg "ros_ui_bridge|robot_serial_display|gateway.launch.py|rovi.launch.py" | rg -v rg
```

In **systemd mode**, stacks should be “stack-only” (the gateway plane is owned by `rovi-gateway.service`):
- Prefer `sudo systemctl start rovi-mapping.service`, or
- Run `mapping gateway_enabled:=false`

You can also confirm what `mapping` is doing:
```bash
systemctl is-active rovi-gateway.service && echo "gateway service active"
```

## Gateway lidar

This section helps confirm whether lidar data is making it “out of the UI gateway” (the `ros_ui_bridge` gRPC server).

### Step 1 — Confirm the robot is publishing scans
```bash
ros2 topic list | rg "^/scan$"
ros2 topic hz /scan
ros2 topic echo --once /scan
```

If `/scan` is missing or 0 Hz, troubleshoot the lidar driver / sim first (the gateway can’t stream what doesn’t exist).

### Step 2 — Confirm the gateway is ingesting scans (ROS-side)
The gateway’s lidar stream also republishes a throttled scan topic for RViz (default: `/viz/scan`).

```bash
ros2 node list | rg "ros_ui_bridge|ui_bridge"
ros2 topic list | rg "^/viz/scan$"
ros2 topic hz /viz/scan
```

If `/viz/scan` is >0 Hz, `ros_ui_bridge` is receiving `/scan` and forwarding it.

### Step 3 — Confirm lidar is streaming over gRPC (UI-side)
By default the gateway listens on `0.0.0.0:50051`.

```bash
ss -lntp | rg ":50051" || true
grpcurl -plaintext -proto src/ros_ui_bridge/proto/ui_bridge.proto localhost:50051 list
grpcurl -plaintext -proto src/ros_ui_bridge/proto/ui_bridge.proto -d '{}' \
  localhost:50051 roblibs.ui_bridge.v1.UiBridge/StreamLidar
```

Expected:
- The `list` call includes `roblibs.ui_bridge.v1.UiBridge`.
- `StreamLidar` prints a stream of `LidarUpdate` messages while scans are active.

#### Step 3 — Check the serial device isn’t missing/busy
```bash
ls -la /dev/robot_display || true
sudo lsof /dev/robot_display 2>/dev/null || true
```

If another process holds the port, stop the extra instance (or reboot), then keep a single always-on display client.

### UI: floor topology not showing

Symptom:
- Your UI does not show the floor topology overlay.

Checklist (ROS side):
```bash
# Topic exists and has data?
ros2 topic list | rg "/floor"
ros2 topic info /floor/topology -v
ros2 topic hz /floor/topology
ros2 topic echo --once /floor/topology

# Is the floor node running and is topology enabled?
ros2 node list | rg "rovi_floor_runtime"
ros2 param get /rovi_floor_runtime camera_topology_enabled
```

Checklist (RViz sanity):
```bash
# If RViz can’t show it either, the issue is upstream of the UI bridge.
# In RViz: add a "MarkerArray" display and set topic to /floor/topology.
```

Checklist (gRPC / UI bridge):
```bash
# Is the gRPC server listening?
ss -lntp | rg ":50051" || true

# Does the floor topology stream produce updates?
# (Install grpcurl if needed: sudo snap install --edge grpcurl)
grpcurl -plaintext -import-path ${ROVI_ROS_WS_DIR}/src/ros_ui_bridge/proto -proto ui_bridge.proto \
  localhost:50051 roblibs.ui_bridge.v1.UiBridge/StreamFloorTopology
```

If `grpcurl` shows nothing for a long time:
- Confirm `/floor/topology` is publishing (see ROS checklist above).
- Note: `grpcurl` waits for the next streamed event; it will not print anything if the stream is idle.

If `grpcurl` shows `FloorTopologyUpdate{ polylines: ... }` but the UI still shows nothing:
- Confirm the UI client is using the current proto: `FloorTopologyUpdate` now carries `polylines[]` (not a single `points[]`).

## Runtime environment

### Python deps missing at runtime (e.g. `rosmaster_driver` import errors)

Symptom (example):
- A Python node fails to start due to missing imports (venv-installed deps like `rosmaster-lib`, `pyserial`, etc.).

What to do:
- Ensure the workspace venv is present and synced: `uv sync`
- Ensure `ROVI_ROS_WS_DIR` points at the workspace root (so bringup can find `ROVI_ROS_WS_DIR/.venv` for `robot_mode:=real`).
- If you are launching a node directly (outside bringup), you may still need to activate the venv or set `PYTHONPATH`/`VIRTUAL_ENV` for that process (e.g. systemd):
  ```bash
  export VIRTUAL_ENV="$HOME/dev/rovi_ros_ws/.venv"
  ```
