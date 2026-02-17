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

## UI / display

### `robot_serial_display`: gRPC `Connection refused` at startup

Typical log:
- `Failed to connect ... 127.0.0.1:50051 ... Connection refused (111); retrying ...`

What to do:
- Usually acceptable at startup; the node should connect once `ros_ui_bridge` is listening.
- If it never connects (or keeps disconnecting), treat it as a real fault.

See also: `docs/runtime.md`

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
