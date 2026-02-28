# LiDAR (`/scan`) — notes + troubleshooting (ROVI)

ROVI uses an RPLidar (via `rplidar_ros`) as the primary ranging sensor for SLAM/Nav.

This page is the “single place” to troubleshoot LiDAR from USB → ROS → gRPC.

## What should exist (contract)

- ROS topic: `/scan` (`sensor_msgs/msg/LaserScan`)
- Frame: `laser_link` (expected)
- TF chain (typical): `map -> odom -> base_footprint -> base_link -> laser_link`

In this repo, `/scan` is expected in all normal **real robot** sessions (because it comes from the always-on gateway plane).

## Where it is started

### Real robot (systemd / gateway plane)

The LiDAR driver is started by the gateway plane:

- systemd: `rovi-gateway.service`
- launch: `rovi_bringup/gateway.launch.py`
- backend: `rovi_bringup/robot_bringup.launch.py`
- node: `rplidar_ros` executable `rplidar_composition` (node name: `rplidar_composition`)

Relevant launch args in `robot_bringup.launch.py`:
- `lidar_enabled` (default `true`)
- `lidar_serial_port` (default `$ROVI_LIDAR_PORT` or `/dev/robot_lidar`)
- `lidar_serial_baudrate` (default `115200`)
- `lidar_frame` (default `laser_link`)

### Simulation (Gazebo)

In `robot_mode:=sim`, `/scan` is bridged from Gazebo (not from `rplidar_ros`).

### Offline

No LiDAR is expected.

## USB: device mapping (most common failure point)

ROVI relies on stable device symlinks:
- `/dev/robot_lidar` (LiDAR CH340)

Those are set up by:
```bash
sudo python3 tools/rovi_usb_setup.py
```

### Quick USB checks
```bash
ls -l /dev/robot_lidar || true
lsusb | rg -i "rplidar|slamtec|1a86:7523" || true
lsusb -t
udevadm info -q path -n /dev/robot_lidar 2>/dev/null || true
sudo lsof /dev/robot_lidar 2>/dev/null || true
```

If `/dev/robot_lidar` is missing, rerun `tools/rovi_usb_setup.py` and replug the USB cable/hub.

See also: `docs/usb.md`.

## ROS: is the driver alive and publishing?

### 1) Node + params
```bash
ros2 node list | rg "rplidar|rplidar_composition"
ros2 param get /rplidar_composition serial_port
ros2 param get /rplidar_composition frame_id
```

### 2) Topic presence + rate
```bash
ros2 topic list | rg "^/scan$"
ros2 topic hz /scan
ros2 topic echo --once /scan | rg "frame_id:|angle_min:|angle_max:|ranges:"
```

### 3) TF sanity (RViz won’t show scans without TF)
```bash
ros2 run tf2_ros tf2_echo base_footprint laser_link
```

If TF is missing, confirm `robot_state_publisher` is running and the URDF includes `laser_link`.

## Gateway / gRPC: is LiDAR available to the UI?

`ros_ui_bridge` provides:
- `StreamLidar` (gRPC)
- status field `hz_lidar` (in `GetStatus` / `StreamStatus`)
- a ROS visualization republish: `/viz/scan` (throttled scan for RViz)

### Check status field `hz_lidar`
```bash
grpcurl -plaintext -import-path ${ROVI_ROS_WS_DIR}/src/ros_ui_bridge/proto -proto ui_bridge.proto \
  localhost:50051 roblibs.ui_bridge.v1.UiBridge/GetStatus | rg "hz_lidar|values" -n
```

### Check the gRPC lidar stream
```bash
grpcurl -plaintext -import-path ${ROVI_ROS_WS_DIR}/src/ros_ui_bridge/proto -proto ui_bridge.proto \
  localhost:50051 roblibs.ui_bridge.v1.UiBridge/StreamLidar
```

### Check `/viz/scan`
```bash
ros2 topic list | rg "^/viz/scan$"
ros2 topic hz /viz/scan
```

## If LiDAR “disappears” right now (fast triage)

1) Is the USB serial device still present?
```bash
ls -l /dev/robot_lidar /dev/ttyUSB* 2>/dev/null || true
```

2) Is the driver still running?
```bash
ros2 node list | rg "rplidar_composition"
```

3) If you’re in systemd mode, check the gateway service:
```bash
systemctl is-active rovi-gateway.service && echo "gateway active"
journalctl -u rovi-gateway.service -n 200 --no-pager | rg -i "rplidar|scan|serial_port|ttyUSB|robot_lidar|error|warn"
```

4) Restart the gateway plane (brings LiDAR back if it was stuck):
```bash
sudo systemctl restart rovi-gateway.service
```

## Common root causes

- **/dev path changed** (two CH340 devices swapped): rerun `tools/rovi_usb_setup.py`.
- **USB power / hub issues**: move LiDAR off the camera hub tree; use a powered hub; try another port.
- **Port already opened** by a different process: `lsof /dev/robot_lidar`.
- **TF missing**: `/scan` exists but RViz shows nothing; fix the `laser_link` transform.
