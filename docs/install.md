# Install

## Dependency policy

- PC sim/view: use `apt` (no `pip`, no workspace `.venv` required).
- Robot real: use `apt` first; use workspace `.venv` via `uv sync` only for robot-only Python deps that are not packaged as Ubuntu/ROS debs.

* Install ros jazzy : https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

* Install uv (Python package and venv manager):

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```
* Recommended: clone this repo under `~/dev/rovi_ros_ws` (robot: `wass` user). The robot systemd units currently hardcode `/home/wass/dev/rovi_ros_ws`; if you move the workspace, update the unit files under `services/` before installing.
* set `ROVI_ROS_WS_DIR` + source `rovi_env.sh` (see [Config in ~/.bashrc](#config-in-bashrc)), then build (uses system ROS colcon):
```bash
ws
build
```

If you want to run `robot_mode:=real` using the workspace venv deps:
```bash
uv sync
```

You do not need to manually activate the venv for normal `robot_mode:=real` launches; bringup will pick up `ROVI_ROS_WS_DIR/.venv` automatically when present.

## Robot services

Install the robot systemd units + polkit rules:

```bash
sudo ./services/install.sh
```

Allow operator users to start/stop stack services without sudo (log out/in after):

```bash
sudo usermod -aG rovi-ops "$USER"
```

* Install the required ROS packages

```bash
sudo apt install -y \
  ros-jazzy-joy \
  ros-jazzy-teleop-twist-joy \
  ros-jazzy-twist-mux \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-rosbag2 \
  ros-jazzy-rosbag2-compression-zstd \
  ros-jazzy-diagnostic-updater \
  protobuf-compiler \
  protobuf-compiler-grpc \
  python3-grpcio \
  python3-protobuf \
  python3-psutil \
  python3-serial \
  python3-yaml \
  python3-catkin-pkg \
  python3-lark \
  python3-numpy \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-rviz2 \
  ros-jazzy-foxglove-bridge \
  ros-jazzy-camera-info-manager \
  ros-jazzy-v4l2-camera \
  ros-jazzy-camera-calibration \
  ros-jazzy-rplidar-ros \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-rviz-plugins \
  ros-jazzy-imu-filter-madgwick
```

Notes:
- PC sim/view stacks are intended to run without workspace `.venv`. Keep Python runtime deps like `grpcio` installed via `apt` (system Python).
- The workspace `.venv` (via `uv sync`) is reserved for robot-only Python deps that are not packaged as Ubuntu/ROS debs (for example GitHub-only libs).

### Protobuf/gRPC codegen note (PC builds)

`ros_ui_bridge` generates Python gRPC/protobuf stubs from `src/ros_ui_bridge/proto/ui_bridge.proto` during `colcon build` using system `protoc` + the gRPC Python plugin (from `protobuf-compiler` / `protobuf-compiler-grpc`).

If you see older instructions suggesting `python3 -m pip install --user grpcio-tools ...`, you can ignore them on Ubuntu: we intentionally rely on `apt` for both runtime (`python3-protobuf`, `python3-grpcio`) and codegen (`protobuf-compiler`, `protobuf-compiler-grpc`).

## USB device setup (robot)
Connect the Rosmaster control board, RPLidar, and the ESP32-S3 display, then run:

```bash
sudo python3 tools/rovi_usb_setup.py
```

This creates stable symlinks:
- `/dev/robot_control` (Rosmaster board)
- `/dev/robot_lidar` (RPLidar)
- `/dev/robot_display` (ESP32-S3 display)

Manual overrides (no udev install) are available for debugging:

```bash
export ROVI_ROSMASTER_PORT=/dev/ttyUSB0
export ROVI_LIDAR_PORT=/dev/ttyUSB1
export ROVI_DISPLAY_PORT=/dev/ttyACM0
```

Verify the mapping (recommended after setup):

```bash
ls -l /dev/robot_control /dev/robot_lidar /dev/robot_display
env | rg "ROVI_.*_PORT"
unset ROVI_ROSMASTER_PORT ROVI_LIDAR_PORT ROVI_DISPLAY_PORT
```

With `teleop` running in another terminal:

```bash
ros2 param get /rplidar_composition serial_port
ros2 topic echo /scan --once
ros2 topic echo /voltage --once
```

If you still see `/dev/ttyUSB0` or `/dev/ttyACM0` in logs, an override is active.

## Config in `~/.bashrc`
Add these lines to your real `~/.bashrc`:

```bash
export ROVI_ROS_WS_DIR="$HOME/dev/rovi_ros_ws"
source "$ROVI_ROS_WS_DIR/rovi_env.sh"

# Optional (PC sim): pin Gazebo Transport to a stable local IPv4
# export GZ_IP="<your-ipv4>"

# Optional: skip OpenNI2 camera build/runtime helpers if not needed
# export ROVI_SKIP_OPENNI2=1

# Optional (PC sim): keep this terminal off the robot DDS network
# export ROS_LOCALHOST_ONLY=1
```

## wsl
When using Windows Subsystem for Linux, it is necessary to ensure the following:
- Windows Network: network is private, not public.
- WSL Settings: Networking, Networking mode 'Mirrored'
- Firewall config :

```bash
Set-NetConnectionProfile -InterfaceAlias "WiFi" -NetworkCategory Private

New-NetFirewallRule -DisplayName "Allow ICMPv4 Echo from robot" -Direction Inbound -Action Allow -Protocol ICMPv4 -IcmpType 8 -RemoteAddress 10.0.0.180 -Profile Private

New-NetFirewallRule -DisplayName "Allow ROS2 DDS UDP from robot" -Direction Inbound -Action Allow -Protocol UDP -LocalPort 7400-7600 -RemoteAddress 10.0.0.180 -Profile Private
```

USB management in WSL:

- install https://github.com/dorssel/usbipd-win/releases

```bash
usbipd bind --busid 2-4
usbipd attach --wsl --busid 2-4
usbipd detach --busid 2-4
```
In WSL or Linux:
```bash
sudo usermod -a -G dialout "$USER"
```
