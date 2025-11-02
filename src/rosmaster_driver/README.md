# rosmaster_driver

ROS 2 (Jazzy) Python package that bridges a Rosmaster-based robot controller.

- Subscribes: `cmd_vel` (geometry_msgs/Twist)
- Publishes:
  - `edition` (std_msgs/Float32)
  - `voltage` (std_msgs/Float32)
  - `joint_states` (sensor_msgs/JointState)
  - `vel_raw` (geometry_msgs/Twist)
  - `/imu/data_raw` (sensor_msgs/Imu)
  - `/imu/mag` (sensor_msgs/MagneticField)

## Target platform

Raspberry Pi 5 running Ubuntu 24.04 (ROS 2 Jazzy). These steps assume you are on the Pi.

## Prerequisites

```bash
cd ~/dev/rovi_ros_ws
uv sync
```

## Build (on the Pi)

```bash
cd ~/dev/rovi_ros_ws
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
colcon build --merge-install
source install/setup.bash
```

## Run

```bash
ros2 launch rosmaster_driver rosmaster_driver.launch.py
```

Parameters:
- `imu_link` (string, default `imu_link`)
- `prefix` (string, default empty) — prepended to joint names
- `publish_rate` (double, default `10.0` Hz)
- `port` (string, default `/dev/my_ros_board`) — serial device for the controller
- `debug` (bool, default `False`) — enable Rosmaster debug logging

## Notes

- The virtual environment isolates Python user-space dependencies (like `pyserial` and `rosmaster-lib`). ROS 2 packages
  such as `rclpy` come from the system (apt) and are not installed via pip/uv.
- The package also declares the direct Git URL in `setup.py`. Using the venv with uv ensures the dependency is
  present even if your build environment doesn’t auto-install Python deps during `colcon build`.
