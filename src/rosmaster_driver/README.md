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

1) Install ROS 2 Jazzy (Ubuntu Debs):
   https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

2) Install Python dependency for the hardware library (from your GitHub repo):

```bash
python3 -m pip install --upgrade pip
python3 -m pip install "rosmaster-lib @ git+https://github.com/RobLibs/Rosmaster_Lib"
```

This makes `from Rosmaster_Lib import Rosmaster` available to the node.

## Build (on the Pi)

```bash
cd ~/dev/rovi_ros_ws
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

- This package declares a pip dependency on your GitHub repo in `setup.py` using a PEP 508 direct URL. If
  `colcon build` doesn’t fetch it automatically in your environment, install it with pip first as shown above.
  After that, (re)build and source the workspace, then launch the node.
