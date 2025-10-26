# rovi_ros_ws
ROS2 Jazzy workspace for Room View Bot

# Getting started
install : https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

install joystick and rplidar ros packages

```bash
# Teleop + mux
sudo apt install -y ros-jazzy-joy ros-jazzy-teleop-twist-joy ros-jazzy-twist-mux

# Lidar: try apt first; if nothing, use Slamtec’s ROS 2 driver
sudo apt install -y ros-jazzy-rplidar-ros || true
```

test the rplidar
```bash
source /opt/ros/jazzy/setup.bash
ros2 launch rplidar_ros view_rplidar.launch.py
```

# Data Flow
![packafe_flow](./docs/package_flow.drawio.svg)
