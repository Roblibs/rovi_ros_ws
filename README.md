# rovi_ros_ws
ROS2 Jazzy workspace for Room View Bot

# Getting started
1) Install : https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

2) Install uv (Python package and venv manager):

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

3) Install joystick and rplidar ros packages

```bash
# Teleop + mux
sudo apt install -y ros-jazzy-joy ros-jazzy-teleop-twist-joy ros-jazzy-twist-mux

# Lidar: try apt first; if nothing, use Slamtec’s ROS 2 driver
sudo apt install -y ros-jazzy-rplidar-ros || true
```

4) Test the rplidar
```bash
source /opt/ros/jazzy/setup.bash
ros2 launch rplidar_ros view_rplidar.launch.py
```

# Usage

## remote robot on windows
when the robot is running in headless mode, it is possible to see the laser output on a windows PC.

As a prerequisites make sure to troubleshout the connection with `ros2 multicast send` and `ros2 multicast receive`, e.g. wsl even with mirror mode might not work on some windows wifi adapters.

Steps :
- run the simple rplidar launch with

```bash
ros2 launch rplidar_ros rplidar.launch.py
```
- add a transform for visualization
```bash
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 \
  map laser
```
- then on windows open rviz
```cmd
>pixi shell
>call C:\pixi_ws\ros2-windows\local_setup.bat
>ros2 run rviz2 rviz2
```
- select default `map` on `Global Options/Fixed Frame` add a LaserScan and configure its topic to `/scan`


## Teleop bringup

1. Build the workspace (single merged install tree keeps the sourced path short when you add more packages later):

```bash
colcon build --merge-install
source install/setup.bash
```

2. Check joystick → twist without touching the robot hardware:

```bash
ros2 launch rovi_bringup joy.launch.py \
  joy_dev:=0 \
  cmd_vel_topic:=/cmd_vel

ros2 topic echo /cmd_vel
```

  - `joy_dev` is the SDL device index (0 ≈ `/dev/input/js0`).
  - Override `joy_params_file` or `teleop_params_file` if you keep custom YAMLs elsewhere.

3. Drive the robot with the Rosmaster board attached:

```bash
ros2 launch rovi_bringup teleop.launch.py \
  joy_dev:=0 \
  rosmaster_port:=/dev/ttyUSB0 \
  cmd_vel_topic:=/cmd_vel
```

  - `rosmaster_port` must match the serial device exposed by your controller board.
  - Set `rosmaster_debug:=true` if you want verbose logs while tuning.

4. Re-source `install/setup.bash` whenever you rebuild so the launch files and configs stay on your ROS 2 path.


# Data Flow
![packafe_flow](./docs/package_flow.drawio.svg)


```mermaid
flowchart TD
  Rosmaster[rosmaster_driver node]

  CMD(["cmd_vel<br/>(Twist)"])
  CMD -->|subscribe| Rosmaster

  EDT(["edition<br/>(Float32)"])
  Rosmaster -->|publish| EDT

  VOL(["voltage<br/>(Float32)"])
  Rosmaster -->|publish| VOL

  VRAW(["vel_raw<br/>(Twist)"])
  Rosmaster -->|publish| VRAW

  IMU(["/imu/data_raw<br/>(Imu)"])
  Rosmaster -->|publish| IMU

  MAG(["/imu/mag<br/>(MagneticField)"])
  Rosmaster -->|publish| MAG

  Joystick["ros-jazzy-joy node"]
  JOY(["/joy<br/>(Joy)"])
  Joystick -->|publish| JOY

  RoviJoy["ros-jazzy-teleop-twist-joy node"]
  JOY -->|subscribe| RoviJoy
  RoviJoy -->|publish| CMD

  RoviBase["rovi_base node"]
  VRAW -->|subscribe| RoviBase
  ODRAW(["/odom_raw<br/>(Odometry)"])
  RoviBase -->|publish| ODRAW

```
# Devices
## Joystick
| Control           | Axis   | Direction | Value trend | Robot action        |
|-------------------|--------|-----------|-------------|---------------------|
| Left stick right  | axis 0 | right     | -           | turn clockwise      |
| Left stick left   | axis 0 | left      | +           | turn anti-clockwise |
| Right stick left  | axis 3 | left      | -           | move left           |
| Right stick right | axis 3 | right     | +           | move right          |
| Right stick down  | axis 4 | down      | +           | move rear           |
| Right stick up    | axis 4 | up        | -           | move front          |


![Joystick Control](./docs/joystick_control.drawio.svg)

## ELP Stereo camera

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext
```
see more details in [stereo camera](./docs/stereo.md)

