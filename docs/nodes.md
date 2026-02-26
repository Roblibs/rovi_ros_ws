# Rosbags record/play

`record` and `play` are lightweight helpers around `ros2 bag`.

- Default bag directory: `~/.ros/rovi/bags/`
- Current launch is published on: `/rovi/session/current_launch_ref` (written automatically when running `teleop`, `camera`, `mapping`, `localization`, `nav`)
- Topic filters are configured in: `src/rovi_bringup/config/bag_topics.yaml`

`bag_topics.yaml` format: top-level keys are launch *base names* (`teleop`, `nav`, ...).

- If the value is a list: it is the full topic list to record (legacy override).
- If the value is a mapping: `add`/`drop` are applied as deltas on top of `docs/contract.yaml` required topics for that stack.

Examples:

```bash
record
record nav
play
play nav
```

# Robot backend contract
Goal: keep the higher-level stack (`rovi_localization`, `rovi_slam`, `rovi_nav`, RViz) unchanged while selecting *how the robot is provided* via a single `robot_mode` argument.

## Robot interface contract (what the rest of the stack assumes)
- `/cmd_vel` (`geometry_msgs/msg/Twist`) holonomic X/Y + yaw
- `/scan` (`sensor_msgs/msg/LaserScan`)
- `/vel_raw` (`geometry_msgs/msg/Twist`) base feedback (used by `rovi_odom_integrator`)
- `/odom_raw` (`nav_msgs/msg/Odometry`) and TF `odom -> base_footprint` (raw or filtered depending on `odom_mode`)
- `/imu/data_raw` (`sensor_msgs/msg/Imu`) (needed for `odom_mode=fusion_wheels_imu`)
- `/clock` (sim only, with `use_sim_time:=true`)

## Modes (overview)
See [stacks.md](./stacks.md) for the full `robot_mode` + `stack` matrix, RViz policy, and sensor/topic expectations (real vs sim).

## Control inputs
All velocity sources feed into `twist_mux` and produce the single `/cmd_vel` topic:
- joystick: `/cmd_vel_joy`
- keyboard: `/cmd_vel_keyboard` (via `tools/rovi_keyboard.py`)
- navigation: `/cmd_vel_nav`

## Status
See [stacks.md](./stacks.md) for stack composition and RViz defaults per `robot_mode`.

# rosmaster driver
![package_flow](./rosmaster.drawio.svg)

| Movement | ros axis | rosmaster Axis |
|----------|----------|----------------|
| forward/backwards | X | -Y |
| left/right        | Y | -X  |
| Rotation          | Z | Z  |

MPU axis

![axis](./MPU9250-axis.png)
- Chip orientation: pin1 pointing to the buzzer edge of the rosmaster control board.
- Buzzer edge of the rosmaster control board points to the robot front-left corner.

From raw board measurement, the sensor reveals a 180° flip on its X axis.

| Robot | IMU | ROS axis |
|-------|-----|----------|
| Right | X  | -Y |
| Front | -Y  | X |
| Up    | -Z  | Z |

This rotation will be compensated in the driver so that output of `/imu/data_raw` `imu.linear_acceleration` should be as follows.

| Position | Axis | Value |
|----------|------|-------|
| Back with the front facing up | X | 9.8 |
| On right side, left facing up | Y | 9.8 |
| Up facing up | Z | 9.8 |

# Power control

![design](./power.drawio.svg)

# Wheels

- `ROS-Driver-Board\1.Code\Factory STM32 firmware\Rosmaster_V3.5.1\ControlBoard_Rosmaster\Source\APP\app_mecanum.h`
- `ROS-Driver-Board\1.Code\Factory STM32 firmware\Rosmaster_V3.5.1\V3.5.1\Source\APP\app_motion.h`

```c++
#define MECANUM_MAX_CIRCLE_MM        (251.327f)

#define ENCODER_CIRCLE_205           (2464.0f)

typedef enum _car_type
{
    CAR_MECANUM_MAX = 0x02,//X3 PLUS
} car_type_t;
```
| Motor | wheel | direction |
|-------|-------|-----------|
|m1     | Front Left | flip |
|m2     | Front Right | |
|m3     | Back Left | |
|m4     | Back Right | flip |


| Robot Model        | Wheel Diameter (mm) | Wheel circumference (mm) | Wheel encoder steps |
|--------------------|---------------------|--------------------------|---------------------|
| X3                 | 80                  | 251.327                  | 2464                |


# Joystick
| Control           | Axis   | Axis sign | Robot action        | Robot command | command scale sign |
|-------------------|--------|-----------|---------------------|---------------|---------------|
| Left stick right  | axis 0  | +     | turn clockwise      | axis_angular.yaw | + |
| Left stick left   | axis 0  | -     | turn anti-clockwise | axis_angular.yaw | + |
| Right stick right | axis 3  | +     | move right          | axis_linear.y    | - |
| Right stick left  | axis 3  | -     | move left           | axis_linear.y    | - |
| Right stick down  | axis 4  | +     | move rear           | axis_linear.x    | - |
| Right stick up    | axis 4  | -     | move front          | axis_linear.x    | - |


![Joystick Control](./joystick_control.drawio.svg)

Check your joystick before starting:
```bash
jstest /dev/input/js0
```

Check joystick → twist without touching the robot hardware:

```bash
ros2 launch rovi_bringup joy.launch.py \
  joy_dev:=0 \
  cmd_vel_topic:=/cmd_vel_joy

ros2 topic echo /cmd_vel_joy
```

- `joy_dev` is the SDL device index (0 ≈ `/dev/input/js0`).
- Override `joy_params_file` or `teleop_params_file` if you keep custom YAMLs elsewhere.



# Cameras
```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext
```
## Stereo camera
ELP stereo camera.

See more details in [stereo camera](./stereo_elp.md).

# Depth camera
Orbbec `Astra Stereo S U3`, aka Yahboom `AI View Depth Camera`

* product https://store.orbbec.com/products/astra-stereo-s-u3
* used library https://github.com/orbbec/OpenNI_SDK
* API reference https://github.com/orbbec/OpenNI_SDK/blob/main/Doc/English/0400_API.md

See more details in [depth camera](./depth_camera_astra_stereo_s_u3.md).


# rplidar
When testing the rplidar alone, it is necessary to add a transform for visualization.

Steps:
- Run the simple rplidar launch with

```bash
ros2 launch rplidar_ros rplidar.launch.py
```
- Add a transform for visualization
```bash
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 \
  map laser
```
- Then on Windows open RViz
```cmd
>ros2 run rviz2 rviz2
```
- Select default `map` on `Global Options/Fixed Frame`, add a LaserScan, and configure its topic to `/scan`

# WiFi adapter
* using AX1800 from BrosTrend model No.: AX4L
* Linux install doc https://linux.brostrend.com/
