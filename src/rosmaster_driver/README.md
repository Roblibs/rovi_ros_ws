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

## Build (Windows cmd.exe)

Adjust the ROS 2 setup path to your installation if different.

```bat
cd /d d:\Projects\room_view_bot\rovi_ros_ws
call C:\dev\ros2_jazzy\local_setup.bat
colcon build --merge-install
call install\local_setup.bat
```

## Run

```bat
ros2 launch rosmaster_driver rosmaster_driver.launch.py
```

Parameters:
- `imu_link` (string, default `imu_link`)
- `prefix` (string, default empty) — prepended to joint names
- `publish_rate` (double, default `10.0` Hz)

## Notes

- Requires the hardware library `Rosmaster_Lib` to be importable by Python. If it
  is missing, the node will log a warning and publish zeros while ignoring `cmd_vel`.
