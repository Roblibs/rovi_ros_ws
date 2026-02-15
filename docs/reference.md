# Commands
All commands in this section are provided by `rovi_env.sh`

| Command | Description |
|---|---|
| `sim` | PC simulation: `sim` (default `mapping`) or `sim teleop|camera|mapping|localization|nav` runs `rovi_bringup/rovi.launch.py robot_mode:=sim stack:=...` and starts Gazebo + RViz by default (`rviz:=false` for headless). |
| `view` | PC visualization: `view` (default `nav`) or `view teleop|camera|mapping|nav`. Use `view offline` for local URDF inspection (no hardware). |
| `teleop` | Robot (Pi): runs `rovi_bringup/rovi.launch.py` with `robot_mode:=real stack:=teleop` (headless; no RViz). |
| `camera` | Robot (Pi): runs `rovi_bringup/rovi.launch.py` with `robot_mode:=real stack:=camera` (teleop + depth + RGB; headless; no RViz). |
| `calib` | Runs camera intrinsics calibration (GUI): `ros2 run camera_calibration cameracalibrator --size 8x5 --square 0.028 --ros-args --remap image:=/camera/color/image --remap camera/set_camera_info:=/camera/color/v4l2_camera/set_camera_info`. |
| `mapping` | Robot (Pi): runs `rovi_bringup/rovi.launch.py` with `robot_mode:=real stack:=mapping` (headless; no RViz). |
| `localization` | Robot (Pi): runs `rovi_bringup/rovi.launch.py` with `robot_mode:=real stack:=localization` (headless; no RViz). |
| `nav` | Robot (Pi): runs `rovi_bringup/rovi.launch.py` with `robot_mode:=real stack:=nav` (headless; no RViz). |
| `ws` | Changes to `$ROVI_ROS_WS_DIR`, runs `setup`, and activates `.venv`. |
| `build` | Runs system `colcon build` (ROS Jazzy toolchain), while keeping the workspace `.venv` active for Python deps. This generates/updates the `install/` overlay used by `ros2 launch`. |
| `stop` | Stops ROS/Gazebo/RViz processes that belong to this workspace (use `stop --all` for an aggressive full user hard-stop). |
| `setup` | Sources `rovi_ros_ws/install/setup.bash` (after a successful build). This overlays workspace packages (e.g., `rovi_bringup`) into your current shell. |
| `activate` | Activates `rovi_ros_ws/.venv` (created by `uv sync`). This provides Python dependencies needed by the real-robot stack and Python nodes. |
| `keyboard` | Runs `tools/rovi_keyboard.py` in the current terminal and publishes `/cmd_vel_keyboard` for `twist_mux` (run in its own terminal). |
| `foxglove` | PC only: runs `ros2 launch foxglove_bridge foxglove_bridge_launch.xml` (Foxglove WebSocket bridge). |

# Packages
Packages of this repo are listed in this table

| Package | Role |
|---|---|
| `rovi_bringup` | Top-level launch entry points (teleop, camera, visualization, mapping/localization stacks) |
| `rovi_sim` | Gazebo Sim backend: worlds + bridges + `rovi_sim_base` + `rovi_gz_odom`; spawns the shared robot model from `rovi_description` into Gazebo Sim |
| `rovi_description` | URDF + meshes + RViz configs; provides static TF like `base_footprint -> base_link -> laser_link` |
| `rosmaster_driver` | Hardware bridge: `/cmd_vel` → MCU, publishes `/vel_raw`, `/joint_states`, `/imu/data_raw`, `/imu/mag`, etc. |
| `rovi_odom_integrator` | Odometry integrator (real robot): `/vel_raw` → `/odom_raw`; can publish TF `odom -> base_footprint` (raw odom). In simulation, `/odom_raw` comes from Gazebo ground truth. |
| `rovi_localization` | Odometry filtering pipeline: IMU orientation filter + EKF; publishes `/odometry/filtered` and TF `odom -> base_footprint` |
| `rovi_slam` | SLAM pipeline (`slam_toolbox`): publishes `/map` and TF `map -> odom` when enabled |
| `rovi_nav` | Nav2 integration package: configuration + component launch for autonomous navigation |
| `ros_ui_bridge` | UI bridge: status fields (CPU + ROS topic values/rates + TF rates) with ROS-time staleness filtering and gRPC streaming; also serves robot pose, lidar, map, and model metadata |
| `robot_serial_display` | Serial UI client: consumes `ros_ui_bridge` gRPC status stream and pushes JSON lines to the ESP32-S3 display |

External ROS packages installed via apt (and a couple of local tools) and how they are used in this workspace:

| Dependency | Description |
|---|---|
| `ros-jazzy-joy` | Used by `rovi_bringup/teleop.launch.py` to start `joy_node` and publish `/joy` from the joystick device. |
| `ros-jazzy-teleop-twist-joy` | Used by `rovi_bringup/teleop.launch.py` to convert `/joy` into `/cmd_vel_joy`, using `rovi_bringup/config/teleop_twist_joy.yaml`. |
| `tools/rovi_keyboard.py` | Keyboard teleop (`keyboard` command) that publishes `/cmd_vel_keyboard` (`geometry_msgs/msg/Twist`) and is muxed via `twist_mux` like joystick and Nav2. |
| `ros-jazzy-twist-mux` | Used by `rovi_bringup/teleop.launch.py` to mux joystick `/cmd_vel_joy`, keyboard `/cmd_vel_keyboard`, and Nav2 `/cmd_vel_nav` into the final `/cmd_vel` topic consumed by the robot backend. |
| `ros-jazzy-diagnostic-updater` | Used by `rovi_odom_integrator` to publish runtime diagnostics (for example on `/diagnostics`). |
| `ros-jazzy-robot-state-publisher` | Used by `rovi_bringup/robot_bringup.launch.py` (all modes) to publish the TF tree from the `rovi_description` URDF and provide `/robot_description` to RViz. |
| `ros-jazzy-joint-state-publisher-gui` | Used by `rovi_bringup/robot_bringup.launch.py` in `robot_mode=offline` for interactive URDF inspection. |
| `ros-jazzy-rviz2` | Used by `view` (PC viewer command) and by `rovi_bringup/rovi.launch.py` in `robot_mode=sim|offline` (optional auto-start). |
| `ros-jazzy-rplidar-ros` | Used by `rovi_bringup/robot_bringup.launch.py` (robot_mode=real, `lidar_enabled:=true`) to publish `/scan` for SLAM and Nav2. |
| `ros-jazzy-ros-gz-sim` | Used by `rovi_sim/gazebo_sim.launch.py` to start Gazebo Sim and spawn the simulated robot + world. |
| `ros-jazzy-ros-gz-bridge` | Used by `rovi_sim/gazebo_sim.launch.py` to bridge Gazebo topics (e.g., LiDAR + `/clock`) into ROS 2 topics like `/scan` and `/clock`. |
| `ros-jazzy-foxglove-bridge` | Optional WebSocket bridge for Foxglove Studio (run via the `foxglove` command). |
| `python3-psutil` | Used by `ros_ui_bridge` to read CPU utilization. |
| `pyserial` | Used by `robot_serial_display` to talk to the ESP32-S3 display over USB serial. |
| `ros-jazzy-slam-toolbox` | Used via `rovi_slam/slam_toolbox.launch.py` (included by `mapping`, `localization`, and `nav`) to publish `/map` and TF `map -> odom`. |
| `ros-jazzy-robot-localization` | Used by `rovi_localization/ekf.launch.py` (included by `mapping`, `localization`, and `nav`) to publish `/odometry/filtered` and TF `odom -> base_footprint`. |
| `ros-jazzy-nav2-bringup` | Provides Nav2 servers started by `rovi_nav/launch/nav.launch.py` for goal-based navigation (planner/controller/BT navigator/behaviors). |
| `ros-jazzy-nav2-rviz-plugins` | Required by `rovi_nav.rviz` so RViz can show the Nav2 panel and tools (initial pose + goal) when `nav` is running. |
| `ros-jazzy-imu-filter-madgwick` | Used when `odom_mode=fusion_wheels_imu` to filter `/imu/data_raw` (and optionally `/imu/mag`) into `/imu/data` for the EKF. |

# Launches
| Launch | Package | Description |
|---|---|---|
| `rovi.launch.py` | `rovi_bringup` | Unified entrypoint (selects `stack` + `robot_mode`, owns RViz startup policy). |
| `robot_bringup.launch.py` | `rovi_bringup` | Backend selector: real drivers / Gazebo sim / offline URDF, exposing the stable topic contract (`/scan`, `/imu/data_raw`, `/cmd_vel`, etc.). |
| `teleop.launch.py` | `rovi_bringup` | Control stack only: joystick → `/cmd_vel_joy` + `twist_mux` → `/cmd_vel` (backend must already be running). |
| `mapping.launch.py` | `rovi_bringup` | Mapping stack only: EKF + `slam_toolbox` (mapping mode). |
| `localization.launch.py` | `rovi_bringup` | Localization stack only: EKF + `slam_toolbox` (localization mode, loads `map_file_name`). |
| `nav.launch.py` | `rovi_bringup` | Navigation stack only: mapping/localization + Nav2 (publishes `/cmd_vel_nav`). |
| `offline_view.launch.py` | `rovi_bringup` | Legacy offline inspection: `robot_mode=offline` + RViz (superseded by `view offline` / `rovi.launch.py`). |
| `joy.launch.py` | `rovi_bringup` | Debug joystick → `/cmd_vel` only (no hardware required) |
| `gazebo_sim.launch.py` | `rovi_sim` | Gazebo Sim backend: starts Gazebo + bridges + spawns the robot model (used when `robot_mode=sim`) |
| `rosmaster_driver.launch.py` | `rosmaster_driver` | Hardware driver only (serial/IMU/joints sanity checks) |
| `ekf.launch.py` | `rovi_localization` | Component launch: odometry pipeline (`odom_mode` selects raw/filtered/fusion_wheels_imu; useful without SLAM) |
| `slam_toolbox.launch.py` | `rovi_slam` | Component launch: `slam_toolbox` (mapping/localization selected by params) |
| `nav.launch.py` | `rovi_nav` | Component launch: Nav2 servers + lifecycle manager |

# Nodes
ROS nodes started by the launches above (some are conditional based on params). For per-node notes and hardware details, see [docs/nodes.md](./nodes.md).

| Node | Package | Description |
|---|---|---|
| `joy_node` | `joy` | Reads a joystick device and publishes `/joy` (`sensor_msgs/msg/Joy`). |
| `teleop_twist_joy` | `teleop_twist_joy` | Converts `/joy` into velocity commands on `/cmd_vel_joy` (`geometry_msgs/msg/Twist`). |
| `rovi_keyboard` | `tools/rovi_keyboard.py` | Keyboard teleop tool that publishes `/cmd_vel_keyboard` (`geometry_msgs/msg/Twist`) (run via the `keyboard` command in its own terminal). |
| `twist_mux` | `twist_mux` | Selects the active velocity command source (e.g., `/cmd_vel_nav` vs `/cmd_vel_joy`) and publishes `/cmd_vel`. |
| `rosmaster_driver` | `rosmaster_driver` | Hardware bridge for the Rosmaster base board: subscribes to `/cmd_vel` and publishes feedback like `/vel_raw`, `/joint_states`, `/imu/data_raw`, `/imu/mag`, and `/voltage`. |
| `rovi_odom_integrator` | `rovi_odom_integrator` | Integrates `/vel_raw` into `/odom_raw` and can broadcast TF `odom -> base_footprint` when enabled. |
| `robot_state_publisher` | `robot_state_publisher` | Publishes the robot TF tree from the URDF (`robot_description`) and `/joint_states`. |
| `rovi_local_joint_states` | `rovi_sim` | Local (stub) publisher for `/joint_states` in `robot_mode=sim` (zeros for non-fixed URDF joints). Exists to avoid a known noisy shutdown race in upstream `joint_state_publisher` on Jazzy. |
| `rplidar_composition` | `rplidar_ros` | Publishes `/scan` (`sensor_msgs/msg/LaserScan`) from an RPLIDAR (only when `lidar_enabled:=true`). |
| `parameter_bridge` | `ros_gz_bridge` | Bridges Gazebo Transport topics into ROS 2 topics (used by simulation for `/scan`, `/clock`, and `/imu/data_raw`). |
| `rovi_sim_base` | `rovi_sim` | Simulation base: subscribes `/cmd_vel`, applies acceleration limits, and publishes `/cmd_vel_sim` (to Gazebo) + `/vel_raw` (to `rovi_odom_integrator`). |
| `imu_filter` | `imu_filter_madgwick` | Filters `/imu/data_raw` (and optionally `/imu/mag`) into `/imu/data` (only when `odom_mode:=fusion_wheels_imu`). |
| `ekf_filter_node` | `robot_localization` | EKF that produces `/odometry/filtered` and TF `odom -> base_footprint` from `/odom_raw` (and `/imu/data` in `fusion_wheels_imu`). |
| `slam_toolbox` | `slam_toolbox` | Lifecycle SLAM node that publishes TF `map -> odom` and (in mapping mode) `/map` (only when `slam_enabled:=true`). |
| `odom_to_basefootprint` | `tf2_ros` | Static TF publisher used by `offline_view.launch.py` to provide `odom -> base_footprint` without hardware. |
| `joint_state_publisher_gui` | `joint_state_publisher_gui` | GUI for publishing `/joint_states` for offline URDF inspection (`robot_mode=offline`). |
| `rviz2` | `rviz2` | Visualization client (used by `offline_view.launch.py` and standalone `rviz2 -d ...`). |
| `bt_navigator` | `nav2_bt_navigator` | Nav2 behavior tree navigator; provides the `/navigate_to_pose` action server. |
| `planner_server` | `nav2_planner` | Nav2 global planner that produces paths in the `map` frame. |
| `controller_server` | `nav2_controller` | Nav2 local controller that publishes velocity commands on `/cmd_vel_nav`. |
| `behavior_server` | `nav2_behaviors` | Nav2 recovery/behavior actions (spin, backup, wait) used by the BT. |
| `lifecycle_manager_navigation` | `nav2_lifecycle_manager` | Configures and activates the Nav2 lifecycle nodes (autostart). |
| `ui_bridge_metrics` | `ros_ui_bridge` | Metrics node: collects configured status fields (CPU, ROS topic values, topic Hz, TF Hz), enforces ROS-time staleness, and supports TF demux republishing to `/viz/tf/<parent>_<child>`. |
| `ui_bridge_robot_state` | `ros_ui_bridge` | Robot state node: subscribes to `/odom_raw`, `/joint_states`, and TF; throttle-forwards robot pose/wheels to gRPC clients. |
| `ui_bridge_lidar` | `ros_ui_bridge` | Lidar node: subscribes to `/scan`, throttle-forwards to gRPC clients, and republishes to `/viz/scan`. |
| `serial_display` | `robot_serial_display` | Serial display client: consumes `ros_ui_bridge` gRPC status stream and pushes JSON lines to the ESP32-S3 display over USB serial. |

# Params
Only the parameters toggling node activation are listed here.

| Param | Package | Launch | Default | Explanation |
|---|---|---|---|---|
| `robot_mode` | `rovi_bringup` | `rovi.launch.py`, `gateway.launch.py`, `robot_bringup.launch.py` | `real` | Selects the robot backend: `real`, `sim`, or `offline`. |
| `stack` | `rovi_bringup` | `rovi.launch.py` | `teleop` | Selects the stack to run: `teleop`, `camera`, `mapping`, `localization`, `nav`, `offline`, `bringup`. |
| `gateway_enabled` | `rovi_bringup` | `rovi.launch.py` | `true` | Starts the gateway plane (backend + UI) as part of the same launch. Set `false` when systemd owns the gateway (`rovi-gateway.service`). |
| `rviz` | `rovi_bringup` | `rovi.launch.py` | auto | Starts RViz automatically (default **off** for `real`, **on** for `sim`/`offline`). |
| `joy_enabled` | `rovi_bringup` | `rovi.launch.py`, `teleop.launch.py` | auto | Starts joystick nodes (default **on** for `real`, **off** for `sim`). |
| `lidar_enabled` | `rovi_bringup` | `robot_bringup.launch.py` | `true` | Starts LiDAR driver (`rplidar_ros`) in `robot_mode=real`; without it there is no `/scan`. |
| `slam_enabled` | `rovi_bringup` | `mapping.launch.py`, `localization.launch.py`, `rovi.launch.py` | `true` | Starts `slam_toolbox`; publishes TF `map -> odom` (and `/map` in mapping mode). |
| `slam_enabled` | `rovi_slam` | `slam_toolbox.launch.py` | `true` | Starts `slam_toolbox`; publishes TF `map -> odom` (and `/map` in mapping mode) |
| `slam_mode` | `rovi_slam` | `slam_toolbox.launch.py` | `mapping` | Selects SLAM mode: `mapping` or `localization`. |
| `slam_mode` | `rovi_bringup` | `nav.launch.py`, `rovi.launch.py` | `mapping` | Selects SLAM mode when running Nav2. |
| `odom_mode` | `rovi_bringup` | `mapping.launch.py`, `localization.launch.py`, `nav.launch.py`, `rovi.launch.py` | `filtered` | Selects the odometry filter pipeline (and whether IMU is used). TF publisher selection depends on how the gateway plane is started (single-command `rovi.launch.py` vs systemd `rovi-gateway.service`). |
| `odom_mode` | `rovi_localization` | `ekf.launch.py` | `filtered` | Same as above, but for running the odometry pipeline without SLAM |
| `mag_enabled` | `rovi_bringup` | `mapping.launch.py`, `localization.launch.py`, `nav.launch.py`, `rovi.launch.py` | `false` | Enables magnetometer input for the IMU filter (used in `odom_mode=fusion_wheels_imu`; disabled by default due to interference risk) |
| `mag_enabled` | `rovi_localization` | `ekf.launch.py` | `false` | Enables magnetometer input for the IMU filter (used in `odom_mode=fusion_wheels_imu`; disabled by default due to interference risk) |
| `map_file_name` | `rovi_bringup` | `localization.launch.py`, `nav.launch.py`, `rovi.launch.py` | `~/.ros/rovi/maps/latest.posegraph` | Pose-graph file to load when `slam_mode=localization`. |

## Robot modes
| `robot_mode` | Intended machine | Backend (started by `gateway.launch.py`, includes `robot_bringup.launch.py`) | `use_sim_time` | RViz default |
|---|---|---|---|---|
| `real` | robot (Pi) | real drivers + sensors | `false` | `off` |
| `sim` | PC | Gazebo Sim backend (+ ground-truth odom) | `true` | `on` |
| `offline` | PC | URDF inspection only | `false` | `on` |

## Odometry Modes
Used by: `rovi_bringup/mapping.launch.py`, `rovi_bringup/localization.launch.py`, and `rovi_localization/ekf.launch.py`.

Note:
- In systemd mode (`rovi-gateway.service`), the gateway’s TF publisher choice is configured via `ROVI_ODOM_INTEGRATOR_PUBLISH_TF` and requires restarting the gateway to change.

| `odom_mode` | `odom_integrator_publish_tf` (bringup) | TF `odom -> base_footprint` | Nodes started | Notes |
|---|---|---|---|---|
| `raw` | `true` | `rovi_odom_integrator` (real) / `rovi_gz_odom` (sim) | (none) | Fastest/simplest; no `/odometry/filtered` |
| `filtered` | `false` | `robot_localization/ekf_node` | `robot_localization/ekf_node` | Filters wheel odom (`/odom_raw` → `/odometry/filtered`) |
| `fusion_wheels_imu` | `false` | `robot_localization/ekf_node` | `imu_filter_madgwick` + `robot_localization/ekf_node` | Adds IMU yaw + yaw rate (`/imu/data_raw` → `/imu/data`); set `mag_enabled:=true` to use `/imu/mag` |
