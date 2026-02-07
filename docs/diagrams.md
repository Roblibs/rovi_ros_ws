# Diagrams

## Unified Robot Interface
Command Interface and Feedback Interface are the contractual topics used by both the real robot and the simulation.

```mermaid
flowchart TB

    TELEOP["Teleop (joystick/keyboard)"]
    NAV2["Nav2"]
    GUI["State Gui"]

  subgraph L2["Command interface"]
    direction TB
    CMD(["/cmd_vel<br/>(Twist)"])
  end

  subgraph L3["robot_mode"]
    BACKEND["real | sim"]
    OFFLINE[offline]
  end

  subgraph L4["Feedback  interface"]
    direction TB
    SCAN(["/scan<br/>(LaserScan)"])
    IMU_RAW(["/imu/data_raw<br/>(Imu)"])
    VRAW(["/vel_raw<br/>(Twist)"])
    ODRAW(["/odom_raw<br/>(Odometry)"])
    CLOCK(["/clock<br/>(Clock, sim only)"])
    TF(["/tf"])
  end

  TELEOP --> CMD
  NAV2 --> CMD

  GUI --> OFFLINE

  CMD --> BACKEND

  BACKEND --> SCAN
  BACKEND --> IMU_RAW
  BACKEND --> VRAW
  BACKEND --> ODRAW
  BACKEND --> CLOCK

  BACKEND --> TF
  OFFLINE --> TF

```

## UI bridge (gRPC)
`ros_ui_bridge` subscribes to ROS topics for robot state/telemetry and exposes them over gRPC (plus a few ROS viz helper topics). Topics shown match `src/ros_ui_bridge/config/default.yaml`.

```mermaid
flowchart LR

  SYS_CPU("system: cpu_percent (psutil)")
  TOPIC_SCAN[("/scan<br/>(LaserScan)")]
  TOPIC_ODOM[("/odom_raw<br/>(Odometry)")]
  TOPIC_JOINTS[("/joint_states<br/>(JointState)")]
  TOPIC_TF[("/tf<br/>(TFMessage)")]
  TOPIC_MAP[("/map<br/>(OccupancyGrid)")]
  TOPIC_VOLTAGE[("/voltage<br/>(Float32)")]
  TOPIC_VIZ_SCAN[("/viz/scan<br/>(LaserScan)")]
  TOPIC_VIZ_TF[("/viz/tf/{parent}_{child}<br/>(TFMessage)")]

  subgraph BRIDGE["ros_ui_bridge (single process)"]
    direction TB
    NODE_LIDAR("ui_bridge_lidar")
    NODE_MAP("ui_bridge_map")
    NODE_STATE("ui_bridge_robot_state")
    NODE_METRICS("ui_bridge_metrics")
    NODE_STATUS("ui_bridge_node (status publish loop)")
    NODE_MODEL("RobotModelProvider")
    NODE_GRPC("gRPC: UiBridge service")
    MODEL_GLB("robot model: package://rovi_description/models/rovi.glb (+ .meta.json)")
  end

  subgraph CONSUMERS["Consumers"]
    direction TB
    RVIZ(rviz2)
    UI_CLIENTS("UI clients (web/desktop)")
    DISPLAY_CLIENT("robot_serial_display (ESP32 display)")
    Plot
  end

  %% Lidar: ROS -> (throttle) -> ROS + gRPC
  TOPIC_SCAN -->|subscribe| NODE_LIDAR
  NODE_LIDAR -->|"publish (throttled)"| TOPIC_VIZ_SCAN
  NODE_LIDAR -->|StreamLidar| NODE_GRPC
  TOPIC_VIZ_SCAN -->|subscribe| RVIZ

  %% Map: ROS -> gRPC
  TOPIC_MAP -->|subscribe| NODE_MAP
  NODE_MAP -->|StreamMap| NODE_GRPC

  %% Robot state: ROS -> gRPC
  TOPIC_ODOM -->|subscribe| NODE_STATE
  TOPIC_JOINTS -->|subscribe| NODE_STATE
  TOPIC_TF -->|"subscribe (map->odom)"| NODE_STATE
  NODE_STATE -->|StreamRobotState| NODE_GRPC

  %% Status: system + ROS metrics -> gRPC
  SYS_CPU -->|sample| NODE_STATUS
  TOPIC_VOLTAGE -->|"subscribe (value + rate)"| NODE_METRICS
  TOPIC_ODOM -->|"subscribe (rate)"| NODE_METRICS
  TOPIC_SCAN -->|"subscribe (rate)"| NODE_METRICS
  TOPIC_TF -->|"subscribe (tf_rate + demux)"| NODE_METRICS
  NODE_METRICS -->|"publish (demux)"| TOPIC_VIZ_TF
  NODE_METRICS -->|values + rates| NODE_STATUS
  NODE_STATUS -->|GetStatus + StreamStatus| NODE_GRPC
  TOPIC_VIZ_TF -->|subscribe| Plot

  %% Robot model: file -> gRPC
  MODEL_GLB -->|load| NODE_MODEL
  NODE_MODEL -->|GetRobotModelMeta + GetRobotModel| NODE_GRPC

  %% gRPC: server -> clients
  NODE_GRPC -->|gRPC| UI_CLIENTS
  NODE_GRPC -->|gRPC| DISPLAY_CLIENT
```

## Real robot
`robot_mode=real`

```mermaid
flowchart TD
  CMD(["/cmd_vel<br/>(Twist)"])
  Rosmaster["rosmaster_driver<br>(control board)"]
  CMD -->|subscribe| Rosmaster

  EDT(["/edition<br/>(Float32)"])
  VOL(["/voltage<br/>(Float32)"])
  VRAW(["/vel_raw<br/>(Twist)"])
  JSTATE(["/joint_states<br/>(JointState)"])
  ENC(["/encoders<br/>(Int32MultiArray)"])
  IMU_RAW(["/imu/data_raw<br/>(Imu)"])
  MAG(["/imu/mag<br/>(MagneticField)"])
  Rosmaster -->|publish| EDT
  Rosmaster -->|publish| VOL
  Rosmaster -->|publish| VRAW
  Rosmaster -->|publish| ENC
  Rosmaster -->|publish| JSTATE
  Rosmaster -->|publish| IMU_RAW
  Rosmaster -->|publish| MAG

  RoviOdom["rovi_odom_integrator"]
  VRAW -->|subscribe| RoviOdom
  ODRAW(["/odom_raw<br/>(Odometry)"])

  subgraph TF["/tf"]
    TF_ODOM_BASE("odom->base_footprint")
    TF_LINKS_JOINTS("links/joints")
  end

  RoviOdom -->|publish| ODRAW
  RoviOdom -->|publish| TF_ODOM_BASE

  RSP["robot_state_publisher"]
  JSTATE -->|subscribe| RSP
  RSP -->|publish| TF_LINKS_JOINTS

  RPLidar["rplidar_ros (LiDAR device)"]
  SCAN(["/scan<br/>(LaserScan)"])
  RPLidar -->|publish| SCAN

```

## Gazebo Simulation
`robot_mode=sim`

In this mode, the real robot is replaced by a Gazebo simulation while `rovi_localization` / `rovi_slam` / `rovi_nav` remain unchanged.

```mermaid
flowchart TD
  CMD(["/cmd_vel<br/>(Twist)"])

  RoviSimBase["rovi_sim_base<br/>(holonomic base + accel limits)"]
  CMD -->|subscribe| RoviSimBase
  CMD_SIM(["/cmd_vel_sim<br/>(Twist)"])
  VRAW(["/vel_raw<br/>(Twist)"])
  RoviSimBase -->|publish| CMD_SIM
  RoviSimBase -->|publish| VRAW

  ODRAW(["/odom_raw<br/>(Odometry)"])

  subgraph GZ["Gazebo Sim (gz sim)"]
    WORLD["room world"]
    ROBOT["rovi robot model (URDF from rovi_description)"]
    LIDAR["lidar sensor"]
    IMU_GZ["imu sensor"]
    ODOM_GZ(["/model/rovi/odometry<br/>(gz.msgs.Odometry)"])
  end

  BRIDGE["ros_gz_bridge<br/>(parameter_bridge)"]

  CMD_SIM --> BRIDGE
  BRIDGE -->|bridge| ROBOT
  LIDAR --> BRIDGE
  IMU_GZ --> BRIDGE
  ODOM_GZ --> BRIDGE

  SCAN(["/scan<br/>(LaserScan)"])
  CLOCK(["/clock"])
  IMU_RAW(["/imu/data_raw<br/>(Imu)"])
  ODOM_BRIDGED(["/odom_gz<br/>(Odometry)"])
  BRIDGE -->|bridge| SCAN
  BRIDGE -->|bridge| CLOCK
  BRIDGE -->|bridge| IMU_RAW
  BRIDGE -->|bridge| ODOM_BRIDGED

  TF_ODOM_BASE(["/tf<br>odom->base_footprint"])

  RoviGzOdom[rovi_gz_odom]
  ODOM_BRIDGED -->|subscribe| RoviGzOdom
  RoviGzOdom -->|publish| ODRAW
  RoviGzOdom -.->|publish| TF_ODOM_BASE

```

## Offline model
`robot_mode=offline`

This launch helps to visualize the robot to inspect its 3D model without other dependencies.

```mermaid
flowchart TD
  RoviDesc["rovi_description package"]
  RSP["robot_state_publisher"]
  JSPG["joint_state_publisher_gui"]
  JSTATE(["/joint_states<br/>(JointState)"])
  TF(["/tf<br/>(TF)"])
  RVIZ["rviz2"]

  RoviDesc -->|provides URDF| RSP
  JSPG -->|publish| JSTATE
  JSTATE -->|subscribe| RSP
  RSP -->|publish| TF
  TF -->|subscribe| RVIZ
```

## Sim joint states
`robot_mode=sim`

In simulation we still publish `/joint_states` so `robot_state_publisher` (and any consumers like the UI bridge) can operate the same way as on the real robot.

We **do not** use upstream `joint_state_publisher` here because on ROS 2 Jazzy it can occasionally throw an `RCLError: context is not valid` during Ctrl-C shutdown (a shutdown race in `rclpy` executors). Instead we run a small local publisher.


## Robot Control

```mermaid
flowchart TD
  Joystick["joy_node <br> (joystick)"]
  JOY(["/joy<br/>(Joy)"])
  Joystick -->|publish| JOY

  TeleopJoy["teleop_twist_joy"]
  JOY -->|subscribe| TeleopJoy
  CMD_JOY(["/cmd_vel_joy<br/>(Twist)"])
  TeleopJoy -->|publish| CMD_JOY

  Keyboard["rovi_keyboard (tools/rovi_keyboard.py)"]
  CMD_KEY(["/cmd_vel_keyboard<br/>(Twist)"])
  Keyboard -->|publish| CMD_KEY

  Nav2["nav2 controller"]
  CMD_NAV(["/cmd_vel_nav<br/>(Twist)"])
  Nav2 -->|publish| CMD_NAV

  TwistMux["twist_mux"]
  CMD_JOY -->|subscribe| TwistMux
  CMD_KEY -->|subscribe| TwistMux
  CMD_NAV -->|subscribe| TwistMux
  CMD(["/cmd_vel<br/>(Twist)"])
  TwistMux -->|publish| CMD

```

## Odometry filtering
Odometry filtering is only applicable for the real robot. When odom_mode=fusion_wheels_imu, this produces a single authoritative `odom -> base_footprint` transform using EKF + IMU. For `odom_mode=filtered`, omit the IMU branch; magnetometer usage is gated by `mag_enabled`.

```mermaid
flowchart TD
  Rosmaster[rosmaster_driver] -->|publish| IMU_RAW(["/imu/data_raw<br/>(Imu)"])
  Rosmaster -->|publish| MAG(["/imu/mag<br/>(MagneticField)"])
  RoviOdom[rovi_odom_integrator] -->|publish| ODRAW(["/odom_raw<br/>(Odometry)"])

  ImuFilter["imu_filter_madgwick (odom_mode=fusion_wheels_imu)"]
  IMU_RAW -->|subscribe| ImuFilter
  MAG -.->|"subscribe (mag_enabled)"| ImuFilter
  IMU(["/imu/data<br/>(Imu, orientation)"])
  ImuFilter -->|publish| IMU

  EKF["robot_localization/ekf_node"]
  ODRAW -->|subscribe| EKF
  IMU -.->|"subscribe (odom_mode=fusion_wheels_imu)"| EKF
  ODOMF(["/odometry/filtered<br/>(Odometry)"])
  EKF -->|publish| ODOMF
  EKF -->|publish| TF(["/tf<br/>odom -> base_footprint"])
```

## SLAM
slam_toolbox produces `map -> odom` so the robot pose is expressed in a stable map frame. `slam_mode` selects `mapping` (build/update the map) or `localization` (load a saved pose-graph from `map_file_name`).

```mermaid
flowchart TD
  subgraph TF_GROUP["/tf"]
    TF(["map -> odom"])
    TFCHAIN(["odom -> base_footprint -> laser_link"])
  end
  RPLidar[rplidar_ros] -->|publish| SCAN(["/scan<br/>(LaserScan)"])
  TFCHAIN -->|subscribe| Slam[slam_toolbox]
  SCAN -->|subscribe| Slam
  MAP(["/map<br/>(OccupancyGrid)"])
  Slam -->|publish| MAP
  Slam -->|publish| TF
```

## Navigation
Nav2 consumes map + TF + sensors to compute `/cmd_vel` for the base.
For navigation, SLAM can run in `slam_mode=mapping` (default) or `slam_mode=localization` (requires `map_file_name`).

Lifecycle-managed nodes (configured/activated by `nav2_lifecycle_manager`): `nav2_bt_navigator/bt_navigator`, `nav2_planner/planner_server`, `nav2_controller/controller_server`, `nav2_behaviors/behavior_server`.

```mermaid
flowchart TD
  GoalCLI["ros2cli<br/>(ros2 action send_goal)"]
  GoalRVIZ["nav2_rviz_plugins<br/>(rviz2 'Nav2 Goal' / '2D Goal Pose')"]
  NAV_ACTION(["/navigate_to_pose<br/>(nav2_msgs/action/NavigateToPose)"])
  GoalCLI -->|send goal| NAV_ACTION
  GoalRVIZ -->|send goal| NAV_ACTION

  Btnav["bt_navigator"]
  NAV_ACTION -->|action server| Btnav

  Planner["planner_server"]
  Controller["controller_server"]
  Behaviors["behavior_server"]
  Btnav -.->|"services/actions"| Planner
  Btnav -.->|"services/actions"| Controller
  Btnav -.->|"services/actions"| Behaviors

  MAP(["/map<br/>(OccupancyGrid)"])
  TF(["/tf<br/>(map->odom + odom->base_footprint + links)"])
  SCAN(["/scan<br/>(LaserScan)"])
  ODOMF(["/odometry/filtered<br/>(Odometry)"])

  MAP -->|subscribe| Planner
  TF -->|subscribe| Planner

  MAP -->|subscribe| Controller
  TF -->|subscribe| Controller
  SCAN -->|subscribe| Controller
  ODOMF -->|subscribe| Controller

  CMD_NAV(["/cmd_vel_nav<br/>(Twist)"])
  Controller -->|publish| CMD_NAV

  TwistMux["twist_mux"]
  CMD_NAV -->|subscribe| TwistMux
  CMD_JOY(["/cmd_vel_joy<br/>(Twist)"])
  CMD_JOY -.->|"subscribe (teleop)"| TwistMux
  CMD_KEY(["/cmd_vel_keyboard<br/>(Twist)"])
  CMD_KEY -.->|"subscribe (keyboard)"| TwistMux
  CMD(["/cmd_vel<br/>(Twist)"])
  TwistMux -->|publish| CMD

  Rosmaster["rosmaster_driver"]
  CMD -->|subscribe| Rosmaster
```

## Visualization

```mermaid
flowchart TD
  TF(["/tf<br/>(TF)"])
  ROBOT_DESC(["/robot_description<br/>(String)"])
  SCAN(["/scan<br/>(LaserScan)"])
  MAP(["/map<br/>(OccupancyGrid)"])

  RVIZ_ODOM["rviz2<br/>(rovi_odom.rviz, Fixed Frame: odom)"]
  RVIZ_MAP["rviz2<br/>(rovi_map.rviz, Fixed Frame: map)"]
  RVIZ_NAV["rviz2<br/>(rovi_nav.rviz, Fixed Frame: map)"]

  TF -->|subscribe| RVIZ_ODOM
  TF -->|subscribe| RVIZ_MAP
  TF -->|subscribe| RVIZ_NAV
  ROBOT_DESC -->|subscribe| RVIZ_ODOM
  ROBOT_DESC -->|subscribe| RVIZ_MAP
  ROBOT_DESC -->|subscribe| RVIZ_NAV
  SCAN -->|subscribe| RVIZ_ODOM
  SCAN -->|subscribe| RVIZ_MAP
  SCAN -->|subscribe| RVIZ_NAV
  MAP -->|subscribe| RVIZ_MAP
  MAP -->|subscribe| RVIZ_NAV
```


## TF Tree
TF frames are not regular ROS topics; this is the chain RViz and SLAM use at runtime.

```mermaid
flowchart TD
  MAP[map] -->|slam_toolbox| ODOM[odom]
  ODOM -->|"ekf_node OR rovi_odom_integrator (real) OR rovi_gz_odom (sim)"| BASEF[base_footprint]
  BASEF -->|robot_state_publisher| BASEL[base_link]
  BASEL -->|robot_state_publisher| LASER[laser_link]
  BASEL -->|robot_state_publisher| IMU[imu_link]
```

## Launch wiring
`rovi_bringup/robot_bringup.launch.py` selects the backend (`robot_mode:=real|sim|offline`), and `rovi_bringup/rovi.launch.py` is the single high-level entrypoint that chooses the stack and owns RViz startup policy.

```mermaid
flowchart LR
  Rovi["rovi_bringup/rovi.launch.py"]
  RobotBringup["rovi_bringup/robot_bringup.launch.py"]

  Rovi --> RobotBringup
  Rovi --> Teleop["rovi_bringup/teleop.launch.py"]
  Rovi --> Mapping["rovi_bringup/mapping.launch.py"]
  Rovi --> Localization["rovi_bringup/localization.launch.py"]
  Rovi --> NavLaunch["rovi_bringup/nav.launch.py"]

  EkfLaunch["rovi_localization/launch/ekf.launch.py"]
  SlamLaunch["rovi_slam/launch/slam_toolbox.launch.py"]
  NavStackLaunch["rovi_nav/launch/nav.launch.py"]

  Mapping -->|IncludeLaunchDescription| EkfLaunch
  Mapping -->|IncludeLaunchDescription| SlamLaunch

  Localization -->|IncludeLaunchDescription| EkfLaunch
  Localization -->|IncludeLaunchDescription| SlamLaunch

  NavLaunch -->|IncludeLaunchDescription| NavStackLaunch
  NavLaunch -->|"IncludeLaunchDescription<br/>(slam_mode=mapping)"| Mapping
  NavLaunch -->|"IncludeLaunchDescription<br/>(slam_mode=localization)"| Localization

  RobotBringup -->|"IncludeLaunchDescription<br/>(robot_mode=sim)"| GazeboSim["rovi_sim/launch/gazebo_sim.launch.py"]
```

## Package dependencies
```mermaid
flowchart LR
    RoviBringup[rovi_bringup]
    RoviDesc[rovi_description]
    Rosmaster[rosmaster_driver]
    RoviOdom[rovi_odom_integrator]
    RoviLoc[rovi_localization]
    RoviSlam[rovi_slam]
    RoviNav[rovi_nav]
    RoviSim[rovi_sim]

  
  Joy[ros-jazzy-joy]
  Teleop[ros-jazzy-teleop-twist-joy]
  TeleopKey[tools/rovi_keyboard.py]
  TwistMux[twist_mux]
  RSP[robot_state_publisher]
  LocalJoints[rovi_local_joint_states]
  JSPG[joint_state_publisher_gui]
  RVIZ[rviz2]
  Diag[diagnostic_updater]
  RPLidar[rplidar_ros]
  RosGzSim[ros_gz_sim]
  RosGzBridge[ros_gz_bridge]
  
  EKF[robot_localization]
  ImuFilter[imu_filter_madgwick]
  SlamToolbox[slam_toolbox]

  Nav2[nav2_bringup]

  RoviBringup --> RoviDesc
  RoviBringup --> Rosmaster
  RoviBringup --> RoviOdom
  RoviBringup -.-> RoviLoc
  RoviBringup -.-> RoviSlam
  RoviBringup --> Joy
  RoviBringup --> Teleop
  RoviBringup -.-> TeleopKey
  RoviBringup --> TwistMux
  RoviBringup --> RSP
  RoviSim -.-> LocalJoints
  RoviBringup --> JSPG
  RoviBringup --> RVIZ
  RoviBringup --> RPLidar
  RoviBringup -.-> RoviSim
  RoviBringup -.-> RoviNav

  RoviSim --> RosGzSim
  RoviSim --> RosGzBridge
  RoviSim --> RoviDesc

  RoviOdom --> Diag

  RoviLoc --> EKF
  RoviLoc -.-> ImuFilter
  RoviSlam --> SlamToolbox
  RoviNav --> Nav2

```
