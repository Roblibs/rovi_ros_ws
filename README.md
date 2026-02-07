# rovi_ros_ws
ROVI is the short name of "Room View Bot", a ROS robot platform for indoor virtual presence in homes and rooms, mostly flat ground. ROVI is based on the Yahboom X3+ chassis and control board equipped with mecanum wheels, a lidar depth camera, and a stereo camera. Videos, gallery, and more details about the robot hardware are available on this wiki page https://homesmartmesh.github.io/robotics/rovi/

This repo provides a ROS2 Jazzy software stack that covers multiple run modes including simulation.

This project is still in development; the current stage is functional navigation with LiDAR on the real robot and in simulation.

Related projects:
* webapp : https://github.com/MicroWebStacks/tanstack-robot-space
* LCD monitor firmware : https://github.com/ESP32Home/robot_serial_display

# Detailed docs

| Doc | What it covers |
|---|---|
| [install.md](./docs/install.md) | Installation and environment setup (ROS Jazzy, `uv`, dependencies, USB, and WSL notes). |
| [diagrams.md](./docs/diagrams.md) | Architecture and dataflow diagrams for interfaces, stacks, launch wiring, and dependencies. |
| [reference.md](./docs/reference.md) | Full command list and deeper reference tables. |
| [nodes.md](./docs/nodes.md) | Quick index of packages/nodes plus useful debug commands. |
| [depth_camera_astra_stereo_s_u3.md](./docs/depth_camera_astra_stereo_s_u3.md) | Astra Stereo S U3 depth camera (depth via OpenNI2 + RGB via UVC): run, topics/TF, install notes, calibration pointers. |
| [stereo_elp.md](./docs/stereo_elp.md) | ELP stereo camera notes and V4L2 formats. |
| [runtime.md](./docs/runtime.md) | Runtime warnings/errors description. |
| [troubleshooting.md](./docs/troubleshooting.md) | Common “if you see X, do Y” fixes (build + runtime). |

# Usage
Before first time usage, start with the [install guide](./docs/install.md).

Main Operation commands are listed below; for the full list see [commands](./docs/reference.md#commands).

| Command | Description |
|---|---|
| `sim` | PC Simulation: `sim` (default mapping) or `sim teleop|camera|mapping|nav|localization` → runs `rovi_bringup/rovi.launch.py robot_mode:=sim stack:=...` and starts Gazebo + RViz (use `rviz:=false` for headless). |
| `view` | PC view: `view` (default `nav`) or `view teleop|camera|mapping|nav`; `view offline` for local URDF inspection (no hardware). |
| `teleop` | Robot (Pi): runs `rovi_bringup/rovi.launch.py` with `robot_mode:=real stack:=teleop` (headless; no RViz). |
| `camera` | Robot (Pi): runs `rovi_bringup/rovi.launch.py` with `robot_mode:=real stack:=camera` (teleop + depth + RGB; headless; no RViz). |
| `nav` | Robot (Pi): runs `rovi_bringup/rovi.launch.py` with `robot_mode:=real stack:=nav` (headless; no RViz). |
