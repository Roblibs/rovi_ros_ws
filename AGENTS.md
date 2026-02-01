# AGENTS.md — rovi_ros_ws (ROVI / Room-View Bot)

## workflow traceability
Three types of documents shall be maintained at all times to keep track of past, present and future developmet :
- present : in `README.md` in any folder with hirerchical scope, always suggest changes to keep readme content up to date but keep it minimal without too much details.
- past : log all important impactful decisions in `gen.md` which should have the generation log and reasons why design decision was taken and changes made, these are important to know so that design does not keep being changed looping in circles and learn lessons to avoid in future concepts. Additions in `gen.md` are always on demand or subject to confirmations, can be suggested by an agent at the end of the change but never updated spontaneously.
- future is noted in `plan.md` which is an open log for ideas and handing over to future agents, there we can coordinate conflicts avoidance when designing something that it does not fundamentally contradict future plans otherwise it should be mentioned and revised. Changes in `plan.md` are always on demand.

## What this workspace is
ROS 2 Jazzy workspace for the ROVI robot with three runtime backends:
- `robot_mode:=real` (hardware)
- `robot_mode:=sim` (Gazebo Sim)
- `robot_mode:=offline` (model-only inspection)
The single high-level entrypoint is `rovi_bringup/rovi.launch.py`, which selects the stack and RViz policy; it includes `rovi_bringup/robot_bringup.launch.py` which selects the backend. :contentReference[oaicite:1]{index=1}

## Golden rule (don’t break this)
**Real and sim must look the same to the rest of the stack**: same `/cmd_vel` input contract, same key feedback topics, and a consistent TF tree so Nav2/SLAM/RViz behave identically. :contentReference[oaicite:2]{index=2}

## Quickstart (developer shell)
This repo expects a helper shell script:
- `rovi_env.sh` provides commands like `ws`, `build`, `sim`, `view`, `teleop`, `mapping`, `localization`, `nav`, `stop`, etc. :contentReference[oaicite:3]{index=3}
- Typical env setup exports `ROVI_ROS_WS_DIR` and sources `rovi_env.sh` from `~/.bashrc`. :contentReference[oaicite:4]{index=4}
- Python deps for the robot side are managed with `uv` and a `.venv` created by `uv sync`. :contentReference[oaicite:5]{index=5}

## Build / verify (minimum)
Prefer these in order when making changes:
1) `ws` (cd + setup; activates `.venv` if present) :contentReference[oaicite:6]{index=6}  
2) `build` (runs `colcon build`) :contentReference[oaicite:7]{index=7}  
3) Smoke run one of:
   - `view offline` (model only)
   - `sim` (simulation)
   - `view` / `nav` / `teleop` depending on target
If you add new dependencies, keep them consistent with the repo’s Jazzy apt-based approach. :contentReference[oaicite:8]{index=8}

## More details (docs)
This file is intentionally short and focuses on the "golden path". For deeper references, see:

- `docs/nodes.md`: commands, packages, launches, nodes, and key params (good as a quick index / source of truth).
- `docs/reference.md`: deeper reference notes (robot interface contract, joystick mapping, rosbags helpers, rosmaster driver details, sensor notes).
- `docs/stereo_elp.md`: stereo camera notes.

## Repo map (where to look first)
Core packages (owned here): :contentReference[oaicite:9]{index=9}
- `rovi_bringup`: top-level launch entrypoints and stack selection
- `rovi_sim`: Gazebo Sim backend (worlds/bridges/base/odom)
- `rovi_description`: URDF/meshes/RViz configs + static TF
- `rosmaster_driver`: hardware bridge (`/cmd_vel` → MCU; publishes raw sensors)
- `rovi_odom_integrator`: real-robot odom from `/vel_raw` → `/odom_raw` (+ TF)
- `rovi_localization`: IMU/EKF filtering → `/odometry/filtered` + TF
- `rovi_slam`: SLAM Toolbox integration (`map -> odom`)
- `rovi_nav`: Nav2 integration/config/launch
- `ros_ui_bridge`: status/metrics collector + streaming server
- `robot_serial_display`: consumes UI stream and drives the ESP32 display client

## Entry points you should prefer
When changing behavior, start from these:
- `rovi_bringup/rovi.launch.py` (single high-level entrypoint) :contentReference[oaicite:10]{index=10}
- `rovi_bringup/robot_bringup.launch.py` (backend selection) :contentReference[oaicite:11]{index=11}
- Stack launches included by the above:
  - `rovi_bringup/teleop.launch.py`
  - `rovi_bringup/mapping.launch.py`
  - `rovi_bringup/localization.launch.py`
  - `rovi_bringup/nav.launch.py` :contentReference[oaicite:12]{index=12}
- Pipeline launches included downstream:
  - `rovi_localization/launch/ekf.launch.py`
  - `rovi_slam/launch/slam_toolbox.launch.py`
  - `rovi_nav/launch/nav.launch.py`
  - `rovi_sim/launch/gazebo_sim.launch.py` :contentReference[oaicite:13]{index=13}

## Interface contract (topics / TF)
### Command path (authoritative)
- `/cmd_vel` (`geometry_msgs/Twist`) is the unified command interface for both real and sim. :contentReference[oaicite:14]{index=14}
- Teleop and Nav2 are muxed via `twist_mux`:
  - `/cmd_vel_joy`, `/cmd_vel_keyboard`, `/cmd_vel_nav` → `/cmd_vel` :contentReference[oaicite:15]{index=15}

### Key feedback topics (commonly used)
- `/scan` (`sensor_msgs/LaserScan`)
- `/imu/data_raw` (`sensor_msgs/Imu`)
- `/vel_raw` (`geometry_msgs/Twist`)
- `/odom_raw` (`nav_msgs/Odometry`)
- `/tf` (and `/clock` in sim) :contentReference[oaicite:16]{index=16}

### TF chain expectation
Nav/SLAM/RViz typically rely on:
`map -> odom -> base_footprint -> base_link -> laser_link` (+ imu link) :contentReference[oaicite:17]{index=17}

## Change policy
- Prefer **small, reviewable commits** over sweeping refactors.
- Before renaming/remapping topics/frames/launch args, search for consumers (Nav2 configs, RViz configs, UI bridge).
- Keep `robot_mode:=real|sim|offline` parity: if you add a feature to one backend, either add it to the other or explicitly gate it with a launch arg and document the difference.
- Don’t “fix” by editing generated artifacts under `install/` / build outputs; change source and rebuild.

## Diagrams (on-demand)
- Diagrams are **on-demand only**: suggest them during planning, and **get confirmation before adding/updating** any diagram in the repo.
- Preferred format is **Mermaid** (unless the user requests something else or there’s a clear reason to justify a different format).
- Keep a consistent Mermaid convention:
  - ROS nodes use **rounded rectangles**: `node_id("node_name")`
  - ROS topics use **cylinders** (database shape): `topic_id[("topic_name")]` (i.e. `topic_id[(...)]`)
  - Direction is explicit: `node --> topic` (publishes), `topic --> node` (subscribes)
- Quoting rule: if a label contains any special characters (e.g. `/`, `-`, `:`, `.`, `(`, `)`, `[`, `]`), **wrap the label in double quotes** (e.g. `topic_cmd_vel[("/cmd_vel")]`, `node_twist_mux("twist_mux")`).

### Clarify-first rule (important)
If there is any ambiguity, missing requirement, or open design choice:

- **Do not change code by default.** Ask for clarification first and propose 1–3 options.
- Only proceed once the intended behavior and constraints are confirmed.

## Where configuration usually lives
- Launch wiring & mode/stack selection: `rovi_bringup`
- Teleop config (joy mapping): referenced by bringup teleop launch :contentReference[oaicite:20]{index=20}
- Nav2 configs: `rovi_nav`
- Localization params: `rovi_localization`
- SLAM params: `rovi_slam`

## When you’re unsure: preferred workflow
1) Identify the entry launch / node that owns the behavior.
2) `rg` for the topic/frame/param name.
3) Make the smallest change that preserves contracts.
4) `build` + run a smoke launch (`view offline` or `sim`).

## UI bridge (concept + where to look)
The UI path is intentionally out-of-band from ROS visualization tools:
- `ros_ui_bridge` exposes low-rate status/metrics and robot state to UI clients via **gRPC** (not ROS topics).
- `robot_serial_display` consumes the gRPC stream and forwards selected fields over USB serial to the ESP32 display.
- The UI bridge node is started by `rovi_bringup/robot_bringup.launch.py` by default.

Where to learn more:
- UI bridge README: `src/ros_ui_bridge/README.md`
- UI bridge proto: `src/ros_ui_bridge/proto/ui_bridge.proto`
- UI bridge default config: `src/ros_ui_bridge/config/default.yaml` (installed into the ROS share directory on build)
- Display client README: `src/robot_serial_display/README.md`
