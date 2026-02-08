# services (systemd + stack manager)

## update (2026-02-08)
- Add a dedicated always-on **gateway** launch (backend + UI bridge + manager).
- Split UI plane to **gateway only**; stacks assume gateway is already running and systemd enforces that.

## implementation status
- Current phase: **Phase 3** (`services/` artifacts + systemd wiring).
- Implemented:
  - UI bridge proto + config support for **text** status fields; `system/service` + `system/process` status sources.
  - Gateway split: `rovi_bringup/gateway.launch.py` and UI plane extracted from `robot_bringup.launch.py`.
  - In-process conductor module scaffold (`ros_ui_bridge.conductor`) for systemd probes.
- Not started yet: `services/` artifacts (unit files, `run_stack.sh`, install tooling, polkit) and the executor start/stop API.

## bring up phases
0) Decisions + plan (this document)
1) Gateway launch: add `rovi_bringup/gateway.launch.py` (backend + UI bridge + manager; robot-only `robot_mode:=real`)
2) Extract UI plane: remove UI bridge + display + manager from backend/stack launches; gateway is the only owner
3) Robot `services/`: add `rovi-*.service`, `run_stack.sh`, `install.sh`, `.env.robot`, `.env.pc`
4) Polkit allowlist + group: enable non-sudo stack start/stop
5) Executor API: UI bridge exposes start/stop stack units via systemd (polkit-backed; implemented via manager package)
6) Expand stacks + process list: teleop/mapping/localization/nav/camera + expected process names

## goal
Provide a two-level runtime management system for ROVI:
- Level 1 (**systemd**) owns process supervision, restart policy, dependencies, and **exclusivity** between stacks.
- Level 2 (**stack manager node/service**) exposes a safe control API (**start/stop stacks + status**) for the UI bridge/display without “full sudo”.

Constraint refinements:
- Exactly **one** permanently running service: `rovi-gateway.service` (enabled at boot).
- Managing **core** (enable/disable/start/stop) is admin-only; sudo is acceptable here.
- Non-sudo control is only required for **on-demand stacks** (start/stop).
- The manager is expected to run as part of **gateway**.
- Safe dev mode: on failure, **stop and surface logs** (no auto-restart loops).
- Observability is owned by `ros_ui_bridge`; execution is decoupled (the stack executor only starts/stops stacks and reports systemd state).
- Robot services are **robot-only** and assume `robot_mode:=real` (no backend switching via services).

Non-goals:
- Do not require ROS 2 lifecycle support in third-party packages.
- Do not embed “supervisor” logic inside every launch file.

## terminology
- **core**: always-on baseline (drivers/TF/clock/UI bridge, etc.) that other stacks depend on.
- **stack**: an on-demand launch group (teleop, mapping, localization, nav, camera, …).
- **manager**: the proxy/delegate that receives UI commands and talks to systemd; also reports status.
- **executor**: the minimal “execution” part of the manager (start/stop stacks; read systemd state).
- **observer**: the observability part (ROS graph + UI topics + health probes), implemented in `ros_ui_bridge`.

## gateway launch split (required)
We need an explicit always-on launch that is *not* a stack, and stacks must not start the UI plane.

Requirements:
- `ros_ui_bridge` and the manager/executor are assumed to be **always running**.
- Stack services must **hard fail** when gateway is not running (enforce via systemd `Requires=` / `After=`).
- UI/manager logic must be extracted from backend/stack launches; gateway becomes the only owner.

Proposed new launch entrypoint:
- `rovi_bringup/gateway.launch.py`:
  - starts the robot backend contract (`robot_bringup.launch.py`) **backend only**
  - starts `ros_ui_bridge` **always on**
  - starts the manager/executor (in-process module `ros_ui_bridge.conductor`) **always on**
  - optionally starts `robot_serial_display` **always on (robot-only)**

## proposed architecture

### level 1: systemd services (source of truth)
Recommended service split:
- `rovi-gateway.service` (enabled at boot)
  - Starts the minimal robot backend + UI bridge path.
  - Owns hardware connections / sim bridge / shared infra.
  - Runs/includes the stack executor endpoint (start/stop stacks) and `ros_ui_bridge` (observability + UI stream).
- One service per on-demand stack (started/stopped by the manager):
  - `rovi-teleop.service`
  - `rovi-mapping.service`
  - `rovi-localization.service`
  - `rovi-nav.service`
  - `rovi-camera.service` (robot-side camera pipeline needed for UI/recording)

Not on the robot:
- No `rovi-view.service` (RViz/view is managed on the PC).

Dependencies:
- Each on-demand stack unit declares:
  - `Requires=rovi-gateway.service`
  - `After=rovi-gateway.service`
  - `PartOf=rovi-gateway.service` (stacks stop when gateway stops)

Restart policy (safe dev mode):
- Core: no auto-restart (fail fast; inspect logs; avoid hardware restart loops)
- Stacks: no auto-restart (operator decides next action)

Implementation detail:
- Omit `Restart=` (systemd default is no restart), or set `Restart=no` explicitly for clarity.

Shutdown semantics:
- Use `KillSignal=SIGINT` and sane `TimeoutStopSec=` so ROS nodes shut down cleanly.

#### unit file skeleton (example)
This is a sketch to standardize conventions (final values TBD):
```ini
[Unit]
Description=ROVI stack: nav
Requires=rovi-gateway.service
After=rovi-gateway.service
PartOf=rovi-gateway.service

[Service]
Type=simple
User=rovi
WorkingDirectory=%h/dev/Roblibs/rovi_ros_ws
EnvironmentFile=%h/dev/Roblibs/rovi_ros_ws/.env.robot
RuntimeDirectory=rovi
ExecStart=%h/dev/Roblibs/rovi_ros_ws/services/run_stack.sh nav
KillSignal=SIGINT
TimeoutStopSec=20
Restart=no

[Install]
WantedBy=multi-user.target
```

Env file sketch (`.env.robot` in repo; committed, no secrets):
```bash
ROVI_ROS_WS_DIR=/home/rovi/dev/Roblibs/rovi_ros_ws
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ROVI_ENV=robot
```

Notes:
- `run_stack.sh` should be responsible for sourcing ROS + workspace setup and applying exclusivity via `flock` (so the unit stays single-line).
- Hardcode the workspace path for systemd units (keep it predictable for boot).

### level 2: stack manager (UI control + policy)
Run a single always-on execution endpoint (“executor”) as part of gateway. Keep observability in `ros_ui_bridge`.

Responsibilities:
- Executor: accept commands (operator/UI scope): `start(stack)`, `stop(stack)`, `get_systemd_status()`.
- UI bridge: collect and publish observability (systemd + ROS) for the UI stream.
- Keep admin actions out of the non-sudo path (gateway enable/disable/start/stop; optional stack enable/disable at boot).
- Enforce intent-level policy (beyond systemd):
  - “one active stack at a time” UX (even if ROS could technically run in parallel)
  - “gateway must be up before any stack”
  - optional: “don’t allow stack switch while robot moving”
- Map systemd + process presence into UI status fields; topic freshness remains a separate UI concept.

Status model (suggested):
- `STOPPED` (inactive)
- `STARTING` (activating)
- `RUNNING` (active)
- `STOPPING` (deactivating)
- `FAILED` (failed)

Checks:
- systemd-level: unit `ActiveState/SubState` + exit status (authoritative for “process running”)
- process-level: expected process signatures present in the unit cgroup (coarse boolean; avoids node-derived status)

## exclusivity (no parallel stacks)
Systemd should be the final enforcement point.

### hard lock via `flock`
Wrap each *exclusive* stack’s `ExecStart` with a shared lock file:
- lock: `/run/rovi/stack.lock`
- effect: systemd can *try* to start two stacks, but only one will acquire the lock and run.

Benefits:
- Scales with number of stacks (no N×N `Conflicts=` matrix).
- Survives “out-of-band” starts (operator runs `systemctl start ...` manually).

Design detail:
- Put lock file under a systemd-managed runtime dir (`RuntimeDirectory=rovi`).
- All stacks are exclusive by design; use a single lock for all stack units.

## privilege model (no full sudo)
We need a safe way for the manager (running as an unprivileged user, inside gateway) to start/stop only the **stack** units.

Explicitly not required:
- Non-sudo control of `rovi-gateway.service` (admin manages gateway with sudo).
- Non-sudo enable/disable of units (keep this as admin/install tooling).

### polkit allowlist for stack start/stop
Use systemd’s D-Bus API and a polkit rule that allows:
- a dedicated group (e.g. `rovi-ops`)
- to `StartUnit/StopUnit` **only** for stack unit names (e.g. `rovi-teleop.service`, `rovi-nav.service`, …) and explicitly **not** `rovi-gateway.service`

Pros:
- No passwordless sudo.
- Least privilege; auditable.
Cons:
- Requires installing a polkit rule under `/etc/polkit-1/rules.d/`.

Polkit rule sketch (exact API calls and constraints TBD):
- Allow only `StartUnit`, `StopUnit` (and optionally `RestartUnit`)
- Allow only unit names with prefix `rovi-`
- Allow only members of group `rovi-ops`

Note:
- Keep “enable/disable at boot” (`EnableUnitFiles/DisableUnitFiles`) restricted to admin/install tooling; operator/UI control is start/stop only.
- Keep things simple for now (local network): no additional auth beyond “local client can reach UI bridge” + polkit group membership.

## file layout in-repo (proposed)
Keep service artifacts at repo root so they are not “hidden” under a ROS package.

Example:
- `services/rovi-gateway.service`
- `services/rovi-teleop.service`
- `services/rovi-mapping.service`
- `services/rovi-localization.service`
- `services/rovi-nav.service`
- `services/rovi-camera.service`
- `services/run_stack.sh`
- `services/install.sh`
- `${ROVI_ROS_WS_DIR}/.env.robot` (committed)
- `${ROVI_ROS_WS_DIR}/.env.pc` (committed)

Install script (`install.sh`) responsibilities:
- Copy unit files into `/etc/systemd/system/`.
- Validate a unified environment source exists (see “environment unification” below) and fail with a clear message if missing.
- Install the polkit rule for stack start/stop (and create the `rovi-ops` group).
- `systemctl daemon-reload`
- `systemctl enable --now rovi-gateway.service`

Notes:
- Installation should be idempotent (safe to re-run).
- Keep system modifications out of `colcon build` and out of package install hooks.

## launch composition guidance
Keep the “golden rule” parity (real/sim/offline) by ensuring the service split doesn’t change ROS contracts.

Suggested ownership:
- `rovi-gateway.service` runs `rovi_bringup/gateway.launch.py` (backend + UI bridge + manager + display client).
- Stack services run via `rovi.launch.py` using `stack:=...` (unified stack contract).

Important:
- `rovi.launch.py` supports `gateway_enabled:=false` for systemd stack units so stacks can start without starting the gateway plane.

Odometry TF ownership (real robot):
- The gateway must be configured to avoid TF conflicts with EKF stacks.
- For mapping/localization/nav (EKF publishes `odom->base_footprint`), configure the gateway service with:
  - `ROVI_ODOM_INTEGRATOR_PUBLISH_TF=0` and restart `rovi-gateway.service`
- For teleop (raw odom integrator publishes `odom->base_footprint`), use:
  - `ROVI_ODOM_INTEGRATOR_PUBLISH_TF=1`

This is intentionally a **restart-time** choice for systemd: switching TF publishers at runtime is out of scope.

### stack catalog (argument → launch)
`run_stack.sh` should accept a single stack argument and map it to a `ros2 launch`:
- `teleop` → `ros2 launch rovi_bringup rovi.launch.py robot_mode:=real gateway_enabled:=false stack:=teleop rviz:=false`
- `mapping` → `ros2 launch rovi_bringup rovi.launch.py robot_mode:=real gateway_enabled:=false stack:=mapping rviz:=false`
- `localization` → `ros2 launch rovi_bringup rovi.launch.py robot_mode:=real gateway_enabled:=false stack:=localization rviz:=false`
- `nav` → `ros2 launch rovi_bringup rovi.launch.py robot_mode:=real gateway_enabled:=false stack:=nav rviz:=false`
- `camera` → `ros2 launch rovi_bringup rovi.launch.py robot_mode:=real gateway_enabled:=false stack:=camera rviz:=false`

`rovi-<stack>.service` should pass `<stack>` to `run_stack.sh`.

Session context (for UI bridge fixed-frame policy):
- Keep `~/.ros/rovi/session/current_launch` up to date so `ros_ui_bridge` can resolve `stack` and `fixed_frame`.
- Preferred: `run_stack.sh` writes `rovi_bringup/<stack>.launch.py` after acquiring the stack lock and before starting the launch, and clears it on exit.

## UI status fields (flat + text support)
Keep the UI config simple and flat: represent service and process state as fields.

Config concept:
- Stay in `streams.status.fields` (flat list).
- Use `provider: system` with `type: service` or `type: process`.
- Service names are configured without the `.service` suffix.
- Fields default to float; use `type: text` only for text fields.

Example:
```yaml
streams:
  status:
    fields:
      - id: "svc_gateway"
        type: "text"
        source: { provider: "system", type: "service", service: "rovi-gateway" }

      - id: "svc_nav"
        type: "text"
        source: { provider: "system", type: "service", service: "rovi-nav" }

      - id: "proc_nav_controller"
        type: "text"
        source:
          provider: "system"
          type: "process"
          process: "controller_server"   # matching strategy is an implementation detail
          service: "rovi-nav"            # optional scope hint
```

Encoding on the wire:
- Float fields: fill `StatusFieldValue.value` (existing behavior).
- Text fields: set field meta `type=text` and fill `StatusFieldValue.text` with a short string (e.g. `active`, `inactive`, `failed`, `running`, `missing`).

Implementation notes:
- Proto:
  - Add optional `StatusFieldMeta.type` (defaults to float when omitted).
  - Add optional `StatusFieldValue.text` for text-typed fields.
- Config loader:
  - For float fields, keep current schema (`unit` required, optional `min/max/target`).
  - For text fields (`type: text`), omit `unit/min/max/target`.

## environment unification (interactive shell vs systemd)
Goal: avoid divergence between:
- developer/operator interactive shell (`~/.bashrc` sourcing `rovi_env.sh`)
- systemd services (non-interactive)

Proposed concept:
- Keep environment *values* in a repo-local env file (committed), and keep `rovi_env.sh` as the single loader.
- Use per-host/per-role env files to separate robot vs PC:
  - `${ROVI_ROS_WS_DIR}/.env.robot` (robot defaults: domain id, robot_mode, device paths)
  - `${ROVI_ROS_WS_DIR}/.env.pc` (PC defaults: visualization-only, sim isolation settings)
- `rovi_env.sh` loads `.env.${ROVI_ENV}` (where `ROVI_ENV=robot|pc`), and exports variables.
- systemd units use `EnvironmentFile=` pointing to the same `.env.robot` (and set `ROVI_ENV=robot` if needed).

This keeps “one place” for variables while allowing robot/PC divergence without baking it into `~/.bashrc`.
`~/.bashrc` should only define:
- `ROVI_ROS_WS_DIR` (workspace location)
- `ROVI_ENV` (`robot` or `pc`)
and source `rovi_env.sh` (avoid defining “real” variables in `.bashrc`).

## simulation on PC vs real robot (topic conflicts)
ROS 2 discovery is typically networked DDS; simulation on a PC is **not** automatically “localhost only”.

To avoid the PC accidentally joining the robot graph (and vice versa), use one of:
- Different `ROS_DOMAIN_ID` for robot vs PC sim (recommended default separation).
- Or set `ROS_LOCALHOST_ONLY=1` on the PC sim environment to keep it strictly local.

This should be documented as part of `.env.pc` and `.env.robot` defaults.

## “standard” node status
Most ROS nodes do not publish a universal “status”. Avoid node-derived state and rely on process/service presence only:
- **Service status**: systemd unit state (`active`, `failed`, `inactive`) per stack.
- **Process presence**: expected process signatures present in the unit cgroup (boolean).

## manager package naming (proposal)
We want a dedicated package for the stack execution/control plane (systemd D-Bus + policy), separate from `ros_ui_bridge` (observer/transport).

Candidate names:
- (Chosen) `ros_ui_bridge.conductor`: an in-process module used only by `ros_ui_bridge`.

Initial scope:
- Python library: systemd unit start/stop/query (D-Bus).
- In-process “executor” logic invoked by `ros_ui_bridge` (started by `gateway.launch.py`).
- UI bridge calls into this package to execute actions; polkit remains the privilege boundary.
