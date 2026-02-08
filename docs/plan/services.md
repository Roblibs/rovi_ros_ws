# services (systemd + stack manager)

## goal
Provide a two-level runtime management system for ROVI:
- Level 1 (**systemd**) owns process supervision, restart policy, dependencies, and **exclusivity** between stacks.
- Level 2 (**stack manager node/service**) exposes a safe control API (start/stop/enable/disable + status) for the UI bridge/display without “full sudo”.

Non-goals:
- Do not require ROS 2 lifecycle support in third-party packages.
- Do not embed “supervisor” logic inside every launch file.

## terminology
- **core**: always-on baseline (drivers/TF/clock/UI bridge, etc.) that other stacks depend on.
- **stack**: an on-demand launch group (teleop, mapping, localization, nav, view, …).
- **manager**: the proxy/delegate that receives UI commands and talks to systemd; also reports status.

## proposed architecture

### level 1: systemd services (source of truth)
Recommended service split:
- `rovi-core.service` (enabled at boot)
  - Starts the minimal robot backend + UI bridge path.
  - Owns hardware connections / sim bridge / shared infra.
- One service per on-demand stack (started/stopped by the manager):
  - `rovi-teleop.service`
  - `rovi-mapping.service`
  - `rovi-localization.service`
  - `rovi-nav.service`
  - optional: `rovi-view.service` (RViz) and other operator tools

Dependencies:
- Each on-demand stack unit declares:
  - `Requires=rovi-core.service`
  - `After=rovi-core.service`
  - `PartOf=rovi-core.service` (stacks stop when core stops)

Restart policy:
- Core: `Restart=on-failure` (or `always` if we want “keep alive” semantics)
- Stacks: typically `Restart=on-failure` (avoid “fight the operator” on manual stop)

Shutdown semantics:
- Use `KillSignal=SIGINT` and sane `TimeoutStopSec=` so ROS nodes shut down cleanly.

#### unit file skeleton (example)
This is a sketch to standardize conventions (final values TBD):
```ini
[Unit]
Description=ROVI stack: nav
Requires=rovi-core.service
After=rovi-core.service
PartOf=rovi-core.service

[Service]
Type=simple
User=rovi
WorkingDirectory=%h/dev/Roblibs/rovi_ros_ws
EnvironmentFile=/etc/rovi/ros.env
RuntimeDirectory=rovi
ExecStart=/usr/bin/flock -n /run/rovi/stack_mode.lock /bin/bash -lc '\
  source /opt/ros/jazzy/setup.bash && \
  source ${ROVI_ROS_WS_DIR}/install/setup.bash && \
  ros2 launch rovi_bringup nav.launch.py'
KillSignal=SIGINT
TimeoutStopSec=20
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
```

Env file sketch (`/etc/rovi/ros.env`):
```bash
ROVI_ROS_WS_DIR=/home/rovi/dev/Roblibs/rovi_ros_ws
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### level 2: stack manager (UI control + policy)
Run a single always-on “manager” component (either inside `ros_ui_bridge` or as a sibling service).

Responsibilities:
- Accept commands: `start(stack)`, `stop(stack)`, `enable(stack)`, `disable(stack)`, `get_status()`.
- Enforce intent-level policy (beyond systemd):
  - “one active stack at a time” UX (even if ROS could technically run in parallel)
  - “core must be up before any stack”
  - optional: “don’t allow stack switch while robot moving”
- Map systemd + ROS health into a clean UI status model.

Status model (suggested):
- `STOPPED` (inactive)
- `STARTING` (activating)
- `RUNNING` (active)
- `STOPPING` (deactivating)
- `FAILED` (failed)
- optional: `DEGRADED` (active but missing required ROS signals)

Health checks (optional, layered):
- systemd-level: unit `ActiveState/SubState` + exit status (authoritative for “process running”)
- ROS-level: check required nodes/topics/TF/lifecycle states for “ready” (authoritative for “stack usable”)

## exclusivity (no parallel stacks)
Systemd should be the final enforcement point.

Two practical alternatives (can be combined):

### A) hard lock via `flock` (recommended)
Wrap each *exclusive* stack’s `ExecStart` with a shared lock file:
- lock: `/run/rovi/stack.lock`
- effect: systemd can *try* to start two stacks, but only one will acquire the lock and run.

Benefits:
- Scales with number of stacks (no N×N `Conflicts=` matrix).
- Survives “out-of-band” starts (operator runs `systemctl start ...` manually).

Design detail:
- Put lock file under a systemd-managed runtime dir (`RuntimeDirectory=rovi`).
- Use a dedicated lock class if we want multiple concurrency groups later:
  - `/run/rovi/stack_mode.lock` (teleop/mapping/localization/nav)
  - `/run/rovi/tools.lock` (rviz/view tools, allowed parallel)

### B) explicit `Conflicts=` graph
Declare pairwise conflicts between stacks (e.g. `rovi-nav.service` conflicts with `rovi-teleop.service`).

Benefits:
- “self-documenting” in unit metadata.
Downside:
- Grows quadratically as stacks increase; easy to drift.

## privilege model (no full sudo)
We need a safe way for the manager (running as an unprivileged user) to start/stop only the ROVI units.

### option 1 (recommended): polkit allowlist for specific units
Use systemd’s D-Bus API and a polkit rule that allows:
- a dedicated group (e.g. `rovi-ops`)
- to `StartUnit/StopUnit` **only** for unit names matching `rovi-*.service`

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
- “enable/disable at boot” maps to `EnableUnitFiles/DisableUnitFiles` and should likely be restricted to an admin mode (or done only by install tooling), even if start/stop is allowed for operators.

### option 2: systemd user services (+ lingering)
Run core + stacks as `systemctl --user` services, enable “linger” for the `rovi` user.

Pros:
- UI bridge/manager can control units without elevated privileges.
Cons:
- Device access / startup ordering sometimes easier with system services.
- Needs a one-time `loginctl enable-linger rovi` and udev permissions.

### option 3: privileged manager daemon + local IPC
Run a tiny root-owned manager service that controls systemd, and expose a restricted local socket to UI bridge.

Pros:
- No polkit complexity.
Cons:
- You own the security boundary and auth (group perms, message framing, hardening).

Decision point:
- Pick **Option 1** if we keep *system services* but want safe non-root control.
- Pick **Option 2** if we want simplest control path and can solve device perms cleanly.

## file layout in-repo (proposed)
Keep unit files close to the launch package that owns them (likely `rovi_bringup`).

Example:
- `src/rovi_bringup/services/systemd/rovi-core.service`
- `src/rovi_bringup/services/systemd/rovi-teleop.service`
- `src/rovi_bringup/services/systemd/rovi-mapping.service`
- `src/rovi_bringup/services/systemd/rovi-localization.service`
- `src/rovi_bringup/services/systemd/rovi-nav.service`
- `src/rovi_bringup/services/systemd/rovi-view.service` (optional)
- `src/rovi_bringup/services/install.sh`

Install script (`install.sh`) responsibilities:
- Copy unit files into `/etc/systemd/system/` (or user dir for option 2).
- Install a single env file (e.g. `/etc/rovi/ros.env`) referenced by all units:
  - `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, workspace path, robot mode defaults, etc.
- Install polkit rule (option 1) or enable linger (option 2).
- `systemctl daemon-reload`
- `systemctl enable --now rovi-core.service`

Notes:
- Installation should be idempotent (safe to re-run).
- Keep system modifications out of `colcon build` and out of package install hooks.

## launch composition guidance
Keep the “golden rule” parity (real/sim/offline) by ensuring the service split doesn’t change ROS contracts.

Suggested ownership:
- `rovi-core.service` runs `rovi_bringup/rovi.launch.py` (or `robot_bringup.launch.py`) with a “core-only” arg/profile.
- Stack services run stack-specific launches already present in `rovi_bringup`:
  - `teleop.launch.py`, `mapping.launch.py`, `localization.launch.py`, `nav.launch.py`

If core currently launches some of these stacks by default, add launch args to gate them so systemd becomes the top-level selector.

## implementation steps (phased)
1) Decide privilege option (polkit vs user services) and document chosen approach.
2) Define the stack inventory + concurrency groups (what is mutually exclusive).
3) Add `services/systemd/` unit files for `rovi-core` + **one** stack (pilot).
4) Add exclusivity enforcement (prefer `flock`) and validate start/stop race cases.
5) Implement the manager API surface (likely in `ros_ui_bridge` gRPC) and map systemd states.
6) Add ROS-level readiness probes for stacks where UI needs “ready vs merely running”.
7) Add install tooling (`install.sh`) and a minimal operator doc for `systemctl` usage.
8) Expand to all stacks; add explicit conflicts only where needed for clarity.

## open points / design decisions to settle
- **Service type**: system services (Option 1) vs user services (Option 2).
- **Stack exclusivity scope**:
  - Are `teleop` and `nav` mutually exclusive as “modes”, or can they run together with `twist_mux` priority?
  - Should `view` (RViz) be allowed concurrently with other stacks?
- **Backend switching**:
  - Is `robot_mode:=real|sim|offline` chosen at boot only, or can we switch live?
  - If switchable: do we model backend selection as separate services (`rovi-backend-real.service`, etc.)?
- **Readiness definition**:
  - What minimum ROS signals define “stack is ready” (topics, TF, lifecycle state)?
  - Do we block UI “RUNNING” until readiness passes, or show `DEGRADED`?
- **Failure policy**:
  - If a stack crashes, should systemd auto-restart it, or should it stay failed until operator action?
  - Do we auto-fallback to a safe mode (e.g. stop stacks, keep core)?
- **Security boundary**:
  - UI commands local-only (display on robot) vs remote network clients.
  - Authentication/authorization for remote requests (token, mTLS, allowed clients).
