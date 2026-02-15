# Improvement: Remove launch-time `PYTHONPATH` mutation (keep venv robot-only)

## Problem
Bringup mutates `PYTHONPATH` to “help” venv-based Python dependencies, which can produce environment-specific behavior.

We want:
- **build + sim/offline** to work with system Python, without any venv involvement
- **robot_mode:=real** to keep using a venv *only* for robot-only Python deps (e.g., GitHub `rosmaster-lib`, serial/display tooling), without leaking that policy into sim/offline.

## Rework
1) Gate all venv/PYTHONPATH logic to `robot_mode:=real` only.
2) Keep build venv-free (system `/usr/bin/python3`).
3) Keep sim/offline venv-free; if a Python node needs non-ROS deps in sim/offline, it must be satisfied via system packages or vendored as a workspace package.

## Independent scope
- `src/rovi_bringup/launch/robot_bringup.launch.py` (currently injects venv site-packages)
- `src/rosmaster_driver/launch/rosmaster_driver.launch.py` (similar pattern)
- Dependency ownership:
  - `pyproject.toml` / `uv.lock` / `.venv` workflow (to be removed once no longer needed)
  - system deps docs (`docs/install.md`, `docs/troubleshooting.md`) as needed

## Value
- Complexity reduction: medium (sim/offline become predictable; venv scope limited to real robot).
- Technical debt reduction: medium-high (fewer environment-specific failures; fewer “it works on my shell” cases).

## Acceptance criteria
- `colcon build` succeeds in a fresh shell with no venv activated.
- `ros2 launch rovi_bringup rovi.launch.py robot_mode:=sim ...` works without venv activation.
- `ros2 launch rovi_bringup rovi.launch.py robot_mode:=offline ...` works without venv activation.
- `robot_mode:=real` works with either:
  - system deps installed, or
  - a workspace `.venv` present (created by `uv sync`) without requiring manual `source .venv/bin/activate`.

## Open points
- Which target OS(s) must be supported (robot image + WSL/Ubuntu version), to choose apt package names/versions.
- Whether `rosmaster-lib` should be vendored as a workspace package vs turned into a system package.
