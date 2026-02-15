# Implementation: Robot-only venv + gated launch-time `PYTHONPATH`

This summarizes what was implemented relative to `docs/implementations/2026-02/15-pythonpath-venv/plan.md`.

## Outcomes

- `robot_mode:=sim` and `robot_mode:=offline` no longer inherit any venv-driven `PYTHONPATH` changes from the robot backend bringup.
- `robot_mode:=real` continues to support “venv deps without manual activation” by optionally prepending the venv `site-packages` to `PYTHONPATH` at launch time.
- Developer workflow supports a venv-free default shell (`ws`). When `robot_mode:=real` runs, bringup automatically uses `ROVI_ROS_WS_DIR/.venv` when present.

## Code changes

### Gate venv `PYTHONPATH` injection to `robot_mode:=real`

- `src/rovi_bringup/launch/robot_bringup.launch.py`
  - Wrap the existing `SetEnvironmentVariable(PYTHONPATH=...)` logic in a `GroupAction(condition=is_real, ...)`.
  - Keep `PYTHONUNBUFFERED=1` for immediate logs, also gated to `robot_mode:=real`.
  - Continue to locate the venv via:
    - active `VIRTUAL_ENV`, else
    - `$ROVI_ROS_WS_DIR/.venv` fallback.

Behavior change:
- Previously: any stack that included `robot_bringup.launch.py` would mutate `PYTHONPATH` whenever a venv was discovered.
- Now: only `robot_mode:=real` mutates `PYTHONPATH`; sim/offline do not.

### Stop auto-activating venv for common `ws`

- `rovi_env.sh`
  - `ws` now does: `cd` + `setup` only.
  - Venv activation remains available via `activate` for ad-hoc debugging, but is not required for normal `robot_mode:=real` bringup.

## Docs updates

- `docs/reference.md`: documents `ws` (no venv) and the real-robot auto-venv behavior.
- `docs/install.md`: default build path is `ws` + `build`; venv is optional for `robot_mode:=real` (`uv sync`).
- `docs/troubleshooting.md`: build-time missing `lark` now points to installing `python3-lark` via apt.

## Notes / remaining gaps

- `src/rosmaster_driver/launch/rosmaster_driver.launch.py` still has its own venv/PYTHONPATH logic when launched directly; the gating in `robot_bringup.launch.py` reduces blast radius for normal stack entrypoints.
