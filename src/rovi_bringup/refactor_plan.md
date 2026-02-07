# rovi_bringup

## Purpose
- Own top-level launch composition and backend/stack selection.
- Keep real/sim/offline contracts stable for the rest of the system.

## Detailed Plan: Launch Orchestration Refactor

### Goal
- Remove duplicated launch argument wiring between `rovi.launch.py`, `robot_bringup.launch.py`, and stack launches.
- Keep behavior identical while reducing hidden coupling and breakage risk.

### Scope
- In scope:
  - `src/rovi_bringup/launch/rovi.launch.py`
  - `src/rovi_bringup/launch/robot_bringup.launch.py`
  - `src/rovi_bringup/launch/camera.launch.py`
  - shared launch helper module(s) under `src/rovi_bringup/launch/lib/`
- Out of scope:
  - topic/TF contract changes
  - camera algorithm changes
  - UI protocol changes

### Architecture Target
- `rovi.launch.py`:
  - orchestration only (mode, stack, RViz policy, session marker)
  - no stack-specific argument ownership
- `robot_bringup.launch.py`:
  - backend process ownership only
- stack launch files (`camera`, `mapping`, `localization`, `nav`):
  - own stack-local arguments and node params
- shared helpers:
  - typed/validated mode values
  - reusable include wiring and argument pass-through helpers

### Phase Plan
1. Phase A: Contract inventory and ownership freeze
- Create a single argument matrix:
  - argument name
  - owner launch file
  - consumers
  - default source
- Freeze current runtime behavior from logs before refactor.
- Exit criteria:
  - no ambiguous argument ownership remains.

2. Phase B: Shared launch helper extraction
- Add helper module(s), e.g.:
  - `launch/lib/modes.py`
  - `launch/lib/includes.py`
  - `launch/lib/args.py`
- Move repeated condition and include patterns into helpers.
- Exit criteria:
  - duplicated condition/include snippets removed from top-level files.

3. Phase C: `rovi.launch.py` slimming
- Keep only global orchestration arguments.
- Remove stack-local pass-through duplication where helpers can route cleanly.
- Preserve same external CLI behavior.
- Exit criteria:
  - top-level file is composition-focused and shorter.

4. Phase D: Backend launch hardening cleanup
- Keep one canonical executable-resolution strategy for python nodes.
- If packaging/toolchain is stabilized, remove temporary fallback logic and keep the simplest valid path.
- Exit criteria:
  - backend nodes start reliably after clean build.

5. Phase E: Documentation and handover
- Update this README with final ownership matrix summary.
- Keep root README unchanged except minimal link/update if needed.
- Exit criteria:
  - next agent can modify launches without rediscovering ownership rules.

### Validation Strategy (manual, no added test code)
- Use real runs + logs as requested.
- Validation matrix:
  - `camera color_mode:=yuyv`
  - `camera color_mode:=mjpeg`
  - `teleop`
  - `mapping`
  - `localization`
  - `nav`
  - `sim camera`
- For each run, verify:
  - launch starts expected nodes
  - expected key topics exist
  - no new launch-time exceptions
  - no contract drift on `/cmd_vel` and core TF chain

### Packaging/Toolchain Decision (important)
- This is not just bad luck.
- Current environment can fail builds due to setuptools/colcon compatibility (`setup.py develop --uninstall --editable` path).
- If ignored, refactor work can appear broken even when launch logic is correct.
- Recommended minimum action:
  - pin a known-good setuptools range in workspace tooling docs (or uv lock workflow)
  - treat this as a prerequisite stability item, not a feature item
- Can it be skipped:
  - yes, but with high risk of intermittent false failures and wasted debugging cycles.

### Packaging/Toolchain Compatibility Plan (detailed)
1. What failed and why
- `colcon` (via `colcon-python-setup-py`) calls `setup.py develop --uninstall --editable ...`.
- Newer `setuptools` versions can reject that invocation pattern.
- Result: Python ROS packages partially install, then launch breaks in non-obvious ways.

2. Upstream signal (where this comes from)
- `colcon-core` issue tracking setuptools v80 breakage:
  - `https://github.com/colcon/colcon-core/issues/696`
- `colcon-core` fix PR pins setuptools below 80:
  - `https://github.com/colcon/colcon-core/pull/699`
- PyPI metadata confirms dependency change:
  - `colcon-core 0.19.0`: `setuptools>=30.3.0`
  - `colcon-core 0.20.x`: `setuptools<80,>=30.3.0`
- Setuptools side context:
  - `https://github.com/pypa/setuptools/issues/4877`

3. How to check versions (local)
- Workspace venv:
  - `python -c "import setuptools, colcon_core; print(setuptools.__version__, colcon_core.__version__)"`
- Dependency metadata:
  - `python -c "import importlib.metadata as m; print(m.distribution('colcon-core').requires)"`
- Lock consistency:
  - inspect `uv.lock` entries for `setuptools`, `colcon-core`, `colcon-python-setup-py`.

4. What to freeze, where to freeze
- Freeze in workspace Python dependency policy (single source of truth):
  - `pyproject.toml` constraints
  - regenerated `uv.lock`
- Recommended stable policy for now:
  - `colcon-core >=0.20.0`
  - `setuptools <80`
- Keep ROS apt packages independent (Jazzy runtime deps), but ensure the workspace build venv is pinned.

5. Execution options
- Option A (minimum, fast):
  - Pin `setuptools<80` only.
  - Keep existing colcon versions.
  - Good for immediate stability; medium future drift risk.
- Option B (cleaner baseline, recommended):
  - Pin `colcon-core>=0.20.0` and `setuptools<80`.
  - Regenerate lock and treat as required baseline for all dev machines.
  - Better reproducibility and fewer hidden failures.

6. Can we ignore this?
- You can temporarily ignore it if everyone uses an already-working venv snapshot.
- For shared development and refactor work, ignoring it is high-risk:
  - false launch regressions
  - inconsistent installs
  - wasted debugging time on non-functional failures.
- Conclusion: do not treat this as optional for the refactor stream; treat it as prerequisite hygiene.

7. Guard rails without automated tests (your preference)
- Keep verification runbook-based:
  - fresh `uv sync`
  - `build`
  - run `teleop`, `camera color_mode:=yuyv`, `camera color_mode:=mjpeg`
  - confirm logs and process starts.
- This gives deterministic validation without adding test code to the repo.

### Current Position vs Final Fix
- Direction: forward on runtime reliability, partial backward on structural cleanliness.
- To fully close debt:
  - complete Phases A-E in one focused refactor branch
  - avoid adding more local pass-through patches before helper extraction
