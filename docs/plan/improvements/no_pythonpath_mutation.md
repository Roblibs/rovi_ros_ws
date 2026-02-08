# Improvement: Remove launch-time PYTHONPATH mutation where possible

## Problem
Bringup mutates `PYTHONPATH` for venv fallback, which can produce environment-specific behavior.

## Rework
Package Python deps and startup paths so runtime works without launch-time environment surgery.

## Independent scope
- `src/rovi_bringup/launch/robot_bringup.launch.py`
- Packaging/install docs (as needed)

## Value
- Complexity reduction: medium.
- Technical debt reduction: medium-high (fewer environment-specific failures).
