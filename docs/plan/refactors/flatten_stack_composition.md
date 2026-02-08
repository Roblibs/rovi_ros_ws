# Refactor: Flatten stack composition (mapping/localization/nav)

## Problem
`nav` includes `mapping`/`localization` which include `ekf` + `slam`, creating nested orchestration and repeated parameter wiring.

## Rework
Introduce explicit reusable blocks (for example: `state_estimation.launch.py`, `slam_mode.launch.py`) and have `mapping`, `localization`, `nav` compose the same blocks.

## Independent scope
- `src/rovi_bringup/launch/nav.launch.py`
- `src/rovi_bringup/launch/mapping.launch.py`
- `src/rovi_bringup/launch/localization.launch.py`

## Value
- Complexity reduction: high (shallower launch graph and clearer ownership).
- Technical debt reduction: medium-high (less duplicated parameter plumbing).
