# Refactor: Split UI bridge into explicit data-plane and transport-plane responsibilities

## Problem
Single-process/single-executor bridge can accumulate mixed responsibilities (sampling, conversion, transport, session policy).

## Rework
Separate data collection/normalization from transport serving and isolate heavy map/image transformations from status/state paths.

## Independent scope
- `src/ros_ui_bridge/ros_ui_bridge/*`

## Value
- Complexity reduction: medium-high (clearer module boundaries).
- Technical debt reduction: high (better scalability and fault isolation).
