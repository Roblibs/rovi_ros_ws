# Improvement: Improve observability for throttled forwarding paths

## Problem
Forwarder swallows callback exceptions and can hide failures.

## Rework
Add structured warning/error logging with bounded rate limiting; keep forwarder resilient but observable.

## Independent scope
- `src/ros_ui_bridge/ros_ui_bridge/throttled_forwarder.py`

## Value
- Complexity reduction: low-medium (debugging becomes linear).
- Technical debt reduction: medium.
