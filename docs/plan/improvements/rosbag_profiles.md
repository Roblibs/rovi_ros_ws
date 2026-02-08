# Improvement: Harden rosbag profile management

## Problem
Bag profile behavior depends on session conventions and manual YAML maintenance.

## Rework
Validate that each known stack has a bag profile; warn/fail when stack/profile mismatch.

## Independent scope
- `src/rovi_bringup/rovi_bringup/cli/bag.py`
- `src/rovi_bringup/config/bag_topics.yaml`

## Value
- Complexity reduction: medium.
- Technical debt reduction: medium.
