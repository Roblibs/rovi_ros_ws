# Improvement: Harden rosbag profile management

## Plan

### Problem
Bag profile behavior depends on session conventions and manual YAML maintenance.

### Rework
Validate that each known stack has a bag profile; warn/fail when stack/profile mismatch.

### Independent scope
- `src/rovi_bringup/rovi_bringup/cli/bag.py`
- `src/rovi_bringup/config/bag_topics.yaml`

### Value
- Complexity reduction: medium.
- Technical debt reduction: medium.

## Implementation details

### Contract-first bag topics
`rovi_bag` now uses `docs/contract.yaml` as the canonical *minimum* bag topic set per stack:
- Base topics come from `stacks.<stack>.required_topics` plus the union of `stacks.<stack>.required_topics_by_mode.*`.
- `bag_topics.yaml` becomes optional “delta” config layered on top.

### Delta profiles (backward compatible)
`src/rovi_bringup/config/bag_topics.yaml` supports:
- **Legacy override**: `stack: ["/topic_a", "/topic_b", ...]` (full list).
- **Delta mapping**: `stack: { add: [...], drop: [...] }` applied on top of the contract topics.

The file was migrated to delta form to reduce duplication and maintenance.

### Packaging and discovery
To make the contract discoverable even when `ROVI_ROS_WS_DIR` is not set, `docs/contract.yaml` is installed into the `rovi_bringup` share directory as `config/contract.yaml` during build.
