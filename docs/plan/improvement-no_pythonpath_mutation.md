# Improvement: Remove launch-time `PYTHONPATH` mutation where possible

This item has an implementation record and an updated plan under:
- `docs/implementations/2026-02/15-pythonpath-venv/plan.md`
- `docs/implementations/2026-02/15-pythonpath-venv/implementation.md`

Current direction: keep venv usage **robot-only** (`robot_mode:=real`) and keep sim/offline venv-free.
