# Refactor: Formalize backend contract as a schema plus contract tests

## Problem
Contract parity (`/cmd_vel`, key feedback topics, TF chain) is policy-driven but not enforced automatically.

## Rework
Define machine-readable contract (`docs/contract.yaml` or equivalent) and add parity tests for `real|sim|offline` launch outputs.

## Independent scope
- New contract file
- Test utilities in `tools/` or package tests

## Value
- Complexity reduction: medium (faster reasoning and onboarding).
- Technical debt reduction: very high (prevents silent real/sim divergence).
