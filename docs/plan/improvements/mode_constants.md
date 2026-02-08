# Improvement: Replace stringly mode logic with validated constants/enums

## Problem
Mode checks are string expressions across launch conditions, easy to mistype and hard to validate.

## Rework
Introduce central constants plus a validation helper that fails fast on invalid mode values.

## Independent scope
- `src/rovi_bringup/launch/*.py`

## Value
- Complexity reduction: medium.
- Technical debt reduction: medium.
