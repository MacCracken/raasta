# 001 — Flat Library Crate

## Status
Accepted

## Context
raasta could be organized as a workspace with separate crates for grid, mesh, steering, etc. This would allow consumers to depend only on what they need.

## Decision
Keep raasta as a single flat library crate. All modules are in `src/` at the top level.

## Consequences
- Simpler dependency management for consumers
- Single version to track
- All features available without multi-crate coordination
- Larger compile unit (mitigated by feature gates for optional deps)
