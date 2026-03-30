# 003 — Feature-Gated Parallel Pathfinding

## Status
Accepted

## Context
Parallel pathfinding via rayon provides significant speedups for batch queries, but adds a non-trivial dependency.

## Decision
Gate parallel pathfinding behind the `parallel` feature flag. The `raasta::parallel` module is only compiled when enabled.

## Consequences
- Zero cost for consumers who don't need parallelism
- rayon only pulled in when explicitly opted-in
- Parallel functions live in their own module (`raasta::parallel::*`)
- Tests run both with and without the feature
