# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/).

## [0.1.0] — 2026-03-23

### Added

- Initial scaffold
- `NavGrid` — 2D grid with walkability and movement costs
- A* pathfinding on grids (cardinal + diagonal, no corner-cutting)
- `NavMesh` — convex polygon navigation mesh with graph-based A*
- `GridPos` — grid coordinate type with Manhattan and octile distance
- Flow field generation from any goal position
- Path smoothing via funnel algorithm
- Steering behaviors: seek, flee, arrive
- `PathRequest` / `PathResult` / `PathStatus` types
- Serde support for all public types
- Grid ↔ world coordinate conversion
- Optional logging via `RAASTA_LOG` (feature-gated)
- 55+ unit tests, 6 integration tests
- 10 criterion benchmarks (grid A*, flow field, navmesh, steering, batch)
- Example: `basic` — grid pathfinding demo
