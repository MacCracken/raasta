# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/).

## [Unreleased]

### Added

- **hisab Vec2/Vec3 integration** — all public APIs use `hisab::Vec2` / `Vec3` instead of `[f32; 2]`
- **Jump Point Search** — `find_path_jps()` on NavGrid, 35x faster than A* on obstacle-heavy grids
- **Theta\* any-angle pathfinding** — `find_path_theta()` with Bresenham LOS, smoother paths without post-processing
- **Hierarchical pathfinding (HPA\*)** — `GridClusters`, `AbstractGraph` with cached intra-cluster paths, 2x faster than A* on batch queries
- **NavMesh baking** — `NavMesh::bake()` from polygon boundaries via ear-clipping triangulation + convex merge + auto neighbor detection
- **Obstacle avoidance steering** — ray-cone `avoid_obstacles()` function
- **Pursuit / Evade / Wander** — predictive chase, flee, and random exploration steering
- **Flocking** — `separation()`, `alignment()`, `cohesion()` for Reynolds-style group behaviors
- **Path following** — `PathFollower` with waypoint progression, seek/arrive steering
- **Agent component** — `Agent` struct with position, velocity, path following, obstacle avoidance
- **RVO/ORCA local avoidance** — `RvoSimulation` with ORCA half-planes + 2D incremental LP solver
- **Crowd simulation** — `CrowdSimulation` with density-aware velocity damping on top of RVO
- **3D steering** — `SteerBehavior3D`, `compute_steer_3d()` for seek/flee/arrive in 3D
- **3D NavMesh** — `NavMesh3D` with projected point-in-polygon, 3D A* pathfinding
- **Navigation queries** — `NavMesh::closest_point()`, `raycast()`, `NavGrid::nearest_walkable()`
- **Dynamic obstacle updates** — `block_rect`, `unblock_rect`, `block_circle`, `unblock_circle`, `set_cost_rect`, `clear`
- **Debug visualization** — `DebugDraw` geometry output (navmesh wireframe, path overlay, flow field arrows, grid walkability)
- **Ear-clipping triangulation** — `triangulate()`, `triangulate_points()`, `merge_convex()`
- **Line of sight** — `NavGrid::has_line_of_sight()` via Bresenham's line

### Changed

- Bumped `hisab` dependency 0.22 → 0.24 (transforms feature only)
- Made `tracing` optional (gated behind `logging` feature)
- Removed unused `thiserror` dependency
- License identifier `GPL-3.0` → `GPL-3.0-only` (SPDX compliance)

### Performance

- **Bit-packed walkability grid** — `Vec<bool>` → `Vec<u64>` bitset (8x memory reduction)
- **Zero-alloc neighbor iteration** — stack-based `[(i32,i32,f32); 8]` buffer replaces `Vec` in A* inner loop
- **Closed-set optimization** — bitset-based closed set for grid A*, Dijkstra, and NavMesh A*
- **HPA\* path cache** — O(1) refinement via precomputed intra-cluster paths
- **Persistent crowd density** — reusable HashMap instead of per-frame allocation
- **Vec2 SIMD** — steering behaviors use glam Vec2 (sub-nanosecond per call)
- `#[inline]` on all hot-path functions, `#[must_use]` on all pure functions
- `index_unchecked()` with `debug_assert` for safe unchecked grid access

### Tests

- 242 tests (up from 55): 230 unit + 6 integration + 1 doc + 5 should_panic

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
