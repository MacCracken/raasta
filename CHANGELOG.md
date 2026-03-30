# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/).

## [0.27.0]

### Added
- **bridge** — cross-crate primitive-value bridges for impetus (collider to obstacle, velocity 3D to 2D), jantu (group target to destination, flee point to repulsion), pavan (wind to movement cost, slope to speed scale)
- **integration/soorat** — feature-gated `soorat-compat` module with visualization data structures: `NavMeshWireframe` (edge pairs), `PathVisualization` (waypoints with cost), `FlowFieldVisualization` (direction grid), `CrowdVisualization` (agent positions/velocities/radii), `HpaOverlay` (cluster boundaries)

### Updated
- zerocopy 0.8.47 -> 0.8.48

## [0.26.3] — 2026-03-26

### Fixed

- **mesh** — `NavMesh::get_poly()` and `NavMesh3D::get_poly()` O(n) linear scan → O(1) index lookup
- **mesh** — `NavMesh` and `NavMesh3D` A* pathfinding: `HashMap`/`HashSet` → `Vec`-indexed arrays (eliminates hashing overhead)
- **grid** — JPS `jump()` converted from recursive to iterative (prevents stack overflow on large grids)
- **grid** — Theta\* now respects per-cell movement costs via line-cost accumulation along LOS shortcuts
- **triangulate** — `point_in_triangle` guarded against division by zero on degenerate triangles

### Added

- **tracing** — `#[instrument]` spans on all public pathfinding, baking, simulation, and query operations (gated behind `logging` feature)
- **serde** — `Serialize`/`Deserialize` on `SteerOutput`, `SteerOutput3D`, `HalfPlane`, `AbstractNodeId`, `Entrance`, `DebugLine`, `DebugPoint`, `DebugDraw`
- **tests** — 8 new tests: Theta\* cost awareness, flow field with costs, JPS large-grid maze stress test, RVO 20-agent simulation, serde roundtrips for new types
- **benchmarks** — 9 new benchmarks: Theta\* (open + obstacles), RVO step, crowd step, NavMesh bake (square + L-shape), triangulation, funnel smoothing

### Performance

- **navmesh_path_10_polys**: 1.67 µs → 381 ns (−77%)
- **navmesh_path_100_polys**: 21.1 µs → 2.52 µs (−88%)
- **navmesh_path_500_polys**: 193 µs → 11.5 µs (−94%)
- **navmesh_path_1000_polys**: 636 µs → 22.6 µs (−96%)
- **jps_50x50_obstacles**: 1.03 µs → 972 ns (−6%, iterative jump)

## [0.24.3] — 2026-03-24

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
