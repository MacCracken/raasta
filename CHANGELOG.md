# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/).

## [1.0.0] — 2026-03-29

### Added
- **partial path** — `NavGrid::find_path_partial()` and `NavMesh::find_path_partial()` return path to closest reachable point when goal is unreachable; `PathStatus::Partial` variant and `PathResult::partial()` constructor
- **area costs** — `NavPoly.cost` field for per-polygon traversal cost; `AreaCostMultiplier` for per-agent cost overrides; `NavMesh::find_path_with_costs()` with combined polygon + agent multipliers
- **spatial hashing** — `RvoSimulation::step()` now uses spatial hash grid for neighbor queries (O(n²) → O(n·k)); `RvoSimulation::with_neighbor_dist()` for custom interaction radius
- **behavior blending** — `blend_weighted()` for weighted steering combination; `blend_priority()` for priority-based selection with fallback; `WeightedSteer` and `PrioritizedSteer` types
- **off-mesh links** — `OffMeshLinkRegistry` for managing jump/ladder/teleport/door links; `NavMesh::find_path_with_links()` integrates links as A* edges; enable/disable support for dynamic doors
- **funnel (SSFA)** — `funnel_portals()` proper Simple Stupid Funnel Algorithm on portal edges with agent radius shrinking; `extract_portals()` extracts shared edges from polygon paths; `Portal` type
- **path corridor** — `PathCorridor` sliding window over navmesh polygon paths; `move_position()`, `trim_passed()`, `replan()`, `replan_local()` for local replanning; `smooth_path()` integration with funnel algorithm
- **incremental pathfinding** — `IncrementalGridPath` time-sliced A* with per-frame iteration budget; `step()` returns `IncrementalStatus` for cooperative scheduling
- **request batching** — `PathBatcher` queues path requests with `RequestPriority`; processes within per-frame budget using incremental A*; `max_active` concurrent query limit
- **bridge** — cross-crate primitive-value bridges returning raasta types: impetus (collider → `Obstacle`, velocity 3D → `Vec2`), jantu (group target → `Vec2` destination, flee point → `(Vec2, f32)` repulsion), pavan (wind → movement cost, slope → speed scale)
- **integration/soorat** — feature-gated `soorat-compat` module with visualization data structures and conversion functions: `NavMeshWireframe::from_navmesh()`, `PathVisualization::from_path_result()`, `FlowFieldVisualization::from_flow_field()`, `CrowdVisualization::from_crowd()`, `HpaOverlay::from_clusters()`
- **query filters** — `NavQueryFilter` with include-only and exclude lists; `NavMesh::find_path_filtered()` for per-agent polygon restrictions
- **random point** — `NavMesh::random_point()` and `NavMesh3D::random_point()` area-weighted sampling for wander targets and spawn points
- **height queries** — `NavMesh3D::sample_height()` ground-snap (x,z) → y; `snap_to_surface()` convenience method
- **serialization** — `NavMesh::to_bytes()`/`from_bytes()` and `NavMesh3D::to_bytes()`/`from_bytes()` compact binary format (RNAV/RNV3 magic, versioned)
- **lazy theta\*** — `NavGrid::find_path_lazy_theta()` deferred LOS checks for fewer line-of-sight tests than standard Theta*
- **query objects** — `GridPathQuery` reusable A* query with pre-allocated scratch buffers for zero per-path allocation
- **tiled navmesh** — `TiledNavMesh` with tile streaming (`load_tile`/`unload_tile`), cross-tile A* pathfinding, `rebuild_connections()` for border polygon matching
- **dynamic rebuild** — `TiledNavMesh::rebake_tile()` localized re-baking; `rebuild_tile_connections()` for single-tile connection updates
- **obstacle carving** — `ObstacleCarver` marks polygons blocked by circle/rect obstacles at runtime; `NavMesh::find_path_carved()` avoids blocked polygons
- **D\* Lite** — `DStarLite` incremental replanning: `update_cell()` + `compute_path()` for efficient dynamic environment changes
- **connected components** — `NavGrid::connected_components()` flood-fill component IDs for instant unreachable-query rejection
- **bidirectional A\*** — `NavGrid::find_path_bidirectional()` dual-frontier search expanding fewer nodes
- **fringe search** — `NavGrid::find_path_fringe()` IDA*-like cache-friendly pathfinding
- **weighted A\*** — `NavGrid::find_path_weighted()` bounded-suboptimal search with inflated heuristic
- **nav layers** — `NavPoly.layer` field + `NavMesh::find_path_on_layers()` per-polygon layer filtering
- **agent erosion** — `erode_navmesh()` shrinks polygons inward by agent radius for conservative paths
- **formation** — `Formation` with `FormationShape` (line, wedge, circle, grid, custom) + slot steering
- **influence maps** — `InfluenceMap` 2D overlay grid with stamp, decay, sample for danger/value annotations
- **multi-layer** — `MultiLayerNavMesh` overlapping navigation surfaces with cross-layer connections (stairs, elevators)
- **voxel nav** — `NavVolume` 3D voxel grid with 26-connected A* for flying/swimming agents
- **heightfield baking** — `Heightfield` voxelized 3D geometry representation; `bake_navmesh_from_geometry()` full pipeline (rasterize → walkability → clearance → erosion → region flood-fill → contour → convex hull → navmesh); `HeightfieldConfig` with slope, clearance, radius settings
- **tests** — 220+ new tests across all P3 features (494 unit + 6 integration + 2 doc = 502 total)

### Fixed
- **bridge** — `wind_to_movement_cost` had inverted physics: tailwind was increasing cost instead of decreasing it. Sign of dot-product contribution corrected; wind-speed normalization removed so 10 m/s headwind properly yields 2× cost

### Changed
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
