# Raasta Roadmap

> **Raasta** (راستہ) — navigation and pathfinding engine for AGNOS

## Completed (0.1.0)

### V0.1 — Initial Scaffold
- NavGrid with A* pathfinding (cardinal + diagonal)
- NavMesh with polygon-graph A*
- Flow field generation
- Path smoothing (funnel)
- Steering behaviors (seek, flee, arrive)
- Serde support, logging, tests, benchmarks

## P0 — Core Features ✅

- [x] **NavMesh generation from geometry** — ear-clipping triangulation + convex merge + neighbor detection
- [x] **hisab Vec2/Vec3 integration** — all public APIs use hisab types
- [x] **Agent component** — navigation agent with speed, radius, avoidance
- [x] **Path following** — waypoint-based path following with seek/arrive
- [x] **Obstacle avoidance steering** — ray-cone obstacle detection and avoidance

## P1 — Production Quality ✅

- [x] **Jump point search** — 35x faster grid pathfinding for uniform-cost grids
- [x] **Hierarchical pathfinding** — HPA* with cluster graph + cached intra-cluster paths
- [x] **NavMesh baking** — `NavMesh::bake()` from polygon boundaries
- [x] **Local avoidance (RVO/ORCA)** — ORCA half-planes + LP solver + simulation
- [x] **3D navigation** — 3D steering behaviors + NavMesh3D
- [x] **Dynamic obstacle updates** — block/unblock rect/circle at runtime

## P2 — Advanced ✅

- [x] **Theta\* / any-angle pathfinding** — LOS-based parent shortcutting
- [x] **Crowd simulation** — density-aware movement on top of RVO
- [x] **Navigation queries** — closest point, raycast, nearest walkable
- [x] **Debug visualization** — geometry output for navmesh, paths, flow fields

## P3 — Advanced II (from gap analysis + external research)

### Tier 1 — High Impact (production-critical) ✅
- [x] Partial path return — when destination unreachable, return path to closest reachable point
- [x] Path corridor — sliding window with local replanning on world changes
- [x] Off-mesh links — jumps, ladders, teleporters, doors (custom traversal edges)
- [x] NavMesh area costs — per-polygon traversal cost with per-agent multipliers
- [x] Async / incremental pathfinding — time-sliced A* with per-frame budget
- [x] Path request batching + priority queue — cap per-frame pathfinding cost
- [x] Spatial hashing for RVO neighbor queries (O(n²) → O(n·k))
- [x] Behavior blending / priority system for combining multiple steerings
- [x] Proper funnel algorithm on portal edges (SSFA) with agent radius

### Tier 2 — Expected by serious consumers ✅
- [x] Tiled navmesh — streaming, localized re-baking for open worlds
- [x] Dynamic navmesh rebuild — rebuild only affected tiles on geometry change
- [x] NavMesh obstacle carving — subtract shapes from navmesh topology at runtime
- [x] Navigation query filters — per-agent include/exclude flags without re-baking
- [x] Random point on navmesh — for wander targets, spawn points
- [x] Height/elevation queries — ground-snap (x,z) → y
- [x] NavMesh serialization — save/load baked navmesh to avoid re-baking
- [x] D* Lite (incremental replanning)
- [x] Lazy Theta* (deferred LOS checks)
- [x] Query object pattern (concurrent pathfinding)

### Tier 3 — Differentiators ✅
- [x] Multi-floor / multi-layer navmesh — overlapping layers for bridges, buildings
- [x] Formation movement — slots, leader-follow, formation maintenance
- [x] Influence maps / cost annotations — danger zones, strategic value overlays
- [x] Pre-allocated A* scratch buffers / node pool (zero per-path allocation)
- [x] Connected-component IDs — reject unreachable queries instantly
- [x] Bidirectional A*
- [x] Fringe search
- [x] Weighted A* (focal search) — bounded suboptimality for faster pathfinding
- [x] Sliced/incremental pathfinding (spread across frames)
- [x] Heightfield-based navmesh baking from 3D geometry
- [x] Navigation layers/groups
- [x] Agent radius erosion
- [x] 3D navigation volumes (voxel nav)

## Cross-Crate Bridges ✅

- [x] `bridge.rs` module — primitive-value conversions for cross-crate navigation
- [x] **impetus bridge**: collider positions/radii → `Obstacle`; rigid body velocity → `Vec2`
- [x] **jantu bridge**: group target position → `Vec2` destination; flee point → `(Vec2, f32)` repulsion
- [x] **pavan bridge**: wind velocity → movement cost modifier; terrain slope → traversal speed scaling

## Soorat Integration ✅

- [x] `integration/soorat.rs` module — feature-gated `soorat-compat`
- [x] **NavMesh wireframe**: `NavMeshWireframe::from_navmesh()` — polygon edges for debug line rendering
- [x] **Path visualization**: `PathVisualization::from_path_result()` — waypoints with cumulative cost
- [x] **Flow field**: `FlowFieldVisualization::from_flow_field()` — normalized direction grid
- [x] **Crowd positions**: `CrowdVisualization::from_crowd()` — agent positions, velocities, radii
- [x] **HPA graph**: `HpaOverlay::from_clusters()` — cluster boundaries + connection edges

## v1.0 Status

Released 2026-03-29. All P0–P3 roadmap items complete (32/32). 502 tests, full clippy + fmt + doc clean.

## Future

- [x] Heightfield-based navmesh baking from 3D geometry (Recast-equivalent)
- [ ] SIMD-accelerated spatial hash / RVO
- [ ] Parallel pathfinding (rayon integration)
- [ ] NavMesh auto-generation from physics colliders
- [ ] Benchmark suite with history tracking

## Dependency Map

```
raasta (navigation/pathfinding)
  └── hisab (math — Vec2, Vec3, geometry)
        └── glam (SIMD linear algebra)
```
