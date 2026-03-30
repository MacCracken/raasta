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

## P3 — Future (from gap analysis + external research)

### Tier 1 — High Impact (production-critical)
- [ ] Partial path return — when destination unreachable, return path to closest reachable point
- [ ] Path corridor — sliding window with local replanning on world changes
- [ ] Off-mesh links — jumps, ladders, teleporters, doors (custom traversal edges)
- [ ] NavMesh area costs — per-polygon traversal cost with per-agent multipliers
- [ ] Async / incremental pathfinding — time-sliced A* with per-frame budget
- [ ] Path request batching + priority queue — cap per-frame pathfinding cost
- [ ] Spatial hashing for RVO neighbor queries (O(n²) → O(n·k))
- [ ] Behavior blending / priority system for combining multiple steerings
- [ ] Proper funnel algorithm on portal edges (SSFA) with agent radius

### Tier 2 — Expected by serious consumers
- [ ] Tiled navmesh — streaming, localized re-baking for open worlds
- [ ] Dynamic navmesh rebuild — rebuild only affected tiles on geometry change
- [ ] NavMesh obstacle carving — subtract shapes from navmesh topology at runtime
- [ ] Navigation query filters — per-agent include/exclude flags without re-baking
- [ ] Random point on navmesh — for wander targets, spawn points
- [ ] Height/elevation queries — ground-snap (x,z) → y
- [ ] NavMesh serialization — save/load baked navmesh to avoid re-baking
- [ ] D* Lite (incremental replanning)
- [ ] Lazy Theta* (deferred LOS checks)
- [ ] Query object pattern (concurrent pathfinding)

### Tier 3 — Differentiators
- [ ] Multi-floor / multi-layer navmesh — overlapping layers for bridges, buildings
- [ ] Formation movement — slots, leader-follow, formation maintenance
- [ ] Influence maps / cost annotations — danger zones, strategic value overlays
- [ ] Pre-allocated A* scratch buffers / node pool (zero per-path allocation)
- [ ] Connected-component IDs — reject unreachable queries instantly
- [ ] Bidirectional A*
- [ ] Fringe search
- [ ] Goal bounding / differential heuristics
- [ ] Sliced/incremental pathfinding (spread across frames)
- [ ] Heightfield-based navmesh baking from 3D geometry
- [ ] Navigation layers/groups
- [ ] Agent radius erosion
- [ ] 3D navigation volumes (voxel nav)

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

## Dependency Map

```
raasta (navigation/pathfinding)
  └── hisab (math — Vec2, Vec3, geometry)
        └── glam (SIMD linear algebra)
```
