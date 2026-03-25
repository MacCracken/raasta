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

## P3 — Future (from gap analysis)

### Tier 1 — High Impact
- [ ] Spatial hashing for RVO neighbor queries (O(n²) → O(n·k))
- [ ] Behavior blending / priority system for combining multiple steerings
- [ ] Proper funnel algorithm on portal edges (SSFA)
- [ ] NavMesh area costs (per-polygon traversal cost)
- [ ] Path corridor (sliding window with local replanning)

### Tier 2
- [ ] Off-mesh links (jumps, ladders, teleporters)
- [ ] D* Lite (incremental replanning)
- [ ] Lazy Theta* (deferred LOS checks)
- [ ] Query object pattern (concurrent pathfinding)
- [ ] Tiled navmesh (streaming, localized re-baking)
- [ ] Navigation query filters (per-agent traversal rules)
- [ ] NavMesh obstacle carving

### Tier 3
- [ ] Bidirectional A*
- [ ] Fringe search
- [ ] Pre-allocated A* scratch buffers
- [ ] Goal bounding / differential heuristics
- [ ] Sliced/incremental pathfinding (spread across frames)
- [ ] Heightfield-based navmesh baking from 3D geometry
- [ ] Navigation layers/groups
- [ ] Agent radius erosion
- [ ] 3D navigation volumes (voxel nav)

## Dependency Map

```
raasta (navigation/pathfinding)
  └── hisab (math — Vec2, Vec3, geometry)
        └── glam (SIMD linear algebra)
```
