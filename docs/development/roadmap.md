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

## P0 — Core Features

- [ ] **NavMesh generation from geometry** — auto-generate navmesh from obstacle polygons
- [ ] **hisab Vec2/Vec3 integration** — use hisab types instead of `[f32; 2]` arrays
- [ ] **Agent component** — navigation agent with speed, radius, avoidance
- [ ] **Path following** — smooth path-following system with lookahead
- [ ] **Obstacle avoidance steering** — dynamic obstacle detection and avoidance

## P1 — Production Quality

- [ ] **Jump point search** — faster grid pathfinding for uniform-cost grids
- [ ] **Hierarchical pathfinding** — HPA* for large grids
- [ ] **NavMesh baking** — offline navmesh generation and serialization
- [ ] **Local avoidance (RVO/ORCA)** — crowd simulation without collisions
- [ ] **3D navigation** — NavMesh on 3D surfaces, multi-level buildings
- [ ] **Dynamic obstacle updates** — add/remove obstacles at runtime

## P2 — Advanced

- [ ] **Theta* / any-angle pathfinding** — smoother paths without post-processing
- [ ] **Crowd simulation** — large-scale agent movement with density awareness
- [ ] **Navigation queries** — closest point on navmesh, random reachable point
- [ ] **Debug visualization** — navmesh wireframe, path overlay, flow field arrows

## Dependency Map

```
raasta (navigation/pathfinding)
  └── hisab (math — Vec2, Vec3, geometry)
        └── glam (SIMD linear algebra)
```
