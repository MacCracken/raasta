# Raasta

**Raasta** (راستہ — Urdu/Hindi for "path/way/route") — navigation and pathfinding engine for the [AGNOS](https://github.com/MacCracken/agnosticos) ecosystem.

Built on [hisab](https://github.com/MacCracken/hisab) for math. Used by [kiran](https://github.com/MacCracken/kiran) (game engine), [joshua](https://github.com/MacCracken/joshua) (simulation), and [impetus](https://github.com/MacCracken/impetus) (physics).

## Features

### Grid Pathfinding
- **A\*** — standard optimal pathfinding with per-cell costs
- **Jump Point Search** — 35x faster on uniform-cost grids
- **Theta\*** — any-angle paths via LOS shortcuts
- **Lazy Theta\*** — deferred LOS checks, fewer line-of-sight tests
- **Bidirectional A\*** — dual-frontier search, fewer node expansions
- **Fringe search** — IDA\*-like cache-friendly algorithm
- **Weighted A\*** — bounded-suboptimal for faster results
- **D\* Lite** — incremental replanning for dynamic environments
- **Flow fields** — precomputed direction grids for crowd movement
- **Connected components** — instant unreachable-query rejection

### NavMesh
- **Baking** — ear-clipping triangulation + convex merge from polygon boundaries
- **Heightfield baking** — Recast-style pipeline from 3D triangle geometry
- **Collider generation** — NavMesh from physics collider shapes
- **A\* pathfinding** — with area costs, query filters, off-mesh links, obstacle carving
- **Tiled navmesh** — streaming, localized re-baking for open worlds
- **Multi-layer** — overlapping surfaces for bridges and multi-story buildings
- **Serialization** — compact binary save/load (RNAV format)
- **Navigation queries** — closest point, raycast, random point, height snap

### Steering & Agents
- **Core behaviors** — seek, flee, arrive, pursuit, evade, wander
- **Flocking** — separation, alignment, cohesion
- **Obstacle avoidance** — ray-cone detection with urgency scaling
- **Behavior blending** — weighted combination + priority-based selection
- **Formation movement** — line, wedge, circle, grid, custom slot patterns
- **Path following** — waypoint progression with arrive steering
- **Path corridor** — sliding window with local replanning

### Simulation
- **RVO/ORCA** — reciprocal collision avoidance with spatial hashing
- **Crowd simulation** — density-aware velocity damping
- **Influence maps** — 2D overlay grids for danger zones and strategic value

### 3D Navigation
- **NavMesh3D** — 3D polygon mesh with projected pathfinding
- **3D steering** — seek, flee, arrive in 3D space
- **Voxel navigation** — 26-connected A\* for flying/swimming agents
- **Height queries** — ground-snap (x,z) → y

### Performance
- **Incremental pathfinding** — time-sliced A\* across frames
- **Request batching** — priority queue with per-frame budget
- **Query objects** — pre-allocated scratch buffers, zero per-path allocation
- **SOA RVO** — struct-of-arrays layout with pre-allocated work buffers
- **Parallel pathfinding** — rayon-powered batch operations (feature-gated)
- **51 criterion benchmarks** with CSV history tracking

## Quick Start

```rust
use raasta::{NavGrid, GridPos};

let mut grid = NavGrid::new(20, 20, 1.0);
grid.set_walkable(10, 5, false); // wall

let path = grid.find_path(GridPos::new(0, 0), GridPos::new(19, 19));
assert!(path.is_some());
```

### NavMesh Pathfinding

```rust
use raasta::{NavMesh, Vec2};

let mesh = NavMesh::bake(&[
    Vec2::new(0.0, 0.0),
    Vec2::new(100.0, 0.0),
    Vec2::new(100.0, 100.0),
    Vec2::new(0.0, 100.0),
]);

let path = mesh.find_path(Vec2::new(5.0, 5.0), Vec2::new(95.0, 95.0));
```

### RVO Collision Avoidance

```rust
use raasta::{RvoSimulation, RvoAgent, Vec2};

let mut sim = RvoSimulation::new(2.0);
let a = sim.add_agent(RvoAgent::new(Vec2::ZERO, 0.5, 2.0));
sim.set_preferred_velocity(a, Vec2::new(1.0, 0.0));
sim.step(0.016);
```

## Feature Flags

| Flag | Description | Default |
|------|-------------|---------|
| `logging` | Structured tracing on all operations | off |
| `soorat-compat` | Visualization data for soorat renderer | off |
| `parallel` | Rayon-powered parallel pathfinding | off |

## Dependency Stack

```
raasta (navigation/pathfinding)
  └── hisab (math — Vec2, Vec3, geometry)
        └── glam (SIMD linear algebra)
```

## Documentation

- [Architecture Overview](docs/architecture/overview.md)
- [Roadmap](docs/development/roadmap.md)
- [Usage Guide](docs/guides/usage.md)
- [Testing Guide](docs/guides/testing.md)
- [Changelog](CHANGELOG.md)

## License

GPL-3.0-only
