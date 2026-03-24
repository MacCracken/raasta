# Raasta

**Raasta** (راستہ — Urdu/Hindi for "path/way/route") — navigation and pathfinding engine for the [AGNOS](https://github.com/MacCracken/agnosticos) ecosystem.

Built on [hisab](https://github.com/MacCracken/hisab) for math. Used by [kiran](https://github.com/MacCracken/kiran) (game engine) and [joshua](https://github.com/MacCracken/joshua) (simulation).

## Features

- **Grid pathfinding** — A* on 2D grids with walkability, movement costs, diagonal control
- **NavMesh** — convex polygon navigation mesh with A* graph search
- **Flow fields** — precomputed direction fields for crowd/swarm movement
- **Path smoothing** — funnel algorithm for natural-looking paths
- **Steering behaviors** — seek, flee, arrive with configurable parameters
- **Serde support** — all types serializable for scene save/load

## Quick Start

```rust
use raasta::{NavGrid, GridPos};

let mut grid = NavGrid::new(20, 20, 1.0);
grid.set_walkable(10, 5, false); // wall

let path = grid.find_path(GridPos::new(0, 0), GridPos::new(19, 19));
assert!(path.is_some());
```

## Feature Flags

| Flag | Description | Default |
|------|-------------|---------|
| `logging` | Enable `RAASTA_LOG` environment variable for tracing output | off |

## Architecture

```
raasta
├── grid.rs      — NavGrid, GridPos, A* grid pathfinding, flow fields
├── mesh.rs      — NavMesh, NavPoly, A* polygon-graph pathfinding
├── path.rs      — PathRequest, PathResult, PathStatus types
├── smooth.rs    — Funnel path smoothing
├── steer.rs     — Steering behaviors (seek, flee, arrive)
└── logging.rs   — Optional tracing init (feature-gated)
```

## Dependency Stack

```
raasta (navigation/pathfinding)
  └── hisab (math — Vec2, Vec3, geometry)
        └── glam (SIMD linear algebra)
```

## License

GPL-3.0
