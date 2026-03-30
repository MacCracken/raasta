# Usage Guide

## Grid Pathfinding

raasta provides multiple grid pathfinding algorithms, each suited to different scenarios:

| Algorithm | Method | Best For |
|-----------|--------|----------|
| A* | `find_path()` | General-purpose, weighted costs |
| JPS | `find_path_jps()` | Uniform-cost grids (35x faster) |
| Theta* | `find_path_theta()` | Smooth any-angle paths |
| Lazy Theta* | `find_path_lazy_theta()` | Any-angle with fewer LOS checks |
| Bidirectional | `find_path_bidirectional()` | Long paths, fewer expansions |
| Fringe | `find_path_fringe()` | Cache-friendly, large grids |
| Weighted | `find_path_weighted()` | Fast approximate paths |
| D* Lite | `DStarLite` | Dynamic environments |
| Partial | `find_path_partial()` | Closest reachable when goal blocked |

### Basic Grid Setup

```rust
use raasta::{NavGrid, GridPos};

let mut grid = NavGrid::new(100, 100, 1.0);

// Block areas
grid.block_rect(10, 10, 20, 20);
grid.block_circle(Vec2::new(50.0, 50.0), 5.0);

// Set movement costs (1.0 = normal, higher = slower)
grid.set_cost_rect(30, 30, 40, 40, 3.0); // swamp area

let path = grid.find_path(GridPos::new(0, 0), GridPos::new(99, 99));
```

### Incremental Pathfinding

For spreading pathfinding across frames:

```rust
use raasta::{IncrementalGridPath, IncrementalStatus};

let mut query = IncrementalGridPath::new(&grid, start, goal).unwrap();

// Each frame:
match query.step(&grid, 100) { // 100 node budget per frame
    IncrementalStatus::InProgress => {}, // continue next frame
    IncrementalStatus::Found => { let path = query.path(); }
    IncrementalStatus::NotFound => { /* no path */ }
}
```

## NavMesh

### Baking from Boundaries

```rust
use raasta::{NavMesh, Vec2};

let boundary = vec![
    Vec2::new(0.0, 0.0), Vec2::new(100.0, 0.0),
    Vec2::new(100.0, 100.0), Vec2::new(0.0, 100.0),
];
let mesh = NavMesh::bake(&boundary);
```

### From 3D Geometry (Heightfield)

```rust
use raasta::{bake_navmesh_from_geometry, HeightfieldConfig, Vec3};

let triangles = vec![
    [Vec3::new(0.0, 0.0, 0.0), Vec3::new(100.0, 0.0, 0.0), Vec3::new(100.0, 0.0, 100.0)],
    [Vec3::new(0.0, 0.0, 0.0), Vec3::new(100.0, 0.0, 100.0), Vec3::new(0.0, 0.0, 100.0)],
];
let config = HeightfieldConfig::default();
let mesh = bake_navmesh_from_geometry(&triangles, &config);
```

### From Colliders

```rust
use raasta::{navmesh_from_colliders_rect, ColliderShape, ColliderNavConfig, Vec2};

let colliders = vec![
    ColliderShape::Circle { center: Vec2::new(50.0, 50.0), radius: 10.0 },
    ColliderShape::Aabb { min: Vec2::new(20.0, 20.0), max: Vec2::new(30.0, 30.0) },
];
let mesh = navmesh_from_colliders_rect(
    Vec2::ZERO, Vec2::new(100.0, 100.0),
    &colliders, &ColliderNavConfig::default(),
);
```

## Steering Behaviors

```rust
use raasta::{compute_steer, SteerBehavior, Vec2, blend_weighted, WeightedSteer};

// Single behavior
let seek = compute_steer(
    &SteerBehavior::Seek { target: Vec2::new(100.0, 0.0) },
    Vec2::ZERO, 5.0,
);

// Blend multiple
let result = blend_weighted(&[
    WeightedSteer { output: seek_output, weight: 1.0 },
    WeightedSteer { output: avoid_output, weight: 2.0 },
], 5.0);
```

## Serialization

All types support serde. NavMesh also has compact binary format:

```rust
let bytes = mesh.to_bytes();
let loaded = NavMesh::from_bytes(&bytes).unwrap();
```
