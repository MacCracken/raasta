# Raasta Architecture

## Overview

Raasta provides navigation and pathfinding for the AGNOS ecosystem. It is a flat library crate with feature-gated optional dependencies.

## Modules

| Module | Responsibility |
|--------|---------------|
| `grid` | 2D grid navigation — A*, JPS, Theta*, Lazy Theta*, fringe, bidirectional, weighted A*, flow fields, connected components |
| `mesh` | Convex polygon navmesh — bake, A* pathfinding, area costs, query filters, obstacle carving, off-mesh links, serialization, erosion |
| `mesh3d` | 3D navmesh — projected point-in-polygon, A*, height queries, random point |
| `multilayer` | Multi-layer navmesh — overlapping surfaces with cross-layer connections |
| `tiled` | Tiled navmesh — streaming tiles, localized re-baking, cross-tile pathfinding |
| `voxel` | 3D navigation volumes — voxel grid with 26-connected A* for flying/swimming |
| `smooth` | Path smoothing — simple string-pulling + portal-based SSFA funnel with agent radius |
| `corridor` | Path corridor — sliding window over polygon paths with local replanning |
| `path` | Path request/result types, PathStatus |
| `steer` | 2D steering — seek, flee, arrive, pursuit, evade, wander, flocking, obstacle avoidance |
| `steer3d` | 3D steering behaviors |
| `blend` | Steering combination — weighted blending + priority-based selection |
| `formation` | Formation movement — line, wedge, circle, grid, custom slot patterns |
| `rvo` | RVO/ORCA — local collision avoidance with spatial hashing |
| `crowd` | Crowd simulation — density-aware movement on top of RVO |
| `agent` | Navigation agent — position, velocity, path following, obstacle avoidance |
| `follow` | Path follower — waypoint progression with seek/arrive |
| `hpa` | Hierarchical pathfinding — HPA* with cluster graph + cached intra-cluster paths |
| `incremental` | Time-sliced A* — spread pathfinding across frames |
| `batch` | Request batching — priority queue with per-frame budget |
| `dstar` | D* Lite — incremental replanning for dynamic environments |
| `query` | Reusable query objects — pre-allocated scratch buffers |
| `influence` | Influence maps — 2D overlay grids for danger zones, strategic value |
| `triangulate` | Ear-clipping triangulation + convex merge |
| `offmesh` | Off-mesh links — jumps, ladders, teleporters, doors |
| `debug_draw` | Debug visualization geometry output |
| `bridge` | Cross-crate bridges — impetus, jantu, pavan conversions |
| `integration` | Downstream consumer APIs (soorat rendering) |
| `error` | Error types |
| `logging` | Optional tracing setup |

## Consumers

- **kiran** — game engine (NPC navigation, crowd movement)
- **joshua** — simulation (agent pathfinding)
- **impetus** — physics (collision-aware steering)

## Dependencies

- `hisab` — math types (Vec2, Vec3, geometry)
- `serde` — serialization for all public types
- `tracing` — structured logging (feature-gated)
