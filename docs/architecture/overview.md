# Raasta Architecture

## Overview

Raasta provides navigation and pathfinding for the AGNOS game engine stack. It is a flat crate with feature-gated optional dependencies.

## Modules

| Module | Responsibility |
|--------|---------------|
| `grid` | 2D grid navigation, A* pathfinding, flow fields |
| `mesh` | Convex polygon navmesh, graph-based A* |
| `path` | Path request/result types |
| `smooth` | Funnel path smoothing |
| `steer` | Steering behaviors (seek, flee, arrive) |

## Consumers

- **kiran** — game engine uses raasta for NPC/entity navigation
- **joshua** — simulation uses raasta for agent pathfinding

## Dependencies

- `hisab` — math types (Vec2, Vec3, geometry)
- `serde` — serialization for all public types
- `tracing` — structured logging
