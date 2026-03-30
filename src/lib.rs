//! # Raasta
//!
//! **Raasta** (راستہ — Urdu/Hindi for "path/way/route") — navigation and
//! pathfinding engine for the AGNOS ecosystem.
//!
//! Provides navmesh generation, A* pathfinding, grid-based search, flow fields,
//! path smoothing, and steering behaviors. Built on [`hisab`] for math.
//!
//! ## Features
//!
//! - **NavMesh** — polygon-based navigation mesh with connectivity graph
//! - **A\* pathfinding** — optimal shortest-path on navmesh or grid
//! - **Grid pathfinding** — 2D grid with walkability and costs
//! - **Flow fields** — precomputed direction fields for crowd movement
//! - **Path smoothing** — funnel algorithm for natural-looking paths
//! - **Steering behaviors** — seek, flee, arrive, obstacle avoidance
//!
//! ## Example
//!
//! ```rust
//! use raasta::{NavGrid, GridPos};
//!
//! let mut grid = NavGrid::new(10, 10, 1.0);
//! grid.set_walkable(5, 3, false); // place a wall
//!
//! let path = grid.find_path(GridPos::new(0, 0), GridPos::new(9, 9));
//! assert!(path.is_some());
//! ```

mod agent;
mod batch;
mod blend;
/// Cross-crate bridges — primitive-value conversions from other AGNOS science crates.
pub mod bridge;
mod corridor;
mod crowd;
mod debug_draw;
mod dstar;
mod error;
mod follow;
mod formation;
mod grid;
mod heightfield;
mod hpa;
mod incremental;
mod influence;
/// Integration APIs for downstream consumers (soorat rendering).
pub mod integration;
mod mesh;
mod mesh3d;
mod multilayer;
mod offmesh;
mod path;
mod query;
mod rvo;
mod smooth;
mod steer;
mod steer3d;
mod tiled;
mod triangulate;
mod voxel;

#[cfg(feature = "logging")]
pub mod logging;

// Re-export hisab math types used in our public API
pub use hisab::{Vec2, Vec3};

pub use agent::Agent;
pub use batch::{BatchedResult, PathBatcher, PathRequestId, QueuedRequest, RequestPriority};
pub use blend::{PrioritizedSteer, WeightedSteer, blend_priority, blend_weighted};
pub use corridor::PathCorridor;
pub use crowd::CrowdSimulation;
pub use debug_draw::{DebugDraw, DebugLine, DebugPoint};
pub use dstar::DStarLite;
pub use error::NavError;
pub use follow::PathFollower;
pub use formation::{Formation, FormationShape};
pub use grid::{GridPos, NavGrid};
pub use heightfield::{
    HeightSpan, Heightfield, HeightfieldConfig, bake_from_heightfield, bake_navmesh_from_geometry,
};
pub use hpa::{AbstractGraph, AbstractNodeId, ClusterId, Entrance, GridClusters};
pub use incremental::{IncrementalGridPath, IncrementalStatus};
pub use influence::InfluenceMap;
pub use mesh::{
    AreaCostMultiplier, CarveShape, CarvedObstacle, CarvedObstacleId, NavMesh, NavPoly, NavPolyId,
    NavQueryFilter, ObstacleCarver, erode_navmesh,
};
pub use mesh3d::{NavMesh3D, NavPoly3D};
pub use multilayer::{LayerConnection, LayerId, LayeredPolyId, MultiLayerNavMesh};
pub use offmesh::{LinkType, OffMeshLink, OffMeshLinkDesc, OffMeshLinkId, OffMeshLinkRegistry};
pub use path::{PathRequest, PathResult, PathStatus};
pub use query::GridPathQuery;
pub use rvo::{HalfPlane, RvoAgent, RvoSimulation, compute_orca_half_plane, solve_velocity};
pub use smooth::{Portal, extract_portals, funnel_portals, funnel_smooth};
pub use steer::{
    Obstacle, SteerBehavior, SteerOutput, alignment, avoid_obstacles, cohesion, compute_steer,
    evade, pursuit, separation, wander,
};
pub use steer3d::{SteerBehavior3D, SteerOutput3D, compute_steer_3d};
pub use tiled::{GlobalPolyId, NavTile, TileCoord, TiledNavMesh};
pub use triangulate::{merge_convex, triangulate, triangulate_points};
pub use voxel::{NavVolume, VoxelPos};
