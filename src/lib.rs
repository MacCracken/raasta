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
mod follow;
mod grid;
mod hpa;
mod mesh;
mod mesh3d;
mod path;
mod rvo;
mod smooth;
mod steer;
mod steer3d;
mod triangulate;

#[cfg(feature = "logging")]
pub mod logging;

// Re-export hisab math types used in our public API
pub use hisab::{Vec2, Vec3};

pub use agent::Agent;
pub use follow::PathFollower;
pub use grid::{GridPos, NavGrid};
pub use hpa::{AbstractGraph, AbstractNodeId, ClusterId, Entrance, GridClusters};
pub use mesh::{NavMesh, NavPoly, NavPolyId};
pub use mesh3d::{NavMesh3D, NavPoly3D};
pub use path::{PathRequest, PathResult, PathStatus};
pub use rvo::{HalfPlane, RvoAgent, RvoSimulation, compute_orca_half_plane, solve_velocity};
pub use smooth::funnel_smooth;
pub use steer::{Obstacle, SteerBehavior, SteerOutput, avoid_obstacles, compute_steer};
pub use steer3d::{SteerBehavior3D, SteerOutput3D, compute_steer_3d};
pub use triangulate::{merge_convex, triangulate, triangulate_points};
