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

mod grid;
mod mesh;
mod path;
mod smooth;
mod steer;

#[cfg(feature = "logging")]
pub mod logging;

// Re-export hisab math types used in our public API
pub use hisab::Vec2;

pub use grid::{GridPos, NavGrid};
pub use mesh::{NavMesh, NavPoly, NavPolyId};
pub use path::{PathRequest, PathResult, PathStatus};
pub use smooth::funnel_smooth;
pub use steer::{SteerBehavior, SteerOutput, compute_steer};
