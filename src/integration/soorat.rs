//! Soorat integration — visualization data structures for navigation/pathfinding.
//!
//! Feature-gated behind `soorat-compat`. Provides structs that soorat can
//! consume directly for debug rendering, plus conversion functions from raasta
//! core types.

#[cfg(feature = "logging")]
use tracing::instrument;
use serde::{Deserialize, Serialize};

use crate::crowd::CrowdSimulation;
use crate::grid::NavGrid;
use crate::hpa::{ClusterId, GridClusters};
use crate::mesh::NavMesh;
use crate::path::PathResult;

/// NavMesh wireframe for debug line rendering.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NavMeshWireframe {
    /// Edge segments: `(start, end)` as `[x, y, z]` pairs.
    pub edges: Vec<([f32; 3], [f32; 3])>,
    /// Polygon count.
    pub poly_count: usize,
}

impl NavMeshWireframe {
    /// Build a wireframe from a [`NavMesh`].
    ///
    /// Vertices are projected to Y=0 in world space.
    #[must_use]
    #[cfg_attr(feature = "logging", tracing::instrument(skip_all))]
    pub fn from_navmesh(mesh: &NavMesh) -> Self {
        let mut edges = Vec::new();
        for poly in mesh.polys() {
            let verts = &poly.vertices;
            let n = verts.len();
            for i in 0..n {
                let a = verts[i];
                let b = verts[(i + 1) % n];
                edges.push(([a.x, 0.0, a.y], [b.x, 0.0, b.y]));
            }
        }
        Self {
            poly_count: mesh.poly_count(),
            edges,
        }
    }
}

/// Path visualization for colored line rendering.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PathVisualization {
    /// Waypoints `[x, y, z]`.
    pub waypoints: Vec<[f32; 3]>,
    /// Cost at each waypoint (for color mapping).
    pub costs: Vec<f32>,
    /// Total path cost.
    pub total_cost: f32,
}

impl PathVisualization {
    /// Build a path visualization from a [`PathResult`].
    ///
    /// Waypoints are projected to Y=0. Cost at each waypoint is the
    /// cumulative Euclidean distance from the start.
    #[must_use]
    #[cfg_attr(feature = "logging", tracing::instrument(skip_all))]
    pub fn from_path_result(result: &PathResult) -> Self {
        let waypoints: Vec<[f32; 3]> = result.waypoints.iter().map(|v| [v.x, 0.0, v.y]).collect();

        let mut costs = Vec::with_capacity(waypoints.len());
        let mut cumulative = 0.0f32;
        costs.push(0.0);
        for i in 1..result.waypoints.len() {
            let dx = result.waypoints[i].x - result.waypoints[i - 1].x;
            let dy = result.waypoints[i].y - result.waypoints[i - 1].y;
            cumulative += (dx * dx + dy * dy).sqrt();
            costs.push(cumulative);
        }
        let total_cost = cumulative;

        Self {
            waypoints,
            costs,
            total_cost,
        }
    }
}

/// Flow field for arrow rendering.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FlowFieldVisualization {
    /// Direction at each cell `[dx, dz]` (unit vectors).
    pub directions: Vec<[f32; 2]>,
    /// Grid dimensions (nx, nz).
    pub dimensions: [usize; 2],
    /// Cell size in metres.
    pub cell_size: f32,
    /// World-space origin `[x, z]`.
    pub origin: [f32; 2],
}

impl FlowFieldVisualization {
    /// Build a flow field visualization from a raw flow field and its source grid.
    ///
    /// `flow` is the direction array from [`NavGrid::flow_field`].
    #[must_use]
    #[cfg_attr(feature = "logging", tracing::instrument(skip_all))]
    pub fn from_flow_field(flow: &[(i32, i32)], grid: &NavGrid) -> Self {
        let directions: Vec<[f32; 2]> = flow
            .iter()
            .map(|&(dx, dy)| {
                let len = ((dx * dx + dy * dy) as f32).sqrt();
                if len > 0.0 {
                    [dx as f32 / len, dy as f32 / len]
                } else {
                    [0.0, 0.0]
                }
            })
            .collect();

        Self {
            directions,
            dimensions: [grid.width(), grid.height()],
            cell_size: grid.cell_size(),
            origin: [0.0, 0.0],
        }
    }
}

/// Crowd agent positions for instanced rendering.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CrowdVisualization {
    /// Agent positions `[x, y, z]`.
    pub positions: Vec<[f32; 3]>,
    /// Agent velocities `[vx, vz]`.
    pub velocities: Vec<[f32; 2]>,
    /// Agent radii.
    pub radii: Vec<f32>,
}

impl CrowdVisualization {
    /// Build a crowd visualization from a [`CrowdSimulation`].
    #[must_use]
    #[cfg_attr(feature = "logging", tracing::instrument(skip_all))]
    pub fn from_crowd(crowd: &CrowdSimulation) -> Self {
        let count = crowd.agent_count();
        let mut positions = Vec::with_capacity(count);
        let mut velocities = Vec::with_capacity(count);
        let mut radii = Vec::with_capacity(count);

        for i in 0..count {
            let agent = crowd.agent(i);
            positions.push([agent.position.x, 0.0, agent.position.y]);
            velocities.push([agent.velocity.x, agent.velocity.y]);
            radii.push(agent.radius);
        }

        Self {
            positions,
            velocities,
            radii,
        }
    }
}

/// HPA cluster boundaries for debug overlay rendering.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HpaOverlay {
    /// Cluster boundaries as axis-aligned rectangles: `[min_x, min_z, max_x, max_z]`.
    pub clusters: Vec<[f32; 4]>,
    /// Inter-cluster edges: `(cluster_a, cluster_b)`.
    pub connections: Vec<(usize, usize)>,
}

impl HpaOverlay {
    /// Build an HPA overlay from [`GridClusters`] and its source [`NavGrid`].
    #[must_use]
    #[cfg_attr(feature = "logging", tracing::instrument(skip_all))]
    pub fn from_clusters(clusters: &GridClusters, grid: &NavGrid) -> Self {
        let cw = clusters.clusters_wide();
        let ch = clusters.clusters_high();

        let mut rects = Vec::with_capacity((cw * ch) as usize);
        for cy in 0..ch {
            for cx in 0..cw {
                let (min_pos, max_pos) = clusters.cluster_bounds(ClusterId { cx, cy }, grid);
                let min_world = grid.grid_to_world(min_pos);
                let max_world = grid.grid_to_world(max_pos);
                let half = grid.cell_size() * 0.5;
                rects.push([
                    min_world.x - half,
                    min_world.y - half,
                    max_world.x + half,
                    max_world.y + half,
                ]);
            }
        }

        // Connections: each entrance connects two clusters
        let mut connections = Vec::new();
        let mut seen = std::collections::HashSet::new();
        for entrance in clusters.entrances() {
            let a_idx = (entrance.cluster_a.cy * cw + entrance.cluster_a.cx) as usize;
            let b_idx = (entrance.cluster_b.cy * cw + entrance.cluster_b.cx) as usize;
            let key = if a_idx <= b_idx {
                (a_idx, b_idx)
            } else {
                (b_idx, a_idx)
            };
            if seen.insert(key) {
                connections.push(key);
            }
        }

        Self {
            clusters: rects,
            connections,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::GridPos;
    use crate::mesh::NavMesh;

    #[test]
    fn navmesh_wireframe_from_mesh() {
        let mesh = NavMesh::bake(&[
            hisab::Vec2::new(0.0, 0.0),
            hisab::Vec2::new(10.0, 0.0),
            hisab::Vec2::new(10.0, 10.0),
            hisab::Vec2::new(0.0, 10.0),
        ]);
        let wf = NavMeshWireframe::from_navmesh(&mesh);
        assert!(wf.poly_count > 0);
        assert!(!wf.edges.is_empty());
    }

    #[test]
    fn navmesh_wireframe_serializes() {
        let wf = NavMeshWireframe {
            edges: vec![([0.0; 3], [1.0, 0.0, 0.0])],
            poly_count: 1,
        };
        let json = serde_json::to_string(&wf).unwrap();
        let back: NavMeshWireframe = serde_json::from_str(&json).unwrap();
        assert_eq!(wf, back);
    }

    #[test]
    fn path_viz_from_result() {
        let result = PathResult::found(vec![
            hisab::Vec2::new(0.0, 0.0),
            hisab::Vec2::new(3.0, 4.0),
            hisab::Vec2::new(6.0, 8.0),
        ]);
        let viz = PathVisualization::from_path_result(&result);
        assert_eq!(viz.waypoints.len(), 3);
        assert!((viz.costs[0]).abs() < 0.001);
        assert!((viz.costs[1] - 5.0).abs() < 0.01);
        assert!((viz.total_cost - 10.0).abs() < 0.01);
    }

    #[test]
    fn path_viz_empty() {
        let result = PathResult::not_found();
        let viz = PathVisualization::from_path_result(&result);
        assert!(viz.waypoints.is_empty());
        assert!((viz.total_cost).abs() < 0.001);
    }

    #[test]
    fn path_viz_serde_roundtrip() {
        let path = PathVisualization {
            waypoints: vec![[0.0; 3], [5.0, 0.0, 5.0]],
            costs: vec![0.0, 7.07],
            total_cost: 7.07,
        };
        let json = serde_json::to_string(&path).unwrap();
        let back: PathVisualization = serde_json::from_str(&json).unwrap();
        assert_eq!(path, back);
    }

    #[test]
    fn flow_field_from_grid() {
        let grid = NavGrid::new(4, 4, 1.0);
        let flow = grid.flow_field(GridPos::new(3, 3));
        let viz = FlowFieldVisualization::from_flow_field(&flow, &grid);
        assert_eq!(viz.dimensions, [4, 4]);
        assert_eq!(viz.directions.len(), 16);
        assert!((viz.cell_size - 1.0).abs() < 0.001);
    }

    #[test]
    fn flow_field_serde_roundtrip() {
        let ff = FlowFieldVisualization {
            directions: vec![[1.0, 0.0]; 4],
            dimensions: [2, 2],
            cell_size: 1.0,
            origin: [0.0, 0.0],
        };
        let json = serde_json::to_string(&ff).unwrap();
        let back: FlowFieldVisualization = serde_json::from_str(&json).unwrap();
        assert_eq!(ff, back);
    }

    #[test]
    fn crowd_viz_serde_roundtrip() {
        let crowd = CrowdVisualization {
            positions: vec![[1.0, 0.0, 2.0]],
            velocities: vec![[0.5, 0.0]],
            radii: vec![0.4],
        };
        let json = serde_json::to_string(&crowd).unwrap();
        let back: CrowdVisualization = serde_json::from_str(&json).unwrap();
        assert_eq!(crowd, back);
    }

    #[test]
    fn crowd_viz_from_simulation() {
        use crate::rvo::RvoAgent;
        let mut crowd = CrowdSimulation::new(1.0, 2.0, 4.0);
        crowd.add_agent(RvoAgent::new(hisab::Vec2::new(1.0, 2.0), 0.4, 2.0));
        let viz = CrowdVisualization::from_crowd(&crowd);
        assert_eq!(viz.positions.len(), 1);
        assert!((viz.positions[0][0] - 1.0).abs() < 0.001);
        assert!((viz.positions[0][2] - 2.0).abs() < 0.001);
        assert!((viz.radii[0] - 0.4).abs() < 0.001);
    }

    #[test]
    fn hpa_overlay_from_clusters() {
        let grid = NavGrid::new(16, 16, 1.0);
        let clusters = GridClusters::build(&grid, 8);
        let overlay = HpaOverlay::from_clusters(&clusters, &grid);
        assert_eq!(overlay.clusters.len(), 4); // 2x2 clusters
    }

    #[test]
    fn hpa_overlay_serde_roundtrip() {
        let hpa = HpaOverlay {
            clusters: vec![[0.0, 0.0, 5.0, 5.0]],
            connections: vec![(0, 1)],
        };
        let json = serde_json::to_string(&hpa).unwrap();
        let back: HpaOverlay = serde_json::from_str(&json).unwrap();
        assert_eq!(hpa, back);
    }
}
