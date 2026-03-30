//! Soorat integration — visualization data structures for navigation/pathfinding.

use serde::{Deserialize, Serialize};

/// NavMesh wireframe for debug line rendering.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NavMeshWireframe {
    /// Edge segments: `(start, end)` as `[x, y, z]` pairs.
    pub edges: Vec<([f32; 3], [f32; 3])>,
    /// Polygon count.
    pub poly_count: usize,
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

/// HPA cluster boundaries for debug overlay rendering.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HpaOverlay {
    /// Cluster boundaries as axis-aligned rectangles: `(min_x, min_z, max_x, max_z)`.
    pub clusters: Vec<[f32; 4]>,
    /// Inter-cluster edges: `(cluster_a, cluster_b)`.
    pub connections: Vec<(usize, usize)>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn navmesh_wireframe_serializes() {
        let wf = NavMeshWireframe {
            edges: vec![([0.0; 3], [1.0, 0.0, 0.0])],
            poly_count: 1,
        };
        let json = serde_json::to_string(&wf);
        assert!(json.is_ok());
    }

    #[test]
    fn path_viz_manual() {
        let path = PathVisualization {
            waypoints: vec![[0.0; 3], [5.0, 0.0, 5.0], [10.0, 0.0, 10.0]],
            costs: vec![0.0, 7.07, 14.14],
            total_cost: 14.14,
        };
        assert_eq!(path.waypoints.len(), 3);
    }

    #[test]
    fn flow_field_manual() {
        let ff = FlowFieldVisualization {
            directions: vec![[1.0, 0.0]; 4],
            dimensions: [2, 2],
            cell_size: 1.0,
            origin: [0.0, 0.0],
        };
        assert_eq!(ff.directions.len(), 4);
    }

    #[test]
    fn crowd_viz_manual() {
        let crowd = CrowdVisualization {
            positions: vec![[1.0, 0.0, 2.0], [3.0, 0.0, 4.0]],
            velocities: vec![[0.5, 0.0], [-0.3, 0.2]],
            radii: vec![0.4, 0.5],
        };
        assert_eq!(crowd.positions.len(), 2);
    }

    #[test]
    fn hpa_overlay_manual() {
        let hpa = HpaOverlay {
            clusters: vec![[0.0, 0.0, 5.0, 5.0], [5.0, 0.0, 10.0, 5.0]],
            connections: vec![(0, 1)],
        };
        assert_eq!(hpa.clusters.len(), 2);
    }
}
