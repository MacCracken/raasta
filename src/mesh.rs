//! NavMesh — polygon-based navigation mesh.

use std::collections::BinaryHeap;
use std::cmp::Ordering;

use serde::{Deserialize, Serialize};

/// Unique identifier for a navigation polygon.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct NavPolyId(pub u32);

/// A convex polygon in the navigation mesh.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NavPoly {
    pub id: NavPolyId,
    /// Vertices of this polygon (in order, convex).
    pub vertices: Vec<[f32; 2]>,
    /// IDs of neighboring polygons.
    pub neighbors: Vec<NavPolyId>,
}

impl NavPoly {
    /// Compute the centroid of this polygon.
    pub fn centroid(&self) -> [f32; 2] {
        if self.vertices.is_empty() {
            return [0.0, 0.0];
        }
        let n = self.vertices.len() as f32;
        let (sx, sy) = self.vertices.iter().fold((0.0f32, 0.0f32), |(ax, ay), v| {
            (ax + v[0], ay + v[1])
        });
        [sx / n, sy / n]
    }

    /// Check if a point is inside this convex polygon (2D, XY plane).
    pub fn contains_point(&self, point: [f32; 2]) -> bool {
        let n = self.vertices.len();
        if n < 3 {
            return false;
        }
        let mut sign = None;
        for i in 0..n {
            let j = (i + 1) % n;
            let ex = self.vertices[j][0] - self.vertices[i][0];
            let ey = self.vertices[j][1] - self.vertices[i][1];
            let px = point[0] - self.vertices[i][0];
            let py = point[1] - self.vertices[i][1];
            let cross = ex * py - ey * px;
            let s = cross >= 0.0;
            match sign {
                None => sign = Some(s),
                Some(prev) if prev != s => return false,
                _ => {}
            }
        }
        true
    }
}

/// A navigation mesh composed of convex polygons.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct NavMesh {
    polys: Vec<NavPoly>,
}

impl NavMesh {
    pub fn new() -> Self {
        Self { polys: Vec::new() }
    }

    /// Add a polygon to the mesh.
    pub fn add_poly(&mut self, poly: NavPoly) {
        self.polys.push(poly);
    }

    /// Get all polygons.
    pub fn polys(&self) -> &[NavPoly] {
        &self.polys
    }

    /// Get a polygon by its ID.
    pub fn get_poly(&self, id: NavPolyId) -> Option<&NavPoly> {
        self.polys.iter().find(|p| p.id == id)
    }

    /// Find which polygon contains the given point.
    pub fn find_poly_at(&self, point: [f32; 2]) -> Option<NavPolyId> {
        self.polys
            .iter()
            .find(|p| p.contains_point(point))
            .map(|p| p.id)
    }

    /// Find a path between two points on the navmesh.
    ///
    /// Returns a sequence of polygon IDs from start to goal.
    pub fn find_path(&self, start: [f32; 2], goal: [f32; 2]) -> Option<Vec<NavPolyId>> {
        let start_id = self.find_poly_at(start)?;
        let goal_id = self.find_poly_at(goal)?;

        if start_id == goal_id {
            return Some(vec![start_id]);
        }

        // A* over polygon graph
        let mut g_score = std::collections::HashMap::new();
        let mut came_from = std::collections::HashMap::new();
        let mut open = BinaryHeap::new();

        g_score.insert(start_id, 0.0f32);
        let goal_centroid = self.get_poly(goal_id)?.centroid();
        let start_centroid = self.get_poly(start_id)?.centroid();
        let h = dist2d(start_centroid, goal_centroid);

        open.push(MeshNode {
            id: start_id,
            f_score: h,
        });

        while let Some(current) = open.pop() {
            if current.id == goal_id {
                return Some(self.reconstruct_path(&came_from, goal_id));
            }

            let current_g = g_score[&current.id];
            let poly = self.get_poly(current.id)?;
            let current_centroid = poly.centroid();

            for &neighbor_id in &poly.neighbors {
                let neighbor = self.get_poly(neighbor_id)?;
                let neighbor_centroid = neighbor.centroid();
                let edge_cost = dist2d(current_centroid, neighbor_centroid);
                let tentative_g = current_g + edge_cost;

                let prev_g = g_score.get(&neighbor_id).copied().unwrap_or(f32::INFINITY);
                if tentative_g < prev_g {
                    came_from.insert(neighbor_id, current.id);
                    g_score.insert(neighbor_id, tentative_g);
                    let h = dist2d(neighbor_centroid, goal_centroid);
                    open.push(MeshNode {
                        id: neighbor_id,
                        f_score: tentative_g + h,
                    });
                }
            }
        }

        None
    }

    /// Number of polygons in the mesh.
    pub fn poly_count(&self) -> usize {
        self.polys.len()
    }

    fn reconstruct_path(
        &self,
        came_from: &std::collections::HashMap<NavPolyId, NavPolyId>,
        goal_id: NavPolyId,
    ) -> Vec<NavPolyId> {
        let mut path = Vec::new();
        let mut current = goal_id;
        loop {
            path.push(current);
            match came_from.get(&current) {
                Some(&prev) => current = prev,
                None => break,
            }
        }
        path.reverse();
        path
    }
}

#[inline]
fn dist2d(a: [f32; 2], b: [f32; 2]) -> f32 {
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    (dx * dx + dy * dy).sqrt()
}

#[derive(Clone, Copy)]
struct MeshNode {
    id: NavPolyId,
    f_score: f32,
}

impl PartialEq for MeshNode {
    fn eq(&self, other: &Self) -> bool {
        self.f_score == other.f_score
    }
}

impl Eq for MeshNode {}

impl PartialOrd for MeshNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for MeshNode {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .f_score
            .partial_cmp(&self.f_score)
            .unwrap_or(Ordering::Equal)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn make_square_mesh() -> NavMesh {
        // Two adjacent squares: [0,0]-[1,1] and [1,0]-[2,1]
        let mut mesh = NavMesh::new();
        mesh.add_poly(NavPoly {
            id: NavPolyId(0),
            vertices: vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]],
            neighbors: vec![NavPolyId(1)],
        });
        mesh.add_poly(NavPoly {
            id: NavPolyId(1),
            vertices: vec![[1.0, 0.0], [2.0, 0.0], [2.0, 1.0], [1.0, 1.0]],
            neighbors: vec![NavPolyId(0)],
        });
        mesh
    }

    #[test]
    fn centroid() {
        let poly = NavPoly {
            id: NavPolyId(0),
            vertices: vec![[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]],
            neighbors: vec![],
        };
        let c = poly.centroid();
        assert!((c[0] - 1.0).abs() < f32::EPSILON);
        assert!((c[1] - 1.0).abs() < f32::EPSILON);
    }

    #[test]
    fn contains_point_inside() {
        let poly = NavPoly {
            id: NavPolyId(0),
            vertices: vec![[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]],
            neighbors: vec![],
        };
        assert!(poly.contains_point([2.0, 2.0]));
    }

    #[test]
    fn contains_point_outside() {
        let poly = NavPoly {
            id: NavPolyId(0),
            vertices: vec![[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]],
            neighbors: vec![],
        };
        assert!(!poly.contains_point([5.0, 5.0]));
    }

    #[test]
    fn find_poly_at() {
        let mesh = make_square_mesh();
        assert_eq!(mesh.find_poly_at([0.5, 0.5]), Some(NavPolyId(0)));
        assert_eq!(mesh.find_poly_at([1.5, 0.5]), Some(NavPolyId(1)));
        assert_eq!(mesh.find_poly_at([3.0, 0.5]), None);
    }

    #[test]
    fn navmesh_path_same_poly() {
        let mesh = make_square_mesh();
        let path = mesh.find_path([0.2, 0.2], [0.8, 0.8]);
        assert_eq!(path, Some(vec![NavPolyId(0)]));
    }

    #[test]
    fn navmesh_path_across_polys() {
        let mesh = make_square_mesh();
        let path = mesh.find_path([0.5, 0.5], [1.5, 0.5]);
        assert_eq!(path, Some(vec![NavPolyId(0), NavPolyId(1)]));
    }

    #[test]
    fn navmesh_path_unreachable() {
        let mesh = make_square_mesh();
        let path = mesh.find_path([0.5, 0.5], [5.0, 5.0]);
        assert!(path.is_none());
    }

    #[test]
    fn navmesh_poly_count() {
        let mesh = make_square_mesh();
        assert_eq!(mesh.poly_count(), 2);
    }

    #[test]
    fn empty_poly_centroid() {
        let poly = NavPoly {
            id: NavPolyId(0),
            vertices: vec![],
            neighbors: vec![],
        };
        assert_eq!(poly.centroid(), [0.0, 0.0]);
    }

    #[test]
    fn triangle_contains() {
        let tri = NavPoly {
            id: NavPolyId(0),
            vertices: vec![[0.0, 0.0], [4.0, 0.0], [2.0, 4.0]],
            neighbors: vec![],
        };
        assert!(tri.contains_point([2.0, 1.0]));
        assert!(!tri.contains_point([0.0, 4.0]));
    }

    #[test]
    fn navmesh_get_poly() {
        let mesh = make_square_mesh();
        assert!(mesh.get_poly(NavPolyId(0)).is_some());
        assert!(mesh.get_poly(NavPolyId(99)).is_none());
    }

    #[test]
    fn navmesh_serde_roundtrip() {
        let mesh = make_square_mesh();
        let json = serde_json::to_string(&mesh).unwrap();
        let deserialized: NavMesh = serde_json::from_str(&json).unwrap();
        assert_eq!(deserialized.poly_count(), 2);
    }
}
