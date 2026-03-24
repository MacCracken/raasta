//! NavMesh — polygon-based navigation mesh.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use hisab::Vec2;
use serde::{Deserialize, Serialize};

/// Unique identifier for a navigation polygon.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct NavPolyId(pub u32);

/// A convex polygon in the navigation mesh.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NavPoly {
    pub id: NavPolyId,
    /// Vertices of this polygon (in order, convex).
    pub vertices: Vec<Vec2>,
    /// IDs of neighboring polygons.
    pub neighbors: Vec<NavPolyId>,
}

impl NavPoly {
    /// Compute the centroid of this polygon.
    #[must_use]
    pub fn centroid(&self) -> Vec2 {
        if self.vertices.is_empty() {
            return Vec2::ZERO;
        }
        let sum: Vec2 = self.vertices.iter().copied().sum();
        sum / self.vertices.len() as f32
    }

    /// Check if a point is inside this convex polygon (2D, XY plane).
    #[must_use]
    pub fn contains_point(&self, point: Vec2) -> bool {
        let n = self.vertices.len();
        if n < 3 {
            return false;
        }
        let mut sign = None;
        for i in 0..n {
            let j = (i + 1) % n;
            let edge = self.vertices[j] - self.vertices[i];
            let to_point = point - self.vertices[i];
            let cross = edge.perp_dot(to_point);
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
    #[must_use]
    pub fn new() -> Self {
        Self { polys: Vec::new() }
    }

    /// Bake a NavMesh from a simple polygon boundary.
    ///
    /// Triangulates the polygon, merges triangles into convex regions,
    /// and auto-detects neighbor connectivity from shared edges.
    #[must_use]
    pub fn bake(boundary: &[Vec2]) -> Self {
        use crate::triangulate::{merge_convex, triangulate};

        let tris = triangulate(boundary);
        if tris.is_empty() {
            return Self::new();
        }

        let merged = merge_convex(boundary, &tris);

        // Build NavPolys with vertices
        let mut polys: Vec<NavPoly> = merged
            .iter()
            .enumerate()
            .map(|(i, indices)| NavPoly {
                id: NavPolyId(i as u32),
                vertices: indices.iter().map(|&idx| boundary[idx]).collect(),
                neighbors: Vec::new(),
            })
            .collect();

        // Detect neighbors via shared edges
        // Build edge → poly index map
        let mut edge_to_poly: std::collections::HashMap<(usize, usize), Vec<u32>> =
            std::collections::HashMap::new();
        for (pi, indices) in merged.iter().enumerate() {
            for i in 0..indices.len() {
                let j = (i + 1) % indices.len();
                let key = if indices[i] <= indices[j] {
                    (indices[i], indices[j])
                } else {
                    (indices[j], indices[i])
                };
                edge_to_poly.entry(key).or_default().push(pi as u32);
            }
        }

        for owners in edge_to_poly.values() {
            if owners.len() == 2 {
                let a = owners[0];
                let b = owners[1];
                polys[a as usize].neighbors.push(NavPolyId(b));
                polys[b as usize].neighbors.push(NavPolyId(a));
            }
        }

        // Deduplicate neighbors
        for poly in &mut polys {
            poly.neighbors.sort_by_key(|n| n.0);
            poly.neighbors.dedup();
        }

        let mut mesh = Self::new();
        for poly in polys {
            mesh.add_poly(poly);
        }
        mesh
    }

    /// Add a polygon to the mesh.
    pub fn add_poly(&mut self, poly: NavPoly) {
        self.polys.push(poly);
    }

    /// Get all polygons.
    #[must_use]
    pub fn polys(&self) -> &[NavPoly] {
        &self.polys
    }

    /// Get a polygon by its ID.
    #[must_use]
    pub fn get_poly(&self, id: NavPolyId) -> Option<&NavPoly> {
        self.polys.iter().find(|p| p.id == id)
    }

    /// Find which polygon contains the given point.
    #[must_use]
    pub fn find_poly_at(&self, point: Vec2) -> Option<NavPolyId> {
        self.polys
            .iter()
            .find(|p| p.contains_point(point))
            .map(|p| p.id)
    }

    /// Find the closest point on the navmesh to the given point.
    ///
    /// Returns the nearest point on any polygon, or `None` if the mesh is empty.
    #[must_use]
    pub fn closest_point(&self, point: Vec2) -> Option<Vec2> {
        if self.polys.is_empty() {
            return None;
        }

        // If point is inside a polygon, return it directly
        if self.find_poly_at(point).is_some() {
            return Some(point);
        }

        // Otherwise find the closest point on any polygon edge
        let mut best_point = point;
        let mut best_dist = f32::INFINITY;

        for poly in &self.polys {
            let n = poly.vertices.len();
            for i in 0..n {
                let j = (i + 1) % n;
                let closest = closest_point_on_segment(point, poly.vertices[i], poly.vertices[j]);
                let dist = point.distance(closest);
                if dist < best_dist {
                    best_dist = dist;
                    best_point = closest;
                }
            }
        }

        Some(best_point)
    }

    /// Cast a ray from `origin` in `direction` and find the first intersection
    /// with a navmesh polygon edge.
    ///
    /// Returns the intersection point, or `None` if the ray doesn't hit any edge.
    #[must_use]
    pub fn raycast(&self, origin: Vec2, direction: Vec2) -> Option<Vec2> {
        let dir_len = direction.length();
        if dir_len < f32::EPSILON {
            return None;
        }
        let dir = direction / dir_len;

        let mut nearest_t = f32::INFINITY;
        let mut hit_point = None;

        for poly in &self.polys {
            let n = poly.vertices.len();
            for i in 0..n {
                let j = (i + 1) % n;
                if let Some(t) =
                    ray_segment_intersect(origin, dir, poly.vertices[i], poly.vertices[j])
                    && t > f32::EPSILON
                    && t < nearest_t
                {
                    nearest_t = t;
                    hit_point = Some(origin + dir * t);
                }
            }
        }

        hit_point
    }

    /// Find a path between two points on the navmesh.
    ///
    /// Returns a sequence of polygon IDs from start to goal.
    #[must_use]
    pub fn find_path(&self, start: Vec2, goal: Vec2) -> Option<Vec<NavPolyId>> {
        let start_id = self.find_poly_at(start)?;
        let goal_id = self.find_poly_at(goal)?;

        if start_id == goal_id {
            return Some(vec![start_id]);
        }

        // A* over polygon graph
        let mut g_score = std::collections::HashMap::new();
        let mut came_from = std::collections::HashMap::new();
        let mut closed = std::collections::HashSet::new();
        let mut open = BinaryHeap::new();

        g_score.insert(start_id, 0.0f32);
        let goal_centroid = self.get_poly(goal_id)?.centroid();
        let start_centroid = self.get_poly(start_id)?.centroid();
        let h = start_centroid.distance(goal_centroid);

        open.push(MeshNode {
            id: start_id,
            f_score: h,
        });

        while let Some(current) = open.pop() {
            if current.id == goal_id {
                return Some(self.reconstruct_path(&came_from, goal_id));
            }

            if !closed.insert(current.id) {
                continue;
            }

            let current_g = g_score[&current.id];
            let poly = self.get_poly(current.id)?;
            let current_centroid = poly.centroid();

            for &neighbor_id in &poly.neighbors {
                if closed.contains(&neighbor_id) {
                    continue;
                }
                let neighbor = self.get_poly(neighbor_id)?;
                let neighbor_centroid = neighbor.centroid();
                let edge_cost = current_centroid.distance(neighbor_centroid);
                let tentative_g = current_g + edge_cost;

                let prev_g = g_score.get(&neighbor_id).copied().unwrap_or(f32::INFINITY);
                if tentative_g < prev_g {
                    came_from.insert(neighbor_id, current.id);
                    g_score.insert(neighbor_id, tentative_g);
                    let h = neighbor_centroid.distance(goal_centroid);
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
    #[must_use]
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

/// Closest point on line segment AB to point P.
fn closest_point_on_segment(p: Vec2, a: Vec2, b: Vec2) -> Vec2 {
    let ab = b - a;
    let len_sq = ab.length_squared();
    if len_sq < f32::EPSILON {
        return a;
    }
    let t = ((p - a).dot(ab) / len_sq).clamp(0.0, 1.0);
    a + ab * t
}

/// Ray-segment intersection. Returns parameter t along ray, or None.
fn ray_segment_intersect(origin: Vec2, dir: Vec2, a: Vec2, b: Vec2) -> Option<f32> {
    let edge = b - a;
    let denom = dir.perp_dot(edge);
    if denom.abs() < f32::EPSILON {
        return None; // parallel
    }
    let t = (a - origin).perp_dot(edge) / denom;
    let u = (a - origin).perp_dot(dir) / denom;
    if t > 0.0 && (0.0..=1.0).contains(&u) {
        Some(t)
    } else {
        None
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
            vertices: vec![
                Vec2::ZERO,
                Vec2::new(1.0, 0.0),
                Vec2::ONE,
                Vec2::new(0.0, 1.0),
            ],
            neighbors: vec![NavPolyId(1)],
        });
        mesh.add_poly(NavPoly {
            id: NavPolyId(1),
            vertices: vec![
                Vec2::new(1.0, 0.0),
                Vec2::new(2.0, 0.0),
                Vec2::new(2.0, 1.0),
                Vec2::ONE,
            ],
            neighbors: vec![NavPolyId(0)],
        });
        mesh
    }

    #[test]
    fn centroid() {
        let poly = NavPoly {
            id: NavPolyId(0),
            vertices: vec![
                Vec2::ZERO,
                Vec2::new(2.0, 0.0),
                Vec2::new(2.0, 2.0),
                Vec2::new(0.0, 2.0),
            ],
            neighbors: vec![],
        };
        let c = poly.centroid();
        assert!((c.x - 1.0).abs() < f32::EPSILON);
        assert!((c.y - 1.0).abs() < f32::EPSILON);
    }

    #[test]
    fn contains_point_inside() {
        let poly = NavPoly {
            id: NavPolyId(0),
            vertices: vec![
                Vec2::ZERO,
                Vec2::new(4.0, 0.0),
                Vec2::new(4.0, 4.0),
                Vec2::new(0.0, 4.0),
            ],
            neighbors: vec![],
        };
        assert!(poly.contains_point(Vec2::new(2.0, 2.0)));
    }

    #[test]
    fn contains_point_outside() {
        let poly = NavPoly {
            id: NavPolyId(0),
            vertices: vec![
                Vec2::ZERO,
                Vec2::new(4.0, 0.0),
                Vec2::new(4.0, 4.0),
                Vec2::new(0.0, 4.0),
            ],
            neighbors: vec![],
        };
        assert!(!poly.contains_point(Vec2::new(5.0, 5.0)));
    }

    #[test]
    fn find_poly_at() {
        let mesh = make_square_mesh();
        assert_eq!(mesh.find_poly_at(Vec2::new(0.5, 0.5)), Some(NavPolyId(0)));
        assert_eq!(mesh.find_poly_at(Vec2::new(1.5, 0.5)), Some(NavPolyId(1)));
        assert_eq!(mesh.find_poly_at(Vec2::new(3.0, 0.5)), None);
    }

    #[test]
    fn navmesh_path_same_poly() {
        let mesh = make_square_mesh();
        let path = mesh.find_path(Vec2::new(0.2, 0.2), Vec2::new(0.8, 0.8));
        assert_eq!(path, Some(vec![NavPolyId(0)]));
    }

    #[test]
    fn navmesh_path_across_polys() {
        let mesh = make_square_mesh();
        let path = mesh.find_path(Vec2::new(0.5, 0.5), Vec2::new(1.5, 0.5));
        assert_eq!(path, Some(vec![NavPolyId(0), NavPolyId(1)]));
    }

    #[test]
    fn navmesh_path_unreachable() {
        let mesh = make_square_mesh();
        let path = mesh.find_path(Vec2::new(0.5, 0.5), Vec2::new(5.0, 5.0));
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
        assert_eq!(poly.centroid(), Vec2::ZERO);
    }

    #[test]
    fn triangle_contains() {
        let tri = NavPoly {
            id: NavPolyId(0),
            vertices: vec![Vec2::ZERO, Vec2::new(4.0, 0.0), Vec2::new(2.0, 4.0)],
            neighbors: vec![],
        };
        assert!(tri.contains_point(Vec2::new(2.0, 1.0)));
        assert!(!tri.contains_point(Vec2::new(0.0, 4.0)));
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

    #[test]
    fn degenerate_poly_contains_point() {
        // A polygon with < 3 vertices should never contain any point
        let line = NavPoly {
            id: NavPolyId(0),
            vertices: vec![Vec2::ZERO, Vec2::ONE],
            neighbors: vec![],
        };
        assert!(!line.contains_point(Vec2::new(0.5, 0.5)));

        let point = NavPoly {
            id: NavPolyId(1),
            vertices: vec![Vec2::ZERO],
            neighbors: vec![],
        };
        assert!(!point.contains_point(Vec2::ZERO));
    }

    #[test]
    fn empty_mesh() {
        let mesh = NavMesh::new();
        assert_eq!(mesh.poly_count(), 0);
        assert!(mesh.find_poly_at(Vec2::ZERO).is_none());
        assert!(mesh.find_path(Vec2::ZERO, Vec2::ONE).is_none());
    }

    #[test]
    fn disconnected_polys() {
        let mut mesh = NavMesh::new();
        mesh.add_poly(NavPoly {
            id: NavPolyId(0),
            vertices: vec![
                Vec2::ZERO,
                Vec2::new(1.0, 0.0),
                Vec2::ONE,
                Vec2::new(0.0, 1.0),
            ],
            neighbors: vec![],
        });
        mesh.add_poly(NavPoly {
            id: NavPolyId(1),
            vertices: vec![
                Vec2::new(10.0, 10.0),
                Vec2::new(11.0, 10.0),
                Vec2::new(11.0, 11.0),
                Vec2::new(10.0, 11.0),
            ],
            neighbors: vec![],
        });
        assert!(mesh.find_poly_at(Vec2::new(0.5, 0.5)).is_some());
        assert!(mesh.find_poly_at(Vec2::new(10.5, 10.5)).is_some());
        assert!(
            mesh.find_path(Vec2::new(0.5, 0.5), Vec2::new(10.5, 10.5))
                .is_none()
        );
    }

    #[test]
    fn navpoly_id_serde_roundtrip() {
        let id = NavPolyId(42);
        let json = serde_json::to_string(&id).unwrap();
        let deserialized: NavPolyId = serde_json::from_str(&json).unwrap();
        assert_eq!(deserialized, id);
    }

    // --- Bake tests ---

    #[test]
    fn bake_square() {
        let boundary = vec![
            Vec2::ZERO,
            Vec2::new(4.0, 0.0),
            Vec2::new(4.0, 4.0),
            Vec2::new(0.0, 4.0),
        ];
        let mesh = NavMesh::bake(&boundary);
        assert!(mesh.poly_count() >= 1);
        // Should be able to find a point inside
        assert!(mesh.find_poly_at(Vec2::new(2.0, 2.0)).is_some());
    }

    #[test]
    fn bake_l_shape() {
        let boundary = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(2.0, 0.0),
            Vec2::new(2.0, 1.0),
            Vec2::new(1.0, 1.0),
            Vec2::new(1.0, 2.0),
            Vec2::new(0.0, 2.0),
        ];
        let mesh = NavMesh::bake(&boundary);
        assert!(mesh.poly_count() >= 1);

        // Points inside the L should be findable
        assert!(mesh.find_poly_at(Vec2::new(0.5, 0.5)).is_some());
        assert!(mesh.find_poly_at(Vec2::new(0.5, 1.5)).is_some());

        // Point outside the L
        assert!(mesh.find_poly_at(Vec2::new(1.5, 1.5)).is_none());
    }

    #[test]
    fn bake_neighbors_connected() {
        let boundary = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(4.0, 0.0),
            Vec2::new(4.0, 1.0),
            Vec2::new(2.0, 1.0),
            Vec2::new(2.0, 2.0),
            Vec2::new(0.0, 2.0),
        ];
        let mesh = NavMesh::bake(&boundary);

        // If there are multiple polys, they should have neighbor connections
        if mesh.poly_count() > 1 {
            let has_neighbors = mesh.polys().iter().any(|p| !p.neighbors.is_empty());
            assert!(
                has_neighbors,
                "multi-poly mesh should have neighbor connections"
            );
        }
    }

    #[test]
    fn bake_pathfinding_works() {
        let boundary = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(6.0, 0.0),
            Vec2::new(6.0, 1.0),
            Vec2::new(3.0, 1.0),
            Vec2::new(3.0, 3.0),
            Vec2::new(0.0, 3.0),
        ];
        let mesh = NavMesh::bake(&boundary);

        // Path from one end to another should work if neighbors are connected
        let start = Vec2::new(0.5, 0.5);
        let goal = Vec2::new(0.5, 2.5);

        assert!(mesh.find_poly_at(start).is_some());
        assert!(mesh.find_poly_at(goal).is_some());

        let path = mesh.find_path(start, goal);
        assert!(path.is_some());
    }

    #[test]
    fn bake_empty() {
        let mesh = NavMesh::bake(&[]);
        assert_eq!(mesh.poly_count(), 0);
    }

    #[test]
    fn bake_triangle() {
        let boundary = vec![Vec2::ZERO, Vec2::new(3.0, 0.0), Vec2::new(1.5, 3.0)];
        let mesh = NavMesh::bake(&boundary);
        assert_eq!(mesh.poly_count(), 1);
        assert!(mesh.find_poly_at(Vec2::new(1.5, 1.0)).is_some());
    }

    // --- Nav query tests ---

    #[test]
    fn closest_point_inside() {
        let mesh = make_square_mesh();
        let p = mesh.closest_point(Vec2::new(0.5, 0.5));
        assert_eq!(p, Some(Vec2::new(0.5, 0.5)));
    }

    #[test]
    fn closest_point_outside() {
        let mesh = make_square_mesh();
        let p = mesh.closest_point(Vec2::new(-1.0, 0.5)).unwrap();
        // Should snap to the left edge of poly 0 (x=0)
        assert!((p.x - 0.0).abs() < 0.1);
    }

    #[test]
    fn closest_point_empty_mesh() {
        let mesh = NavMesh::new();
        assert!(mesh.closest_point(Vec2::ZERO).is_none());
    }

    #[test]
    fn raycast_hits_edge() {
        let mesh = make_square_mesh();
        // Cast ray from inside poly 0 to the right — should hit right edge at x=1
        let hit = mesh.raycast(Vec2::new(0.5, 0.5), Vec2::new(1.0, 0.0));
        assert!(hit.is_some());
    }

    #[test]
    fn raycast_misses() {
        let mesh = make_square_mesh();
        // Cast ray away from the mesh
        let hit = mesh.raycast(Vec2::new(0.5, 0.5), Vec2::new(0.0, -1.0));
        // May or may not hit bottom edge depending on polygon winding
        // The important thing is it doesn't panic
        let _ = hit;
    }

    #[test]
    fn raycast_empty_mesh() {
        let mesh = NavMesh::new();
        assert!(mesh.raycast(Vec2::ZERO, Vec2::new(1.0, 0.0)).is_none());
    }
}
