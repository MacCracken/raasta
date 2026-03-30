//! 3D NavMesh — polygon-based navigation in 3D space.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use hisab::Vec3;
use serde::{Deserialize, Serialize};

#[cfg(feature = "logging")]
use tracing::instrument;

use crate::mesh::NavPolyId;

/// A convex polygon in 3D space.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NavPoly3D {
    pub id: NavPolyId,
    pub vertices: Vec<Vec3>,
    pub neighbors: Vec<NavPolyId>,
}

impl NavPoly3D {
    /// Compute the centroid of this polygon.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    #[must_use]
    pub fn centroid(&self) -> Vec3 {
        if self.vertices.is_empty() {
            return Vec3::ZERO;
        }
        let sum: Vec3 = self.vertices.iter().copied().sum();
        sum / self.vertices.len() as f32
    }

    /// Normal of the polygon plane (assumes convex, CCW winding).
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    #[must_use]
    pub fn normal(&self) -> Vec3 {
        if self.vertices.len() < 3 {
            return Vec3::Y; // default up
        }
        let e1 = self.vertices[1] - self.vertices[0];
        let e2 = self.vertices[2] - self.vertices[0];
        e1.cross(e2).normalize_or_zero()
    }

    /// Project a 3D point onto this polygon's plane and check if it's inside.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    #[must_use]
    pub fn contains_point_projected(&self, point: Vec3) -> bool {
        let n = self.vertices.len();
        if n < 3 {
            return false;
        }
        let normal = self.normal();

        // Find the best 2D projection axis (drop the axis most aligned with normal)
        let abs_normal = normal.abs();
        let (ax1, ax2) = if abs_normal.x >= abs_normal.y && abs_normal.x >= abs_normal.z {
            // Drop X, project to YZ
            (1, 2)
        } else if abs_normal.y >= abs_normal.z {
            // Drop Y, project to XZ
            (0, 2)
        } else {
            // Drop Z, project to XY
            (0, 1)
        };

        let p = [point[ax1], point[ax2]];
        let mut sign = None;
        for i in 0..n {
            let j = (i + 1) % n;
            let vi = [self.vertices[i][ax1], self.vertices[i][ax2]];
            let vj = [self.vertices[j][ax1], self.vertices[j][ax2]];
            let ex = vj[0] - vi[0];
            let ey = vj[1] - vi[1];
            let px = p[0] - vi[0];
            let py = p[1] - vi[1];
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

/// A 3D navigation mesh.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct NavMesh3D {
    polys: Vec<NavPoly3D>,
}

impl NavMesh3D {
    #[cfg_attr(feature = "logging", instrument)]
    #[must_use]
    pub fn new() -> Self {
        Self { polys: Vec::new() }
    }

    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    pub fn add_poly(&mut self, poly: NavPoly3D) {
        self.polys.push(poly);
    }

    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    #[must_use]
    pub fn polys(&self) -> &[NavPoly3D] {
        &self.polys
    }

    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    #[must_use]
    pub fn poly_count(&self) -> usize {
        self.polys.len()
    }

    /// Get a polygon by its ID.
    ///
    /// Assumes poly IDs are contiguous indices `0..poly_count()`.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    #[inline]
    #[must_use]
    pub fn get_poly(&self, id: NavPolyId) -> Option<&NavPoly3D> {
        self.polys.get(id.0 as usize).filter(|p| p.id == id)
    }

    /// Find which polygon contains the given point (projected onto polygon planes).
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    #[must_use]
    pub fn find_poly_at(&self, point: Vec3) -> Option<NavPolyId> {
        self.polys
            .iter()
            .find(|p| p.contains_point_projected(point))
            .map(|p| p.id)
    }

    /// Find a path between two points. Returns polygon IDs.
    #[cfg_attr(feature = "logging", instrument(skip(self), fields(polys = self.polys.len())))]
    #[must_use]
    pub fn find_path(&self, start: Vec3, goal: Vec3) -> Option<Vec<NavPolyId>> {
        let start_id = self.find_poly_at(start)?;
        let goal_id = self.find_poly_at(goal)?;

        if start_id == goal_id {
            return Some(vec![start_id]);
        }

        let n = self.polys.len();
        let mut g_score = vec![f32::INFINITY; n];
        let mut came_from: Vec<Option<u32>> = vec![None; n];
        let mut closed = vec![false; n];
        let mut open = BinaryHeap::new();

        let start_idx = start_id.0 as usize;
        g_score[start_idx] = 0.0;
        let goal_centroid = self.get_poly(goal_id)?.centroid();
        let start_centroid = self.get_poly(start_id)?.centroid();
        let h = start_centroid.distance(goal_centroid);

        open.push(MeshNode3D {
            id: start_id,
            f_score: h,
        });

        while let Some(current) = open.pop() {
            if current.id == goal_id {
                return Some(Self::reconstruct_path_vec(&came_from, start_id, goal_id));
            }

            let ci = current.id.0 as usize;
            if closed[ci] {
                continue;
            }
            closed[ci] = true;

            let current_g = g_score[ci];
            let poly = self.get_poly(current.id)?;
            let current_centroid = poly.centroid();

            for &neighbor_id in &poly.neighbors {
                let ni = neighbor_id.0 as usize;
                if ni >= n || closed[ni] {
                    continue;
                }
                let neighbor = self.get_poly(neighbor_id)?;
                let neighbor_centroid = neighbor.centroid();
                let edge_cost = current_centroid.distance(neighbor_centroid);
                let tentative_g = current_g + edge_cost;

                if tentative_g < g_score[ni] {
                    came_from[ni] = Some(current.id.0);
                    g_score[ni] = tentative_g;
                    let h = neighbor_centroid.distance(goal_centroid);
                    open.push(MeshNode3D {
                        id: neighbor_id,
                        f_score: tentative_g + h,
                    });
                }
            }
        }

        None
    }

    fn reconstruct_path_vec(
        came_from: &[Option<u32>],
        start_id: NavPolyId,
        goal_id: NavPolyId,
    ) -> Vec<NavPolyId> {
        let mut path = Vec::new();
        let mut current = goal_id;
        loop {
            path.push(current);
            if current == start_id {
                break;
            }
            match came_from[current.0 as usize] {
                Some(prev) => current = NavPolyId(prev),
                None => break,
            }
        }
        path.reverse();
        path
    }
}

#[derive(Clone, Copy)]
struct MeshNode3D {
    id: NavPolyId,
    f_score: f32,
}

impl PartialEq for MeshNode3D {
    fn eq(&self, other: &Self) -> bool {
        self.f_score == other.f_score
    }
}

impl Eq for MeshNode3D {}

impl PartialOrd for MeshNode3D {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for MeshNode3D {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .f_score
            .partial_cmp(&self.f_score)
            .unwrap_or(Ordering::Equal)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_ramp_mesh() -> NavMesh3D {
        // Two quads forming a ramp: flat + angled
        let mut mesh = NavMesh3D::new();
        mesh.add_poly(NavPoly3D {
            id: NavPolyId(0),
            vertices: vec![
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(2.0, 0.0, 0.0),
                Vec3::new(2.0, 0.0, 2.0),
                Vec3::new(0.0, 0.0, 2.0),
            ],
            neighbors: vec![NavPolyId(1)],
        });
        mesh.add_poly(NavPoly3D {
            id: NavPolyId(1),
            vertices: vec![
                Vec3::new(2.0, 0.0, 0.0),
                Vec3::new(4.0, 2.0, 0.0),
                Vec3::new(4.0, 2.0, 2.0),
                Vec3::new(2.0, 0.0, 2.0),
            ],
            neighbors: vec![NavPolyId(0)],
        });
        mesh
    }

    #[test]
    fn centroid_3d() {
        let poly = NavPoly3D {
            id: NavPolyId(0),
            vertices: vec![
                Vec3::ZERO,
                Vec3::new(2.0, 0.0, 0.0),
                Vec3::new(2.0, 2.0, 0.0),
                Vec3::new(0.0, 2.0, 0.0),
            ],
            neighbors: vec![],
        };
        let c = poly.centroid();
        assert!((c.x - 1.0).abs() < f32::EPSILON);
        assert!((c.y - 1.0).abs() < f32::EPSILON);
    }

    #[test]
    fn normal_3d() {
        let poly = NavPoly3D {
            id: NavPolyId(0),
            vertices: vec![
                Vec3::ZERO,
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            ],
            neighbors: vec![],
        };
        let n = poly.normal();
        // For XZ plane triangle, normal should point in +Y or -Y
        assert!(n.y.abs() > 0.9);
    }

    #[test]
    fn contains_point_3d() {
        let poly = NavPoly3D {
            id: NavPolyId(0),
            vertices: vec![
                Vec3::ZERO,
                Vec3::new(4.0, 0.0, 0.0),
                Vec3::new(4.0, 0.0, 4.0),
                Vec3::new(0.0, 0.0, 4.0),
            ],
            neighbors: vec![],
        };
        assert!(poly.contains_point_projected(Vec3::new(2.0, 0.0, 2.0)));
        assert!(!poly.contains_point_projected(Vec3::new(5.0, 0.0, 5.0)));
    }

    #[test]
    fn find_poly_at_3d() {
        let mesh = make_ramp_mesh();
        assert_eq!(
            mesh.find_poly_at(Vec3::new(1.0, 0.0, 1.0)),
            Some(NavPolyId(0))
        );
        assert!(mesh.find_poly_at(Vec3::new(10.0, 0.0, 10.0)).is_none());
    }

    #[test]
    fn path_across_ramp() {
        let mesh = make_ramp_mesh();
        let path = mesh.find_path(Vec3::new(1.0, 0.0, 1.0), Vec3::new(3.0, 1.0, 1.0));
        assert!(path.is_some());
        let path = path.unwrap();
        assert_eq!(path.len(), 2);
    }

    #[test]
    fn path_same_poly_3d() {
        let mesh = make_ramp_mesh();
        let path = mesh.find_path(Vec3::new(0.5, 0.0, 0.5), Vec3::new(1.5, 0.0, 1.5));
        assert_eq!(path, Some(vec![NavPolyId(0)]));
    }

    #[test]
    fn empty_mesh_3d() {
        let mesh = NavMesh3D::new();
        assert_eq!(mesh.poly_count(), 0);
        assert!(mesh.find_path(Vec3::ZERO, Vec3::ONE).is_none());
    }

    #[test]
    fn mesh3d_serde_roundtrip() {
        let mesh = make_ramp_mesh();
        let json = serde_json::to_string(&mesh).unwrap();
        let deserialized: NavMesh3D = serde_json::from_str(&json).unwrap();
        assert_eq!(deserialized.poly_count(), 2);
    }
}
