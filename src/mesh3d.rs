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
    /// Traversal cost multiplier (1.0 = normal, higher = more expensive).
    pub cost: f32,
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

    /// Serialize the 3D navmesh to a compact binary format.
    ///
    /// The binary format stores a magic header (`RNV3`), a version number,
    /// polygon count, then for each polygon: ID, vertex count, vertices
    /// (x,y,z triples as little-endian `f32`), neighbor count, neighbor IDs,
    /// and cost.
    ///
    /// Use [`NavMesh3D::from_bytes`] to deserialize.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    #[must_use]
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut buf = Vec::new();

        // Magic bytes + version
        buf.extend_from_slice(b"RNV3");
        buf.extend_from_slice(&1u32.to_le_bytes()); // version 1

        // Polygon count
        buf.extend_from_slice(&(self.polys.len() as u32).to_le_bytes());

        for poly in &self.polys {
            // Poly ID
            buf.extend_from_slice(&poly.id.0.to_le_bytes());
            // Vertex count
            buf.extend_from_slice(&(poly.vertices.len() as u32).to_le_bytes());
            // Vertices
            for v in &poly.vertices {
                buf.extend_from_slice(&v.x.to_le_bytes());
                buf.extend_from_slice(&v.y.to_le_bytes());
                buf.extend_from_slice(&v.z.to_le_bytes());
            }
            // Neighbor count
            buf.extend_from_slice(&(poly.neighbors.len() as u32).to_le_bytes());
            // Neighbors
            for n in &poly.neighbors {
                buf.extend_from_slice(&n.0.to_le_bytes());
            }
            // Cost
            buf.extend_from_slice(&poly.cost.to_le_bytes());
        }

        buf
    }

    /// Deserialize a 3D navmesh from binary data produced by [`NavMesh3D::to_bytes`].
    ///
    /// Returns `None` if the data is malformed or has an unsupported version.
    #[cfg_attr(feature = "logging", instrument)]
    #[must_use]
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        // Magic
        if data.len() < 8 {
            return None;
        }
        if &data[0..4] != b"RNV3" {
            return None;
        }
        let mut cursor = 4usize;

        // Version
        let version = read_u32_3d(&mut cursor, data)?;
        if version != 1 {
            return None;
        }

        // Polygon count
        let poly_count = read_u32_3d(&mut cursor, data)? as usize;

        let mut mesh = Self::new();
        for _ in 0..poly_count {
            let id = NavPolyId(read_u32_3d(&mut cursor, data)?);

            let vert_count = read_u32_3d(&mut cursor, data)? as usize;
            let mut vertices = Vec::with_capacity(vert_count);
            for _ in 0..vert_count {
                let x = read_f32_3d(&mut cursor, data)?;
                let y = read_f32_3d(&mut cursor, data)?;
                let z = read_f32_3d(&mut cursor, data)?;
                vertices.push(Vec3::new(x, y, z));
            }

            let neighbor_count = read_u32_3d(&mut cursor, data)? as usize;
            let mut neighbors = Vec::with_capacity(neighbor_count);
            for _ in 0..neighbor_count {
                neighbors.push(NavPolyId(read_u32_3d(&mut cursor, data)?));
            }

            let cost = read_f32_3d(&mut cursor, data)?;

            mesh.add_poly(NavPoly3D {
                id,
                vertices,
                neighbors,
                cost,
            });
        }

        Some(mesh)
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

    /// Pick a uniformly random point on the 3D navmesh surface.
    ///
    /// Uses area-weighted polygon selection, then uniform sampling within
    /// the selected polygon via barycentric coordinates.
    ///
    /// `rng_value` should be a random `f32` in `[0.0, 1.0)` for polygon selection.
    /// `u` and `v` should be random `f32`s in `[0.0, 1.0)` for point sampling.
    ///
    /// Returns `None` if the mesh is empty or has zero total area.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    #[must_use]
    pub fn random_point(&self, rng_value: f32, u: f32, v: f32) -> Option<Vec3> {
        if self.polys.is_empty() {
            return None;
        }

        let areas: Vec<f32> = self
            .polys
            .iter()
            .map(|p| polygon_area_3d(&p.vertices))
            .collect();
        let total_area: f32 = areas.iter().sum();
        if total_area < f32::EPSILON {
            return None;
        }

        let target = rng_value * total_area;
        let mut cumulative = 0.0f32;
        let mut selected = 0;
        for (i, &area) in areas.iter().enumerate() {
            cumulative += area;
            if cumulative >= target {
                selected = i;
                break;
            }
        }

        let poly = &self.polys[selected];
        Some(sample_point_in_convex_polygon_3d(&poly.vertices, u, v))
    }

    /// Sample the height (Y coordinate) at a given (X, Z) position.
    ///
    /// Projects the position vertically onto the navmesh surface.
    /// Returns the Y value at that point, or `None` if the position
    /// is not over any polygon.
    ///
    /// This is useful for ground-snapping agents to the navmesh surface.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    #[must_use]
    pub fn sample_height(&self, x: f32, z: f32) -> Option<f32> {
        for poly in &self.polys {
            if poly.vertices.len() < 3 {
                continue;
            }
            if contains_point_xz(&poly.vertices, x, z) {
                let normal = poly.normal();
                if normal.y.abs() < f32::EPSILON {
                    continue; // Vertical polygon, skip
                }
                // Plane equation: normal . (P - v0) = 0
                // Solve for y: y = v0.y - (normal.x*(x - v0.x) + normal.z*(z - v0.z)) / normal.y
                let v0 = poly.vertices[0];
                let y = v0.y - (normal.x * (x - v0.x) + normal.z * (z - v0.z)) / normal.y;
                return Some(y);
            }
        }
        None
    }

    /// Snap a 3D position to the navmesh surface (project Y onto the mesh).
    ///
    /// Returns the position with Y adjusted to match the navmesh surface,
    /// or `None` if the XZ position is not over any polygon.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    #[must_use]
    pub fn snap_to_surface(&self, position: Vec3) -> Option<Vec3> {
        self.sample_height(position.x, position.z)
            .map(|y| Vec3::new(position.x, y, position.z))
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
                let edge_cost = current_centroid.distance(neighbor_centroid) * neighbor.cost;
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

/// Area of a 3D polygon via cross-product magnitudes.
#[inline]
fn polygon_area_3d(vertices: &[Vec3]) -> f32 {
    if vertices.len() < 3 {
        return 0.0;
    }
    let mut area = 0.0f32;
    for i in 1..vertices.len() - 1 {
        let e1 = vertices[i] - vertices[0];
        let e2 = vertices[i + 1] - vertices[0];
        area += e1.cross(e2).length() * 0.5;
    }
    area
}

/// Sample a uniform random point in a 3D triangle using barycentric coordinates.
#[inline]
fn sample_triangle_3d(a: Vec3, b: Vec3, c: Vec3, u: f32, v: f32) -> Vec3 {
    let (s, t) = if u + v > 1.0 {
        (1.0 - u, 1.0 - v)
    } else {
        (u, v)
    };
    a + (b - a) * s + (c - a) * t
}

/// Sample a uniform random point in a convex 3D polygon using triangle fan + barycentric.
fn sample_point_in_convex_polygon_3d(vertices: &[Vec3], u: f32, v: f32) -> Vec3 {
    if vertices.len() < 3 {
        return vertices.first().copied().unwrap_or(Vec3::ZERO);
    }

    let n_tris = vertices.len() - 2;
    if n_tris == 1 {
        return sample_triangle_3d(vertices[0], vertices[1], vertices[2], u, v);
    }

    // Weight triangle selection by area
    let mut cum_areas = Vec::with_capacity(n_tris);
    let mut total = 0.0f32;
    for i in 0..n_tris {
        let e1 = vertices[i + 1] - vertices[0];
        let e2 = vertices[i + 2] - vertices[0];
        let area = e1.cross(e2).length() * 0.5;
        total += area;
        cum_areas.push(total);
    }

    if total < f32::EPSILON {
        return vertices[0];
    }

    let target = u * total;
    let mut tri_idx = 0;
    for (i, &cum) in cum_areas.iter().enumerate() {
        if cum >= target {
            tri_idx = i;
            break;
        }
    }

    sample_triangle_3d(
        vertices[0],
        vertices[tri_idx + 1],
        vertices[tri_idx + 2],
        u,
        v,
    )
}

/// Check if a point (x, z) is inside a polygon projected onto the XZ plane.
fn contains_point_xz(vertices: &[Vec3], x: f32, z: f32) -> bool {
    let n = vertices.len();
    if n < 3 {
        return false;
    }
    let mut sign = None;
    for i in 0..n {
        let j = (i + 1) % n;
        let ex = vertices[j].x - vertices[i].x;
        let ez = vertices[j].z - vertices[i].z;
        let px = x - vertices[i].x;
        let pz = z - vertices[i].z;
        let cross = ex * pz - ez * px;
        let s = cross >= 0.0;
        match sign {
            None => sign = Some(s),
            Some(prev) if prev != s => return false,
            _ => {}
        }
    }
    true
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

// ---------------------------------------------------------------------------
// Binary format helpers
// ---------------------------------------------------------------------------

/// Read a little-endian `u32` from `data` at `cursor`, advancing the cursor.
fn read_u32_3d(cursor: &mut usize, data: &[u8]) -> Option<u32> {
    if *cursor + 4 > data.len() {
        return None;
    }
    let val = u32::from_le_bytes([
        data[*cursor],
        data[*cursor + 1],
        data[*cursor + 2],
        data[*cursor + 3],
    ]);
    *cursor += 4;
    Some(val)
}

/// Read a little-endian `f32` from `data` at `cursor`, advancing the cursor.
fn read_f32_3d(cursor: &mut usize, data: &[u8]) -> Option<f32> {
    if *cursor + 4 > data.len() {
        return None;
    }
    let val = f32::from_le_bytes([
        data[*cursor],
        data[*cursor + 1],
        data[*cursor + 2],
        data[*cursor + 3],
    ]);
    *cursor += 4;
    Some(val)
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
            cost: 1.0,
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
            cost: 1.0,
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
            cost: 1.0,
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
            cost: 1.0,
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
            cost: 1.0,
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

    // --- Height / elevation query tests ---

    #[test]
    fn sample_height_flat() {
        let mut mesh = NavMesh3D::new();
        mesh.add_poly(NavPoly3D {
            id: NavPolyId(0),
            vertices: vec![
                Vec3::new(0.0, 5.0, 0.0),
                Vec3::new(10.0, 5.0, 0.0),
                Vec3::new(10.0, 5.0, 10.0),
                Vec3::new(0.0, 5.0, 10.0),
            ],
            neighbors: vec![],
            cost: 1.0,
        });
        let h = mesh.sample_height(5.0, 5.0);
        assert!(h.is_some());
        assert!((h.unwrap() - 5.0).abs() < 0.01);
    }

    #[test]
    fn sample_height_sloped() {
        let mut mesh = NavMesh3D::new();
        // Sloped polygon: Y increases with X
        mesh.add_poly(NavPoly3D {
            id: NavPolyId(0),
            vertices: vec![
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(10.0, 10.0, 0.0),
                Vec3::new(10.0, 10.0, 10.0),
                Vec3::new(0.0, 0.0, 10.0),
            ],
            neighbors: vec![],
            cost: 1.0,
        });
        let h = mesh.sample_height(5.0, 5.0);
        assert!(h.is_some());
        assert!((h.unwrap() - 5.0).abs() < 0.1);
    }

    #[test]
    fn sample_height_off_mesh() {
        let mesh = NavMesh3D::new();
        assert!(mesh.sample_height(5.0, 5.0).is_none());
    }

    #[test]
    fn snap_to_surface_flat() {
        let mut mesh = NavMesh3D::new();
        mesh.add_poly(NavPoly3D {
            id: NavPolyId(0),
            vertices: vec![
                Vec3::new(0.0, 3.0, 0.0),
                Vec3::new(10.0, 3.0, 0.0),
                Vec3::new(10.0, 3.0, 10.0),
                Vec3::new(0.0, 3.0, 10.0),
            ],
            neighbors: vec![],
            cost: 1.0,
        });
        let snapped = mesh.snap_to_surface(Vec3::new(5.0, 100.0, 5.0));
        assert!(snapped.is_some());
        let s = snapped.unwrap();
        assert!((s.x - 5.0).abs() < 0.01);
        assert!((s.y - 3.0).abs() < 0.01);
        assert!((s.z - 5.0).abs() < 0.01);
    }

    #[test]
    fn snap_to_surface_off_mesh() {
        let mesh = NavMesh3D::new();
        assert!(mesh.snap_to_surface(Vec3::new(5.0, 0.0, 5.0)).is_none());
    }

    // --- Random point 3D tests ---

    #[test]
    fn random_point_3d() {
        let mut mesh = NavMesh3D::new();
        mesh.add_poly(NavPoly3D {
            id: NavPolyId(0),
            vertices: vec![
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(10.0, 0.0, 0.0),
                Vec3::new(5.0, 0.0, 10.0),
            ],
            neighbors: vec![],
            cost: 1.0,
        });
        let point = mesh.random_point(0.5, 0.3, 0.3);
        assert!(point.is_some());
    }

    #[test]
    fn random_point_3d_empty() {
        let mesh = NavMesh3D::new();
        assert!(mesh.random_point(0.5, 0.5, 0.5).is_none());
    }

    #[test]
    fn random_point_3d_multiple_samples() {
        let mut mesh = NavMesh3D::new();
        mesh.add_poly(NavPoly3D {
            id: NavPolyId(0),
            vertices: vec![
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(10.0, 0.0, 0.0),
                Vec3::new(10.0, 0.0, 10.0),
                Vec3::new(0.0, 0.0, 10.0),
            ],
            neighbors: vec![],
            cost: 1.0,
        });
        for i in 0..10 {
            let rng = i as f32 / 10.0;
            let point = mesh.random_point(rng, 0.4, 0.2);
            assert!(point.is_some());
            let p = point.unwrap();
            // All Y should be 0 for flat mesh
            assert!(p.y.abs() < 0.01);
        }
    }

    // -----------------------------------------------------------------------
    // Binary serialization tests
    // -----------------------------------------------------------------------

    #[test]
    fn navmesh3d_bytes_roundtrip() {
        let mesh = make_ramp_mesh();
        let bytes = mesh.to_bytes();
        let loaded = NavMesh3D::from_bytes(&bytes);
        assert!(loaded.is_some());
        let loaded = loaded.expect("roundtrip should succeed");
        assert_eq!(loaded.poly_count(), mesh.poly_count());

        for (a, b) in mesh.polys().iter().zip(loaded.polys().iter()) {
            assert_eq!(a.id, b.id);
            assert_eq!(a.vertices.len(), b.vertices.len());
            for (va, vb) in a.vertices.iter().zip(b.vertices.iter()) {
                assert!((va.x - vb.x).abs() < f32::EPSILON);
                assert!((va.y - vb.y).abs() < f32::EPSILON);
                assert!((va.z - vb.z).abs() < f32::EPSILON);
            }
            assert_eq!(a.neighbors.len(), b.neighbors.len());
            for (na, nb) in a.neighbors.iter().zip(b.neighbors.iter()) {
                assert_eq!(na, nb);
            }
            assert!((a.cost - b.cost).abs() < f32::EPSILON);
        }
    }

    #[test]
    fn navmesh3d_bytes_empty() {
        let mesh = NavMesh3D::new();
        let bytes = mesh.to_bytes();
        let loaded = NavMesh3D::from_bytes(&bytes).expect("empty mesh roundtrip");
        assert_eq!(loaded.poly_count(), 0);
    }

    #[test]
    fn navmesh3d_bytes_invalid() {
        assert!(NavMesh3D::from_bytes(b"BAAD").is_none());
        assert!(NavMesh3D::from_bytes(&[]).is_none());
        assert!(NavMesh3D::from_bytes(b"RNV3").is_none()); // truncated — no version
    }

    #[test]
    fn navmesh3d_bytes_single_triangle() {
        let mut mesh = NavMesh3D::new();
        mesh.add_poly(NavPoly3D {
            id: NavPolyId(0),
            vertices: vec![
                Vec3::ZERO,
                Vec3::new(10.0, 0.0, 0.0),
                Vec3::new(5.0, 0.0, 10.0),
            ],
            neighbors: vec![],
            cost: 1.0,
        });
        let bytes = mesh.to_bytes();
        let loaded = NavMesh3D::from_bytes(&bytes).expect("single triangle roundtrip");
        assert_eq!(loaded.poly_count(), 1);
        let poly = loaded.get_poly(NavPolyId(0)).expect("poly 0");
        assert_eq!(poly.vertices.len(), 3);
        assert!(poly.neighbors.is_empty());
    }

    #[test]
    fn navmesh3d_bytes_wrong_magic() {
        // RNAV (2D magic) should not parse as 3D
        let mut data = b"RNAV".to_vec();
        data.extend_from_slice(&1u32.to_le_bytes());
        data.extend_from_slice(&0u32.to_le_bytes());
        assert!(NavMesh3D::from_bytes(&data).is_none());
    }
}
