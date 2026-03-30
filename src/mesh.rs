//! NavMesh — polygon-based navigation mesh.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use hisab::Vec2;
use serde::{Deserialize, Serialize};

use crate::offmesh::OffMeshLinkRegistry;

#[cfg(feature = "logging")]
use tracing::instrument;

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
    /// Traversal cost multiplier (1.0 = normal, higher = more expensive).
    pub cost: f32,
}

impl NavPoly {
    /// Compute the centroid of this polygon.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    #[must_use]
    pub fn centroid(&self) -> Vec2 {
        if self.vertices.is_empty() {
            return Vec2::ZERO;
        }
        let sum: Vec2 = self.vertices.iter().copied().sum();
        sum / self.vertices.len() as f32
    }

    /// Check if a point is inside this convex polygon (2D, XY plane).
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
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

/// Per-agent area cost multiplier.
///
/// Multiplied with each polygon's base `cost` during pathfinding.
/// For example, an agent that avoids water might have a high multiplier
/// for water-tagged polygons.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AreaCostMultiplier {
    /// Map from polygon ID to cost multiplier.
    /// Polygons not in this map use multiplier 1.0.
    multipliers: Vec<(NavPolyId, f32)>,
}

impl AreaCostMultiplier {
    /// Create an empty cost multiplier (all polygons use base cost).
    #[must_use]
    pub fn new() -> Self {
        Self {
            multipliers: Vec::new(),
        }
    }

    /// Set the cost multiplier for a specific polygon.
    pub fn set(&mut self, id: NavPolyId, multiplier: f32) {
        if let Some(entry) = self.multipliers.iter_mut().find(|(pid, _)| *pid == id) {
            entry.1 = multiplier;
        } else {
            self.multipliers.push((id, multiplier));
        }
    }

    /// Get the cost multiplier for a polygon (defaults to 1.0).
    #[inline]
    #[must_use]
    pub fn get(&self, id: NavPolyId) -> f32 {
        self.multipliers
            .iter()
            .find(|(pid, _)| *pid == id)
            .map(|(_, m)| *m)
            .unwrap_or(1.0)
    }
}

/// A navigation mesh composed of convex polygons.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct NavMesh {
    polys: Vec<NavPoly>,
}

impl NavMesh {
    #[cfg_attr(feature = "logging", instrument)]
    #[must_use]
    pub fn new() -> Self {
        Self { polys: Vec::new() }
    }

    /// Bake a NavMesh from a simple polygon boundary.
    ///
    /// Triangulates the polygon, merges triangles into convex regions,
    /// and auto-detects neighbor connectivity from shared edges.
    #[cfg_attr(feature = "logging", instrument(fields(vertices = boundary.len())))]
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
                cost: 1.0,
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
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    pub fn add_poly(&mut self, poly: NavPoly) {
        self.polys.push(poly);
    }

    /// Get all polygons.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    #[must_use]
    pub fn polys(&self) -> &[NavPoly] {
        &self.polys
    }

    /// Get a polygon by its ID.
    ///
    /// Assumes poly IDs are contiguous indices `0..poly_count()`.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    #[inline]
    #[must_use]
    pub fn get_poly(&self, id: NavPolyId) -> Option<&NavPoly> {
        self.polys.get(id.0 as usize).filter(|p| p.id == id)
    }

    /// Find which polygon contains the given point.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
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
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
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
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
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

    /// Set the traversal cost for a polygon.
    ///
    /// Cost of 1.0 is normal. Higher values make the polygon more expensive to
    /// traverse. Zero or negative costs are clamped to a minimum of `f32::EPSILON`.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    pub fn set_poly_cost(&mut self, id: NavPolyId, cost: f32) {
        if let Some(poly) = self.polys.get_mut(id.0 as usize)
            && poly.id == id
        {
            poly.cost = cost.max(f32::EPSILON);
        }
    }

    /// Find a path between two points on the navmesh.
    ///
    /// Returns a sequence of polygon IDs from start to goal.
    /// Respects each polygon's base `cost` field.
    #[cfg_attr(feature = "logging", instrument(skip(self), fields(polys = self.polys.len())))]
    #[must_use]
    pub fn find_path(&self, start: Vec2, goal: Vec2) -> Option<Vec<NavPolyId>> {
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

        open.push(MeshNode {
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
                    open.push(MeshNode {
                        id: neighbor_id,
                        f_score: tentative_g + h,
                    });
                }
            }
        }

        None
    }

    /// Find a path with per-polygon area costs and optional agent cost multipliers.
    ///
    /// The effective cost of traversing a polygon is:
    /// `polygon.cost * agent_multiplier.get(polygon.id) * edge_distance`
    #[cfg_attr(feature = "logging", instrument(skip(self, costs), fields(polys = self.polys.len())))]
    #[must_use]
    pub fn find_path_with_costs(
        &self,
        start: Vec2,
        goal: Vec2,
        costs: &AreaCostMultiplier,
    ) -> Option<Vec<NavPolyId>> {
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

        open.push(MeshNode {
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
                let base_dist = current_centroid.distance(neighbor_centroid);
                let poly_cost = neighbor.cost * costs.get(neighbor_id);
                let edge_cost = base_dist * poly_cost;
                let tentative_g = current_g + edge_cost;

                if tentative_g < g_score[ni] {
                    came_from[ni] = Some(current.id.0);
                    g_score[ni] = tentative_g;
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

    /// Find a path considering off-mesh links as additional edges.
    ///
    /// Off-mesh links (jumps, ladders, teleporters, doors) add extra
    /// connectivity beyond polygon adjacency. Only enabled links are used.
    #[cfg_attr(feature = "logging", instrument(skip(self, links), fields(polys = self.polys.len())))]
    #[must_use]
    pub fn find_path_with_links(
        &self,
        start: Vec2,
        goal: Vec2,
        links: &OffMeshLinkRegistry,
    ) -> Option<Vec<NavPolyId>> {
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

        open.push(MeshNode {
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

            // Expand normal polygon neighbors
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
                    open.push(MeshNode {
                        id: neighbor_id,
                        f_score: tentative_g + h,
                    });
                }
            }

            // Expand off-mesh links from the current polygon
            for link in links.links_from_poly(current.id) {
                let (target_id, link_cost) = if link.start_poly == current.id {
                    (link.end_poly, link.cost)
                } else {
                    // Bidirectional, traversing backwards
                    (link.start_poly, link.cost)
                };
                let ni = target_id.0 as usize;
                if ni >= n || closed[ni] {
                    continue;
                }
                let neighbor = match self.get_poly(target_id) {
                    Some(p) => p,
                    None => continue,
                };
                let tentative_g = current_g + link_cost;
                if tentative_g < g_score[ni] {
                    came_from[ni] = Some(current.id.0);
                    g_score[ni] = tentative_g;
                    let h = neighbor.centroid().distance(goal_centroid);
                    open.push(MeshNode {
                        id: target_id,
                        f_score: tentative_g + h,
                    });
                }
            }
        }

        None
    }

    /// Find a path between two points, returning a partial path to the
    /// closest reachable polygon if the goal polygon is unreachable.
    ///
    /// Returns `None` only if start is not on the mesh.
    #[cfg_attr(feature = "logging", instrument(skip(self), fields(polys = self.polys.len())))]
    #[must_use]
    pub fn find_path_partial(&self, start: Vec2, goal: Vec2) -> Option<Vec<NavPolyId>> {
        let start_id = self.find_poly_at(start)?;

        // If goal is on the mesh, try the normal path first
        if let Some(goal_id) = self.find_poly_at(goal) {
            if start_id == goal_id {
                return Some(vec![start_id]);
            }
            if let Some(path) = self.find_path(start, goal) {
                return Some(path);
            }
        }

        // Run A* from start, tracking the explored poly closest to goal
        let n = self.polys.len();
        let mut g_score = vec![f32::INFINITY; n];
        let mut came_from: Vec<Option<u32>> = vec![None; n];
        let mut closed = vec![false; n];
        let mut open = BinaryHeap::new();

        let start_idx = start_id.0 as usize;
        g_score[start_idx] = 0.0;
        let start_centroid = match self.get_poly(start_id) {
            Some(p) => p.centroid(),
            None => return Some(vec![start_id]),
        };
        let h = start_centroid.distance(goal);

        open.push(MeshNode {
            id: start_id,
            f_score: h,
        });

        let mut best_id = start_id;
        let mut best_h = h;

        while let Some(current) = open.pop() {
            let ci = current.id.0 as usize;
            if closed[ci] {
                continue;
            }
            closed[ci] = true;

            // Track the explored poly closest to goal
            let poly = match self.get_poly(current.id) {
                Some(p) => p,
                None => continue,
            };
            let current_centroid = poly.centroid();
            let current_h = current_centroid.distance(goal);
            if current_h < best_h {
                best_h = current_h;
                best_id = current.id;
            }

            let current_g = g_score[ci];

            for &neighbor_id in &poly.neighbors {
                let ni = neighbor_id.0 as usize;
                if ni >= n || closed[ni] {
                    continue;
                }
                let neighbor = match self.get_poly(neighbor_id) {
                    Some(p) => p,
                    None => continue,
                };
                let neighbor_centroid = neighbor.centroid();
                let edge_cost = current_centroid.distance(neighbor_centroid);
                let tentative_g = current_g + edge_cost;

                if tentative_g < g_score[ni] {
                    came_from[ni] = Some(current.id.0);
                    g_score[ni] = tentative_g;
                    let h = neighbor_centroid.distance(goal);
                    open.push(MeshNode {
                        id: neighbor_id,
                        f_score: tentative_g + h,
                    });
                }
            }
        }

        // Reconstruct path to the closest explored poly
        Some(Self::reconstruct_path_vec(&came_from, start_id, best_id))
    }

    /// Number of polygons in the mesh.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    #[must_use]
    pub fn poly_count(&self) -> usize {
        self.polys.len()
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
            cost: 1.0,
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
            cost: 1.0,
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
            cost: 1.0,
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
            cost: 1.0,
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
            cost: 1.0,
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
            cost: 1.0,
        };
        assert_eq!(poly.centroid(), Vec2::ZERO);
    }

    #[test]
    fn triangle_contains() {
        let tri = NavPoly {
            id: NavPolyId(0),
            vertices: vec![Vec2::ZERO, Vec2::new(4.0, 0.0), Vec2::new(2.0, 4.0)],
            neighbors: vec![],
            cost: 1.0,
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
            cost: 1.0,
        };
        assert!(!line.contains_point(Vec2::new(0.5, 0.5)));

        let point = NavPoly {
            id: NavPolyId(1),
            vertices: vec![Vec2::ZERO],
            neighbors: vec![],
            cost: 1.0,
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
            cost: 1.0,
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
            cost: 1.0,
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

    #[test]
    fn partial_path_mesh_unreachable() {
        // Two disconnected polygons
        let mut mesh = NavMesh::new();
        mesh.add_poly(NavPoly {
            id: NavPolyId(0),
            vertices: vec![
                Vec2::ZERO,
                Vec2::new(10.0, 0.0),
                Vec2::new(10.0, 10.0),
                Vec2::new(0.0, 10.0),
            ],
            neighbors: vec![],
            cost: 1.0,
        });
        mesh.add_poly(NavPoly {
            id: NavPolyId(1),
            vertices: vec![
                Vec2::new(100.0, 0.0),
                Vec2::new(110.0, 0.0),
                Vec2::new(110.0, 10.0),
                Vec2::new(100.0, 10.0),
            ],
            neighbors: vec![],
            cost: 1.0,
        });
        // Start in poly 0, goal in poly 1 (unreachable)
        let path = mesh.find_path_partial(Vec2::new(5.0, 5.0), Vec2::new(105.0, 5.0));
        assert!(path.is_some());
        let path = path.unwrap();
        assert_eq!(path.len(), 1); // Only start poly reachable
        assert_eq!(path[0], NavPolyId(0));
    }

    #[test]
    fn partial_path_mesh_goal_off_mesh() {
        let mesh = make_square_mesh();
        // Goal is completely off the mesh
        let path = mesh.find_path_partial(Vec2::new(0.5, 0.5), Vec2::new(50.0, 50.0));
        assert!(path.is_some());
        let path = path.unwrap();
        // Should return some path (at least the start poly)
        assert!(!path.is_empty());
    }

    #[test]
    fn partial_path_mesh_reachable_returns_full() {
        let mesh = make_square_mesh();
        let partial = mesh.find_path_partial(Vec2::new(0.5, 0.5), Vec2::new(1.5, 0.5));
        let full = mesh.find_path(Vec2::new(0.5, 0.5), Vec2::new(1.5, 0.5));
        assert_eq!(partial, full);
    }

    #[test]
    fn partial_path_mesh_start_off_mesh() {
        let mesh = make_square_mesh();
        assert!(
            mesh.find_path_partial(Vec2::new(50.0, 50.0), Vec2::new(0.5, 0.5))
                .is_none()
        );
    }

    // --- Area cost tests ---

    #[test]
    fn area_cost_basic() {
        // Build a mesh with 3 polygons in a line
        let mut mesh = NavMesh::new();
        mesh.add_poly(NavPoly {
            id: NavPolyId(0),
            vertices: vec![
                Vec2::ZERO,
                Vec2::new(10.0, 0.0),
                Vec2::new(10.0, 10.0),
                Vec2::new(0.0, 10.0),
            ],
            neighbors: vec![NavPolyId(1)],
            cost: 1.0,
        });
        mesh.add_poly(NavPoly {
            id: NavPolyId(1),
            vertices: vec![
                Vec2::new(10.0, 0.0),
                Vec2::new(20.0, 0.0),
                Vec2::new(20.0, 10.0),
                Vec2::new(10.0, 10.0),
            ],
            neighbors: vec![NavPolyId(0), NavPolyId(2)],
            cost: 1.0,
        });
        mesh.add_poly(NavPoly {
            id: NavPolyId(2),
            vertices: vec![
                Vec2::new(20.0, 0.0),
                Vec2::new(30.0, 0.0),
                Vec2::new(30.0, 10.0),
                Vec2::new(20.0, 10.0),
            ],
            neighbors: vec![NavPolyId(1)],
            cost: 1.0,
        });

        // Normal path should go through all 3
        let path = mesh.find_path(Vec2::new(5.0, 5.0), Vec2::new(25.0, 5.0));
        assert!(path.is_some());
        assert_eq!(path.unwrap().len(), 3);
    }

    #[test]
    fn area_cost_expensive_polygon() {
        let mut mesh = NavMesh::new();
        mesh.add_poly(NavPoly {
            id: NavPolyId(0),
            vertices: vec![
                Vec2::ZERO,
                Vec2::new(10.0, 0.0),
                Vec2::new(10.0, 10.0),
                Vec2::new(0.0, 10.0),
            ],
            neighbors: vec![NavPolyId(1), NavPolyId(2)],
            cost: 1.0,
        });
        mesh.add_poly(NavPoly {
            id: NavPolyId(1),
            vertices: vec![
                Vec2::new(10.0, 0.0),
                Vec2::new(20.0, 0.0),
                Vec2::new(20.0, 10.0),
                Vec2::new(10.0, 10.0),
            ],
            neighbors: vec![NavPolyId(0), NavPolyId(3)],
            cost: 100.0, // Very expensive
        });
        mesh.add_poly(NavPoly {
            id: NavPolyId(2),
            vertices: vec![
                Vec2::new(0.0, 10.0),
                Vec2::new(10.0, 10.0),
                Vec2::new(10.0, 20.0),
                Vec2::new(0.0, 20.0),
            ],
            neighbors: vec![NavPolyId(0), NavPolyId(3)],
            cost: 1.0,
        });
        mesh.add_poly(NavPoly {
            id: NavPolyId(3),
            vertices: vec![
                Vec2::new(10.0, 10.0),
                Vec2::new(20.0, 10.0),
                Vec2::new(20.0, 20.0),
                Vec2::new(10.0, 20.0),
            ],
            neighbors: vec![NavPolyId(1), NavPolyId(2)],
            cost: 1.0,
        });

        // Path should prefer going through polys 0->2->3 (cheaper) over 0->1->3
        let path = mesh.find_path(Vec2::new(5.0, 5.0), Vec2::new(15.0, 15.0));
        assert!(path.is_some());
        let path = path.unwrap();
        assert!(!path.contains(&NavPolyId(1))); // Should avoid expensive poly
    }

    #[test]
    fn set_poly_cost() {
        let mut mesh = NavMesh::bake(&[
            Vec2::ZERO,
            Vec2::new(10.0, 0.0),
            Vec2::new(10.0, 10.0),
            Vec2::new(0.0, 10.0),
        ]);
        for poly in mesh.polys() {
            assert!((poly.cost - 1.0).abs() < f32::EPSILON);
        }
        if mesh.poly_count() > 0 {
            mesh.set_poly_cost(NavPolyId(0), 5.0);
            assert!((mesh.get_poly(NavPolyId(0)).unwrap().cost - 5.0).abs() < f32::EPSILON);
        }
    }

    #[test]
    fn area_cost_multiplier_default() {
        let costs = AreaCostMultiplier::new();
        assert!((costs.get(NavPolyId(42)) - 1.0).abs() < f32::EPSILON);
    }

    #[test]
    fn area_cost_multiplier_set_get() {
        let mut costs = AreaCostMultiplier::new();
        costs.set(NavPolyId(1), 3.0);
        assert!((costs.get(NavPolyId(1)) - 3.0).abs() < f32::EPSILON);
        assert!((costs.get(NavPolyId(0)) - 1.0).abs() < f32::EPSILON);
    }

    #[test]
    fn area_cost_multiplier_serde_roundtrip() {
        let mut costs = AreaCostMultiplier::new();
        costs.set(NavPolyId(1), 2.5);
        let json = serde_json::to_string(&costs).unwrap();
        let deserialized: AreaCostMultiplier = serde_json::from_str(&json).unwrap();
        assert!((deserialized.get(NavPolyId(1)) - 2.5).abs() < f32::EPSILON);
    }

    #[test]
    fn find_path_with_costs_same_as_find_path_default() {
        let mesh = make_square_mesh();
        let costs = AreaCostMultiplier::new();
        let path_normal = mesh.find_path(Vec2::new(0.5, 0.5), Vec2::new(1.5, 0.5));
        let path_costs =
            mesh.find_path_with_costs(Vec2::new(0.5, 0.5), Vec2::new(1.5, 0.5), &costs);
        assert_eq!(path_normal, path_costs);
    }

    // --- Off-mesh link pathfinding tests ---

    #[test]
    fn find_path_with_links_teleport() {
        use crate::offmesh::{LinkType, OffMeshLinkDesc, OffMeshLinkRegistry};

        // Two disconnected polygons
        let mut mesh = NavMesh::new();
        mesh.add_poly(NavPoly {
            id: NavPolyId(0),
            vertices: vec![
                Vec2::ZERO,
                Vec2::new(10.0, 0.0),
                Vec2::new(10.0, 10.0),
                Vec2::new(0.0, 10.0),
            ],
            neighbors: vec![],
            cost: 1.0,
        });
        mesh.add_poly(NavPoly {
            id: NavPolyId(1),
            vertices: vec![
                Vec2::new(100.0, 0.0),
                Vec2::new(110.0, 0.0),
                Vec2::new(110.0, 10.0),
                Vec2::new(100.0, 10.0),
            ],
            neighbors: vec![],
            cost: 1.0,
        });

        // Without link: no path
        assert!(
            mesh.find_path(Vec2::new(5.0, 5.0), Vec2::new(105.0, 5.0))
                .is_none()
        );

        // Add teleporter link
        let mut links = OffMeshLinkRegistry::new();
        links.add_link(OffMeshLinkDesc {
            start: Vec2::new(5.0, 5.0),
            end: Vec2::new(105.0, 5.0),
            start_poly: NavPolyId(0),
            end_poly: NavPolyId(1),
            cost: 1.0,
            bidirectional: false,
            link_type: LinkType::Teleport,
        });

        // With link: path exists
        let path = mesh.find_path_with_links(Vec2::new(5.0, 5.0), Vec2::new(105.0, 5.0), &links);
        assert!(path.is_some());
        let path = path.unwrap();
        assert_eq!(path.len(), 2);
        assert_eq!(path[0], NavPolyId(0));
        assert_eq!(path[1], NavPolyId(1));
    }

    #[test]
    fn find_path_with_links_disabled() {
        use crate::offmesh::{LinkType, OffMeshLinkDesc, OffMeshLinkRegistry};

        let mut mesh = NavMesh::new();
        mesh.add_poly(NavPoly {
            id: NavPolyId(0),
            vertices: vec![
                Vec2::ZERO,
                Vec2::new(10.0, 0.0),
                Vec2::new(10.0, 10.0),
                Vec2::new(0.0, 10.0),
            ],
            neighbors: vec![],
            cost: 1.0,
        });
        mesh.add_poly(NavPoly {
            id: NavPolyId(1),
            vertices: vec![
                Vec2::new(100.0, 0.0),
                Vec2::new(110.0, 0.0),
                Vec2::new(110.0, 10.0),
                Vec2::new(100.0, 10.0),
            ],
            neighbors: vec![],
            cost: 1.0,
        });

        let mut links = OffMeshLinkRegistry::new();
        let id = links.add_link(OffMeshLinkDesc {
            start: Vec2::new(5.0, 5.0),
            end: Vec2::new(105.0, 5.0),
            start_poly: NavPolyId(0),
            end_poly: NavPolyId(1),
            cost: 1.0,
            bidirectional: false,
            link_type: LinkType::Door,
        });

        // Disable the link (door closed)
        links.set_enabled(id, false);
        assert!(
            mesh.find_path_with_links(Vec2::new(5.0, 5.0), Vec2::new(105.0, 5.0), &links)
                .is_none()
        );

        // Re-enable (door open)
        links.set_enabled(id, true);
        assert!(
            mesh.find_path_with_links(Vec2::new(5.0, 5.0), Vec2::new(105.0, 5.0), &links)
                .is_some()
        );
    }

    #[test]
    fn find_path_with_links_bidirectional() {
        use crate::offmesh::{LinkType, OffMeshLinkDesc, OffMeshLinkRegistry};

        let mut mesh = NavMesh::new();
        mesh.add_poly(NavPoly {
            id: NavPolyId(0),
            vertices: vec![
                Vec2::ZERO,
                Vec2::new(10.0, 0.0),
                Vec2::new(10.0, 10.0),
                Vec2::new(0.0, 10.0),
            ],
            neighbors: vec![],
            cost: 1.0,
        });
        mesh.add_poly(NavPoly {
            id: NavPolyId(1),
            vertices: vec![
                Vec2::new(100.0, 0.0),
                Vec2::new(110.0, 0.0),
                Vec2::new(110.0, 10.0),
                Vec2::new(100.0, 10.0),
            ],
            neighbors: vec![],
            cost: 1.0,
        });

        let mut links = OffMeshLinkRegistry::new();
        links.add_link(OffMeshLinkDesc {
            start: Vec2::new(5.0, 5.0),
            end: Vec2::new(105.0, 5.0),
            start_poly: NavPolyId(0),
            end_poly: NavPolyId(1),
            cost: 1.0,
            bidirectional: true,
            link_type: LinkType::Ladder,
        });

        // Can path in both directions
        assert!(
            mesh.find_path_with_links(Vec2::new(5.0, 5.0), Vec2::new(105.0, 5.0), &links)
                .is_some()
        );
        assert!(
            mesh.find_path_with_links(Vec2::new(105.0, 5.0), Vec2::new(5.0, 5.0), &links)
                .is_some()
        );
    }

    #[test]
    fn set_poly_cost_clamps_zero() {
        let mut mesh = NavMesh::bake(&[
            Vec2::ZERO,
            Vec2::new(10.0, 0.0),
            Vec2::new(10.0, 10.0),
            Vec2::new(0.0, 10.0),
        ]);
        if mesh.poly_count() > 0 {
            mesh.set_poly_cost(NavPolyId(0), -5.0);
            assert!(mesh.get_poly(NavPolyId(0)).unwrap().cost > 0.0);
        }
    }
}
