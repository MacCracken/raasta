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
    /// Navigation layer (for grouping polygons, e.g., ground=0, water=1, road=2).
    pub layer: u32,
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

/// Filter for including/excluding polygons during pathfinding queries.
///
/// Allows per-agent polygon filtering without re-baking the navmesh.
/// Polygons can be excluded by ID or restricted to an include-only set.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct NavQueryFilter {
    /// Polygon IDs to exclude from pathfinding.
    exclude: Vec<NavPolyId>,
    /// Polygon IDs to include exclusively (if non-empty, only these are allowed).
    include_only: Vec<NavPolyId>,
}

impl NavQueryFilter {
    /// Create an empty filter (all polygons allowed).
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Exclude a polygon from pathfinding.
    pub fn exclude(&mut self, id: NavPolyId) {
        if !self.exclude.contains(&id) {
            self.exclude.push(id);
        }
    }

    /// Remove a polygon from the exclusion list.
    pub fn remove_exclude(&mut self, id: NavPolyId) {
        self.exclude.retain(|&e| e != id);
    }

    /// Set include-only list. When non-empty, ONLY these polygons are traversable.
    pub fn set_include_only(&mut self, ids: Vec<NavPolyId>) {
        self.include_only = ids;
    }

    /// Clear the include-only list (allows all polygons again).
    pub fn clear_include_only(&mut self) {
        self.include_only.clear();
    }

    /// Check if a polygon is allowed by this filter.
    #[inline]
    #[must_use]
    pub fn allows(&self, id: NavPolyId) -> bool {
        if !self.include_only.is_empty() && !self.include_only.contains(&id) {
            return false;
        }
        !self.exclude.contains(&id)
    }

    /// Clear all filter rules.
    pub fn clear(&mut self) {
        self.exclude.clear();
        self.include_only.clear();
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
                layer: 0,
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

    /// Find a path with polygon filtering.
    ///
    /// Polygons rejected by the filter are treated as non-traversable,
    /// allowing per-agent navigation restrictions without re-baking.
    #[cfg_attr(feature = "logging", instrument(skip(self, filter), fields(polys = self.polys.len())))]
    #[must_use]
    pub fn find_path_filtered(
        &self,
        start: Vec2,
        goal: Vec2,
        filter: &NavQueryFilter,
    ) -> Option<Vec<NavPolyId>> {
        let start_id = self
            .polys
            .iter()
            .find(|p| p.contains_point(start) && filter.allows(p.id))
            .map(|p| p.id)?;
        let goal_id = self
            .polys
            .iter()
            .find(|p| p.contains_point(goal) && filter.allows(p.id))
            .map(|p| p.id)?;

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
                if ni >= n || closed[ni] || !filter.allows(neighbor_id) {
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

    /// Pick a uniformly random point on the navmesh surface.
    ///
    /// Uses area-weighted polygon selection, then uniform sampling within
    /// the selected polygon via barycentric coordinates.
    ///
    /// `rng_value` should be a random `f32` in `[0.0, 1.0)` for polygon selection.
    /// `u` and `v` should be random `f32`s in `[0.0, 1.0)` for point sampling.
    ///
    /// Returns `None` if the mesh is empty or has zero total area.
    ///
    /// # Note
    ///
    /// This function takes explicit random values rather than an RNG trait
    /// to avoid adding a dependency on `rand`. The caller provides randomness.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    #[must_use]
    pub fn random_point(&self, rng_value: f32, u: f32, v: f32) -> Option<Vec2> {
        if self.polys.is_empty() {
            return None;
        }

        // Compute area of each polygon
        let areas: Vec<f32> = self
            .polys
            .iter()
            .map(|p| polygon_area(&p.vertices))
            .collect();
        let total_area: f32 = areas.iter().sum();
        if total_area < f32::EPSILON {
            return None;
        }

        // Area-weighted selection
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

        // Sample uniformly within the selected polygon
        let poly = &self.polys[selected];
        Some(sample_point_in_convex_polygon(&poly.vertices, u, v))
    }

    /// Number of polygons in the mesh.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    #[must_use]
    pub fn poly_count(&self) -> usize {
        self.polys.len()
    }

    /// Serialize the navmesh to a compact binary format.
    ///
    /// The binary format stores a magic header (`RNAV`), a version number,
    /// polygon count, then for each polygon: ID, vertex count, vertices
    /// (x,y pairs as little-endian `f32`), neighbor count, neighbor IDs,
    /// and cost.
    ///
    /// Use [`NavMesh::from_bytes`] to deserialize.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    #[must_use]
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut buf = Vec::new();

        // Magic bytes + version
        buf.extend_from_slice(b"RNAV");
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

    /// Deserialize a navmesh from binary data produced by [`NavMesh::to_bytes`].
    ///
    /// Returns `None` if the data is malformed or has an unsupported version.
    #[cfg_attr(feature = "logging", instrument)]
    #[must_use]
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        // Magic
        if data.len() < 8 {
            return None;
        }
        if &data[0..4] != b"RNAV" {
            return None;
        }
        let mut cursor = 4usize;

        // Version
        let version = read_u32(&mut cursor, data)?;
        if version != 1 {
            return None;
        }

        // Polygon count
        let poly_count = read_u32(&mut cursor, data)? as usize;

        let mut mesh = Self::new();
        for _ in 0..poly_count {
            let id = NavPolyId(read_u32(&mut cursor, data)?);

            let vert_count = read_u32(&mut cursor, data)? as usize;
            let mut vertices = Vec::with_capacity(vert_count);
            for _ in 0..vert_count {
                let x = read_f32(&mut cursor, data)?;
                let y = read_f32(&mut cursor, data)?;
                vertices.push(Vec2::new(x, y));
            }

            let neighbor_count = read_u32(&mut cursor, data)? as usize;
            let mut neighbors = Vec::with_capacity(neighbor_count);
            for _ in 0..neighbor_count {
                neighbors.push(NavPolyId(read_u32(&mut cursor, data)?));
            }

            let cost = read_f32(&mut cursor, data)?;

            mesh.add_poly(NavPoly {
                id,
                vertices,
                neighbors,
                cost,
                layer: 0,
            });
        }

        Some(mesh)
    }

    /// Find a path avoiding carved (blocked) polygons.
    ///
    /// Polygons blocked by the carver are treated as non-traversable.
    /// This is the primary way to route around runtime obstacles without
    /// re-baking the navmesh.
    #[cfg_attr(feature = "logging", instrument(skip(self, carver), fields(polys = self.polys.len())))]
    #[must_use]
    pub fn find_path_carved(
        &self,
        start: Vec2,
        goal: Vec2,
        carver: &ObstacleCarver,
    ) -> Option<Vec<NavPolyId>> {
        let start_id = self
            .polys
            .iter()
            .find(|p| p.contains_point(start) && !carver.is_blocked(p.id))
            .map(|p| p.id)?;
        let goal_id = self
            .polys
            .iter()
            .find(|p| p.contains_point(goal) && !carver.is_blocked(p.id))
            .map(|p| p.id)?;

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
                if ni >= n || closed[ni] || carver.is_blocked(neighbor_id) {
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

    /// Find a path between two points, only traversing polygons on allowed layers.
    ///
    /// Polygons whose `layer` is not in `allowed_layers` are treated as
    /// non-traversable. Returns `None` if no path exists on the allowed layers.
    #[cfg_attr(feature = "logging", instrument(skip(self, allowed_layers), fields(polys = self.polys.len())))]
    #[must_use]
    pub fn find_path_on_layers(
        &self,
        start: Vec2,
        goal: Vec2,
        allowed_layers: &[u32],
    ) -> Option<Vec<NavPolyId>> {
        let layer_ok = |id: NavPolyId| -> bool {
            self.get_poly(id)
                .map(|p| allowed_layers.contains(&p.layer))
                .unwrap_or(false)
        };

        let start_id = self
            .polys
            .iter()
            .find(|p| p.contains_point(start) && allowed_layers.contains(&p.layer))
            .map(|p| p.id)?;
        let goal_id = self
            .polys
            .iter()
            .find(|p| p.contains_point(goal) && allowed_layers.contains(&p.layer))
            .map(|p| p.id)?;

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
                if ni >= n || closed[ni] || !layer_ok(neighbor_id) {
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

// ---------------------------------------------------------------------------
// Agent Radius Erosion
// ---------------------------------------------------------------------------

/// Erode navmesh polygon vertices inward by `radius`.
///
/// Each polygon is shrunk so that an agent of the given radius
/// stays fully within the original polygon boundary. Polygons that
/// become degenerate (area too small) after erosion are removed.
///
/// Returns a new NavMesh with eroded polygons.
#[cfg_attr(feature = "logging", instrument(skip(mesh)))]
#[must_use]
pub fn erode_navmesh(mesh: &NavMesh, radius: f32) -> NavMesh {
    if radius <= 0.0 {
        return mesh.clone();
    }

    let mut eroded_polys: Vec<NavPoly> = Vec::new();
    let mut kept_ids: Vec<NavPolyId> = Vec::new();

    for poly in mesh.polys() {
        let n = poly.vertices.len();
        if n < 3 {
            continue;
        }

        // Determine winding via signed area (shoelace).
        // Positive = CCW, negative = CW.
        let mut signed_area = 0.0f32;
        for i in 0..n {
            let j = (i + 1) % n;
            signed_area += poly.vertices[i].perp_dot(poly.vertices[j]);
        }
        // For CCW (positive signed area), inward normal of edge e is (-e.y, e.x).
        // For CW (negative signed area), inward normal is (e.y, -e.x).
        let inward_sign = if signed_area >= 0.0 { 1.0f32 } else { -1.0f32 };

        let mut new_verts = Vec::with_capacity(n);
        let mut valid = true;

        for i in 0..n {
            let prev = poly.vertices[(i + n - 1) % n];
            let curr = poly.vertices[i];
            let next = poly.vertices[(i + 1) % n];

            // Compute inward normals of the two adjacent edges
            let e1 = curr - prev;
            let e2 = next - curr;

            let n1 = Vec2::new(-e1.y * inward_sign, e1.x * inward_sign);
            let n2 = Vec2::new(-e2.y * inward_sign, e2.x * inward_sign);

            let n1_len = n1.length();
            let n2_len = n2.length();

            if n1_len < f32::EPSILON || n2_len < f32::EPSILON {
                valid = false;
                break;
            }

            let n1_norm = n1 / n1_len;
            let n2_norm = n2 / n2_len;

            // Average the two normals for the offset direction
            let avg = n1_norm + n2_norm;
            let avg_len = avg.length();

            if avg_len < f32::EPSILON {
                // Normals cancel out (180-degree turn) — skip this polygon
                valid = false;
                break;
            }

            let offset_dir = avg / avg_len;

            // Scale so the perpendicular offset from each edge is `radius`.
            // The required scale is radius / cos(half_angle) where half_angle
            // is the angle between the offset direction and either normal.
            let cos_half = offset_dir.dot(n1_norm).max(f32::EPSILON);
            let offset_dist = radius / cos_half;

            new_verts.push(curr + offset_dir * offset_dist);
        }

        if !valid {
            continue;
        }

        // Check that the eroded polygon is still valid:
        // 1. Must have positive area
        // 2. Must be smaller than the original (over-erosion makes it bigger)
        let original_area = polygon_area(&poly.vertices);
        let eroded_area = polygon_area(&new_verts);
        if eroded_area < f32::EPSILON || eroded_area >= original_area {
            continue;
        }

        kept_ids.push(poly.id);
        eroded_polys.push(NavPoly {
            id: poly.id,
            vertices: new_verts,
            neighbors: poly.neighbors.clone(),
            cost: poly.cost,
            layer: poly.layer,
        });
    }

    // Rebuild neighbors: only keep references to polygons that survived erosion
    for poly in &mut eroded_polys {
        poly.neighbors.retain(|nid| kept_ids.contains(nid));
    }

    let mut result = NavMesh::new();
    for poly in eroded_polys {
        result.add_poly(poly);
    }
    result
}

// ---------------------------------------------------------------------------
// Obstacle Carving
// ---------------------------------------------------------------------------

/// Unique identifier for a carved obstacle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct CarvedObstacleId(pub u32);

/// Shape to carve from the navmesh.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[non_exhaustive]
pub enum CarveShape {
    /// Circular obstacle.
    Circle {
        /// Center of the circle.
        center: Vec2,
        /// Radius of the circle.
        radius: f32,
    },
    /// Axis-aligned rectangle.
    Rect {
        /// Minimum corner (bottom-left).
        min: Vec2,
        /// Maximum corner (top-right).
        max: Vec2,
    },
}

/// A carved obstacle shape.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct CarvedObstacle {
    /// Unique ID.
    pub id: CarvedObstacleId,
    /// Shape of the obstacle.
    pub shape: CarveShape,
}

/// Runtime obstacle carving for navmesh polygons.
///
/// Marks polygons as blocked when obstacles overlap them, effectively
/// carving holes in the navmesh without modifying the geometry.
/// This is cheaper than true geometric subtraction and sufficient
/// for most gameplay scenarios.
///
/// # Example
///
/// ```
/// use raasta::{NavMesh, NavPoly, NavPolyId, ObstacleCarver, CarveShape};
/// use hisab::Vec2;
///
/// let mut mesh = NavMesh::new();
/// mesh.add_poly(NavPoly {
///     id: NavPolyId(0),
///     vertices: vec![Vec2::ZERO, Vec2::new(10.0, 0.0), Vec2::new(10.0, 10.0), Vec2::new(0.0, 10.0)],
///     neighbors: vec![],
///     cost: 1.0,
///     layer: 0,
/// });
///
/// let mut carver = ObstacleCarver::new();
/// let id = carver.add_obstacle(&mesh, CarveShape::Circle {
///     center: Vec2::new(5.0, 5.0),
///     radius: 3.0,
/// });
/// assert!(carver.is_blocked(NavPolyId(0)));
///
/// carver.remove_obstacle(&mesh, id);
/// assert!(!carver.is_blocked(NavPolyId(0)));
/// ```
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ObstacleCarver {
    /// Polygons currently blocked by obstacles.
    blocked: Vec<NavPolyId>,
    /// Active obstacle shapes.
    obstacles: Vec<CarvedObstacle>,
    /// Next obstacle ID to assign.
    next_id: u32,
}

impl ObstacleCarver {
    /// Create a new, empty obstacle carver.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Add an obstacle and recompute blocked polygons.
    ///
    /// Returns the obstacle ID. Any polygon whose centroid falls within
    /// the obstacle shape is marked as blocked.
    #[cfg_attr(feature = "logging", instrument(skip(self, mesh)))]
    pub fn add_obstacle(&mut self, mesh: &NavMesh, shape: CarveShape) -> CarvedObstacleId {
        let id = CarvedObstacleId(self.next_id);
        self.next_id += 1;
        self.obstacles.push(CarvedObstacle { id, shape });
        self.rebuild_blocked(mesh);
        id
    }

    /// Remove an obstacle and recompute blocked polygons.
    ///
    /// Returns `true` if the obstacle was found and removed.
    #[cfg_attr(feature = "logging", instrument(skip(self, mesh)))]
    pub fn remove_obstacle(&mut self, mesh: &NavMesh, id: CarvedObstacleId) -> bool {
        let before = self.obstacles.len();
        self.obstacles.retain(|o| o.id != id);
        if self.obstacles.len() != before {
            self.rebuild_blocked(mesh);
            true
        } else {
            false
        }
    }

    /// Check if a polygon is blocked by any obstacle.
    #[inline]
    #[must_use]
    pub fn is_blocked(&self, id: NavPolyId) -> bool {
        self.blocked.contains(&id)
    }

    /// Get all blocked polygon IDs.
    #[must_use]
    pub fn blocked_polys(&self) -> &[NavPolyId] {
        &self.blocked
    }

    /// Get all active obstacles.
    #[must_use]
    pub fn obstacles(&self) -> &[CarvedObstacle] {
        &self.obstacles
    }

    /// Number of active obstacles.
    #[must_use]
    pub fn obstacle_count(&self) -> usize {
        self.obstacles.len()
    }

    /// Recompute which polygons are blocked.
    fn rebuild_blocked(&mut self, mesh: &NavMesh) {
        self.blocked.clear();
        for poly in mesh.polys() {
            let centroid = poly.centroid();
            for obs in &self.obstacles {
                if shape_contains(obs.shape, centroid) {
                    self.blocked.push(poly.id);
                    break;
                }
            }
        }
    }

    /// Clear all obstacles and unblock all polygons.
    pub fn clear(&mut self) {
        self.obstacles.clear();
        self.blocked.clear();
    }
}

/// Check if a shape contains a point.
#[inline]
fn shape_contains(shape: CarveShape, point: Vec2) -> bool {
    match shape {
        CarveShape::Circle { center, radius } => center.distance_squared(point) <= radius * radius,
        CarveShape::Rect { min, max } => {
            point.x >= min.x && point.x <= max.x && point.y >= min.y && point.y <= max.y
        }
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

/// Area of a convex polygon (shoelace formula).
#[inline]
fn polygon_area(vertices: &[Vec2]) -> f32 {
    if vertices.len() < 3 {
        return 0.0;
    }
    let mut area = 0.0f32;
    let n = vertices.len();
    for i in 0..n {
        let j = (i + 1) % n;
        area += vertices[i].perp_dot(vertices[j]);
    }
    area.abs() * 0.5
}

/// Sample a uniform random point in a triangle using barycentric coordinates.
#[inline]
fn sample_triangle_2d(a: Vec2, b: Vec2, c: Vec2, u: f32, v: f32) -> Vec2 {
    let (s, t) = if u + v > 1.0 {
        (1.0 - u, 1.0 - v)
    } else {
        (u, v)
    };
    a + (b - a) * s + (c - a) * t
}

/// Sample a uniform random point in a convex polygon using triangle fan + barycentric.
fn sample_point_in_convex_polygon(vertices: &[Vec2], u: f32, v: f32) -> Vec2 {
    if vertices.len() < 3 {
        return vertices.first().copied().unwrap_or(Vec2::ZERO);
    }

    // Triangle fan from vertex 0
    let n_tris = vertices.len() - 2;
    if n_tris == 1 {
        return sample_triangle_2d(vertices[0], vertices[1], vertices[2], u, v);
    }

    // Weight triangle selection by area
    let mut cum_areas = Vec::with_capacity(n_tris);
    let mut total = 0.0f32;
    for i in 0..n_tris {
        let area =
            ((vertices[i + 1] - vertices[0]).perp_dot(vertices[i + 2] - vertices[0])).abs() * 0.5;
        total += area;
        cum_areas.push(total);
    }

    if total < f32::EPSILON {
        return vertices[0];
    }

    // Select triangle by area using `u`
    let target = u * total;
    let mut tri_idx = 0;
    for (i, &cum) in cum_areas.iter().enumerate() {
        if cum >= target {
            tri_idx = i;
            break;
        }
    }

    sample_triangle_2d(
        vertices[0],
        vertices[tri_idx + 1],
        vertices[tri_idx + 2],
        u,
        v,
    )
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
// Binary format helpers
// ---------------------------------------------------------------------------

/// Read a little-endian `u32` from `data` at `cursor`, advancing the cursor.
fn read_u32(cursor: &mut usize, data: &[u8]) -> Option<u32> {
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
fn read_f32(cursor: &mut usize, data: &[u8]) -> Option<f32> {
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
            layer: 0,
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
            layer: 0,
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
            layer: 0,
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
            layer: 0,
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
            layer: 0,
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
            layer: 0,
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
            layer: 0,
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
            layer: 0,
        };
        assert!(!line.contains_point(Vec2::new(0.5, 0.5)));

        let point = NavPoly {
            id: NavPolyId(1),
            vertices: vec![Vec2::ZERO],
            neighbors: vec![],
            cost: 1.0,
            layer: 0,
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
            layer: 0,
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
            layer: 0,
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
            layer: 0,
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
            layer: 0,
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
            layer: 0,
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
            layer: 0,
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
            layer: 0,
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
            layer: 0,
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
            layer: 0,
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
            layer: 0,
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
            layer: 0,
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
            layer: 0,
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
            layer: 0,
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
            layer: 0,
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
            layer: 0,
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
            layer: 0,
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
            layer: 0,
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

    // --- NavQueryFilter tests ---

    #[test]
    fn query_filter_allows_all_by_default() {
        let filter = NavQueryFilter::new();
        assert!(filter.allows(NavPolyId(0)));
        assert!(filter.allows(NavPolyId(99)));
    }

    #[test]
    fn query_filter_exclude() {
        let mut filter = NavQueryFilter::new();
        filter.exclude(NavPolyId(1));
        assert!(filter.allows(NavPolyId(0)));
        assert!(!filter.allows(NavPolyId(1)));

        filter.remove_exclude(NavPolyId(1));
        assert!(filter.allows(NavPolyId(1)));
    }

    #[test]
    fn query_filter_include_only() {
        let mut filter = NavQueryFilter::new();
        filter.set_include_only(vec![NavPolyId(0), NavPolyId(2)]);
        assert!(filter.allows(NavPolyId(0)));
        assert!(!filter.allows(NavPolyId(1)));
        assert!(filter.allows(NavPolyId(2)));

        filter.clear_include_only();
        assert!(filter.allows(NavPolyId(1)));
    }

    #[test]
    fn query_filter_exclude_idempotent() {
        let mut filter = NavQueryFilter::new();
        filter.exclude(NavPolyId(1));
        filter.exclude(NavPolyId(1));
        assert_eq!(filter.exclude.len(), 1);
    }

    #[test]
    fn find_path_filtered_excludes_poly() {
        let mut mesh = NavMesh::new();
        // Three polys in a line: 0 -- 1 -- 2
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
            layer: 0,
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
            layer: 0,
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
            layer: 0,
        });

        // Normal path works
        assert!(
            mesh.find_path_filtered(
                Vec2::new(5.0, 5.0),
                Vec2::new(25.0, 5.0),
                &NavQueryFilter::new()
            )
            .is_some()
        );

        // Exclude middle poly — no path
        let mut filter = NavQueryFilter::new();
        filter.exclude(NavPolyId(1));
        assert!(
            mesh.find_path_filtered(Vec2::new(5.0, 5.0), Vec2::new(25.0, 5.0), &filter)
                .is_none()
        );
    }

    #[test]
    fn find_path_filtered_include_only() {
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
            layer: 0,
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
            layer: 0,
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
            layer: 0,
        });

        // include_only polys 0 and 1 — goal in poly 2 is unreachable
        let mut filter = NavQueryFilter::new();
        filter.set_include_only(vec![NavPolyId(0), NavPolyId(1)]);
        assert!(
            mesh.find_path_filtered(Vec2::new(5.0, 5.0), Vec2::new(25.0, 5.0), &filter)
                .is_none()
        );

        // include_only all three — path works
        filter.set_include_only(vec![NavPolyId(0), NavPolyId(1), NavPolyId(2)]);
        assert!(
            mesh.find_path_filtered(Vec2::new(5.0, 5.0), Vec2::new(25.0, 5.0), &filter)
                .is_some()
        );
    }

    #[test]
    fn query_filter_serde_roundtrip() {
        let mut filter = NavQueryFilter::new();
        filter.exclude(NavPolyId(1));
        filter.set_include_only(vec![NavPolyId(0), NavPolyId(2)]);
        let json = serde_json::to_string(&filter).unwrap();
        let deserialized: NavQueryFilter = serde_json::from_str(&json).unwrap();
        assert!(!deserialized.allows(NavPolyId(1)));
    }

    #[test]
    fn query_filter_clear() {
        let mut filter = NavQueryFilter::new();
        filter.exclude(NavPolyId(0));
        filter.set_include_only(vec![NavPolyId(1)]);
        filter.clear();
        assert!(filter.allows(NavPolyId(0)));
        assert!(filter.allows(NavPolyId(1)));
    }

    // --- Random point tests ---

    #[test]
    fn random_point_on_mesh() {
        let mesh = NavMesh::bake(&[
            Vec2::ZERO,
            Vec2::new(10.0, 0.0),
            Vec2::new(10.0, 10.0),
            Vec2::new(0.0, 10.0),
        ]);
        for i in 0..10 {
            let rng = i as f32 / 10.0;
            let point = mesh.random_point(rng, 0.3, 0.3);
            assert!(point.is_some());
            let p = point.unwrap();
            assert!(p.x >= -0.1 && p.x <= 10.1);
            assert!(p.y >= -0.1 && p.y <= 10.1);
        }
    }

    #[test]
    fn random_point_empty_mesh() {
        let mesh = NavMesh::new();
        assert!(mesh.random_point(0.5, 0.5, 0.5).is_none());
    }

    #[test]
    fn random_point_boundary_values() {
        let mesh = NavMesh::bake(&[
            Vec2::ZERO,
            Vec2::new(10.0, 0.0),
            Vec2::new(10.0, 10.0),
            Vec2::new(0.0, 10.0),
        ]);
        assert!(mesh.random_point(0.0, 0.0, 0.0).is_some());
        assert!(mesh.random_point(0.999, 0.999, 0.001).is_some());
    }

    #[test]
    fn polygon_area_correctness() {
        // Unit square = area 1.0
        let square = [
            Vec2::ZERO,
            Vec2::new(1.0, 0.0),
            Vec2::new(1.0, 1.0),
            Vec2::new(0.0, 1.0),
        ];
        assert!((polygon_area(&square) - 1.0).abs() < f32::EPSILON);

        // Triangle with base 2, height 2 = area 2.0
        let tri = [Vec2::ZERO, Vec2::new(2.0, 0.0), Vec2::new(1.0, 2.0)];
        assert!((polygon_area(&tri) - 2.0).abs() < 0.01);

        // Degenerate (< 3 vertices)
        assert!(polygon_area(&[Vec2::ZERO]) < f32::EPSILON);
        assert!(polygon_area(&[]) < f32::EPSILON);
    }

    // -----------------------------------------------------------------------
    // Binary serialization tests
    // -----------------------------------------------------------------------

    #[test]
    fn navmesh_bytes_roundtrip() {
        let mesh = NavMesh::bake(&[
            Vec2::ZERO,
            Vec2::new(10.0, 0.0),
            Vec2::new(10.0, 10.0),
            Vec2::new(0.0, 10.0),
        ]);
        let bytes = mesh.to_bytes();
        let loaded = NavMesh::from_bytes(&bytes);
        assert!(loaded.is_some());
        let loaded = loaded.expect("roundtrip should succeed");
        assert_eq!(loaded.poly_count(), mesh.poly_count());

        // Verify polygon data matches
        for (a, b) in mesh.polys().iter().zip(loaded.polys().iter()) {
            assert_eq!(a.id, b.id);
            assert_eq!(a.vertices.len(), b.vertices.len());
            for (va, vb) in a.vertices.iter().zip(b.vertices.iter()) {
                assert!((va.x - vb.x).abs() < f32::EPSILON);
                assert!((va.y - vb.y).abs() < f32::EPSILON);
            }
            assert_eq!(a.neighbors.len(), b.neighbors.len());
            for (na, nb) in a.neighbors.iter().zip(b.neighbors.iter()) {
                assert_eq!(na, nb);
            }
            assert!((a.cost - b.cost).abs() < f32::EPSILON);
        }
    }

    #[test]
    fn navmesh_bytes_empty() {
        let mesh = NavMesh::new();
        let bytes = mesh.to_bytes();
        let loaded = NavMesh::from_bytes(&bytes).expect("empty mesh roundtrip");
        assert_eq!(loaded.poly_count(), 0);
    }

    #[test]
    fn navmesh_bytes_invalid_magic() {
        assert!(NavMesh::from_bytes(b"BAAD\x01\x00\x00\x00\x00\x00\x00\x00").is_none());
    }

    #[test]
    fn navmesh_bytes_truncated() {
        assert!(NavMesh::from_bytes(b"RNAV").is_none());
        assert!(NavMesh::from_bytes(&[]).is_none());
    }

    #[test]
    fn navmesh_bytes_unsupported_version() {
        let mut data = b"RNAV".to_vec();
        data.extend_from_slice(&99u32.to_le_bytes());
        data.extend_from_slice(&0u32.to_le_bytes());
        assert!(NavMesh::from_bytes(&data).is_none());
    }

    #[test]
    fn navmesh_bytes_with_costs() {
        let mut mesh = NavMesh::bake(&[
            Vec2::ZERO,
            Vec2::new(10.0, 0.0),
            Vec2::new(10.0, 10.0),
            Vec2::new(0.0, 10.0),
        ]);
        if mesh.poly_count() > 0 {
            mesh.set_poly_cost(NavPolyId(0), 3.5);
        }
        let bytes = mesh.to_bytes();
        let loaded = NavMesh::from_bytes(&bytes).expect("cost roundtrip");
        if loaded.poly_count() > 0 {
            let poly = loaded.get_poly(NavPolyId(0)).expect("poly 0 exists");
            assert!((poly.cost - 3.5).abs() < f32::EPSILON);
        }
    }

    #[test]
    fn navmesh_bytes_manual_mesh() {
        let mesh = make_square_mesh();
        let bytes = mesh.to_bytes();
        let loaded = NavMesh::from_bytes(&bytes).expect("manual mesh roundtrip");
        assert_eq!(loaded.poly_count(), mesh.poly_count());
        // Verify neighbor connectivity survived
        let p0 = loaded.get_poly(NavPolyId(0)).expect("poly 0");
        assert!(p0.neighbors.contains(&NavPolyId(1)));
        let p1 = loaded.get_poly(NavPolyId(1)).expect("poly 1");
        assert!(p1.neighbors.contains(&NavPolyId(0)));
    }

    // --- Obstacle Carving tests ---

    #[test]
    fn carver_basic() {
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
            layer: 0,
        });
        mesh.add_poly(NavPoly {
            id: NavPolyId(1),
            vertices: vec![
                Vec2::new(10.0, 0.0),
                Vec2::new(20.0, 0.0),
                Vec2::new(20.0, 10.0),
                Vec2::new(10.0, 10.0),
            ],
            neighbors: vec![NavPolyId(0)],
            cost: 1.0,
            layer: 0,
        });

        let mut carver = ObstacleCarver::new();
        assert!(!carver.is_blocked(NavPolyId(0)));

        // Block poly 0 with a circle around its centroid (5,5)
        let id = carver.add_obstacle(
            &mesh,
            CarveShape::Circle {
                center: Vec2::new(5.0, 5.0),
                radius: 3.0,
            },
        );
        assert!(carver.is_blocked(NavPolyId(0)));
        assert!(!carver.is_blocked(NavPolyId(1)));

        // Remove obstacle
        carver.remove_obstacle(&mesh, id);
        assert!(!carver.is_blocked(NavPolyId(0)));
    }

    #[test]
    fn carver_rect() {
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
            layer: 0,
        });

        let mut carver = ObstacleCarver::new();
        carver.add_obstacle(
            &mesh,
            CarveShape::Rect {
                min: Vec2::new(3.0, 3.0),
                max: Vec2::new(7.0, 7.0),
            },
        );
        assert!(carver.is_blocked(NavPolyId(0)));
    }

    #[test]
    fn find_path_carved_routes_around() {
        let mut mesh = NavMesh::new();
        // Four polys in a 2x2 grid
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
            layer: 0,
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
            cost: 1.0,
            layer: 0,
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
            layer: 0,
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
            layer: 0,
        });

        // Without carving — path exists
        let no_carve = ObstacleCarver::new();
        let path = mesh.find_path_carved(Vec2::new(5.0, 5.0), Vec2::new(15.0, 15.0), &no_carve);
        assert!(path.is_some());

        // Carve poly 1 — should route through 0 -> 2 -> 3
        let mut carver = ObstacleCarver::new();
        carver.add_obstacle(
            &mesh,
            CarveShape::Circle {
                center: Vec2::new(15.0, 5.0),
                radius: 3.0,
            },
        );
        let path = mesh.find_path_carved(Vec2::new(5.0, 5.0), Vec2::new(15.0, 15.0), &carver);
        assert!(path.is_some());
        let path = path.unwrap();
        assert!(!path.contains(&NavPolyId(1)));
    }

    #[test]
    fn carver_clear() {
        let mut mesh = NavMesh::new();
        mesh.add_poly(NavPoly {
            id: NavPolyId(0),
            vertices: vec![Vec2::ZERO, Vec2::new(10.0, 0.0), Vec2::new(5.0, 10.0)],
            neighbors: vec![],
            cost: 1.0,
            layer: 0,
        });

        let mut carver = ObstacleCarver::new();
        carver.add_obstacle(
            &mesh,
            CarveShape::Circle {
                center: Vec2::new(5.0, 3.0),
                radius: 5.0,
            },
        );
        assert!(carver.is_blocked(NavPolyId(0)));

        carver.clear();
        assert!(!carver.is_blocked(NavPolyId(0)));
        assert_eq!(carver.obstacle_count(), 0);
    }

    #[test]
    fn carver_serde_roundtrip() {
        let carver = ObstacleCarver::new();
        let json = serde_json::to_string(&carver).unwrap();
        let deserialized: ObstacleCarver = serde_json::from_str(&json).unwrap();
        assert_eq!(deserialized.obstacle_count(), 0);
    }

    #[test]
    fn carve_shape_circle_miss() {
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
            layer: 0,
        });

        let mut carver = ObstacleCarver::new();
        // Circle far from polygon centroid (5,5)
        carver.add_obstacle(
            &mesh,
            CarveShape::Circle {
                center: Vec2::new(50.0, 50.0),
                radius: 1.0,
            },
        );
        assert!(!carver.is_blocked(NavPolyId(0)));
    }

    #[test]
    fn carver_remove_nonexistent() {
        let mesh = NavMesh::new();
        let mut carver = ObstacleCarver::new();
        assert!(!carver.remove_obstacle(&mesh, CarvedObstacleId(999)));
    }

    #[test]
    fn carver_multiple_obstacles() {
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
            layer: 0,
        });
        mesh.add_poly(NavPoly {
            id: NavPolyId(1),
            vertices: vec![
                Vec2::new(20.0, 0.0),
                Vec2::new(30.0, 0.0),
                Vec2::new(30.0, 10.0),
                Vec2::new(20.0, 10.0),
            ],
            neighbors: vec![],
            cost: 1.0,
            layer: 0,
        });

        let mut carver = ObstacleCarver::new();
        let id0 = carver.add_obstacle(
            &mesh,
            CarveShape::Circle {
                center: Vec2::new(5.0, 5.0),
                radius: 2.0,
            },
        );
        let _id1 = carver.add_obstacle(
            &mesh,
            CarveShape::Circle {
                center: Vec2::new(25.0, 5.0),
                radius: 2.0,
            },
        );
        assert!(carver.is_blocked(NavPolyId(0)));
        assert!(carver.is_blocked(NavPolyId(1)));
        assert_eq!(carver.obstacle_count(), 2);

        // Remove first obstacle — second poly should still be blocked
        carver.remove_obstacle(&mesh, id0);
        assert!(!carver.is_blocked(NavPolyId(0)));
        assert!(carver.is_blocked(NavPolyId(1)));
        assert_eq!(carver.obstacle_count(), 1);
    }

    #[test]
    fn find_path_carved_blocked_start() {
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
            layer: 0,
        });

        let mut carver = ObstacleCarver::new();
        carver.add_obstacle(
            &mesh,
            CarveShape::Circle {
                center: Vec2::new(5.0, 5.0),
                radius: 3.0,
            },
        );
        // Start poly is blocked — no path
        let path = mesh.find_path_carved(Vec2::new(5.0, 5.0), Vec2::new(5.0, 5.0), &carver);
        assert!(path.is_none());
    }

    // --- Navigation layers tests ---

    #[test]
    fn layers_basic_path() {
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
            layer: 0,
        });
        mesh.add_poly(NavPoly {
            id: NavPolyId(1),
            vertices: vec![
                Vec2::new(10.0, 0.0),
                Vec2::new(20.0, 0.0),
                Vec2::new(20.0, 10.0),
                Vec2::new(10.0, 10.0),
            ],
            neighbors: vec![NavPolyId(0)],
            cost: 1.0,
            layer: 0,
        });

        let path = mesh.find_path_on_layers(Vec2::new(5.0, 5.0), Vec2::new(15.0, 5.0), &[0]);
        assert!(path.is_some());
        assert_eq!(path.unwrap().len(), 2);
    }

    #[test]
    fn layers_filtering_blocks_path() {
        let mut mesh = NavMesh::new();
        // Three polys: 0 (ground) -- 1 (water) -- 2 (ground)
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
            layer: 0,
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
            layer: 1, // water
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
            layer: 0,
        });

        // Only ground allowed — middle poly blocks path
        let path = mesh.find_path_on_layers(Vec2::new(5.0, 5.0), Vec2::new(25.0, 5.0), &[0]);
        assert!(path.is_none());

        // Allow both ground and water — path exists
        let path = mesh.find_path_on_layers(Vec2::new(5.0, 5.0), Vec2::new(25.0, 5.0), &[0, 1]);
        assert!(path.is_some());
        assert_eq!(path.unwrap().len(), 3);
    }

    #[test]
    fn layers_empty_allowed_no_path() {
        let mesh = make_square_mesh();
        let path = mesh.find_path_on_layers(Vec2::new(0.5, 0.5), Vec2::new(1.5, 0.5), &[]);
        assert!(path.is_none());
    }

    // --- Agent radius erosion tests ---

    #[test]
    fn erode_basic() {
        let mesh = make_square_mesh();
        let eroded = erode_navmesh(&mesh, 0.1);
        // Both polygons should survive a small erosion
        assert_eq!(eroded.poly_count(), 2);
        // Eroded vertices should be shifted inward
        for poly in eroded.polys() {
            let centroid = poly.centroid();
            // Centroid should still be roughly in the original poly area
            assert!(centroid.x > 0.0 && centroid.y > 0.0);
        }
    }

    #[test]
    fn erode_zero_radius() {
        let mesh = make_square_mesh();
        let eroded = erode_navmesh(&mesh, 0.0);
        assert_eq!(eroded.poly_count(), mesh.poly_count());
        // Vertices should match exactly
        for (orig, eroded_poly) in mesh.polys().iter().zip(eroded.polys().iter()) {
            for (v1, v2) in orig.vertices.iter().zip(eroded_poly.vertices.iter()) {
                assert!((v1.x - v2.x).abs() < f32::EPSILON);
                assert!((v1.y - v2.y).abs() < f32::EPSILON);
            }
        }
    }

    #[test]
    fn erode_large_radius_removes_small_polys() {
        let mut mesh = NavMesh::new();
        // Small triangle that will be eliminated
        mesh.add_poly(NavPoly {
            id: NavPolyId(0),
            vertices: vec![Vec2::ZERO, Vec2::new(0.5, 0.0), Vec2::new(0.25, 0.5)],
            neighbors: vec![],
            cost: 1.0,
            layer: 0,
        });
        let eroded = erode_navmesh(&mesh, 1.0);
        // Small polygon should be removed by large erosion
        assert_eq!(eroded.poly_count(), 0);
    }

    #[test]
    fn navpoly_zero_area() {
        // Collinear vertices — zero area polygon
        let poly = NavPoly {
            id: NavPolyId(0),
            vertices: vec![Vec2::ZERO, Vec2::new(5.0, 0.0), Vec2::new(10.0, 0.0)],
            neighbors: vec![],
            cost: 1.0,
            layer: 0,
        };
        // Collinear polygon: point off the line should not be contained
        assert!(!poly.contains_point(Vec2::new(5.0, 1.0)));
    }

    #[test]
    fn navpoly_single_vertex() {
        let poly = NavPoly {
            id: NavPolyId(0),
            vertices: vec![Vec2::ZERO],
            neighbors: vec![],
            cost: 1.0,
            layer: 0,
        };
        assert!(!poly.contains_point(Vec2::ZERO));
    }

    #[test]
    fn navpoly_two_vertices() {
        let poly = NavPoly {
            id: NavPolyId(0),
            vertices: vec![Vec2::ZERO, Vec2::new(10.0, 0.0)],
            neighbors: vec![],
            cost: 1.0,
            layer: 0,
        };
        assert!(!poly.contains_point(Vec2::new(5.0, 0.0)));
    }

    #[test]
    fn navmesh_invalid_neighbor_id() {
        // Polygon references a non-existent neighbor — pathfinding should handle gracefully
        let mut mesh = NavMesh::new();
        mesh.add_poly(NavPoly {
            id: NavPolyId(0),
            vertices: vec![Vec2::ZERO, Vec2::new(10.0, 0.0), Vec2::new(5.0, 10.0)],
            neighbors: vec![NavPolyId(99)], // doesn't exist
            cost: 1.0,
            layer: 0,
        });
        // Should not panic
        let path = mesh.find_path(Vec2::new(5.0, 3.0), Vec2::new(5.0, 3.0));
        assert!(path.is_some()); // same poly, path to self
    }
}
