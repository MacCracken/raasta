//! Heightfield-based navmesh baking from 3D geometry.
//!
//! Provides a Recast-inspired pipeline: voxelize 3D triangles into a
//! heightfield, identify walkable surfaces, and generate a 2D navmesh
//! projected onto the XZ plane.

use hisab::{Vec2, Vec3};
use serde::{Deserialize, Serialize};

use crate::mesh::{NavMesh, NavPoly, NavPolyId};

#[cfg(feature = "logging")]
use tracing::instrument;

/// A span in a heightfield column — a vertical range that is solid.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct HeightSpan {
    /// Bottom of the solid span (voxel units).
    pub min_y: u16,
    /// Top of the solid span (voxel units).
    pub max_y: u16,
    /// Whether the top surface is walkable.
    pub walkable: bool,
}

/// Baking configuration for heightfield navmesh generation.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct HeightfieldConfig {
    /// Size of each voxel in world units (XZ plane).
    pub cell_size: f32,
    /// Height of each voxel in world units (Y axis).
    pub cell_height: f32,
    /// Maximum slope angle (radians) considered walkable.
    pub max_slope: f32,
    /// Minimum clearance height (world units) above a walkable surface.
    pub agent_height: f32,
    /// Agent radius (world units) for erosion.
    pub agent_radius: f32,
    /// Minimum region area (in cells) to keep — smaller regions are removed.
    pub min_region_area: usize,
}

impl Default for HeightfieldConfig {
    fn default() -> Self {
        Self {
            cell_size: 0.3,
            cell_height: 0.2,
            max_slope: std::f32::consts::FRAC_PI_4, // 45 degrees
            agent_height: 2.0,
            agent_radius: 0.6,
            min_region_area: 8,
        }
    }
}

/// A heightfield — voxelized representation of 3D geometry.
///
/// Each cell in the XZ grid contains a stack of solid spans representing
/// the vertical extent of geometry at that position.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Heightfield {
    width: usize,
    depth: usize,
    origin: Vec3,
    cell_size: f32,
    cell_height: f32,
    /// Column-major storage: `spans[z * width + x]` = Vec of spans for that column.
    spans: Vec<Vec<HeightSpan>>,
}

impl Heightfield {
    /// Create an empty heightfield covering the given world bounds.
    #[must_use]
    pub fn new(min: Vec3, max: Vec3, cell_size: f32, cell_height: f32) -> Self {
        let width = ((max.x - min.x) / cell_size).ceil() as usize;
        let depth = ((max.z - min.z) / cell_size).ceil() as usize;
        Self {
            width: width.max(1),
            depth: depth.max(1),
            origin: min,
            cell_size,
            cell_height,
            spans: vec![Vec::new(); width.max(1) * depth.max(1)],
        }
    }

    /// Grid width (number of cells along the X axis).
    #[must_use]
    pub fn width(&self) -> usize {
        self.width
    }

    /// Grid depth (number of cells along the Z axis).
    #[must_use]
    pub fn depth(&self) -> usize {
        self.depth
    }

    /// World-space origin (minimum corner).
    #[must_use]
    pub fn origin(&self) -> Vec3 {
        self.origin
    }

    /// Cell size in world units (XZ plane).
    #[must_use]
    pub fn cell_size(&self) -> f32 {
        self.cell_size
    }

    /// Cell height in world units (Y axis).
    #[must_use]
    pub fn cell_height(&self) -> f32 {
        self.cell_height
    }

    /// Get spans at grid position.
    #[must_use]
    pub fn spans_at(&self, x: usize, z: usize) -> &[HeightSpan] {
        if x < self.width && z < self.depth {
            &self.spans[z * self.width + x]
        } else {
            &[]
        }
    }

    /// Rasterize a single triangle into the heightfield.
    pub fn rasterize_triangle(&mut self, v0: Vec3, v1: Vec3, v2: Vec3) {
        // Compute bounding box in grid coords
        let min_x = ((v0.x.min(v1.x).min(v2.x) - self.origin.x) / self.cell_size).floor() as i32;
        let max_x = ((v0.x.max(v1.x).max(v2.x) - self.origin.x) / self.cell_size).ceil() as i32;
        let min_z = ((v0.z.min(v1.z).min(v2.z) - self.origin.z) / self.cell_size).floor() as i32;
        let max_z = ((v0.z.max(v1.z).max(v2.z) - self.origin.z) / self.cell_size).ceil() as i32;

        let min_x = min_x.max(0) as usize;
        let max_x = (max_x as usize).min(self.width.saturating_sub(1));
        let min_z = min_z.max(0) as usize;
        let max_z = (max_z as usize).min(self.depth.saturating_sub(1));

        for z in min_z..=max_z {
            for x in min_x..=max_x {
                // Cell center in world space
                let cx = self.origin.x + (x as f32 + 0.5) * self.cell_size;
                let cz = self.origin.z + (z as f32 + 0.5) * self.cell_size;

                // Check if cell overlaps triangle (conservative: use point-in-triangle on XZ)
                if !point_in_triangle_xz(cx, cz, v0, v1, v2) {
                    continue;
                }

                // Interpolate Y at this XZ position using barycentric coords
                if let Some(y) = barycentric_y(cx, cz, v0, v1, v2) {
                    let vy = ((y - self.origin.y) / self.cell_height) as u16;
                    self.add_span(x, z, vy, vy.saturating_add(1));
                }
            }
        }
    }

    /// Rasterize multiple triangles (vertex triplets).
    #[cfg_attr(feature = "logging", instrument(skip(self, triangles)))]
    pub fn rasterize_triangles(&mut self, triangles: &[[Vec3; 3]]) {
        for tri in triangles {
            self.rasterize_triangle(tri[0], tri[1], tri[2]);
        }
    }

    /// Rasterize from indexed vertex/index buffers.
    #[cfg_attr(feature = "logging", instrument(skip(self, vertices, indices)))]
    pub fn rasterize_indexed(&mut self, vertices: &[Vec3], indices: &[u32]) {
        for chunk in indices.chunks_exact(3) {
            let i0 = chunk[0] as usize;
            let i1 = chunk[1] as usize;
            let i2 = chunk[2] as usize;
            if i0 < vertices.len() && i1 < vertices.len() && i2 < vertices.len() {
                self.rasterize_triangle(vertices[i0], vertices[i1], vertices[i2]);
            }
        }
    }

    /// Add a span to a column, merging with existing spans if they overlap.
    fn add_span(&mut self, x: usize, z: usize, min_y: u16, max_y: u16) {
        let idx = z * self.width + x;
        let spans = &mut self.spans[idx];

        // Find overlapping or adjacent spans and merge
        let mut new_min = min_y;
        let mut new_max = max_y;
        let mut i = 0;
        while i < spans.len() {
            if spans[i].min_y <= new_max.saturating_add(1)
                && spans[i].max_y >= new_min.saturating_sub(1)
            {
                new_min = new_min.min(spans[i].min_y);
                new_max = new_max.max(spans[i].max_y);
                spans.swap_remove(i);
            } else {
                i += 1;
            }
        }
        spans.push(HeightSpan {
            min_y: new_min,
            max_y: new_max,
            walkable: false,
        });
    }

    /// Mark walkable surfaces based on slope angle.
    ///
    /// A span's top surface is walkable if the triangle normal at that
    /// point has a Y component >= cos(max\_slope).
    pub fn mark_walkable(&mut self, _max_slope: f32) {
        // Simple approach: all top surfaces of spans are potentially walkable.
        // (Real Recast checks triangle normals; we approximate by marking all.
        // Use `mark_walkable_from_triangles` for slope-filtered walkability.)
        for col in &mut self.spans {
            // Sort spans by min_y
            col.sort_by_key(|s| s.min_y);
            for span in col.iter_mut() {
                span.walkable = true;
            }
        }
    }

    /// Mark walkable surfaces using triangle normals for slope filtering.
    #[cfg_attr(feature = "logging", instrument(skip(self, triangles)))]
    pub fn mark_walkable_from_triangles(&mut self, triangles: &[[Vec3; 3]], max_slope: f32) {
        let min_ny = max_slope.cos();

        // First, reset all to non-walkable
        for col in &mut self.spans {
            for span in col.iter_mut() {
                span.walkable = false;
            }
        }

        // For each triangle, compute its normal and mark covered cells
        for tri in triangles {
            let edge1 = tri[1] - tri[0];
            let edge2 = tri[2] - tri[0];
            let normal = edge1.cross(edge2);
            let len = normal.length();
            if len < f32::EPSILON {
                continue;
            }
            let ny = (normal.y / len).abs();
            if ny < min_ny {
                continue; // Too steep
            }

            // Find cells covered by this triangle and mark them walkable
            let min_x = ((tri[0].x.min(tri[1].x).min(tri[2].x) - self.origin.x) / self.cell_size)
                .floor()
                .max(0.0) as usize;
            let max_x = ((tri[0].x.max(tri[1].x).max(tri[2].x) - self.origin.x) / self.cell_size)
                .ceil() as usize;
            let min_z = ((tri[0].z.min(tri[1].z).min(tri[2].z) - self.origin.z) / self.cell_size)
                .floor()
                .max(0.0) as usize;
            let max_z = ((tri[0].z.max(tri[1].z).max(tri[2].z) - self.origin.z) / self.cell_size)
                .ceil() as usize;

            let max_x = max_x.min(self.width.saturating_sub(1));
            let max_z = max_z.min(self.depth.saturating_sub(1));

            for z in min_z..=max_z {
                for x in min_x..=max_x {
                    let cx = self.origin.x + (x as f32 + 0.5) * self.cell_size;
                    let cz = self.origin.z + (z as f32 + 0.5) * self.cell_size;
                    if point_in_triangle_xz(cx, cz, tri[0], tri[1], tri[2]) {
                        // Mark the topmost span in this column as walkable
                        let col = &mut self.spans[z * self.width + x];
                        if let Some(top) = col.last_mut() {
                            top.walkable = true;
                        }
                    }
                }
            }
        }
    }

    /// Filter walkable spans by agent clearance height.
    ///
    /// A walkable span needs at least `agent_height` world units of
    /// empty space above it to the next span.
    pub fn filter_clearance(&mut self, agent_height: f32) {
        let clearance_voxels = (agent_height / self.cell_height).ceil() as u16;

        for col in &mut self.spans {
            col.sort_by_key(|s| s.min_y);
            let len = col.len();
            for i in 0..len {
                if !col[i].walkable {
                    continue;
                }
                // Check clearance to next span above
                if i + 1 < len {
                    let gap = col[i + 1].min_y.saturating_sub(col[i].max_y);
                    if gap < clearance_voxels {
                        col[i].walkable = false;
                    }
                }
                // Top span always has infinite clearance (stays walkable)
            }
        }
    }

    /// Erode walkable areas by agent radius.
    ///
    /// Marks border walkable cells as unwalkable if they are within
    /// `agent_radius` of a non-walkable cell.
    pub fn erode_walkable(&mut self, agent_radius: f32) {
        let radius_cells = (agent_radius / self.cell_size).ceil() as i32;
        if radius_cells <= 0 {
            return;
        }

        // Build a walkable mask
        let len = self.width * self.depth;
        let mut walkable_mask = vec![false; len];
        for z in 0..self.depth {
            for x in 0..self.width {
                let col = &self.spans[z * self.width + x];
                walkable_mask[z * self.width + x] = col.iter().any(|s| s.walkable);
            }
        }

        // Distance-based erosion
        let mut erode = vec![false; len];
        for z in 0..self.depth {
            for x in 0..self.width {
                if !walkable_mask[z * self.width + x] {
                    continue;
                }
                // Check if any non-walkable cell is within radius
                let mut near_edge = false;
                'outer: for dz in -radius_cells..=radius_cells {
                    for dx in -radius_cells..=radius_cells {
                        if dx * dx + dz * dz > radius_cells * radius_cells {
                            continue;
                        }
                        let nx = x as i32 + dx;
                        let nz = z as i32 + dz;
                        if nx < 0 || nz < 0 || nx >= self.width as i32 || nz >= self.depth as i32 {
                            near_edge = true;
                            break 'outer;
                        }
                        if !walkable_mask[nz as usize * self.width + nx as usize] {
                            near_edge = true;
                            break 'outer;
                        }
                    }
                }
                if near_edge {
                    erode[z * self.width + x] = true;
                }
            }
        }

        // Apply erosion
        for z in 0..self.depth {
            for x in 0..self.width {
                if erode[z * self.width + x] {
                    let col = &mut self.spans[z * self.width + x];
                    for span in col.iter_mut() {
                        span.walkable = false;
                    }
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Navmesh generation from heightfield
// ---------------------------------------------------------------------------

/// Bake a 2D NavMesh from a heightfield.
///
/// Extracts walkable regions, traces contours, and generates convex
/// polygons projected onto the XZ plane.
///
/// This is the main entry point for heightfield-based navmesh generation.
#[cfg_attr(feature = "logging", instrument(skip(hf)))]
#[must_use]
pub fn bake_from_heightfield(hf: &Heightfield, config: &HeightfieldConfig) -> NavMesh {
    // Step 1: Build region IDs from walkable cells using flood-fill
    let regions = build_regions(hf, config.min_region_area);

    // Step 2: Trace contours from region boundaries
    let contours = trace_contours(hf, &regions);

    // Step 3: Convert contours to navmesh polygons
    build_navmesh_from_contours(hf, &contours)
}

/// Full pipeline: rasterize triangles and bake a navmesh.
///
/// Convenience function that runs the complete pipeline:
/// rasterize -> mark walkable -> filter clearance -> erode -> bake.
#[cfg_attr(feature = "logging", instrument(skip(triangles)))]
#[must_use]
pub fn bake_navmesh_from_geometry(triangles: &[[Vec3; 3]], config: &HeightfieldConfig) -> NavMesh {
    if triangles.is_empty() {
        return NavMesh::new();
    }

    // Compute bounds
    let mut min = Vec3::new(f32::INFINITY, f32::INFINITY, f32::INFINITY);
    let mut max = Vec3::new(f32::NEG_INFINITY, f32::NEG_INFINITY, f32::NEG_INFINITY);
    for tri in triangles {
        for v in tri {
            min.x = min.x.min(v.x);
            min.y = min.y.min(v.y);
            min.z = min.z.min(v.z);
            max.x = max.x.max(v.x);
            max.y = max.y.max(v.y);
            max.z = max.z.max(v.z);
        }
    }

    // Expand bounds slightly
    min.x -= config.cell_size;
    min.y -= config.cell_height;
    min.z -= config.cell_size;
    max.x += config.cell_size;
    max.y += config.cell_height;
    max.z += config.cell_size;

    let mut hf = Heightfield::new(min, max, config.cell_size, config.cell_height);
    hf.rasterize_triangles(triangles);
    hf.mark_walkable_from_triangles(triangles, config.max_slope);
    hf.filter_clearance(config.agent_height);
    hf.erode_walkable(config.agent_radius);

    bake_from_heightfield(&hf, config)
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Point-in-triangle test on XZ plane.
#[inline]
fn point_in_triangle_xz(px: f32, pz: f32, v0: Vec3, v1: Vec3, v2: Vec3) -> bool {
    let d1 = sign_xz(px, pz, v0.x, v0.z, v1.x, v1.z);
    let d2 = sign_xz(px, pz, v1.x, v1.z, v2.x, v2.z);
    let d3 = sign_xz(px, pz, v2.x, v2.z, v0.x, v0.z);
    let has_neg = (d1 < 0.0) || (d2 < 0.0) || (d3 < 0.0);
    let has_pos = (d1 > 0.0) || (d2 > 0.0) || (d3 > 0.0);
    !(has_neg && has_pos)
}

#[inline]
fn sign_xz(px: f32, pz: f32, x1: f32, z1: f32, x2: f32, z2: f32) -> f32 {
    (px - x2) * (z1 - z2) - (x1 - x2) * (pz - z2)
}

/// Interpolate Y coordinate at (px, pz) using barycentric coords on triangle.
#[inline]
fn barycentric_y(px: f32, pz: f32, v0: Vec3, v1: Vec3, v2: Vec3) -> Option<f32> {
    let denom = (v1.z - v2.z) * (v0.x - v2.x) + (v2.x - v1.x) * (v0.z - v2.z);
    if denom.abs() < f32::EPSILON {
        return None;
    }
    let inv = 1.0 / denom;
    let u = ((v1.z - v2.z) * (px - v2.x) + (v2.x - v1.x) * (pz - v2.z)) * inv;
    let v = ((v2.z - v0.z) * (px - v2.x) + (v0.x - v2.x) * (pz - v2.z)) * inv;
    let w = 1.0 - u - v;
    Some(u * v0.y + v * v1.y + w * v2.y)
}

/// Build regions from walkable cells using flood-fill.
/// Returns a grid of region IDs (0 = non-walkable).
fn build_regions(hf: &Heightfield, min_area: usize) -> Vec<u32> {
    let len = hf.width * hf.depth;
    let mut region_ids = vec![0u32; len];
    let mut current_region = 1u32;

    for z in 0..hf.depth {
        for x in 0..hf.width {
            let idx = z * hf.width + x;
            if region_ids[idx] != 0 {
                continue;
            }
            let col = &hf.spans[idx];
            if !col.iter().any(|s| s.walkable) {
                continue;
            }

            // Flood-fill from this cell
            let mut stack = vec![(x, z)];
            let mut area = 0usize;
            region_ids[idx] = current_region;

            while let Some((cx, cz)) = stack.pop() {
                area += 1;
                for (dx, dz) in [(0i32, 1), (0, -1), (1, 0), (-1, 0)] {
                    let nx = cx as i32 + dx;
                    let nz = cz as i32 + dz;
                    if nx < 0 || nz < 0 || nx >= hf.width as i32 || nz >= hf.depth as i32 {
                        continue;
                    }
                    let nx = nx as usize;
                    let nz = nz as usize;
                    let ni = nz * hf.width + nx;
                    if region_ids[ni] != 0 {
                        continue;
                    }
                    if !hf.spans[ni].iter().any(|s| s.walkable) {
                        continue;
                    }
                    region_ids[ni] = current_region;
                    stack.push((nx, nz));
                }
            }

            // Remove small regions
            if area < min_area {
                for rid in region_ids.iter_mut() {
                    if *rid == current_region {
                        *rid = 0;
                    }
                }
            } else {
                current_region += 1;
            }
        }
    }

    region_ids
}

/// Trace contour polygons from region boundaries.
/// Returns a list of contour vertex lists (in XZ grid coords).
fn trace_contours(hf: &Heightfield, regions: &[u32]) -> Vec<Vec<(usize, usize)>> {
    let max_region = regions.iter().copied().max().unwrap_or(0);
    let mut contours = Vec::new();

    for region_id in 1..=max_region {
        // Find boundary cells of this region
        let mut boundary = Vec::new();
        for z in 0..hf.depth {
            for x in 0..hf.width {
                if regions[z * hf.width + x] != region_id {
                    continue;
                }
                // Check if this cell is on the region boundary
                let is_boundary = [(0i32, 1), (0, -1), (1, 0), (-1, 0)]
                    .iter()
                    .any(|&(dx, dz)| {
                        let nx = x as i32 + dx;
                        let nz = z as i32 + dz;
                        if nx < 0 || nz < 0 || nx >= hf.width as i32 || nz >= hf.depth as i32 {
                            return true;
                        }
                        regions[nz as usize * hf.width + nx as usize] != region_id
                    });
                if is_boundary {
                    boundary.push((x, z));
                }
            }
        }

        if !boundary.is_empty() {
            contours.push(boundary);
        }
    }

    contours
}

/// Build navmesh from contour boundaries.
fn build_navmesh_from_contours(hf: &Heightfield, contours: &[Vec<(usize, usize)>]) -> NavMesh {
    let mut mesh = NavMesh::new();
    let mut poly_id = 0u32;

    for contour in contours {
        if contour.len() < 3 {
            continue;
        }

        // Compute convex hull of boundary cells projected to world coords
        let vertices: Vec<Vec2> = compute_convex_hull(
            &contour
                .iter()
                .map(|&(x, z)| {
                    Vec2::new(
                        hf.origin.x + (x as f32 + 0.5) * hf.cell_size,
                        hf.origin.z + (z as f32 + 0.5) * hf.cell_size,
                    )
                })
                .collect::<Vec<_>>(),
        );

        if vertices.len() < 3 {
            continue;
        }

        mesh.add_poly(NavPoly {
            id: NavPolyId(poly_id),
            vertices,
            neighbors: Vec::new(),
            cost: 1.0,
            layer: 0,
        });
        poly_id += 1;
    }

    // Auto-detect neighbors (shared edges)
    detect_neighbors(&mut mesh);

    mesh
}

/// Compute convex hull of 2D points (Andrew's monotone chain).
fn compute_convex_hull(points: &[Vec2]) -> Vec<Vec2> {
    if points.len() < 3 {
        return points.to_vec();
    }

    let mut sorted = points.to_vec();
    sorted.sort_by(|a, b| {
        a.x.partial_cmp(&b.x)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(a.y.partial_cmp(&b.y).unwrap_or(std::cmp::Ordering::Equal))
    });
    sorted.dedup_by(|a, b| (a.x - b.x).abs() < f32::EPSILON && (a.y - b.y).abs() < f32::EPSILON);

    if sorted.len() < 3 {
        return sorted;
    }

    let mut hull = Vec::with_capacity(sorted.len() * 2);

    // Lower hull
    for &p in &sorted {
        while hull.len() >= 2 {
            let len = hull.len();
            if cross_2d(hull[len - 2], hull[len - 1], p) <= 0.0 {
                hull.pop();
            } else {
                break;
            }
        }
        hull.push(p);
    }

    // Upper hull
    let lower_len = hull.len() + 1;
    for &p in sorted.iter().rev() {
        while hull.len() >= lower_len {
            let len = hull.len();
            if cross_2d(hull[len - 2], hull[len - 1], p) <= 0.0 {
                hull.pop();
            } else {
                break;
            }
        }
        hull.push(p);
    }

    hull.pop(); // Remove last point (same as first)
    hull
}

#[inline]
fn cross_2d(o: Vec2, a: Vec2, b: Vec2) -> f32 {
    (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)
}

/// Auto-detect neighbors by shared edges.
fn detect_neighbors(mesh: &mut NavMesh) {
    let eps = 1e-3;
    let polys: Vec<(NavPolyId, Vec<Vec2>)> = mesh
        .polys()
        .iter()
        .map(|p| (p.id, p.vertices.clone()))
        .collect();

    // Build neighbor map
    let mut neighbor_map: Vec<Vec<NavPolyId>> = vec![Vec::new(); polys.len()];
    for i in 0..polys.len() {
        for j in (i + 1)..polys.len() {
            if shares_edge(&polys[i].1, &polys[j].1, eps) {
                neighbor_map[i].push(polys[j].0);
                neighbor_map[j].push(polys[i].0);
            }
        }
    }

    // Rebuild mesh with neighbors
    let old_polys: Vec<NavPoly> = mesh.polys().to_vec();
    *mesh = NavMesh::new();
    for (idx, poly) in old_polys.into_iter().enumerate() {
        mesh.add_poly(NavPoly {
            id: poly.id,
            vertices: poly.vertices,
            neighbors: neighbor_map.get(idx).cloned().unwrap_or_default(),
            cost: poly.cost,
            layer: poly.layer,
        });
    }
}

fn shares_edge(a: &[Vec2], b: &[Vec2], eps: f32) -> bool {
    let mut shared = 0;
    for va in a {
        for vb in b {
            if va.distance(*vb) < eps {
                shared += 1;
                if shared >= 2 {
                    return true;
                }
            }
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;

    fn flat_floor_triangles() -> Vec<[Vec3; 3]> {
        // Two triangles forming a 10x10 flat floor at y=0
        vec![
            [
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(10.0, 0.0, 0.0),
                Vec3::new(10.0, 0.0, 10.0),
            ],
            [
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(10.0, 0.0, 10.0),
                Vec3::new(0.0, 0.0, 10.0),
            ],
        ]
    }

    #[test]
    fn heightfield_new() {
        let hf = Heightfield::new(Vec3::ZERO, Vec3::new(10.0, 5.0, 10.0), 1.0, 0.5);
        assert_eq!(hf.width(), 10);
        assert_eq!(hf.depth(), 10);
    }

    #[test]
    fn heightfield_rasterize_flat() {
        let tris = flat_floor_triangles();
        let mut hf = Heightfield::new(
            Vec3::new(-1.0, -1.0, -1.0),
            Vec3::new(11.0, 1.0, 11.0),
            1.0,
            0.5,
        );
        hf.rasterize_triangles(&tris);
        // Center cells should have spans
        let spans = hf.spans_at(5, 5);
        assert!(!spans.is_empty());
    }

    #[test]
    fn heightfield_mark_walkable() {
        let tris = flat_floor_triangles();
        let mut hf = Heightfield::new(
            Vec3::new(-1.0, -1.0, -1.0),
            Vec3::new(11.0, 1.0, 11.0),
            1.0,
            0.5,
        );
        hf.rasterize_triangles(&tris);
        hf.mark_walkable(std::f32::consts::FRAC_PI_4);
        // All spans should be walkable (flat floor)
        let spans = hf.spans_at(5, 5);
        assert!(spans.iter().any(|s| s.walkable));
    }

    #[test]
    fn heightfield_filter_clearance() {
        let mut hf = Heightfield::new(Vec3::ZERO, Vec3::new(5.0, 10.0, 5.0), 1.0, 0.5);
        // Add two spans close together (no clearance)
        hf.add_span(2, 2, 0, 1);
        hf.add_span(2, 2, 3, 4); // Only 1 voxel gap
        hf.spans[2 * hf.width + 2].sort_by_key(|s| s.min_y);
        hf.spans[2 * hf.width + 2][0].walkable = true;
        hf.spans[2 * hf.width + 2][1].walkable = true;

        hf.filter_clearance(2.0); // Need 4 voxels clearance at 0.5 cell_height
        let spans = hf.spans_at(2, 2);
        // Bottom span should no longer be walkable (gap too small)
        assert!(!spans[0].walkable);
        // Top span stays walkable (infinite clearance above)
        assert!(spans[1].walkable);
    }

    #[test]
    fn bake_full_pipeline() {
        let tris = flat_floor_triangles();
        let config = HeightfieldConfig {
            cell_size: 1.0,
            cell_height: 0.5,
            max_slope: std::f32::consts::FRAC_PI_4,
            agent_height: 2.0,
            agent_radius: 0.0, // No erosion for simple test
            min_region_area: 1,
        };
        let mesh = bake_navmesh_from_geometry(&tris, &config);
        assert!(mesh.poly_count() > 0);
    }

    #[test]
    fn bake_empty_triangles() {
        let config = HeightfieldConfig::default();
        let mesh = bake_navmesh_from_geometry(&[], &config);
        assert_eq!(mesh.poly_count(), 0);
    }

    #[test]
    fn bake_produces_pathable_mesh() {
        let tris = flat_floor_triangles();
        let config = HeightfieldConfig {
            cell_size: 0.5,
            cell_height: 0.2,
            max_slope: std::f32::consts::FRAC_PI_4,
            agent_height: 2.0,
            agent_radius: 0.0,
            min_region_area: 1,
        };
        let mesh = bake_navmesh_from_geometry(&tris, &config);
        // Should produce a non-empty mesh from the floor geometry
        assert!(mesh.poly_count() > 0);
    }

    #[test]
    fn heightfield_rasterize_indexed() {
        let verts = vec![
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(10.0, 0.0, 0.0),
            Vec3::new(10.0, 0.0, 10.0),
            Vec3::new(0.0, 0.0, 10.0),
        ];
        let indices = vec![0, 1, 2, 0, 2, 3];
        let mut hf = Heightfield::new(
            Vec3::new(-1.0, -1.0, -1.0),
            Vec3::new(11.0, 1.0, 11.0),
            1.0,
            0.5,
        );
        hf.rasterize_indexed(&verts, &indices);
        assert!(!hf.spans_at(5, 5).is_empty());
    }

    #[test]
    fn heightfield_config_default() {
        let config = HeightfieldConfig::default();
        assert!(config.cell_size > 0.0);
        assert!(config.agent_height > 0.0);
    }

    #[test]
    fn heightfield_serde_roundtrip() {
        let hf = Heightfield::new(Vec3::ZERO, Vec3::new(5.0, 5.0, 5.0), 1.0, 0.5);
        let json = serde_json::to_string(&hf).expect("serialize heightfield");
        let deserialized: Heightfield =
            serde_json::from_str(&json).expect("deserialize heightfield");
        assert_eq!(deserialized.width(), hf.width());
    }

    #[test]
    fn heightfield_config_serde_roundtrip() {
        let config = HeightfieldConfig::default();
        let json = serde_json::to_string(&config).expect("serialize config");
        let deserialized: HeightfieldConfig =
            serde_json::from_str(&json).expect("deserialize config");
        assert!((deserialized.cell_size - config.cell_size).abs() < f32::EPSILON);
    }

    #[test]
    fn convex_hull_basic() {
        let points = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(1.0, 0.0),
            Vec2::new(0.5, 0.5),
            Vec2::new(0.0, 1.0),
            Vec2::new(1.0, 1.0),
        ];
        let hull = compute_convex_hull(&points);
        assert_eq!(hull.len(), 4); // Square hull
    }

    #[test]
    fn steep_slope_not_walkable() {
        // Nearly vertical triangle
        let tris = vec![[
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 10.0, 0.0),
            Vec3::new(0.0, 10.0, 10.0),
        ]];
        let mut hf = Heightfield::new(
            Vec3::new(-1.0, -1.0, -1.0),
            Vec3::new(1.0, 11.0, 11.0),
            1.0,
            0.5,
        );
        hf.rasterize_triangles(&tris);
        hf.mark_walkable_from_triangles(&tris, std::f32::consts::FRAC_PI_4);
        // Vertical triangle should not be walkable
        for z in 0..hf.depth() {
            for x in 0..hf.width() {
                for span in hf.spans_at(x, z) {
                    assert!(!span.walkable, "Vertical surface should not be walkable");
                }
            }
        }
    }
}
