//! NavMesh generation from physics colliders.
//!
//! Generates a 2D NavMesh from a walkable boundary with collider
//! obstacles subtracted. Uses grid-based rasterization for robust
//! handling of arbitrary collider shapes.

use hisab::Vec2;
use serde::{Deserialize, Serialize};

use crate::mesh::{NavMesh, NavPoly, NavPolyId};

#[cfg(feature = "logging")]
use tracing::instrument;

/// A collider shape for navmesh generation.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[non_exhaustive]
pub enum ColliderShape {
    /// Circular collider.
    Circle { center: Vec2, radius: f32 },
    /// Axis-aligned bounding box.
    Aabb { min: Vec2, max: Vec2 },
    /// Convex polygon collider (vertices in CCW order).
    ConvexPoly { vertices: Vec<Vec2> },
}

/// Configuration for collider-based navmesh generation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ColliderNavConfig {
    /// Resolution of the internal rasterization grid (world units per cell).
    /// Smaller values produce more accurate navmeshes but are slower.
    pub cell_size: f32,
    /// Extra padding around colliders (agent radius).
    pub agent_radius: f32,
    /// Minimum region area in cells to keep (removes tiny fragments).
    pub min_region_area: usize,
}

impl Default for ColliderNavConfig {
    fn default() -> Self {
        Self {
            cell_size: 0.5,
            agent_radius: 0.0,
            min_region_area: 4,
        }
    }
}

/// Generate a NavMesh from a walkable boundary with collider obstacles.
///
/// The boundary defines the outer walkable area (convex or concave polygon).
/// Colliders are subtracted from this area. The result is a navmesh covering
/// the walkable space.
///
/// # Arguments
///
/// * `boundary` — Outer walkable polygon vertices (CCW order)
/// * `colliders` — Obstacle shapes to subtract
/// * `config` — Generation settings
#[cfg_attr(feature = "logging", instrument(skip(boundary, colliders)))]
#[must_use]
pub fn navmesh_from_colliders(
    boundary: &[Vec2],
    colliders: &[ColliderShape],
    config: &ColliderNavConfig,
) -> NavMesh {
    if boundary.len() < 3 {
        return NavMesh::new();
    }

    // Compute bounds
    let mut min = Vec2::new(f32::INFINITY, f32::INFINITY);
    let mut max = Vec2::new(f32::NEG_INFINITY, f32::NEG_INFINITY);
    for v in boundary {
        min.x = min.x.min(v.x);
        min.y = min.y.min(v.y);
        max.x = max.x.max(v.x);
        max.y = max.y.max(v.y);
    }

    let width = ((max.x - min.x) / config.cell_size).ceil() as usize;
    let height = ((max.y - min.y) / config.cell_size).ceil() as usize;
    if width == 0 || height == 0 {
        return NavMesh::new();
    }

    // Step 1: Rasterize — mark cells as walkable if inside boundary
    let len = width * height;
    let mut walkable = vec![false; len];

    for y in 0..height {
        for x in 0..width {
            let cx = min.x + (x as f32 + 0.5) * config.cell_size;
            let cy = min.y + (y as f32 + 0.5) * config.cell_size;
            let p = Vec2::new(cx, cy);

            if point_in_polygon(p, boundary) {
                walkable[y * width + x] = true;
            }
        }
    }

    // Step 2: Subtract colliders (with agent radius padding)
    let padding = config.agent_radius;
    for collider in colliders {
        for y in 0..height {
            for x in 0..width {
                if !walkable[y * width + x] {
                    continue;
                }
                let cx = min.x + (x as f32 + 0.5) * config.cell_size;
                let cy = min.y + (y as f32 + 0.5) * config.cell_size;
                let p = Vec2::new(cx, cy);

                if collider_contains_padded(collider, p, padding) {
                    walkable[y * width + x] = false;
                }
            }
        }
    }

    // Step 3: Build regions via flood-fill
    let regions = flood_fill_regions(&walkable, width, height, config.min_region_area);

    // Step 4: Extract region contours and build navmesh polygons
    build_navmesh_from_regions(&regions, width, height, min, config.cell_size)
}

/// Simplified variant: generate navmesh from just collider obstacles
/// within a rectangular area.
#[cfg_attr(feature = "logging", instrument(skip(colliders)))]
#[must_use]
pub fn navmesh_from_colliders_rect(
    area_min: Vec2,
    area_max: Vec2,
    colliders: &[ColliderShape],
    config: &ColliderNavConfig,
) -> NavMesh {
    let boundary = vec![
        area_min,
        Vec2::new(area_max.x, area_min.y),
        area_max,
        Vec2::new(area_min.x, area_max.y),
    ];
    navmesh_from_colliders(&boundary, colliders, config)
}

/// Check if a point is inside a polygon (ray-casting).
#[inline]
fn point_in_polygon(point: Vec2, polygon: &[Vec2]) -> bool {
    let n = polygon.len();
    if n < 3 {
        return false;
    }
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let vi = polygon[i];
        let vj = polygon[j];
        if ((vi.y > point.y) != (vj.y > point.y))
            && (point.x < (vj.x - vi.x) * (point.y - vi.y) / (vj.y - vi.y) + vi.x)
        {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// Check if a collider (with padding) contains a point.
#[inline]
fn collider_contains_padded(collider: &ColliderShape, point: Vec2, padding: f32) -> bool {
    match collider {
        ColliderShape::Circle { center, radius } => {
            let r = radius + padding;
            center.distance_squared(point) <= r * r
        }
        ColliderShape::Aabb { min, max } => {
            point.x >= min.x - padding
                && point.x <= max.x + padding
                && point.y >= min.y - padding
                && point.y <= max.y + padding
        }
        ColliderShape::ConvexPoly { vertices } => {
            // For padding, we check point-in-polygon + distance to edges
            if point_in_polygon(point, vertices) {
                return true;
            }
            if padding > f32::EPSILON {
                // Check distance to each edge
                for i in 0..vertices.len() {
                    let j = (i + 1) % vertices.len();
                    let dist = point_to_segment_dist(point, vertices[i], vertices[j]);
                    if dist <= padding {
                        return true;
                    }
                }
            }
            false
        }
    }
}

/// Distance from a point to a line segment.
#[inline]
fn point_to_segment_dist(p: Vec2, a: Vec2, b: Vec2) -> f32 {
    let ab = b - a;
    let len_sq = ab.length_squared();
    if len_sq < f32::EPSILON {
        return p.distance(a);
    }
    let t = ((p - a).dot(ab) / len_sq).clamp(0.0, 1.0);
    p.distance(a + ab * t)
}

/// Flood-fill to identify connected walkable regions.
fn flood_fill_regions(walkable: &[bool], width: usize, height: usize, min_area: usize) -> Vec<u32> {
    let len = width * height;
    let mut regions = vec![0u32; len];
    let mut current = 1u32;

    for start in 0..len {
        if !walkable[start] || regions[start] != 0 {
            continue;
        }

        let mut stack = vec![start];
        let mut area = 0usize;
        regions[start] = current;

        while let Some(idx) = stack.pop() {
            area += 1;
            let x = idx % width;
            let y = idx / width;

            for (dx, dy) in [(0i32, 1), (0, -1), (1, 0), (-1, 0)] {
                let nx = x as i32 + dx;
                let ny = y as i32 + dy;
                if nx < 0 || ny < 0 || nx >= width as i32 || ny >= height as i32 {
                    continue;
                }
                let ni = ny as usize * width + nx as usize;
                if walkable[ni] && regions[ni] == 0 {
                    regions[ni] = current;
                    stack.push(ni);
                }
            }
        }

        if area < min_area {
            for r in regions.iter_mut() {
                if *r == current {
                    *r = 0;
                }
            }
        } else {
            current += 1;
        }
    }

    regions
}

/// Build navmesh from rasterized regions.
fn build_navmesh_from_regions(
    regions: &[u32],
    width: usize,
    height: usize,
    origin: Vec2,
    cell_size: f32,
) -> NavMesh {
    let max_region = regions.iter().copied().max().unwrap_or(0);
    let mut mesh = NavMesh::new();
    let mut poly_id = 0u32;

    for rid in 1..=max_region {
        // Collect boundary cells for this region
        let mut boundary_points = Vec::new();
        for y in 0..height {
            for x in 0..width {
                if regions[y * width + x] != rid {
                    continue;
                }
                // Check if boundary cell (adjacent to non-region cell or grid edge)
                let is_edge = [(0i32, 1), (0, -1), (1, 0), (-1, 0)]
                    .iter()
                    .any(|&(dx, dy)| {
                        let nx = x as i32 + dx;
                        let ny = y as i32 + dy;
                        if nx < 0 || ny < 0 || nx >= width as i32 || ny >= height as i32 {
                            return true;
                        }
                        regions[ny as usize * width + nx as usize] != rid
                    });
                if is_edge {
                    let wx = origin.x + (x as f32 + 0.5) * cell_size;
                    let wy = origin.y + (y as f32 + 0.5) * cell_size;
                    boundary_points.push(Vec2::new(wx, wy));
                }
            }
        }

        if boundary_points.len() < 3 {
            continue;
        }

        // Compute convex hull
        let hull = convex_hull(&boundary_points);
        if hull.len() < 3 {
            continue;
        }

        mesh.add_poly(NavPoly {
            id: NavPolyId(poly_id),
            vertices: hull,
            neighbors: Vec::new(),
            cost: 1.0,
            layer: 0,
        });
        poly_id += 1;
    }

    // Auto-detect neighbors
    auto_detect_neighbors(&mut mesh);

    mesh
}

/// Andrew's monotone chain convex hull.
#[must_use]
fn convex_hull(points: &[Vec2]) -> Vec<Vec2> {
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
            let n = hull.len();
            let cross = cross2d(hull[n - 2], hull[n - 1], p);
            if cross <= 0.0 {
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
            let n = hull.len();
            let cross = cross2d(hull[n - 2], hull[n - 1], p);
            if cross <= 0.0 {
                hull.pop();
            } else {
                break;
            }
        }
        hull.push(p);
    }

    hull.pop();
    hull
}

#[inline]
fn cross2d(o: Vec2, a: Vec2, b: Vec2) -> f32 {
    (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)
}

/// Auto-detect neighbor polygons by shared vertices (within epsilon distance).
fn auto_detect_neighbors(mesh: &mut NavMesh) {
    let eps = 1e-3;
    let polys: Vec<(NavPolyId, Vec<Vec2>)> = mesh
        .polys()
        .iter()
        .map(|p| (p.id, p.vertices.clone()))
        .collect();

    let mut neighbor_map: Vec<Vec<NavPolyId>> = vec![Vec::new(); polys.len()];
    for i in 0..polys.len() {
        for j in (i + 1)..polys.len() {
            let mut shared = 0;
            for va in &polys[i].1 {
                for vb in &polys[j].1 {
                    if va.distance(*vb) < eps {
                        shared += 1;
                        if shared >= 2 {
                            break;
                        }
                    }
                }
                if shared >= 2 {
                    break;
                }
            }
            if shared >= 2 {
                neighbor_map[i].push(polys[j].0);
                neighbor_map[j].push(polys[i].0);
            }
        }
    }

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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_boundary() {
        let mesh = navmesh_from_colliders(&[], &[], &ColliderNavConfig::default());
        assert_eq!(mesh.poly_count(), 0);
    }

    #[test]
    fn simple_rect_no_colliders() {
        let boundary = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(20.0, 0.0),
            Vec2::new(20.0, 20.0),
            Vec2::new(0.0, 20.0),
        ];
        let mesh = navmesh_from_colliders(
            &boundary,
            &[],
            &ColliderNavConfig {
                cell_size: 1.0,
                agent_radius: 0.0,
                min_region_area: 1,
            },
        );
        assert!(mesh.poly_count() > 0);
    }

    #[test]
    fn rect_with_circle_collider() {
        let boundary = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(20.0, 0.0),
            Vec2::new(20.0, 20.0),
            Vec2::new(0.0, 20.0),
        ];
        let colliders = vec![ColliderShape::Circle {
            center: Vec2::new(10.0, 10.0),
            radius: 3.0,
        }];
        let mesh = navmesh_from_colliders(
            &boundary,
            &colliders,
            &ColliderNavConfig {
                cell_size: 1.0,
                agent_radius: 0.0,
                min_region_area: 1,
            },
        );
        // Should still produce walkable area around the circle
        assert!(mesh.poly_count() > 0);
    }

    #[test]
    fn rect_with_aabb_collider() {
        let boundary = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(20.0, 0.0),
            Vec2::new(20.0, 20.0),
            Vec2::new(0.0, 20.0),
        ];
        let colliders = vec![ColliderShape::Aabb {
            min: Vec2::new(8.0, 8.0),
            max: Vec2::new(12.0, 12.0),
        }];
        let mesh = navmesh_from_colliders(
            &boundary,
            &colliders,
            &ColliderNavConfig {
                cell_size: 1.0,
                agent_radius: 0.0,
                min_region_area: 1,
            },
        );
        assert!(mesh.poly_count() > 0);
    }

    #[test]
    fn rect_convenience() {
        let mesh = navmesh_from_colliders_rect(
            Vec2::ZERO,
            Vec2::new(10.0, 10.0),
            &[],
            &ColliderNavConfig::default(),
        );
        assert!(mesh.poly_count() > 0);
    }

    #[test]
    fn with_agent_radius() {
        let boundary = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(20.0, 0.0),
            Vec2::new(20.0, 20.0),
            Vec2::new(0.0, 20.0),
        ];
        let colliders = vec![ColliderShape::Circle {
            center: Vec2::new(10.0, 10.0),
            radius: 2.0,
        }];
        let mesh = navmesh_from_colliders(
            &boundary,
            &colliders,
            &ColliderNavConfig {
                cell_size: 1.0,
                agent_radius: 1.0,
                min_region_area: 1,
            },
        );
        assert!(mesh.poly_count() > 0);
    }

    #[test]
    fn convex_poly_collider() {
        let boundary = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(20.0, 0.0),
            Vec2::new(20.0, 20.0),
            Vec2::new(0.0, 20.0),
        ];
        let colliders = vec![ColliderShape::ConvexPoly {
            vertices: vec![
                Vec2::new(8.0, 8.0),
                Vec2::new(12.0, 8.0),
                Vec2::new(12.0, 12.0),
                Vec2::new(8.0, 12.0),
            ],
        }];
        let mesh = navmesh_from_colliders(
            &boundary,
            &colliders,
            &ColliderNavConfig {
                cell_size: 1.0,
                agent_radius: 0.0,
                min_region_area: 1,
            },
        );
        assert!(mesh.poly_count() > 0);
    }

    #[test]
    fn collider_config_default() {
        let config = ColliderNavConfig::default();
        assert!(config.cell_size > 0.0);
    }

    #[test]
    fn collider_shape_serde_roundtrip() {
        let shape = ColliderShape::Circle {
            center: Vec2::new(5.0, 5.0),
            radius: 2.0,
        };
        let json = serde_json::to_string(&shape).expect("serialize failed");
        let deserialized: ColliderShape = serde_json::from_str(&json).expect("deserialize failed");
        match deserialized {
            ColliderShape::Circle { center, radius } => {
                assert!((center.x - 5.0).abs() < f32::EPSILON);
                assert!((radius - 2.0).abs() < f32::EPSILON);
            }
            _ => panic!("wrong variant"),
        }
    }

    #[test]
    fn full_coverage_collider_blocks_all() {
        // Collider covers the entire boundary
        let boundary = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(5.0, 0.0),
            Vec2::new(5.0, 5.0),
            Vec2::new(0.0, 5.0),
        ];
        let colliders = vec![ColliderShape::Aabb {
            min: Vec2::new(-1.0, -1.0),
            max: Vec2::new(6.0, 6.0),
        }];
        let mesh = navmesh_from_colliders(
            &boundary,
            &colliders,
            &ColliderNavConfig {
                cell_size: 1.0,
                agent_radius: 0.0,
                min_region_area: 1,
            },
        );
        assert_eq!(mesh.poly_count(), 0);
    }

    #[test]
    fn point_in_polygon_basic() {
        let poly = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(10.0, 0.0),
            Vec2::new(10.0, 10.0),
            Vec2::new(0.0, 10.0),
        ];
        assert!(point_in_polygon(Vec2::new(5.0, 5.0), &poly));
        assert!(!point_in_polygon(Vec2::new(15.0, 5.0), &poly));
    }
}
