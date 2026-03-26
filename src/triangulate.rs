//! Ear-clipping triangulation of simple polygons.

use hisab::Vec2;

/// Triangulate a simple polygon using the ear-clipping algorithm.
///
/// Input: vertices of a simple (non-self-intersecting) polygon in CCW order.
/// Returns a list of triangle index triples `(i, j, k)` into the input slice.
///
/// Returns an empty vec if the polygon has fewer than 3 vertices.
#[must_use]
pub fn triangulate(vertices: &[Vec2]) -> Vec<(usize, usize, usize)> {
    let n = vertices.len();
    if n < 3 {
        return Vec::new();
    }
    if n == 3 {
        return vec![(0, 1, 2)];
    }

    // Ensure CCW winding — if CW, reverse the index list
    let mut indices: Vec<usize> = (0..n).collect();
    if signed_area(vertices) < 0.0 {
        indices.reverse();
    }

    let mut triangles = Vec::with_capacity(n - 2);
    ear_clip(vertices, &mut indices, &mut triangles);
    triangles
}

/// Triangulate and return the actual triangle vertices (not indices).
#[must_use]
pub fn triangulate_points(vertices: &[Vec2]) -> Vec<[Vec2; 3]> {
    triangulate(vertices)
        .into_iter()
        .map(|(a, b, c)| [vertices[a], vertices[b], vertices[c]])
        .collect()
}

/// Signed area of a polygon (positive = CCW, negative = CW).
#[must_use]
fn signed_area(vertices: &[Vec2]) -> f32 {
    let n = vertices.len();
    let mut area = 0.0f32;
    for i in 0..n {
        let j = (i + 1) % n;
        area += vertices[i].perp_dot(vertices[j]);
    }
    area * 0.5
}

/// Core ear-clipping loop.
fn ear_clip(
    vertices: &[Vec2],
    indices: &mut Vec<usize>,
    triangles: &mut Vec<(usize, usize, usize)>,
) {
    let mut remaining = indices.len();

    // Safety counter to prevent infinite loops on degenerate input
    let mut max_iter = remaining * remaining;

    let mut i = 0;
    while remaining > 3 && max_iter > 0 {
        max_iter -= 1;

        let prev = indices[wrap_idx(i as isize - 1, remaining)];
        let curr = indices[i % remaining];
        let next = indices[(i + 1) % remaining];

        if is_ear(vertices, indices, remaining, prev, curr, next) {
            triangles.push((prev, curr, next));
            indices.remove(i % remaining);
            remaining -= 1;
            // Don't advance i — the next vertex shifted into this slot
            if i >= remaining {
                i = 0;
            }
        } else {
            i = (i + 1) % remaining;
        }
    }

    // Last triangle
    if remaining == 3 {
        triangles.push((indices[0], indices[1], indices[2]));
    }
}

/// Check if vertex `curr` forms an ear (convex + no other vertex inside).
fn is_ear(
    vertices: &[Vec2],
    indices: &[usize],
    remaining: usize,
    prev: usize,
    curr: usize,
    next: usize,
) -> bool {
    let a = vertices[prev];
    let b = vertices[curr];
    let c = vertices[next];

    // Must be convex (CCW turn)
    let cross = (b - a).perp_dot(c - b);
    if cross <= 0.0 {
        return false;
    }

    // No other vertex inside this triangle
    for &idx in &indices[..remaining] {
        if idx == prev || idx == curr || idx == next {
            continue;
        }
        if point_in_triangle(vertices[idx], a, b, c) {
            return false;
        }
    }

    true
}

/// Check if point p is inside triangle (a, b, c) using barycentric coordinates.
#[inline]
fn point_in_triangle(p: Vec2, a: Vec2, b: Vec2, c: Vec2) -> bool {
    let v0 = c - a;
    let v1 = b - a;
    let v2 = p - a;

    let dot00 = v0.dot(v0);
    let dot01 = v0.dot(v1);
    let dot02 = v0.dot(v2);
    let dot11 = v1.dot(v1);
    let dot12 = v1.dot(v2);

    let denom = dot00 * dot11 - dot01 * dot01;
    if denom.abs() < f32::EPSILON {
        return false; // degenerate triangle (zero area)
    }
    let inv_denom = 1.0 / denom;
    let u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
    let v = (dot00 * dot12 - dot01 * dot02) * inv_denom;

    u >= 0.0 && v >= 0.0 && (u + v) <= 1.0
}

/// Wrap an index for circular access.
#[inline]
fn wrap_idx(i: isize, len: usize) -> usize {
    ((i % len as isize) + len as isize) as usize % len
}

/// Merge a set of triangles into larger convex polygons.
///
/// Takes triangles as index triples into a vertex array and greedily merges
/// pairs that share an edge, as long as the result remains convex.
///
/// Returns polygons as lists of vertex indices (CCW order).
#[must_use]
pub fn merge_convex(vertices: &[Vec2], triangles: &[(usize, usize, usize)]) -> Vec<Vec<usize>> {
    if triangles.is_empty() {
        return Vec::new();
    }

    // Start with each triangle as a polygon
    let mut polys: Vec<Option<Vec<usize>>> = triangles
        .iter()
        .map(|&(a, b, c)| Some(vec![a, b, c]))
        .collect();

    // Build edge → polygon index map for shared-edge detection
    // Edge key: (min_vertex, max_vertex) → list of polygon indices
    let mut edge_map: std::collections::HashMap<(usize, usize), Vec<usize>> =
        std::collections::HashMap::new();

    for (pi, poly) in polys.iter().enumerate() {
        if let Some(verts) = poly {
            for i in 0..verts.len() {
                let j = (i + 1) % verts.len();
                let key = edge_key(verts[i], verts[j]);
                edge_map.entry(key).or_default().push(pi);
            }
        }
    }

    // Greedily merge pairs that share an edge
    let mut merged = true;
    while merged {
        merged = false;
        let keys: Vec<(usize, usize)> = edge_map.keys().copied().collect();

        for key in keys {
            let owners = match edge_map.get(&key) {
                Some(o) if o.len() == 2 => [o[0], o[1]],
                _ => continue,
            };

            let (pa, pb) = (owners[0], owners[1]);
            if polys[pa].is_none() || polys[pb].is_none() {
                continue;
            }

            let poly_a = polys[pa].as_ref().unwrap().clone();
            let poly_b = polys[pb].as_ref().unwrap().clone();

            if let Some(merged_poly) = try_merge(vertices, &poly_a, &poly_b, key) {
                // Remove old edges from map
                remove_poly_edges(&mut edge_map, &poly_a, pa);
                remove_poly_edges(&mut edge_map, &poly_b, pb);

                // Insert new polygon
                let new_idx = pa;
                polys[pa] = Some(merged_poly.clone());
                polys[pb] = None;

                // Add new polygon's edges to map
                for i in 0..merged_poly.len() {
                    let j = (i + 1) % merged_poly.len();
                    let ek = edge_key(merged_poly[i], merged_poly[j]);
                    edge_map.entry(ek).or_default().push(new_idx);
                }

                merged = true;
                break; // restart scan
            }
        }
    }

    polys.into_iter().flatten().collect()
}

/// Try to merge two polygons along a shared edge. Returns the merged polygon
/// if the result is convex, or None if it would be concave.
fn try_merge(
    vertices: &[Vec2],
    poly_a: &[usize],
    poly_b: &[usize],
    shared_edge: (usize, usize),
) -> Option<Vec<usize>> {
    let (ea, eb) = shared_edge;

    // Find shared edge position in poly_a (as consecutive pair)
    let a_len = poly_a.len();
    let edge_in_a = (0..a_len).find(|&i| {
        let j = (i + 1) % a_len;
        edge_key(poly_a[i], poly_a[j]) == (ea, eb)
    })?;
    let a_end = (edge_in_a + 1) % a_len;

    // Find shared edge position in poly_b
    let b_len = poly_b.len();
    let edge_in_b = (0..b_len).find(|&i| {
        let j = (i + 1) % b_len;
        edge_key(poly_b[i], poly_b[j]) == (ea, eb)
    })?;

    // Build merged polygon:
    // Walk poly_a: include all vertices from after shared-edge-end,
    // around back to shared-edge-start (inclusive).
    // Then walk poly_b: include non-shared vertices between the
    // shared edge endpoints.
    let mut result = Vec::with_capacity(a_len + b_len - 2);

    // poly_a: from vertex after shared edge's second endpoint,
    // around to shared edge's first endpoint (inclusive)
    {
        let mut ai = (a_end + 1) % a_len;
        loop {
            result.push(poly_a[ai]);
            if ai == edge_in_a {
                break;
            }
            ai = (ai + 1) % a_len;
        }
    }

    // poly_b: walk from after shared edge's second endpoint to
    // before shared edge's first endpoint (non-shared vertices only)
    {
        let b_end = (edge_in_b + 1) % b_len;
        let mut bi = (b_end + 1) % b_len;
        while bi != edge_in_b {
            result.push(poly_b[bi]);
            bi = (bi + 1) % b_len;
        }
    }

    // Include poly_a's shared edge second endpoint to close the seam
    result.push(poly_a[a_end]);

    if result.len() < 3 {
        return None;
    }

    if is_convex(vertices, &result) {
        Some(result)
    } else {
        None
    }
}

/// Check if a polygon (given as vertex indices) is convex.
fn is_convex(vertices: &[Vec2], indices: &[usize]) -> bool {
    let n = indices.len();
    if n < 3 {
        return false;
    }
    for i in 0..n {
        let a = vertices[indices[i]];
        let b = vertices[indices[(i + 1) % n]];
        let c = vertices[indices[(i + 2) % n]];
        if (b - a).perp_dot(c - b) < -f32::EPSILON {
            return false;
        }
    }
    true
}

/// Canonical edge key (min, max).
#[inline]
fn edge_key(a: usize, b: usize) -> (usize, usize) {
    if a <= b { (a, b) } else { (b, a) }
}

/// Remove a polygon's edges from the edge map.
fn remove_poly_edges(
    edge_map: &mut std::collections::HashMap<(usize, usize), Vec<usize>>,
    poly: &[usize],
    poly_idx: usize,
) {
    for i in 0..poly.len() {
        let j = (i + 1) % poly.len();
        let key = edge_key(poly[i], poly[j]);
        if let Some(owners) = edge_map.get_mut(&key) {
            owners.retain(|&x| x != poly_idx);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn triangle_passthrough() {
        let verts = vec![Vec2::ZERO, Vec2::new(1.0, 0.0), Vec2::new(0.5, 1.0)];
        let tris = triangulate(&verts);
        assert_eq!(tris.len(), 1);
    }

    #[test]
    fn square_two_triangles() {
        let verts = vec![
            Vec2::ZERO,
            Vec2::new(1.0, 0.0),
            Vec2::new(1.0, 1.0),
            Vec2::new(0.0, 1.0),
        ];
        let tris = triangulate(&verts);
        assert_eq!(tris.len(), 2);
    }

    #[test]
    fn pentagon() {
        // Regular pentagon (CCW)
        let verts: Vec<Vec2> = (0..5)
            .map(|i| {
                let angle = std::f32::consts::TAU * i as f32 / 5.0;
                Vec2::new(angle.cos(), angle.sin())
            })
            .collect();
        let tris = triangulate(&verts);
        assert_eq!(tris.len(), 3); // n-2 triangles
    }

    #[test]
    fn concave_l_shape() {
        // L-shaped polygon (6 vertices, CCW)
        let verts = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(2.0, 0.0),
            Vec2::new(2.0, 1.0),
            Vec2::new(1.0, 1.0),
            Vec2::new(1.0, 2.0),
            Vec2::new(0.0, 2.0),
        ];
        let tris = triangulate(&verts);
        assert_eq!(tris.len(), 4); // 6-2 = 4

        // All triangles should have positive area
        for (a, b, c) in &tris {
            let va = verts[*a];
            let vb = verts[*b];
            let vc = verts[*c];
            let area = (vb - va).perp_dot(vc - va);
            assert!(area > 0.0, "degenerate triangle: ({a},{b},{c})");
        }
    }

    #[test]
    fn cw_polygon_handled() {
        // CW square — should still triangulate correctly
        let verts = vec![
            Vec2::ZERO,
            Vec2::new(0.0, 1.0),
            Vec2::new(1.0, 1.0),
            Vec2::new(1.0, 0.0),
        ];
        let tris = triangulate(&verts);
        assert_eq!(tris.len(), 2);
    }

    #[test]
    fn degenerate_too_few() {
        assert!(triangulate(&[]).is_empty());
        assert!(triangulate(&[Vec2::ZERO]).is_empty());
        assert!(triangulate(&[Vec2::ZERO, Vec2::ONE]).is_empty());
    }

    #[test]
    fn triangulate_points_returns_vertices() {
        let verts = vec![
            Vec2::ZERO,
            Vec2::new(1.0, 0.0),
            Vec2::new(1.0, 1.0),
            Vec2::new(0.0, 1.0),
        ];
        let tris = triangulate_points(&verts);
        assert_eq!(tris.len(), 2);
        // Each triangle should have 3 vertices from the original set
        for tri in &tris {
            for v in tri {
                assert!(verts.contains(v));
            }
        }
    }

    #[test]
    fn all_indices_valid() {
        let verts = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(3.0, 0.0),
            Vec2::new(3.0, 1.0),
            Vec2::new(2.0, 1.0),
            Vec2::new(2.0, 2.0),
            Vec2::new(0.0, 2.0),
        ];
        let tris = triangulate(&verts);
        for (a, b, c) in &tris {
            assert!(*a < verts.len());
            assert!(*b < verts.len());
            assert!(*c < verts.len());
        }
    }

    #[test]
    fn large_convex_polygon() {
        // 20-gon
        let verts: Vec<Vec2> = (0..20)
            .map(|i| {
                let angle = std::f32::consts::TAU * i as f32 / 20.0;
                Vec2::new(angle.cos() * 10.0, angle.sin() * 10.0)
            })
            .collect();
        let tris = triangulate(&verts);
        assert_eq!(tris.len(), 18); // n-2
    }

    // --- Merge convex tests ---

    #[test]
    fn merge_square_triangles() {
        // Two triangles forming a square should merge into one quad
        let verts = vec![
            Vec2::ZERO,
            Vec2::new(1.0, 0.0),
            Vec2::new(1.0, 1.0),
            Vec2::new(0.0, 1.0),
        ];
        let tris = triangulate(&verts);
        let merged = merge_convex(&verts, &tris);

        // Should merge into 1 convex polygon (the original square)
        assert_eq!(merged.len(), 1);
        assert_eq!(merged[0].len(), 4);
    }

    #[test]
    fn merge_concave_stays_split() {
        // L-shape: triangles shouldn't all merge (result would be concave)
        let verts = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(2.0, 0.0),
            Vec2::new(2.0, 1.0),
            Vec2::new(1.0, 1.0),
            Vec2::new(1.0, 2.0),
            Vec2::new(0.0, 2.0),
        ];
        let tris = triangulate(&verts);
        let merged = merge_convex(&verts, &tris);

        // Should have fewer polygons than triangles but more than 1
        assert!(merged.len() < tris.len());
        assert!(!merged.is_empty());

        // All resulting polygons should be convex
        for poly in &merged {
            assert!(is_convex(&verts, poly), "non-convex merged polygon");
        }
    }

    #[test]
    fn merge_empty() {
        let merged = merge_convex(&[], &[]);
        assert!(merged.is_empty());
    }

    #[test]
    fn merge_single_triangle() {
        let verts = vec![Vec2::ZERO, Vec2::new(1.0, 0.0), Vec2::new(0.5, 1.0)];
        let tris = vec![(0, 1, 2)];
        let merged = merge_convex(&verts, &tris);
        assert_eq!(merged.len(), 1);
        assert_eq!(merged[0].len(), 3);
    }

    #[test]
    fn merge_convex_polygon_fully_merges() {
        // Regular hexagon → 4 triangles → should merge back to ≤2 convex polygons
        let verts: Vec<Vec2> = (0..6)
            .map(|i| {
                let angle = std::f32::consts::TAU * i as f32 / 6.0;
                Vec2::new(angle.cos(), angle.sin())
            })
            .collect();
        let tris = triangulate(&verts);
        assert_eq!(tris.len(), 4);

        let merged = merge_convex(&verts, &tris);
        // Hexagon is convex, so should merge significantly
        assert!(merged.len() <= 2);
    }
}
