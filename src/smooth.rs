//! Path smoothing — funnel algorithm for natural-looking paths.
//!
//! Contains both the simplified waypoint string-pulling (`funnel_smooth`) and
//! the proper Simple Stupid Funnel Algorithm (`funnel_portals`) that operates
//! on portal edges between navmesh polygons.

use hisab::Vec2;
use serde::{Deserialize, Serialize};

#[cfg(feature = "logging")]
use tracing::instrument;

use crate::mesh::{NavMesh, NavPolyId};

/// A portal edge between two adjacent navmesh polygons.
///
/// The left/right convention is relative to the direction of travel
/// from the polygon sequence's start to end.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Portal {
    /// Left vertex of the portal edge.
    pub left: Vec2,
    /// Right vertex of the portal edge.
    pub right: Vec2,
}

/// Apply the simple string-pulling (funnel) smoothing algorithm.
///
/// Given a sequence of waypoints, removes redundant intermediate points
/// where a direct line of sight exists. This produces smoother, more
/// natural-looking paths.
///
/// This is a simplified version that works on the waypoint list directly.
/// A full funnel algorithm would work on portal edges between navmesh polygons.
#[must_use]
pub fn funnel_smooth(waypoints: &[Vec2]) -> Vec<Vec2> {
    if waypoints.len() <= 2 {
        return waypoints.to_vec();
    }

    let mut result = Vec::with_capacity(waypoints.len());
    result.push(waypoints[0]);

    let mut anchor = 0;
    let mut i = 2;

    while i < waypoints.len() {
        // Check if we can skip the intermediate point
        // Simple approach: keep the point if direction changes significantly
        let prev = waypoints[anchor];
        let mid = waypoints[i - 1];
        let next = waypoints[i];

        let d1 = mid - prev;
        let d2 = next - mid;

        let cross = d1.perp_dot(d2);
        let dot = d1.dot(d2);

        // If direction changes significantly (cross product threshold),
        // keep the intermediate point
        let len1 = d1.length();
        let len2 = d2.length();
        let denom = len1 * len2;

        if denom > f32::EPSILON {
            let sin_angle = cross.abs() / denom;
            let cos_angle = dot / denom;

            // Keep point if turn angle > ~15 degrees or direction reverses
            if sin_angle > 0.26 || cos_angle < 0.0 {
                result.push(mid);
                anchor = i - 1;
            }
        }

        i += 1;
    }

    result.push(waypoints[waypoints.len() - 1]);
    result
}

/// Extract portal edges from a navmesh polygon path.
///
/// Given a sequence of polygon IDs (as returned by `NavMesh::find_path`),
/// finds the shared edges between consecutive polygons.
///
/// Returns `None` if any consecutive pair doesn't share an edge.
#[cfg_attr(feature = "logging", instrument(skip(mesh)))]
#[must_use]
pub fn extract_portals(
    mesh: &NavMesh,
    poly_path: &[NavPolyId],
    start: Vec2,
    goal: Vec2,
) -> Option<Vec<Portal>> {
    if poly_path.len() <= 1 {
        return Some(vec![]);
    }

    let mut portals = Vec::with_capacity(poly_path.len() + 1);

    // Start portal: degenerate (start point)
    portals.push(Portal {
        left: start,
        right: start,
    });

    for i in 0..poly_path.len() - 1 {
        let poly_a = mesh.get_poly(poly_path[i])?;
        let poly_b = mesh.get_poly(poly_path[i + 1])?;

        // Find shared edge between poly_a and poly_b
        let shared = find_shared_edge(&poly_a.vertices, &poly_b.vertices)?;
        portals.push(shared);
    }

    // End portal: degenerate (goal point)
    portals.push(Portal {
        left: goal,
        right: goal,
    });

    Some(portals)
}

/// Simple Stupid Funnel Algorithm — compute a smooth path through portal edges.
///
/// This is the proper portal-based funnel algorithm that produces optimal
/// paths through a navmesh polygon corridor. It processes the portal sequence
/// from [`extract_portals`] and outputs the shortest path that stays within
/// the corridor.
///
/// `agent_radius` shrinks portal edges inward by the agent's radius to
/// ensure the path has clearance. Pass `0.0` for a point agent.
#[cfg_attr(feature = "logging", instrument)]
#[must_use]
pub fn funnel_portals(portals: &[Portal], agent_radius: f32) -> Vec<Vec2> {
    if portals.is_empty() {
        return Vec::new();
    }
    if portals.len() == 1 {
        return vec![portals[0].left];
    }

    // Shrink portals by agent radius
    let shrunk: Vec<Portal> = if agent_radius > f32::EPSILON {
        portals
            .iter()
            .map(|p| shrink_portal(*p, agent_radius))
            .collect()
    } else {
        portals.to_vec()
    };

    let mut path = Vec::new();
    #[allow(unused_assignments)]
    let mut apex_idx: usize = 0;
    let mut left_idx: usize = 0;
    let mut right_idx: usize = 0;

    let mut apex = shrunk[0].left;
    let mut funnel_left = shrunk[0].left;
    let mut funnel_right = shrunk[0].right;

    path.push(apex);

    let n = shrunk.len();
    let mut i = 1;
    while i < n {
        let pl = shrunk[i].left;
        let pr = shrunk[i].right;

        // Update right funnel side
        if tri_area2(apex, funnel_right, pr) <= 0.0 {
            if vec2_approx_eq(apex, funnel_right) || tri_area2(apex, funnel_left, pr) > 0.0 {
                // Tighten the funnel
                funnel_right = pr;
                right_idx = i;
            } else {
                // Right crossed left — add left as new apex
                path.push(funnel_left);
                apex = funnel_left;
                apex_idx = left_idx;

                funnel_left = apex;
                funnel_right = apex;
                left_idx = apex_idx;
                right_idx = apex_idx;
                i = apex_idx + 1;
                continue;
            }
        }

        // Update left funnel side
        if tri_area2(apex, funnel_left, pl) >= 0.0 {
            if vec2_approx_eq(apex, funnel_left) || tri_area2(apex, funnel_right, pl) < 0.0 {
                // Tighten the funnel
                funnel_left = pl;
                left_idx = i;
            } else {
                // Left crossed right — add right as new apex
                path.push(funnel_right);
                apex = funnel_right;
                apex_idx = right_idx;

                funnel_left = apex;
                funnel_right = apex;
                left_idx = apex_idx;
                right_idx = apex_idx;
                i = apex_idx + 1;
                continue;
            }
        }

        i += 1;
    }

    // Add goal
    let goal = shrunk[n - 1].left;
    if path.last().is_none_or(|&p| !vec2_approx_eq(p, goal)) {
        path.push(goal);
    }

    path
}

/// Find the shared edge between two convex polygons.
///
/// Returns the edge as a [`Portal`] oriented so that left is on the left side
/// when walking from polygon A through to polygon B.
fn find_shared_edge(verts_a: &[Vec2], verts_b: &[Vec2]) -> Option<Portal> {
    let eps = 1e-4;
    for i in 0..verts_a.len() {
        let a0 = verts_a[i];
        let a1 = verts_a[(i + 1) % verts_a.len()];
        for j in 0..verts_b.len() {
            let b0 = verts_b[j];
            let b1 = verts_b[(j + 1) % verts_b.len()];
            // Shared edge: vertices match (opposite winding between adjacent polys)
            if (a0.distance(b1) < eps && a1.distance(b0) < eps)
                || (a0.distance(b0) < eps && a1.distance(b1) < eps)
            {
                return Some(Portal {
                    left: a0,
                    right: a1,
                });
            }
        }
    }
    None
}

/// Signed triangle area × 2 (positive = CCW).
#[inline]
fn tri_area2(a: Vec2, b: Vec2, c: Vec2) -> f32 {
    (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)
}

/// Approximate [`Vec2`] equality.
#[inline]
fn vec2_approx_eq(a: Vec2, b: Vec2) -> bool {
    (a.x - b.x).abs() < 1e-5 && (a.y - b.y).abs() < 1e-5
}

/// Shrink a portal edge inward by `radius`.
#[inline]
fn shrink_portal(portal: Portal, radius: f32) -> Portal {
    let edge = portal.right - portal.left;
    let len = edge.length();
    if len < radius * 2.0 + f32::EPSILON {
        // Portal too narrow — collapse to midpoint
        let mid = (portal.left + portal.right) * 0.5;
        return Portal {
            left: mid,
            right: mid,
        };
    }
    let dir = edge / len;
    Portal {
        left: portal.left + dir * radius,
        right: portal.right - dir * radius,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn smooth_empty() {
        let result = funnel_smooth(&[]);
        assert!(result.is_empty());
    }

    #[test]
    fn smooth_single() {
        let result = funnel_smooth(&[Vec2::new(1.0, 2.0)]);
        assert_eq!(result.len(), 1);
    }

    #[test]
    fn smooth_two_points() {
        let result = funnel_smooth(&[Vec2::ZERO, Vec2::ONE]);
        assert_eq!(result.len(), 2);
    }

    #[test]
    fn smooth_straight_line() {
        // Collinear points should be simplified
        let input: Vec<Vec2> = (0..5).map(|i| Vec2::new(i as f32, i as f32)).collect();
        let result = funnel_smooth(&input);
        // Should keep fewer points than input (removes collinear intermediates)
        assert!(result.len() <= input.len());
        assert_eq!(result[0], input[0]);
        assert_eq!(*result.last().unwrap(), *input.last().unwrap());
    }

    #[test]
    fn smooth_right_angle() {
        // 90-degree turn should keep the corner
        let input = vec![Vec2::ZERO, Vec2::new(5.0, 0.0), Vec2::new(5.0, 5.0)];
        let result = funnel_smooth(&input);
        assert_eq!(result.len(), 3); // corner must be kept
    }

    #[test]
    fn smooth_preserves_endpoints() {
        let input = vec![
            Vec2::ZERO,
            Vec2::new(1.0, 0.5),
            Vec2::new(2.0, 0.0),
            Vec2::new(3.0, 0.5),
            Vec2::new(4.0, 0.0),
        ];
        let result = funnel_smooth(&input);
        assert_eq!(result[0], Vec2::ZERO);
        assert_eq!(*result.last().unwrap(), Vec2::new(4.0, 0.0));
    }

    #[test]
    fn smooth_u_turn() {
        // Path goes right, then reverses left — u-turn must be kept
        let input = vec![Vec2::ZERO, Vec2::new(5.0, 0.0), Vec2::ZERO];
        let result = funnel_smooth(&input);
        assert_eq!(result.len(), 3);
    }

    #[test]
    fn smooth_long_collinear() {
        // 20 collinear points should simplify to just 2
        let input: Vec<Vec2> = (0..20).map(|i| Vec2::new(i as f32, 0.0)).collect();
        let result = funnel_smooth(&input);
        assert_eq!(result.len(), 2);
        assert_eq!(result[0], Vec2::ZERO);
        assert_eq!(result[1], Vec2::new(19.0, 0.0));
    }

    // --- Portal shrinking tests ---

    #[test]
    fn portal_shrink_basic() {
        let p = Portal {
            left: Vec2::ZERO,
            right: Vec2::new(10.0, 0.0),
        };
        let shrunk = shrink_portal(p, 1.0);
        assert!((shrunk.left.x - 1.0).abs() < 0.01);
        assert!((shrunk.right.x - 9.0).abs() < 0.01);
    }

    #[test]
    fn portal_shrink_narrow_collapses() {
        let p = Portal {
            left: Vec2::ZERO,
            right: Vec2::new(1.0, 0.0),
        };
        let shrunk = shrink_portal(p, 2.0);
        assert!((shrunk.left.x - 0.5).abs() < 0.01);
        assert!((shrunk.right.x - 0.5).abs() < 0.01);
    }

    // --- Funnel portal tests ---

    #[test]
    fn funnel_straight_corridor() {
        // Simple straight corridor — 3 portals (start, middle, end)
        let portals = vec![
            Portal {
                left: Vec2::new(0.0, 0.0),
                right: Vec2::new(0.0, 0.0),
            },
            Portal {
                left: Vec2::new(5.0, 2.0),
                right: Vec2::new(5.0, -2.0),
            },
            Portal {
                left: Vec2::new(10.0, 0.0),
                right: Vec2::new(10.0, 0.0),
            },
        ];
        let path = funnel_portals(&portals, 0.0);
        assert!(path.len() >= 2);
        assert!((path[0].x).abs() < 0.01);
        assert!((path.last().unwrap().x - 10.0).abs() < 0.01);
    }

    #[test]
    fn funnel_with_agent_radius() {
        let portals = vec![
            Portal {
                left: Vec2::new(0.0, 0.0),
                right: Vec2::new(0.0, 0.0),
            },
            Portal {
                left: Vec2::new(5.0, 2.0),
                right: Vec2::new(5.0, -2.0),
            },
            Portal {
                left: Vec2::new(10.0, 0.0),
                right: Vec2::new(10.0, 0.0),
            },
        ];
        let path = funnel_portals(&portals, 0.5);
        assert!(path.len() >= 2);
    }

    #[test]
    fn funnel_single_portal() {
        let portals = vec![Portal {
            left: Vec2::new(5.0, 5.0),
            right: Vec2::new(5.0, 5.0),
        }];
        let path = funnel_portals(&portals, 0.0);
        assert_eq!(path.len(), 1);
    }

    #[test]
    fn funnel_empty() {
        let path = funnel_portals(&[], 0.0);
        assert!(path.is_empty());
    }

    #[test]
    fn funnel_l_shaped_corridor() {
        // Corridor turns 90 degrees
        let portals = vec![
            Portal {
                left: Vec2::new(0.0, 0.0),
                right: Vec2::new(0.0, 0.0),
            },
            Portal {
                left: Vec2::new(5.0, 1.0),
                right: Vec2::new(5.0, -1.0),
            },
            Portal {
                left: Vec2::new(6.0, 5.0),
                right: Vec2::new(4.0, 5.0),
            },
            Portal {
                left: Vec2::new(10.0, 10.0),
                right: Vec2::new(10.0, 10.0),
            },
        ];
        let path = funnel_portals(&portals, 0.0);
        assert!(path.len() >= 2);
        // First point is start, last is goal
        assert!((path[0] - Vec2::ZERO).length() < 0.01);
        assert!(path.last().unwrap().distance(Vec2::new(10.0, 10.0)) < 0.01);
    }

    #[test]
    fn portal_serde_roundtrip() {
        let p = Portal {
            left: Vec2::new(1.0, 2.0),
            right: Vec2::new(3.0, 4.0),
        };
        let json = serde_json::to_string(&p).unwrap();
        let deserialized: Portal = serde_json::from_str(&json).unwrap();
        assert!((deserialized.left.x - 1.0).abs() < f32::EPSILON);
    }

    #[test]
    fn extract_portals_single_poly() {
        use crate::mesh::{NavMesh, NavPoly, NavPolyId};

        let mut mesh = NavMesh::new();
        mesh.add_poly(NavPoly {
            id: NavPolyId(0),
            vertices: vec![Vec2::ZERO, Vec2::new(10.0, 0.0), Vec2::new(5.0, 10.0)],
            neighbors: vec![],
            cost: 1.0,
            layer: 0,
        });

        let portals = extract_portals(
            &mesh,
            &[NavPolyId(0)],
            Vec2::new(2.0, 2.0),
            Vec2::new(5.0, 5.0),
        );
        assert!(portals.is_some());
        assert!(portals.unwrap().is_empty());
    }
}
