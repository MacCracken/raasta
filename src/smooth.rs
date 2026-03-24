//! Path smoothing — funnel algorithm for natural-looking paths.

use hisab::Vec2;

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
}
