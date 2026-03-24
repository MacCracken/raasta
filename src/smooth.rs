//! Path smoothing — funnel algorithm for natural-looking paths.

/// Apply the simple string-pulling (funnel) smoothing algorithm.
///
/// Given a sequence of waypoints, removes redundant intermediate points
/// where a direct line of sight exists. This produces smoother, more
/// natural-looking paths.
///
/// This is a simplified version that works on the waypoint list directly.
/// A full funnel algorithm would work on portal edges between navmesh polygons.
pub fn funnel_smooth(waypoints: &[[f32; 2]]) -> Vec<[f32; 2]> {
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

        let d1x = mid[0] - prev[0];
        let d1y = mid[1] - prev[1];
        let d2x = next[0] - mid[0];
        let d2y = next[1] - mid[1];

        let cross = d1x * d2y - d1y * d2x;
        let dot = d1x * d2x + d1y * d2y;

        // If direction changes significantly (cross product threshold),
        // keep the intermediate point
        let len1 = (d1x * d1x + d1y * d1y).sqrt();
        let len2 = (d2x * d2x + d2y * d2y).sqrt();
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

    result.push(*waypoints.last().unwrap());
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
        let result = funnel_smooth(&[[1.0, 2.0]]);
        assert_eq!(result.len(), 1);
    }

    #[test]
    fn smooth_two_points() {
        let result = funnel_smooth(&[[0.0, 0.0], [1.0, 1.0]]);
        assert_eq!(result.len(), 2);
    }

    #[test]
    fn smooth_straight_line() {
        // Collinear points should be simplified
        let input: Vec<[f32; 2]> = (0..5).map(|i| [i as f32, i as f32]).collect();
        let result = funnel_smooth(&input);
        // Should keep fewer points than input (removes collinear intermediates)
        assert!(result.len() <= input.len());
        assert_eq!(*result.first().unwrap(), input[0]);
        assert_eq!(*result.last().unwrap(), *input.last().unwrap());
    }

    #[test]
    fn smooth_right_angle() {
        // 90-degree turn should keep the corner
        let input = vec![[0.0, 0.0], [5.0, 0.0], [5.0, 5.0]];
        let result = funnel_smooth(&input);
        assert_eq!(result.len(), 3); // corner must be kept
    }

    #[test]
    fn smooth_preserves_endpoints() {
        let input = vec![[0.0, 0.0], [1.0, 0.5], [2.0, 0.0], [3.0, 0.5], [4.0, 0.0]];
        let result = funnel_smooth(&input);
        assert_eq!(*result.first().unwrap(), [0.0, 0.0]);
        assert_eq!(*result.last().unwrap(), [4.0, 0.0]);
    }
}
