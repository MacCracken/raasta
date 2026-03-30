//! Path request/result types.

use hisab::Vec2;
use serde::{Deserialize, Serialize};

/// Status of a pathfinding request.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
pub enum PathStatus {
    /// Path found successfully.
    Found,
    /// Path could not reach the goal; this is the closest reachable point.
    Partial,
    /// No path exists between start and goal.
    NotFound,
    /// Start or goal is outside navigable area.
    Invalid,
    /// Path computation is still in progress (for async).
    Pending,
}

/// A pathfinding request.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PathRequest {
    /// Start position in world space.
    pub start: Vec2,
    /// Goal position in world space.
    pub goal: Vec2,
}

impl PathRequest {
    #[must_use]
    pub fn new(start: Vec2, goal: Vec2) -> Self {
        Self { start, goal }
    }
}

/// Result of a pathfinding computation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PathResult {
    pub status: PathStatus,
    /// Waypoints from start to goal in world space.
    pub waypoints: Vec<Vec2>,
    /// Total path length.
    pub length: f32,
}

impl PathResult {
    /// Create a successful path result.
    #[must_use]
    pub fn found(waypoints: Vec<Vec2>) -> Self {
        let length = waypoints.windows(2).map(|w| (w[1] - w[0]).length()).sum();
        Self {
            status: PathStatus::Found,
            waypoints,
            length,
        }
    }

    /// Create a partial path result (closest reachable point to unreachable goal).
    #[must_use]
    pub fn partial(waypoints: Vec<Vec2>) -> Self {
        let length = waypoints.windows(2).map(|w| (w[1] - w[0]).length()).sum();
        Self {
            status: PathStatus::Partial,
            waypoints,
            length,
        }
    }

    /// Create a not-found result.
    #[must_use]
    pub fn not_found() -> Self {
        Self {
            status: PathStatus::NotFound,
            waypoints: Vec::new(),
            length: 0.0,
        }
    }

    /// Create an invalid result.
    #[must_use]
    pub fn invalid() -> Self {
        Self {
            status: PathStatus::Invalid,
            waypoints: Vec::new(),
            length: 0.0,
        }
    }

    /// Whether a path was found.
    #[must_use]
    pub fn is_found(&self) -> bool {
        self.status == PathStatus::Found
    }

    /// Whether the path is partial (closest reachable point to unreachable goal).
    #[must_use]
    pub fn is_partial(&self) -> bool {
        self.status == PathStatus::Partial
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn path_result_found() {
        let r = PathResult::found(vec![Vec2::ZERO, Vec2::new(3.0, 4.0)]);
        assert!(r.is_found());
        assert!((r.length - 5.0).abs() < 0.01);
    }

    #[test]
    fn path_result_partial() {
        let r = PathResult::partial(vec![Vec2::ZERO, Vec2::new(3.0, 4.0)]);
        assert!(r.is_partial());
        assert!(!r.is_found());
        assert!((r.length - 5.0).abs() < 0.01);
    }

    #[test]
    fn path_result_not_found() {
        let r = PathResult::not_found();
        assert!(!r.is_found());
        assert_eq!(r.waypoints.len(), 0);
    }

    #[test]
    fn path_result_invalid() {
        let r = PathResult::invalid();
        assert_eq!(r.status, PathStatus::Invalid);
    }

    #[test]
    fn path_request_new() {
        let req = PathRequest::new(Vec2::new(1.0, 2.0), Vec2::new(3.0, 4.0));
        assert_eq!(req.start, Vec2::new(1.0, 2.0));
        assert_eq!(req.goal, Vec2::new(3.0, 4.0));
    }

    #[test]
    fn path_result_zero_length() {
        let r = PathResult::found(vec![Vec2::new(5.0, 5.0)]);
        assert!((r.length - 0.0).abs() < f32::EPSILON);
    }

    #[test]
    fn path_result_serde_roundtrip() {
        let r = PathResult::found(vec![Vec2::ZERO, Vec2::ONE]);
        let json = serde_json::to_string(&r).unwrap();
        let deserialized: PathResult = serde_json::from_str(&json).unwrap();
        assert_eq!(deserialized.status, PathStatus::Found);
        assert_eq!(deserialized.waypoints.len(), 2);
    }

    #[test]
    fn path_result_multi_segment_length() {
        // 3-4-5 triangle: (0,0) -> (3,0) -> (3,4)
        let r = PathResult::found(vec![Vec2::ZERO, Vec2::new(3.0, 0.0), Vec2::new(3.0, 4.0)]);
        // Length should be 3 + 4 = 7
        assert!((r.length - 7.0).abs() < 0.01);
    }

    #[test]
    fn path_result_empty_waypoints() {
        let r = PathResult::found(vec![]);
        assert!(r.is_found());
        assert!((r.length - 0.0).abs() < f32::EPSILON);
    }

    #[test]
    fn path_status_serde_roundtrip() {
        let status = PathStatus::Pending;
        let json = serde_json::to_string(&status).unwrap();
        let deserialized: PathStatus = serde_json::from_str(&json).unwrap();
        assert_eq!(deserialized, status);
    }
}
