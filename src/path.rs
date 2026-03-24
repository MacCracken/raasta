//! Path request/result types.

use serde::{Deserialize, Serialize};

/// Status of a pathfinding request.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
pub enum PathStatus {
    /// Path found successfully.
    Found,
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
    pub start: [f32; 2],
    /// Goal position in world space.
    pub goal: [f32; 2],
}

impl PathRequest {
    pub fn new(start: [f32; 2], goal: [f32; 2]) -> Self {
        Self { start, goal }
    }
}

/// Result of a pathfinding computation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PathResult {
    pub status: PathStatus,
    /// Waypoints from start to goal in world space.
    pub waypoints: Vec<[f32; 2]>,
    /// Total path length.
    pub length: f32,
}

impl PathResult {
    /// Create a successful path result.
    pub fn found(waypoints: Vec<[f32; 2]>) -> Self {
        let length = waypoints
            .windows(2)
            .map(|w| {
                let dx = w[1][0] - w[0][0];
                let dy = w[1][1] - w[0][1];
                (dx * dx + dy * dy).sqrt()
            })
            .sum();
        Self {
            status: PathStatus::Found,
            waypoints,
            length,
        }
    }

    /// Create a not-found result.
    pub fn not_found() -> Self {
        Self {
            status: PathStatus::NotFound,
            waypoints: Vec::new(),
            length: 0.0,
        }
    }

    /// Create an invalid result.
    pub fn invalid() -> Self {
        Self {
            status: PathStatus::Invalid,
            waypoints: Vec::new(),
            length: 0.0,
        }
    }

    /// Whether a path was found.
    pub fn is_found(&self) -> bool {
        self.status == PathStatus::Found
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn path_result_found() {
        let r = PathResult::found(vec![[0.0, 0.0], [3.0, 4.0]]);
        assert!(r.is_found());
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
        let req = PathRequest::new([1.0, 2.0], [3.0, 4.0]);
        assert_eq!(req.start, [1.0, 2.0]);
        assert_eq!(req.goal, [3.0, 4.0]);
    }

    #[test]
    fn path_result_zero_length() {
        let r = PathResult::found(vec![[5.0, 5.0]]);
        assert!((r.length - 0.0).abs() < f32::EPSILON);
    }

    #[test]
    fn path_result_serde_roundtrip() {
        let r = PathResult::found(vec![[0.0, 0.0], [1.0, 1.0]]);
        let json = serde_json::to_string(&r).unwrap();
        let deserialized: PathResult = serde_json::from_str(&json).unwrap();
        assert_eq!(deserialized.status, PathStatus::Found);
        assert_eq!(deserialized.waypoints.len(), 2);
    }
}
