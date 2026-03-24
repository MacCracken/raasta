//! Path following — waypoint-based path progression with steering.

use hisab::Vec2;
use serde::{Deserialize, Serialize};

use crate::steer::{SteerBehavior, SteerOutput, compute_steer};

/// Follows a sequence of waypoints, advancing to the next when close enough.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PathFollower {
    waypoints: Vec<Vec2>,
    /// Index of the current target waypoint.
    current: usize,
    /// Distance threshold to advance to the next waypoint.
    arrival_threshold: f32,
    /// Distance at which to start slowing for the final waypoint.
    slow_radius: f32,
}

impl PathFollower {
    /// Create a new path follower.
    ///
    /// - `waypoints`: ordered list of positions to follow
    /// - `arrival_threshold`: how close to a waypoint before advancing
    /// - `slow_radius`: deceleration radius for the final waypoint
    #[must_use]
    pub fn new(waypoints: Vec<Vec2>, arrival_threshold: f32, slow_radius: f32) -> Self {
        Self {
            waypoints,
            current: 0,
            arrival_threshold,
            slow_radius,
        }
    }

    /// Whether we've reached (or passed) the final waypoint.
    #[must_use]
    pub fn is_finished(&self) -> bool {
        self.current >= self.waypoints.len()
    }

    /// The waypoint currently being targeted, if any.
    #[must_use]
    pub fn current_target(&self) -> Option<Vec2> {
        self.waypoints.get(self.current).copied()
    }

    /// Index of the current waypoint.
    #[must_use]
    pub fn current_index(&self) -> usize {
        self.current
    }

    /// Total number of waypoints.
    #[must_use]
    pub fn waypoint_count(&self) -> usize {
        self.waypoints.len()
    }

    /// Reset to the beginning of the path.
    pub fn reset(&mut self) {
        self.current = 0;
    }

    /// Compute steering for this tick. Advances waypoints automatically.
    ///
    /// Returns zero output if the path is finished or empty.
    #[must_use]
    pub fn steer(&mut self, position: Vec2, max_speed: f32) -> SteerOutput {
        let target = match self.current_target() {
            Some(t) => t,
            None => return SteerOutput::default(),
        };

        let dist = position.distance(target);

        // Check if we've arrived at the current waypoint
        if dist < self.arrival_threshold {
            self.current += 1;
            // Re-check — may have finished or need to steer to next
            return self.steer(position, max_speed);
        }

        let is_final = self.current == self.waypoints.len() - 1;

        let behavior = if is_final {
            SteerBehavior::Arrive {
                target,
                slow_radius: self.slow_radius,
            }
        } else {
            SteerBehavior::Seek { target }
        };

        compute_steer(&behavior, position, max_speed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_path() {
        let mut f = PathFollower::new(vec![], 0.5, 2.0);
        assert!(f.is_finished());
        let out = f.steer(Vec2::ZERO, 5.0);
        assert!(out.speed() < f32::EPSILON);
    }

    #[test]
    fn single_waypoint_arrive() {
        let mut f = PathFollower::new(vec![Vec2::new(10.0, 0.0)], 0.5, 5.0);
        assert!(!f.is_finished());
        assert_eq!(f.current_target(), Some(Vec2::new(10.0, 0.0)));

        let out = f.steer(Vec2::ZERO, 5.0);
        // Should seek/arrive toward (10, 0)
        assert!(out.velocity.x > 0.0);
    }

    #[test]
    fn advances_waypoints() {
        let mut f = PathFollower::new(
            vec![
                Vec2::new(1.0, 0.0),
                Vec2::new(2.0, 0.0),
                Vec2::new(3.0, 0.0),
            ],
            0.5,
            1.0,
        );
        assert_eq!(f.current_index(), 0);

        // Position close to first waypoint — should advance
        let _out = f.steer(Vec2::new(0.9, 0.0), 5.0);
        assert_eq!(f.current_index(), 1);
    }

    #[test]
    fn finishes_at_end() {
        let mut f = PathFollower::new(vec![Vec2::new(1.0, 0.0)], 0.5, 1.0);
        // Close enough to finish
        let _out = f.steer(Vec2::new(0.9, 0.0), 5.0);
        assert!(f.is_finished());
    }

    #[test]
    fn reset_restarts() {
        let mut f = PathFollower::new(vec![Vec2::new(1.0, 0.0)], 0.5, 1.0);
        let _out = f.steer(Vec2::new(0.9, 0.0), 5.0);
        assert!(f.is_finished());
        f.reset();
        assert!(!f.is_finished());
        assert_eq!(f.current_index(), 0);
    }

    #[test]
    fn final_waypoint_uses_arrive() {
        let mut f = PathFollower::new(vec![Vec2::new(3.0, 0.0)], 0.1, 5.0);
        // At distance 3 with slow_radius 5, should be slowing down
        let out = f.steer(Vec2::ZERO, 10.0);
        // Speed should be less than max (3/5 * 10 = 6.0)
        assert!(out.speed() < 10.0);
        assert!((out.speed() - 6.0).abs() < 0.1);
    }

    #[test]
    fn intermediate_uses_seek_full_speed() {
        let mut f = PathFollower::new(vec![Vec2::new(5.0, 0.0), Vec2::new(10.0, 0.0)], 0.5, 3.0);
        let out = f.steer(Vec2::ZERO, 5.0);
        // Intermediate waypoint — seek at full speed
        assert!((out.speed() - 5.0).abs() < 0.01);
    }

    #[test]
    fn waypoint_count() {
        let f = PathFollower::new(vec![Vec2::ZERO, Vec2::ONE, Vec2::new(2.0, 2.0)], 0.5, 1.0);
        assert_eq!(f.waypoint_count(), 3);
    }

    #[test]
    fn follow_serde_roundtrip() {
        let f = PathFollower::new(vec![Vec2::ZERO, Vec2::new(5.0, 5.0)], 0.5, 2.0);
        let json = serde_json::to_string(&f).unwrap();
        let deserialized: PathFollower = serde_json::from_str(&json).unwrap();
        assert_eq!(deserialized.waypoint_count(), 2);
        assert_eq!(deserialized.current_index(), 0);
    }
}
