//! Agent — ties position, velocity, path following, and steering together.

use hisab::Vec2;
use serde::{Deserialize, Serialize};

use crate::follow::PathFollower;
use crate::steer::{Obstacle, SteerOutput, avoid_obstacles};

/// A navigation agent with position, velocity, and optional path following.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Agent {
    pub position: Vec2,
    pub velocity: Vec2,
    pub max_speed: f32,
    pub max_force: f32,
    follower: Option<PathFollower>,
}

impl Agent {
    /// Create a new agent at the given position.
    #[must_use]
    pub fn new(position: Vec2, max_speed: f32, max_force: f32) -> Self {
        Self {
            position,
            velocity: Vec2::ZERO,
            max_speed,
            max_force,
            follower: None,
        }
    }

    /// Assign a path for this agent to follow.
    pub fn set_path(&mut self, follower: PathFollower) {
        self.follower = Some(follower);
    }

    /// Clear the current path.
    pub fn clear_path(&mut self) {
        self.follower = None;
    }

    /// Whether the agent has an active (unfinished) path.
    #[must_use]
    pub fn has_path(&self) -> bool {
        self.follower.as_ref().is_some_and(|f| !f.is_finished())
    }

    /// Reference to the path follower, if any.
    #[must_use]
    pub fn follower(&self) -> Option<&PathFollower> {
        self.follower.as_ref()
    }

    /// Update the agent for one tick.
    ///
    /// Computes path-following steering, applies velocity, and moves the agent.
    /// Pass `dt` as the time step (seconds). Returns the steering output used.
    pub fn update(&mut self, dt: f32, obstacles: &[Obstacle]) -> SteerOutput {
        // Path following steering
        let path_steer = match self.follower.as_mut() {
            Some(f) if !f.is_finished() => f.steer(self.position, self.max_speed),
            _ => SteerOutput::default(),
        };

        // Obstacle avoidance
        let avoid_steer = if !obstacles.is_empty() && self.velocity.length_squared() > f32::EPSILON
        {
            let look_ahead = self.max_speed * 2.0;
            avoid_obstacles(
                self.position,
                self.velocity,
                obstacles,
                look_ahead,
                self.max_force,
            )
        } else {
            SteerOutput::default()
        };

        // Combine forces: path following + avoidance
        let combined = path_steer.velocity + avoid_steer.velocity;

        // Truncate to max force
        let force = if combined.length() > self.max_force {
            combined.normalize() * self.max_force
        } else {
            combined
        };

        // Direct velocity steering — set velocity from force direction
        // (rather than accumulating force, which causes overshoot)
        if force.length_squared() > f32::EPSILON {
            self.velocity = force;
        } else {
            self.velocity = Vec2::ZERO;
        }

        // Clamp to max speed
        let speed = self.velocity.length();
        if speed > self.max_speed {
            self.velocity = self.velocity / speed * self.max_speed;
        }

        // Move
        self.position += self.velocity * dt;

        SteerOutput::from_vec2(force)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn agent_creation() {
        let a = Agent::new(Vec2::ZERO, 5.0, 10.0);
        assert_eq!(a.position, Vec2::ZERO);
        assert_eq!(a.velocity, Vec2::ZERO);
        assert!(!a.has_path());
    }

    #[test]
    fn agent_set_path() {
        let mut a = Agent::new(Vec2::ZERO, 5.0, 10.0);
        let f = PathFollower::new(vec![Vec2::new(10.0, 0.0)], 0.5, 2.0);
        a.set_path(f);
        assert!(a.has_path());
    }

    #[test]
    fn agent_clear_path() {
        let mut a = Agent::new(Vec2::ZERO, 5.0, 10.0);
        a.set_path(PathFollower::new(vec![Vec2::new(10.0, 0.0)], 0.5, 2.0));
        a.clear_path();
        assert!(!a.has_path());
    }

    #[test]
    fn agent_moves_toward_target() {
        let mut a = Agent::new(Vec2::ZERO, 5.0, 10.0);
        a.set_path(PathFollower::new(vec![Vec2::new(10.0, 0.0)], 0.5, 2.0));

        for _ in 0..100 {
            a.update(0.1, &[]);
        }

        // Should have moved toward (10, 0)
        assert!(a.position.x > 5.0);
    }

    #[test]
    fn agent_reaches_target() {
        let mut a = Agent::new(Vec2::ZERO, 10.0, 20.0);
        a.set_path(PathFollower::new(vec![Vec2::new(5.0, 0.0)], 0.5, 2.0));

        for _ in 0..200 {
            a.update(0.05, &[]);
            if !a.has_path() {
                break;
            }
        }

        assert!(a.position.distance(Vec2::new(5.0, 0.0)) < 1.0);
    }

    #[test]
    fn agent_no_path_stays_still() {
        let mut a = Agent::new(Vec2::new(3.0, 3.0), 5.0, 10.0);
        a.update(0.1, &[]);
        // No path — no force — stays still (velocity starts at zero)
        assert!(a.position.distance(Vec2::new(3.0, 3.0)) < f32::EPSILON);
    }

    #[test]
    fn agent_avoids_obstacle() {
        let mut a = Agent::new(Vec2::ZERO, 5.0, 10.0);
        a.set_path(PathFollower::new(vec![Vec2::new(20.0, 0.0)], 0.5, 2.0));

        let obstacle = Obstacle {
            center: Vec2::new(5.0, 0.0),
            radius: 2.0,
        };

        // Run for a while with obstacle avoidance
        for _ in 0..200 {
            a.update(0.05, &[obstacle]);
        }

        // Should have moved past the obstacle (x > 5) and deviated laterally
        assert!(a.position.x > 5.0);
    }

    #[test]
    fn agent_speed_clamped() {
        let mut a = Agent::new(Vec2::ZERO, 5.0, 100.0);
        a.set_path(PathFollower::new(vec![Vec2::new(100.0, 0.0)], 0.5, 2.0));

        // Big force, small max_speed — should clamp
        for _ in 0..10 {
            a.update(0.1, &[]);
        }
        assert!(a.velocity.length() <= 5.0 + f32::EPSILON);
    }

    #[test]
    fn agent_serde_roundtrip() {
        let mut a = Agent::new(Vec2::new(1.0, 2.0), 5.0, 10.0);
        a.set_path(PathFollower::new(vec![Vec2::new(10.0, 0.0)], 0.5, 2.0));

        let json = serde_json::to_string(&a).unwrap();
        let deserialized: Agent = serde_json::from_str(&json).unwrap();
        assert_eq!(deserialized.position, Vec2::new(1.0, 2.0));
        assert!(deserialized.has_path());
    }

    #[test]
    fn agent_multi_waypoint() {
        let mut a = Agent::new(Vec2::ZERO, 10.0, 20.0);
        a.set_path(PathFollower::new(
            vec![
                Vec2::new(5.0, 0.0),
                Vec2::new(5.0, 5.0),
                Vec2::new(10.0, 5.0),
            ],
            1.0,
            2.0,
        ));

        for _ in 0..1000 {
            a.update(0.02, &[]);
            if !a.has_path() {
                break;
            }
        }

        // Should be near the final waypoint
        assert!(a.position.distance(Vec2::new(10.0, 5.0)) < 3.0);
    }
}
