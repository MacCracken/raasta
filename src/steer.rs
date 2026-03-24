//! Steering behaviors — seek, flee, arrive, obstacle avoidance.

use hisab::Vec2;
use serde::{Deserialize, Serialize};

/// A steering behavior that produces a desired velocity/force.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[non_exhaustive]
pub enum SteerBehavior {
    /// Move toward a target position.
    Seek { target: Vec2 },
    /// Move away from a target position.
    Flee { target: Vec2 },
    /// Move toward a target, slowing down as it approaches.
    Arrive {
        target: Vec2,
        /// Distance at which to start slowing down.
        slow_radius: f32,
    },
}

/// Output of a steering computation.
#[derive(Debug, Clone, Copy, Default)]
pub struct SteerOutput {
    /// Desired velocity direction and magnitude.
    pub velocity: Vec2,
}

impl SteerOutput {
    #[must_use]
    pub fn new(vx: f32, vy: f32) -> Self {
        Self {
            velocity: Vec2::new(vx, vy),
        }
    }

    /// Construct from a `Vec2` directly.
    #[must_use]
    pub fn from_vec2(velocity: Vec2) -> Self {
        Self { velocity }
    }

    /// Magnitude of the velocity.
    #[inline]
    #[must_use]
    pub fn speed(&self) -> f32 {
        self.velocity.length()
    }
}

/// Compute steering output for the given behavior.
///
/// - `position`: current agent position
/// - `max_speed`: maximum speed the agent can move
#[inline]
#[must_use]
pub fn compute_steer(behavior: &SteerBehavior, position: Vec2, max_speed: f32) -> SteerOutput {
    match behavior {
        SteerBehavior::Seek { target } => {
            let desired = *target - position;
            let len = desired.length();
            if len < f32::EPSILON {
                return SteerOutput::default();
            }
            SteerOutput::from_vec2(desired / len * max_speed)
        }
        SteerBehavior::Flee { target } => {
            let desired = position - *target;
            let len = desired.length();
            if len < f32::EPSILON {
                return SteerOutput::default();
            }
            SteerOutput::from_vec2(desired / len * max_speed)
        }
        SteerBehavior::Arrive {
            target,
            slow_radius,
        } => {
            let desired = *target - position;
            let dist = desired.length();
            if dist < f32::EPSILON {
                return SteerOutput::default();
            }
            let speed = if dist < *slow_radius {
                max_speed * (dist / slow_radius)
            } else {
                max_speed
            };
            SteerOutput::from_vec2(desired / dist * speed)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn seek_toward_target() {
        let out = compute_steer(
            &SteerBehavior::Seek {
                target: Vec2::new(10.0, 0.0),
            },
            Vec2::ZERO,
            5.0,
        );
        assert!((out.velocity.x - 5.0).abs() < 0.01);
        assert!(out.velocity.y.abs() < 0.01);
    }

    #[test]
    fn flee_from_target() {
        let out = compute_steer(
            &SteerBehavior::Flee {
                target: Vec2::new(10.0, 0.0),
            },
            Vec2::ZERO,
            5.0,
        );
        assert!((out.velocity.x - (-5.0)).abs() < 0.01);
    }

    #[test]
    fn arrive_full_speed() {
        let out = compute_steer(
            &SteerBehavior::Arrive {
                target: Vec2::new(100.0, 0.0),
                slow_radius: 10.0,
            },
            Vec2::ZERO,
            5.0,
        );
        assert!((out.speed() - 5.0).abs() < 0.01);
    }

    #[test]
    fn arrive_slow_down() {
        let out = compute_steer(
            &SteerBehavior::Arrive {
                target: Vec2::new(5.0, 0.0),
                slow_radius: 10.0,
            },
            Vec2::ZERO,
            10.0,
        );
        // At distance 5 with slow_radius 10, speed should be 5.0 (half)
        assert!((out.speed() - 5.0).abs() < 0.01);
    }

    #[test]
    fn arrive_at_target() {
        let out = compute_steer(
            &SteerBehavior::Arrive {
                target: Vec2::new(5.0, 5.0),
                slow_radius: 10.0,
            },
            Vec2::new(5.0, 5.0),
            10.0,
        );
        assert!(out.speed() < f32::EPSILON);
    }

    #[test]
    fn seek_at_target() {
        let out = compute_steer(&SteerBehavior::Seek { target: Vec2::ZERO }, Vec2::ZERO, 5.0);
        assert!(out.speed() < f32::EPSILON);
    }

    #[test]
    fn flee_at_target() {
        let out = compute_steer(&SteerBehavior::Flee { target: Vec2::ZERO }, Vec2::ZERO, 5.0);
        assert!(out.speed() < f32::EPSILON);
    }

    #[test]
    fn steer_output_speed() {
        let out = SteerOutput::new(3.0, 4.0);
        assert!((out.speed() - 5.0).abs() < 0.01);
    }

    #[test]
    fn seek_diagonal() {
        let out = compute_steer(
            &SteerBehavior::Seek {
                target: Vec2::new(10.0, 10.0),
            },
            Vec2::ZERO,
            1.0,
        );
        // Speed should be max_speed
        assert!((out.speed() - 1.0).abs() < 0.01);
        // Direction should be ~45 degrees (equal x and y)
        assert!((out.velocity.x - out.velocity.y).abs() < 0.01);
    }

    #[test]
    fn flee_negative_coords() {
        let out = compute_steer(
            &SteerBehavior::Flee {
                target: Vec2::new(-10.0, -10.0),
            },
            Vec2::ZERO,
            5.0,
        );
        // Should flee in positive direction
        assert!(out.velocity.x > 0.0);
        assert!(out.velocity.y > 0.0);
        assert!((out.speed() - 5.0).abs() < 0.01);
    }

    #[test]
    fn arrive_at_slow_radius_boundary() {
        let out = compute_steer(
            &SteerBehavior::Arrive {
                target: Vec2::new(10.0, 0.0),
                slow_radius: 10.0,
            },
            Vec2::ZERO,
            10.0,
        );
        // Exactly at slow_radius distance — speed should equal max_speed
        assert!((out.speed() - 10.0).abs() < 0.01);
    }

    #[test]
    fn steer_output_zero_speed() {
        let out = SteerOutput::default();
        assert!(out.speed() < f32::EPSILON);
    }

    #[test]
    fn steer_serde_roundtrip() {
        let b = SteerBehavior::Arrive {
            target: Vec2::new(1.0, 2.0),
            slow_radius: 5.0,
        };
        let json = serde_json::to_string(&b).unwrap();
        let deserialized: SteerBehavior = serde_json::from_str(&json).unwrap();
        assert_eq!(deserialized, b);
    }
}
