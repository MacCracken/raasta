//! Steering behaviors — seek, flee, arrive, obstacle avoidance.

use serde::{Deserialize, Serialize};

/// A steering behavior that produces a desired velocity/force.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[non_exhaustive]
pub enum SteerBehavior {
    /// Move toward a target position.
    Seek {
        target: [f32; 2],
    },
    /// Move away from a target position.
    Flee {
        target: [f32; 2],
    },
    /// Move toward a target, slowing down as it approaches.
    Arrive {
        target: [f32; 2],
        /// Distance at which to start slowing down.
        slow_radius: f32,
    },
}

/// Output of a steering computation.
#[derive(Debug, Clone, Copy, Default)]
pub struct SteerOutput {
    /// Desired velocity direction and magnitude.
    pub velocity: [f32; 2],
}

impl SteerOutput {
    pub fn new(vx: f32, vy: f32) -> Self {
        Self {
            velocity: [vx, vy],
        }
    }

    /// Magnitude of the velocity.
    pub fn speed(&self) -> f32 {
        let [vx, vy] = self.velocity;
        (vx * vx + vy * vy).sqrt()
    }
}

/// Compute steering output for the given behavior.
///
/// - `position`: current agent position
/// - `max_speed`: maximum speed the agent can move
pub fn compute_steer(
    behavior: &SteerBehavior,
    position: [f32; 2],
    max_speed: f32,
) -> SteerOutput {
    match behavior {
        SteerBehavior::Seek { target } => {
            let dx = target[0] - position[0];
            let dy = target[1] - position[1];
            let len = (dx * dx + dy * dy).sqrt();
            if len < f32::EPSILON {
                return SteerOutput::default();
            }
            SteerOutput::new(dx / len * max_speed, dy / len * max_speed)
        }
        SteerBehavior::Flee { target } => {
            let dx = position[0] - target[0];
            let dy = position[1] - target[1];
            let len = (dx * dx + dy * dy).sqrt();
            if len < f32::EPSILON {
                return SteerOutput::default();
            }
            SteerOutput::new(dx / len * max_speed, dy / len * max_speed)
        }
        SteerBehavior::Arrive {
            target,
            slow_radius,
        } => {
            let dx = target[0] - position[0];
            let dy = target[1] - position[1];
            let dist = (dx * dx + dy * dy).sqrt();
            if dist < f32::EPSILON {
                return SteerOutput::default();
            }
            let speed = if dist < *slow_radius {
                max_speed * (dist / slow_radius)
            } else {
                max_speed
            };
            SteerOutput::new(dx / dist * speed, dy / dist * speed)
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
                target: [10.0, 0.0],
            },
            [0.0, 0.0],
            5.0,
        );
        assert!((out.velocity[0] - 5.0).abs() < 0.01);
        assert!(out.velocity[1].abs() < 0.01);
    }

    #[test]
    fn flee_from_target() {
        let out = compute_steer(
            &SteerBehavior::Flee {
                target: [10.0, 0.0],
            },
            [0.0, 0.0],
            5.0,
        );
        assert!((out.velocity[0] - (-5.0)).abs() < 0.01);
    }

    #[test]
    fn arrive_full_speed() {
        let out = compute_steer(
            &SteerBehavior::Arrive {
                target: [100.0, 0.0],
                slow_radius: 10.0,
            },
            [0.0, 0.0],
            5.0,
        );
        assert!((out.speed() - 5.0).abs() < 0.01);
    }

    #[test]
    fn arrive_slow_down() {
        let out = compute_steer(
            &SteerBehavior::Arrive {
                target: [5.0, 0.0],
                slow_radius: 10.0,
            },
            [0.0, 0.0],
            10.0,
        );
        // At distance 5 with slow_radius 10, speed should be 5.0 (half)
        assert!((out.speed() - 5.0).abs() < 0.01);
    }

    #[test]
    fn arrive_at_target() {
        let out = compute_steer(
            &SteerBehavior::Arrive {
                target: [5.0, 5.0],
                slow_radius: 10.0,
            },
            [5.0, 5.0],
            10.0,
        );
        assert!(out.speed() < f32::EPSILON);
    }

    #[test]
    fn seek_at_target() {
        let out = compute_steer(
            &SteerBehavior::Seek {
                target: [0.0, 0.0],
            },
            [0.0, 0.0],
            5.0,
        );
        assert!(out.speed() < f32::EPSILON);
    }

    #[test]
    fn flee_at_target() {
        let out = compute_steer(
            &SteerBehavior::Flee {
                target: [0.0, 0.0],
            },
            [0.0, 0.0],
            5.0,
        );
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
                target: [10.0, 10.0],
            },
            [0.0, 0.0],
            1.0,
        );
        // Speed should be max_speed
        assert!((out.speed() - 1.0).abs() < 0.01);
        // Direction should be ~45 degrees (equal x and y)
        assert!((out.velocity[0] - out.velocity[1]).abs() < 0.01);
    }
}
