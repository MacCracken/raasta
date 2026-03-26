//! Steering behaviors — seek, flee, arrive, obstacle avoidance.

use hisab::Vec2;
use serde::{Deserialize, Serialize};

/// A circular obstacle for avoidance steering.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Obstacle {
    pub center: Vec2,
    pub radius: f32,
}

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
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
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

/// Compute pursuit steering — intercept a moving target by predicting its future position.
///
/// `target_pos` and `target_vel` are the target's current position and velocity.
/// `prediction_factor` scales how far ahead to predict (typically 1.0).
#[inline]
#[must_use]
pub fn pursuit(position: Vec2, target_pos: Vec2, target_vel: Vec2, max_speed: f32) -> SteerOutput {
    let to_target = target_pos - position;
    let dist = to_target.length();
    if dist < f32::EPSILON {
        return SteerOutput::default();
    }
    // Predict future position based on distance (further = more prediction)
    let prediction_time = dist / max_speed;
    let predicted = target_pos + target_vel * prediction_time;
    compute_steer(
        &SteerBehavior::Seek { target: predicted },
        position,
        max_speed,
    )
}

/// Compute evasion steering — flee from a moving target's predicted future position.
#[inline]
#[must_use]
pub fn evade(position: Vec2, target_pos: Vec2, target_vel: Vec2, max_speed: f32) -> SteerOutput {
    let to_target = target_pos - position;
    let dist = to_target.length();
    if dist < f32::EPSILON {
        return SteerOutput::default();
    }
    let prediction_time = dist / max_speed;
    let predicted = target_pos + target_vel * prediction_time;
    compute_steer(
        &SteerBehavior::Flee { target: predicted },
        position,
        max_speed,
    )
}

/// Compute wander steering — natural-looking random movement.
///
/// Projects a circle of `wander_radius` at `wander_distance` ahead of the
/// agent, then displaces the target by `wander_angle` radians on that circle.
///
/// Call with a varying `wander_angle` each tick (e.g., previous angle ± small random delta)
/// for smooth wandering.
#[inline]
#[must_use]
pub fn wander(
    position: Vec2,
    velocity: Vec2,
    max_speed: f32,
    wander_distance: f32,
    wander_radius: f32,
    wander_angle: f32,
) -> SteerOutput {
    let speed = velocity.length();
    let forward = if speed > f32::EPSILON {
        velocity / speed
    } else {
        Vec2::new(1.0, 0.0)
    };

    let circle_center = position + forward * wander_distance;
    let displacement = Vec2::new(wander_angle.cos(), wander_angle.sin()) * wander_radius;
    let target = circle_center + displacement;

    compute_steer(&SteerBehavior::Seek { target }, position, max_speed)
}

/// Compute separation steering — steer away from nearby neighbors.
///
/// Returns a force that pushes the agent away from neighbors within `radius`.
/// Closer neighbors produce stronger repulsion.
#[must_use]
pub fn separation(position: Vec2, neighbors: &[Vec2], radius: f32, max_force: f32) -> SteerOutput {
    let mut force = Vec2::ZERO;
    let mut count = 0;

    for &neighbor in neighbors {
        let diff = position - neighbor;
        let dist = diff.length();
        if dist > f32::EPSILON && dist < radius {
            // Weight inversely by distance
            force += diff / (dist * dist);
            count += 1;
        }
    }

    if count > 0 {
        force /= count as f32;
        let len = force.length();
        if len > f32::EPSILON {
            force = force / len * max_force;
        }
    }

    SteerOutput::from_vec2(force)
}

/// Compute alignment steering — steer toward the average heading of neighbors.
#[must_use]
pub fn alignment(velocity: Vec2, neighbor_velocities: &[Vec2], max_force: f32) -> SteerOutput {
    if neighbor_velocities.is_empty() {
        return SteerOutput::default();
    }

    let avg: Vec2 =
        neighbor_velocities.iter().copied().sum::<Vec2>() / neighbor_velocities.len() as f32;
    let desired = avg - velocity;
    let len = desired.length();
    if len < f32::EPSILON {
        return SteerOutput::default();
    }
    SteerOutput::from_vec2(desired / len * max_force)
}

/// Compute cohesion steering — steer toward the center of mass of neighbors.
#[must_use]
pub fn cohesion(position: Vec2, neighbors: &[Vec2], max_speed: f32) -> SteerOutput {
    if neighbors.is_empty() {
        return SteerOutput::default();
    }

    let center: Vec2 = neighbors.iter().copied().sum::<Vec2>() / neighbors.len() as f32;
    compute_steer(&SteerBehavior::Seek { target: center }, position, max_speed)
}

/// Compute an avoidance steering force away from nearby obstacles.
///
/// Casts a ray from `position` along `velocity` up to `look_ahead` distance.
/// If any obstacle intersects that ray, returns a lateral steering force that
/// pushes the agent away from the nearest one. The force magnitude scales
/// with `max_force` and is stronger for closer obstacles.
///
/// Returns zero if no obstacles are within the look-ahead cone.
///
/// - `position`: current agent position
/// - `velocity`: current agent velocity (direction of travel)
/// - `obstacles`: slice of circular obstacles to avoid
/// - `look_ahead`: how far ahead to check for obstacles
/// - `max_force`: maximum avoidance force magnitude
#[inline]
#[must_use]
pub fn avoid_obstacles(
    position: Vec2,
    velocity: Vec2,
    obstacles: &[Obstacle],
    look_ahead: f32,
    max_force: f32,
) -> SteerOutput {
    let speed = velocity.length();
    if speed < f32::EPSILON {
        return SteerOutput::default();
    }

    let forward = velocity / speed;
    // Lateral (perpendicular) direction — always turn the same way for consistency
    let lateral = Vec2::new(-forward.y, forward.x);

    let mut nearest_t = f32::INFINITY;
    let mut nearest_lateral_offset = 0.0f32;
    let mut nearest_dist_sq = f32::INFINITY;

    for obs in obstacles {
        let to_obs = obs.center - position;
        // Project obstacle center onto forward axis
        let forward_dot = to_obs.dot(forward);

        // Behind us or beyond look-ahead — skip
        if forward_dot < -obs.radius || forward_dot > look_ahead + obs.radius {
            continue;
        }

        // Lateral distance from the ray to obstacle center
        let lateral_dot = to_obs.dot(lateral);
        let overlap = obs.radius - lateral_dot.abs();

        // No intersection with the ray corridor
        if overlap < 0.0 {
            continue;
        }

        // Track the nearest obstacle by forward projection
        let dist_sq = to_obs.length_squared();
        if forward_dot < nearest_t || (forward_dot == nearest_t && dist_sq < nearest_dist_sq) {
            nearest_t = forward_dot;
            nearest_dist_sq = dist_sq;
            // Steer away from whichever side the obstacle is on
            nearest_lateral_offset = if lateral_dot >= 0.0 { -1.0 } else { 1.0 };
        }
    }

    if nearest_t == f32::INFINITY {
        return SteerOutput::default();
    }

    // Force scales inversely with distance — closer obstacles get stronger avoidance
    let urgency = 1.0 - (nearest_t / (look_ahead + f32::EPSILON)).clamp(0.0, 1.0);
    let force = lateral * nearest_lateral_offset * max_force * urgency;
    SteerOutput::from_vec2(force)
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

    // --- Obstacle avoidance tests ---

    #[test]
    fn avoid_no_obstacles() {
        let out = avoid_obstacles(Vec2::ZERO, Vec2::new(1.0, 0.0), &[], 10.0, 5.0);
        assert!(out.speed() < f32::EPSILON);
    }

    #[test]
    fn avoid_obstacle_ahead() {
        let obs = Obstacle {
            center: Vec2::new(5.0, 0.5),
            radius: 1.0,
        };
        let out = avoid_obstacles(Vec2::ZERO, Vec2::new(1.0, 0.0), &[obs], 10.0, 5.0);
        // Should produce a lateral force (non-zero y, ~zero x contribution)
        assert!(out.speed() > 0.0);
        // Obstacle is slightly above the ray — should steer downward (negative y)
        assert!(out.velocity.y < 0.0);
    }

    #[test]
    fn avoid_obstacle_behind() {
        let obs = Obstacle {
            center: Vec2::new(-5.0, 0.0),
            radius: 1.0,
        };
        let out = avoid_obstacles(Vec2::ZERO, Vec2::new(1.0, 0.0), &[obs], 10.0, 5.0);
        // Obstacle is behind — no avoidance
        assert!(out.speed() < f32::EPSILON);
    }

    #[test]
    fn avoid_obstacle_far_lateral() {
        let obs = Obstacle {
            center: Vec2::new(5.0, 10.0),
            radius: 1.0,
        };
        let out = avoid_obstacles(Vec2::ZERO, Vec2::new(1.0, 0.0), &[obs], 10.0, 5.0);
        // Obstacle is far to the side — no intersection
        assert!(out.speed() < f32::EPSILON);
    }

    #[test]
    fn avoid_nearer_obstacle_preferred() {
        let near = Obstacle {
            center: Vec2::new(3.0, 0.5),
            radius: 1.0,
        };
        let far = Obstacle {
            center: Vec2::new(8.0, 0.5),
            radius: 1.0,
        };
        let out_both = avoid_obstacles(Vec2::ZERO, Vec2::new(1.0, 0.0), &[far, near], 10.0, 5.0);
        let out_near = avoid_obstacles(Vec2::ZERO, Vec2::new(1.0, 0.0), &[near], 10.0, 5.0);
        // Should react to the nearer obstacle — same direction
        assert!((out_both.velocity.y - out_near.velocity.y).abs() < 0.01);
    }

    #[test]
    fn avoid_zero_velocity() {
        let obs = Obstacle {
            center: Vec2::new(5.0, 0.0),
            radius: 1.0,
        };
        let out = avoid_obstacles(Vec2::ZERO, Vec2::ZERO, &[obs], 10.0, 5.0);
        assert!(out.speed() < f32::EPSILON);
    }

    #[test]
    fn avoid_urgency_scales_with_distance() {
        let close = Obstacle {
            center: Vec2::new(2.0, 0.5),
            radius: 1.0,
        };
        let far = Obstacle {
            center: Vec2::new(9.0, 0.5),
            radius: 1.0,
        };
        let out_close = avoid_obstacles(Vec2::ZERO, Vec2::new(1.0, 0.0), &[close], 10.0, 5.0);
        let out_far = avoid_obstacles(Vec2::ZERO, Vec2::new(1.0, 0.0), &[far], 10.0, 5.0);
        // Closer obstacle should produce stronger force
        assert!(out_close.speed() > out_far.speed());
    }

    #[test]
    fn avoid_beyond_look_ahead() {
        let obs = Obstacle {
            center: Vec2::new(15.0, 0.0),
            radius: 1.0,
        };
        let out = avoid_obstacles(Vec2::ZERO, Vec2::new(1.0, 0.0), &[obs], 10.0, 5.0);
        // Beyond look-ahead — no avoidance
        assert!(out.speed() < f32::EPSILON);
    }

    #[test]
    fn obstacle_serde_roundtrip() {
        let obs = Obstacle {
            center: Vec2::new(1.0, 2.0),
            radius: 3.0,
        };
        let json = serde_json::to_string(&obs).unwrap();
        let deserialized: Obstacle = serde_json::from_str(&json).unwrap();
        assert_eq!(deserialized, obs);
    }

    // --- Pursuit / Evade / Wander / Flocking tests ---

    #[test]
    fn pursuit_intercepts() {
        // Target moving right, agent behind and below
        let out = pursuit(
            Vec2::new(0.0, -5.0),
            Vec2::new(5.0, 0.0),
            Vec2::new(1.0, 0.0),
            5.0,
        );
        // Should aim ahead of target (positive x component)
        assert!(out.velocity.x > 0.0);
        assert!(out.speed() > 0.0);
    }

    #[test]
    fn evade_flees_predicted() {
        let out = evade(
            Vec2::ZERO,
            Vec2::new(5.0, 0.0),
            Vec2::new(-1.0, 0.0), // target approaching
            5.0,
        );
        // Should flee from predicted position (which is closer)
        assert!(out.velocity.x < 0.0);
    }

    #[test]
    fn wander_produces_movement() {
        let out = wander(
            Vec2::ZERO,
            Vec2::new(1.0, 0.0),
            5.0,
            2.0, // distance
            1.0, // radius
            0.5, // angle
        );
        assert!(out.speed() > 0.0);
    }

    #[test]
    fn wander_stationary_agent() {
        let out = wander(Vec2::ZERO, Vec2::ZERO, 5.0, 2.0, 1.0, 0.0);
        // Uses default forward direction, should still produce movement
        assert!(out.speed() > 0.0);
    }

    #[test]
    fn separation_pushes_apart() {
        let neighbors = vec![Vec2::new(1.0, 0.0), Vec2::new(-1.0, 0.0)];
        let out = separation(Vec2::ZERO, &neighbors, 5.0, 10.0);
        // Equal neighbors on both sides — should roughly cancel, low force
        assert!(out.speed() < 1.0);
    }

    #[test]
    fn separation_one_neighbor() {
        let neighbors = vec![Vec2::new(1.0, 0.0)];
        let out = separation(Vec2::ZERO, &neighbors, 5.0, 10.0);
        // Should push away from neighbor (negative x)
        assert!(out.velocity.x < 0.0);
    }

    #[test]
    fn separation_no_neighbors() {
        let out = separation(Vec2::ZERO, &[], 5.0, 10.0);
        assert!(out.speed() < f32::EPSILON);
    }

    #[test]
    fn separation_out_of_range() {
        let neighbors = vec![Vec2::new(100.0, 0.0)];
        let out = separation(Vec2::ZERO, &neighbors, 5.0, 10.0);
        assert!(out.speed() < f32::EPSILON);
    }

    #[test]
    fn alignment_matches_heading() {
        let neighbor_vels = vec![Vec2::new(3.0, 0.0), Vec2::new(3.0, 0.0)];
        let out = alignment(Vec2::new(1.0, 0.0), &neighbor_vels, 5.0);
        // Should steer to match neighbors (increase x velocity)
        assert!(out.velocity.x > 0.0);
    }

    #[test]
    fn alignment_no_neighbors() {
        let out = alignment(Vec2::new(1.0, 0.0), &[], 5.0);
        assert!(out.speed() < f32::EPSILON);
    }

    #[test]
    fn cohesion_toward_center() {
        let neighbors = vec![Vec2::new(10.0, 0.0), Vec2::new(10.0, 10.0)];
        let out = cohesion(Vec2::ZERO, &neighbors, 5.0);
        // Should steer toward center of mass (positive x and y)
        assert!(out.velocity.x > 0.0);
        assert!(out.velocity.y > 0.0);
    }

    #[test]
    fn cohesion_no_neighbors() {
        let out = cohesion(Vec2::ZERO, &[], 5.0);
        assert!(out.speed() < f32::EPSILON);
    }

    #[test]
    fn steer_output_serde_roundtrip() {
        let out = SteerOutput::new(3.0, 4.0);
        let json = serde_json::to_string(&out).unwrap();
        let deserialized: SteerOutput = serde_json::from_str(&json).unwrap();
        assert!((deserialized.speed() - 5.0).abs() < 0.01);
    }
}
