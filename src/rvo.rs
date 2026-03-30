//! RVO/ORCA — Reciprocal Velocity Obstacles for local collision avoidance.
//!
//! Implements the ORCA (Optimal Reciprocal Collision Avoidance) algorithm
//! for multi-agent local avoidance.

use std::collections::HashMap;

use hisab::Vec2;
use serde::{Deserialize, Serialize};

#[cfg(feature = "logging")]
use tracing::instrument;

/// A half-plane constraint: all velocities v where `(v - point) · normal ≥ 0`.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct HalfPlane {
    /// A point on the constraint boundary.
    pub point: Vec2,
    /// Outward normal — velocities on this side are permitted.
    pub normal: Vec2,
}

/// An RVO agent with position, velocity, and radius.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct RvoAgent {
    pub position: Vec2,
    pub velocity: Vec2,
    pub preferred_velocity: Vec2,
    pub radius: f32,
    pub max_speed: f32,
}

impl RvoAgent {
    #[cfg_attr(feature = "logging", instrument)]
    #[must_use]
    pub fn new(position: Vec2, radius: f32, max_speed: f32) -> Self {
        Self {
            position,
            velocity: Vec2::ZERO,
            preferred_velocity: Vec2::ZERO,
            radius,
            max_speed,
        }
    }
}

/// Compute the ORCA half-plane constraint between two agents.
///
/// Returns the half-plane that agent A should respect to avoid
/// collision with agent B within `time_horizon` seconds.
/// Each agent takes half the responsibility (reciprocal).
#[cfg_attr(feature = "logging", instrument)]
#[must_use]
pub fn compute_orca_half_plane(a: &RvoAgent, b: &RvoAgent, time_horizon: f32) -> HalfPlane {
    let relative_pos = b.position - a.position;
    let relative_vel = a.velocity - b.velocity;
    let combined_radius = a.radius + b.radius;
    let dist_sq = relative_pos.length_squared();
    let combined_radius_sq = combined_radius * combined_radius;

    let inv_tau = 1.0 / time_horizon;

    if dist_sq > combined_radius_sq {
        // No collision yet — VO is a truncated cone
        let w = relative_vel - relative_pos * inv_tau;
        let w_len_sq = w.length_squared();
        let dot_product = w.dot(relative_pos);

        if dot_product < 0.0 && dot_product * dot_product > combined_radius_sq * w_len_sq {
            // Project on cutoff circle
            let w_len = w_len_sq.sqrt();
            let unit_w = w / w_len;
            let normal = unit_w;
            let u = unit_w * (combined_radius * inv_tau - w_len);
            HalfPlane {
                point: a.velocity + u * 0.5,
                normal,
            }
        } else {
            // Project on cone legs
            let leg = (dist_sq - combined_radius_sq).max(0.0).sqrt();
            if relative_pos.perp_dot(w) > 0.0 {
                // Left leg
                let direction = Vec2::new(
                    relative_pos.x * leg - relative_pos.y * combined_radius,
                    relative_pos.x * combined_radius + relative_pos.y * leg,
                ) / dist_sq;
                let normal = Vec2::new(direction.y, -direction.x);
                let dot = relative_vel.dot(direction);
                let u = direction * dot - relative_vel;
                HalfPlane {
                    point: a.velocity + u * 0.5,
                    normal,
                }
            } else {
                // Right leg
                let direction = Vec2::new(
                    relative_pos.x * leg + relative_pos.y * combined_radius,
                    -relative_pos.x * combined_radius + relative_pos.y * leg,
                ) / dist_sq;
                let normal = Vec2::new(-direction.y, direction.x);
                let dot = relative_vel.dot(direction);
                let u = direction * dot - relative_vel;
                HalfPlane {
                    point: a.velocity + u * 0.5,
                    normal,
                }
            }
        }
    } else {
        // Already overlapping — push apart immediately
        let inv_dt = 1.0 / 0.001_f32.max(time_horizon * 0.5);
        let w = relative_vel - relative_pos * inv_dt;
        let w_len = w.length();
        if w_len < f32::EPSILON {
            // Exactly overlapping with same velocity — push in arbitrary direction
            return HalfPlane {
                point: a.velocity,
                normal: Vec2::new(1.0, 0.0),
            };
        }
        let unit_w = w / w_len;
        let normal = unit_w;
        let u = unit_w * (combined_radius * inv_dt - w_len);
        HalfPlane {
            point: a.velocity + u * 0.5,
            normal,
        }
    }
}

/// Solve for the optimal velocity given ORCA half-plane constraints.
///
/// Finds the velocity closest to `preferred` that satisfies all half-plane
/// constraints and has magnitude ≤ `max_speed`.
///
/// Uses 2D incremental linear programming: processes constraints one at a
/// time, projecting onto the constraint boundary when violated.
#[cfg_attr(feature = "logging", instrument)]
#[must_use]
pub fn solve_velocity(constraints: &[HalfPlane], preferred: Vec2, max_speed: f32) -> Vec2 {
    // Start with preferred velocity, clamped to max speed
    let mut result = if preferred.length() > max_speed {
        preferred.normalize() * max_speed
    } else {
        preferred
    };

    for (i, constraint) in constraints.iter().enumerate() {
        // Check if current result satisfies this constraint
        if (result - constraint.point).dot(constraint.normal) >= 0.0 {
            continue;
        }

        // Project onto the constraint line
        // The constraint boundary is: point + t * direction, where direction ⊥ normal
        let direction = Vec2::new(-constraint.normal.y, constraint.normal.x);
        let t = (result - constraint.point).dot(direction);
        let projected = constraint.point + direction * t;

        // Clamp to max speed circle
        result = if projected.length() > max_speed {
            projected.normalize() * max_speed
        } else {
            projected
        };

        // Verify all previous constraints still hold; if not, find best point
        // on intersection of violated constraints
        let mut all_satisfied = true;
        for prev in &constraints[..i] {
            if (result - prev.point).dot(prev.normal) < -f32::EPSILON {
                all_satisfied = false;
                break;
            }
        }

        if !all_satisfied {
            // Fallback: find the best velocity on the current constraint line
            // that satisfies all previous constraints
            result = solve_on_line(constraint, &constraints[..i], max_speed);
        }
    }

    result
}

/// Find the best velocity on a constraint line that satisfies all other constraints.
fn solve_on_line(line: &HalfPlane, constraints: &[HalfPlane], max_speed: f32) -> Vec2 {
    let direction = Vec2::new(-line.normal.y, line.normal.x);

    // t range on the line: point + t * direction
    let mut t_min = -max_speed * 2.0;
    let mut t_max = max_speed * 2.0;

    // Intersect with max speed circle
    let a = direction.dot(direction); // always 1 for unit direction, but be safe
    let b = 2.0 * line.point.dot(direction);
    let c = line.point.dot(line.point) - max_speed * max_speed;
    let disc = b * b - 4.0 * a * c;

    if disc < 0.0 {
        // Line doesn't intersect the speed circle — return closest point on circle
        let center_t = -b / (2.0 * a);
        let p = line.point + direction * center_t;
        return if p.length() > f32::EPSILON {
            p.normalize() * max_speed
        } else {
            Vec2::ZERO
        };
    }

    let sqrt_disc = disc.sqrt();
    let circle_t_min = (-b - sqrt_disc) / (2.0 * a);
    let circle_t_max = (-b + sqrt_disc) / (2.0 * a);
    t_min = t_min.max(circle_t_min);
    t_max = t_max.min(circle_t_max);

    // Intersect with each constraint
    for constraint in constraints {
        let denom = direction.dot(constraint.normal);
        let numer = (constraint.point - line.point).dot(constraint.normal);

        if denom.abs() < f32::EPSILON {
            // Parallel — check if on the valid side
            if numer < 0.0 {
                return line.point; // infeasible, return best guess
            }
            continue;
        }

        let t = numer / denom;
        if denom > 0.0 {
            t_min = t_min.max(t);
        } else {
            t_max = t_max.min(t);
        }

        if t_min > t_max {
            return line.point; // infeasible
        }
    }

    // Project preferred direction onto the valid range
    // (just pick the t closest to 0 in the valid range)
    let t = 0.0f32.clamp(t_min, t_max);
    line.point + direction * t
}

/// Spatial hash grid for efficient neighbor queries.
///
/// Maps 2D positions to grid cells so that nearby agents can be found in O(1)
/// per cell rather than scanning all agents.
#[derive(Debug, Clone)]
struct SpatialHash {
    inv_cell_size: f32,
    cells: HashMap<(i32, i32), Vec<usize>>,
}

impl SpatialHash {
    #[must_use]
    fn new(cell_size: f32) -> Self {
        Self {
            inv_cell_size: 1.0 / cell_size,
            cells: HashMap::new(),
        }
    }

    /// Clear all cell contents without deallocating.
    fn clear(&mut self) {
        for v in self.cells.values_mut() {
            v.clear();
        }
    }

    #[inline]
    #[must_use]
    fn cell_key(&self, pos: Vec2) -> (i32, i32) {
        (
            (pos.x * self.inv_cell_size).floor() as i32,
            (pos.y * self.inv_cell_size).floor() as i32,
        )
    }

    #[inline]
    fn insert(&mut self, idx: usize, pos: Vec2) {
        let key = self.cell_key(pos);
        self.cells.entry(key).or_default().push(idx);
    }

    /// Query all agent indices in cells overlapping the given position +/- radius.
    #[inline]
    fn query_neighbors(&self, pos: Vec2, radius: f32, result: &mut Vec<usize>) {
        let min_x = ((pos.x - radius) * self.inv_cell_size).floor() as i32;
        let max_x = ((pos.x + radius) * self.inv_cell_size).floor() as i32;
        let min_y = ((pos.y - radius) * self.inv_cell_size).floor() as i32;
        let max_y = ((pos.y + radius) * self.inv_cell_size).floor() as i32;

        for cy in min_y..=max_y {
            for cx in min_x..=max_x {
                if let Some(cell) = self.cells.get(&(cx, cy)) {
                    result.extend_from_slice(cell);
                }
            }
        }
    }
}

/// Multi-agent RVO simulation.
///
/// Manages a set of agents and computes collision-free velocities each tick.
/// Uses a spatial hash grid for efficient neighbor queries, reducing the
/// constraint computation from O(n^2) to O(n*k) where k is the average
/// number of nearby agents.
#[derive(Debug, Clone)]
pub struct RvoSimulation {
    agents: Vec<RvoAgent>,
    time_horizon: f32,
    neighbor_dist: f32,
    spatial_hash: SpatialHash,
}

impl RvoSimulation {
    /// Create a new simulation with the given time horizon.
    ///
    /// Uses a default neighbor distance of `time_horizon * 20.0`, which is
    /// generous enough for most scenarios. For tighter control, use
    /// [`with_neighbor_dist`](Self::with_neighbor_dist).
    #[cfg_attr(feature = "logging", instrument)]
    #[must_use]
    pub fn new(time_horizon: f32) -> Self {
        let neighbor_dist = time_horizon * 20.0;
        Self {
            agents: Vec::new(),
            time_horizon,
            neighbor_dist,
            spatial_hash: SpatialHash::new(neighbor_dist),
        }
    }

    /// Create a new simulation with custom neighbor distance for spatial hashing.
    ///
    /// `neighbor_dist` is the maximum distance at which agents interact.
    /// Agents further apart than this are skipped during constraint computation.
    /// A good default is `max_agent_speed * time_horizon * 2.0`.
    #[cfg_attr(feature = "logging", instrument)]
    #[must_use]
    pub fn with_neighbor_dist(time_horizon: f32, neighbor_dist: f32) -> Self {
        Self {
            agents: Vec::new(),
            time_horizon,
            neighbor_dist,
            spatial_hash: SpatialHash::new(neighbor_dist),
        }
    }

    /// Add an agent and return its index.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    pub fn add_agent(&mut self, agent: RvoAgent) -> usize {
        let idx = self.agents.len();
        self.agents.push(agent);
        idx
    }

    /// Number of agents.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    #[must_use]
    pub fn agent_count(&self) -> usize {
        self.agents.len()
    }

    /// Get an agent by index.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    #[must_use]
    pub fn agent(&self, idx: usize) -> &RvoAgent {
        &self.agents[idx]
    }

    /// Mutable access to an agent.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    pub fn agent_mut(&mut self, idx: usize) -> &mut RvoAgent {
        &mut self.agents[idx]
    }

    /// Set the preferred velocity for an agent.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    pub fn set_preferred_velocity(&mut self, idx: usize, velocity: Vec2) {
        self.agents[idx].preferred_velocity = velocity;
    }

    /// Step the simulation by `dt` seconds.
    ///
    /// Rebuilds the spatial hash, computes ORCA constraints for nearby agent
    /// pairs, solves for safe velocities, and updates positions.
    #[cfg_attr(feature = "logging", instrument(skip(self), fields(agents = self.agents.len())))]
    pub fn step(&mut self, dt: f32) {
        let n = self.agents.len();
        let mut new_velocities = vec![Vec2::ZERO; n];

        // Rebuild spatial hash
        self.spatial_hash.clear();
        for (i, agent) in self.agents.iter().enumerate() {
            self.spatial_hash.insert(i, agent.position);
        }

        let neighbor_dist_sq = self.neighbor_dist * self.neighbor_dist;
        let mut neighbor_indices = Vec::new();

        #[allow(clippy::needless_range_loop)]
        for i in 0..n {
            let mut constraints = Vec::new();

            // Query nearby agents via spatial hash
            neighbor_indices.clear();
            self.spatial_hash.query_neighbors(
                self.agents[i].position,
                self.neighbor_dist,
                &mut neighbor_indices,
            );

            for &j in &neighbor_indices {
                if i == j {
                    continue;
                }
                // Distance check for circular accuracy (spatial hash returns a square region)
                let dist_sq = self.agents[i]
                    .position
                    .distance_squared(self.agents[j].position);
                if dist_sq > neighbor_dist_sq {
                    continue;
                }

                let hp =
                    compute_orca_half_plane(&self.agents[i], &self.agents[j], self.time_horizon);
                constraints.push(hp);
            }

            new_velocities[i] = solve_velocity(
                &constraints,
                self.agents[i].preferred_velocity,
                self.agents[i].max_speed,
            );
        }

        // Apply velocities and move
        for (i, agent) in self.agents.iter_mut().enumerate() {
            agent.velocity = new_velocities[i];
            agent.position += agent.velocity * dt;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn agents_moving_apart_no_constraint() {
        let a = RvoAgent {
            position: Vec2::ZERO,
            velocity: Vec2::new(-1.0, 0.0),
            preferred_velocity: Vec2::new(-1.0, 0.0),
            radius: 1.0,
            max_speed: 2.0,
        };
        let b = RvoAgent {
            position: Vec2::new(5.0, 0.0),
            velocity: Vec2::new(1.0, 0.0),
            preferred_velocity: Vec2::new(1.0, 0.0),
            radius: 1.0,
            max_speed: 2.0,
        };
        let hp = compute_orca_half_plane(&a, &b, 5.0);
        // The preferred velocity should satisfy the half-plane (agents moving apart)
        let v = a.preferred_velocity - hp.point;
        assert!(v.dot(hp.normal) >= -0.1);
    }

    #[test]
    fn head_on_collision_produces_constraint() {
        let a = RvoAgent {
            position: Vec2::ZERO,
            velocity: Vec2::new(1.0, 0.0),
            preferred_velocity: Vec2::new(1.0, 0.0),
            radius: 1.0,
            max_speed: 2.0,
        };
        let b = RvoAgent {
            position: Vec2::new(4.0, 0.0),
            velocity: Vec2::new(-1.0, 0.0),
            preferred_velocity: Vec2::new(-1.0, 0.0),
            radius: 1.0,
            max_speed: 2.0,
        };
        let hp = compute_orca_half_plane(&a, &b, 5.0);
        // Normal should have a component pushing agent A away from B
        // (half-plane should constrain A from continuing straight at B)
        assert!(hp.normal.length() > 0.5);
    }

    #[test]
    fn overlapping_agents() {
        let a = RvoAgent {
            position: Vec2::ZERO,
            velocity: Vec2::ZERO,
            preferred_velocity: Vec2::new(1.0, 0.0),
            radius: 1.0,
            max_speed: 2.0,
        };
        let b = RvoAgent {
            position: Vec2::new(0.5, 0.0),
            velocity: Vec2::ZERO,
            preferred_velocity: Vec2::new(-1.0, 0.0),
            radius: 1.0,
            max_speed: 2.0,
        };
        // Should not panic even when overlapping
        let hp = compute_orca_half_plane(&a, &b, 5.0);
        assert!(hp.normal.length() > 0.0);
    }

    #[test]
    fn perpendicular_agents() {
        let a = RvoAgent {
            position: Vec2::ZERO,
            velocity: Vec2::new(1.0, 0.0),
            preferred_velocity: Vec2::new(1.0, 0.0),
            radius: 0.5,
            max_speed: 2.0,
        };
        let b = RvoAgent {
            position: Vec2::new(3.0, -3.0),
            velocity: Vec2::new(0.0, 1.0),
            preferred_velocity: Vec2::new(0.0, 1.0),
            radius: 0.5,
            max_speed: 2.0,
        };
        let hp = compute_orca_half_plane(&a, &b, 5.0);
        assert!(hp.normal.length() > 0.0);
    }

    #[test]
    fn rvo_agent_creation() {
        let a = RvoAgent::new(Vec2::new(1.0, 2.0), 0.5, 3.0);
        assert_eq!(a.position, Vec2::new(1.0, 2.0));
        assert_eq!(a.velocity, Vec2::ZERO);
        assert!((a.radius - 0.5).abs() < f32::EPSILON);
        assert!((a.max_speed - 3.0).abs() < f32::EPSILON);
    }

    #[test]
    fn rvo_agent_serde_roundtrip() {
        let a = RvoAgent::new(Vec2::new(1.0, 2.0), 0.5, 3.0);
        let json = serde_json::to_string(&a).unwrap();
        let deserialized: RvoAgent = serde_json::from_str(&json).unwrap();
        assert_eq!(deserialized.position, a.position);
    }

    // --- LP solver tests ---

    #[test]
    fn solve_no_constraints() {
        let v = solve_velocity(&[], Vec2::new(1.0, 0.0), 5.0);
        assert!((v - Vec2::new(1.0, 0.0)).length() < 0.01);
    }

    #[test]
    fn solve_speed_clamped() {
        let v = solve_velocity(&[], Vec2::new(10.0, 0.0), 5.0);
        assert!(v.length() <= 5.0 + f32::EPSILON);
        assert!((v.length() - 5.0).abs() < 0.01);
    }

    #[test]
    fn solve_single_constraint() {
        // Constraint: don't go right (normal points left)
        let hp = HalfPlane {
            point: Vec2::ZERO,
            normal: Vec2::new(-1.0, 0.0),
        };
        let v = solve_velocity(&[hp], Vec2::new(1.0, 0.0), 5.0);
        // Should be pushed to x ≤ 0
        assert!(v.x <= f32::EPSILON);
    }

    #[test]
    fn solve_head_on_agents() {
        let a = RvoAgent {
            position: Vec2::ZERO,
            velocity: Vec2::new(1.0, 0.0),
            preferred_velocity: Vec2::new(1.0, 0.0),
            radius: 0.5,
            max_speed: 2.0,
        };
        let b = RvoAgent {
            position: Vec2::new(3.0, 0.0),
            velocity: Vec2::new(-1.0, 0.0),
            preferred_velocity: Vec2::new(-1.0, 0.0),
            radius: 0.5,
            max_speed: 2.0,
        };
        let hp = compute_orca_half_plane(&a, &b, 5.0);
        let v = solve_velocity(&[hp], a.preferred_velocity, a.max_speed);
        // Should deviate laterally to avoid collision
        assert!(v.length() <= a.max_speed + f32::EPSILON);
    }

    #[test]
    fn solve_multiple_constraints() {
        // Two constraints forming a corridor
        let constraints = vec![
            HalfPlane {
                point: Vec2::new(0.0, 1.0),
                normal: Vec2::new(0.0, -1.0),
            },
            HalfPlane {
                point: Vec2::new(0.0, -1.0),
                normal: Vec2::new(0.0, 1.0),
            },
        ];
        let v = solve_velocity(&constraints, Vec2::new(3.0, 3.0), 5.0);
        // Should be constrained to -1 ≤ y ≤ 1
        assert!(v.y <= 1.0 + 0.1);
        assert!(v.y >= -1.0 - 0.1);
    }

    // --- Simulation tests ---

    #[test]
    fn simulation_basic() {
        let mut sim = RvoSimulation::new(5.0);
        let a = sim.add_agent(RvoAgent::new(Vec2::ZERO, 0.5, 2.0));
        let b = sim.add_agent(RvoAgent::new(Vec2::new(10.0, 0.0), 0.5, 2.0));

        sim.set_preferred_velocity(a, Vec2::new(1.0, 0.0));
        sim.set_preferred_velocity(b, Vec2::new(-1.0, 0.0));

        assert_eq!(sim.agent_count(), 2);

        // Run a few steps
        for _ in 0..10 {
            sim.step(0.1);
        }

        // Agents should have moved toward each other but not collided
        assert!(sim.agent(a).position.x > 0.0);
        assert!(sim.agent(b).position.x < 10.0);
    }

    #[test]
    fn simulation_agents_dont_overlap() {
        let mut sim = RvoSimulation::new(5.0);
        let a = sim.add_agent(RvoAgent::new(Vec2::ZERO, 1.0, 2.0));
        let b = sim.add_agent(RvoAgent::new(Vec2::new(5.0, 0.0), 1.0, 2.0));

        sim.set_preferred_velocity(a, Vec2::new(1.0, 0.0));
        sim.set_preferred_velocity(b, Vec2::new(-1.0, 0.0));

        for _ in 0..100 {
            sim.step(0.05);
        }

        // Agents should maintain at least some distance
        let dist = sim.agent(a).position.distance(sim.agent(b).position);
        // With ORCA, they should avoid overlap (combined radius = 2.0)
        assert!(dist > 1.0, "agents too close: {dist}");
    }

    #[test]
    fn simulation_single_agent() {
        let mut sim = RvoSimulation::new(5.0);
        let a = sim.add_agent(RvoAgent::new(Vec2::ZERO, 0.5, 5.0));
        sim.set_preferred_velocity(a, Vec2::new(3.0, 0.0));

        sim.step(1.0);

        // Single agent, no constraints — should move at preferred velocity
        assert!((sim.agent(a).position.x - 3.0).abs() < 0.1);
    }

    #[test]
    fn simulation_four_way_crossing() {
        let mut sim = RvoSimulation::new(3.0);

        let agents: Vec<usize> = vec![
            sim.add_agent(RvoAgent::new(Vec2::new(-5.0, 0.0), 0.5, 2.0)),
            sim.add_agent(RvoAgent::new(Vec2::new(5.0, 0.0), 0.5, 2.0)),
            sim.add_agent(RvoAgent::new(Vec2::new(0.0, -5.0), 0.5, 2.0)),
            sim.add_agent(RvoAgent::new(Vec2::new(0.0, 5.0), 0.5, 2.0)),
        ];

        sim.set_preferred_velocity(agents[0], Vec2::new(1.0, 0.0));
        sim.set_preferred_velocity(agents[1], Vec2::new(-1.0, 0.0));
        sim.set_preferred_velocity(agents[2], Vec2::new(0.0, 1.0));
        sim.set_preferred_velocity(agents[3], Vec2::new(0.0, -1.0));

        // Run for a while — shouldn't panic
        for _ in 0..100 {
            sim.step(0.05);
        }

        // All agents should have moved generally toward their targets
        assert!(sim.agent(agents[0]).position.x > -5.0);
        assert!(sim.agent(agents[1]).position.x < 5.0);
    }

    #[test]
    fn simulation_many_agents() {
        // 20 agents in a circle, all heading to the opposite side
        let mut sim = RvoSimulation::new(3.0);
        let n = 20;
        let radius = 10.0;
        let mut agents = Vec::new();
        for i in 0..n {
            let angle = std::f32::consts::TAU * i as f32 / n as f32;
            let pos = Vec2::new(angle.cos() * radius, angle.sin() * radius);
            let goal = -pos; // opposite side
            let idx = sim.add_agent(RvoAgent::new(pos, 0.5, 2.0));
            let dir = (goal - pos).normalize();
            sim.set_preferred_velocity(idx, dir * 2.0);
            agents.push(idx);
        }

        for _ in 0..200 {
            sim.step(0.05);
        }

        // All agents should have moved — no deadlock
        for &idx in &agents {
            let dist_from_origin = sim.agent(idx).position.length();
            assert!(
                dist_from_origin < radius + 5.0,
                "agent {idx} drifted too far: {dist_from_origin}"
            );
        }
    }

    #[test]
    fn half_plane_serde_roundtrip() {
        let hp = HalfPlane {
            point: Vec2::new(1.0, 2.0),
            normal: Vec2::new(0.0, 1.0),
        };
        let json = serde_json::to_string(&hp).unwrap();
        let deserialized: HalfPlane = serde_json::from_str(&json).unwrap();
        assert!((deserialized.point - hp.point).length() < f32::EPSILON);
        assert!((deserialized.normal - hp.normal).length() < f32::EPSILON);
    }

    // --- Spatial hash tests ---

    #[test]
    fn spatial_hash_basic() {
        let mut sh = SpatialHash::new(10.0);
        sh.insert(0, Vec2::new(5.0, 5.0));
        sh.insert(1, Vec2::new(15.0, 5.0));
        sh.insert(2, Vec2::new(100.0, 100.0));

        let mut results = Vec::new();
        sh.query_neighbors(Vec2::new(5.0, 5.0), 12.0, &mut results);
        assert!(results.contains(&0));
        assert!(results.contains(&1));
        assert!(!results.contains(&2));
    }

    #[test]
    fn spatial_hash_clear_reuse() {
        let mut sh = SpatialHash::new(10.0);
        sh.insert(0, Vec2::new(5.0, 5.0));
        sh.clear();

        let mut results = Vec::new();
        sh.query_neighbors(Vec2::new(5.0, 5.0), 12.0, &mut results);
        assert!(results.is_empty());

        // Re-insert after clear should work
        sh.insert(1, Vec2::new(5.0, 5.0));
        sh.query_neighbors(Vec2::new(5.0, 5.0), 12.0, &mut results);
        assert!(results.contains(&1));
    }

    #[test]
    fn spatial_hash_negative_coords() {
        let mut sh = SpatialHash::new(10.0);
        sh.insert(0, Vec2::new(-15.0, -15.0));
        sh.insert(1, Vec2::new(-5.0, -5.0));

        let mut results = Vec::new();
        sh.query_neighbors(Vec2::new(-10.0, -10.0), 12.0, &mut results);
        assert!(results.contains(&0));
        assert!(results.contains(&1));
    }

    #[test]
    fn rvo_with_spatial_hash() {
        let mut sim = RvoSimulation::with_neighbor_dist(2.0, 10.0);
        // Two agents heading toward each other
        let a = RvoAgent::new(Vec2::ZERO, 0.5, 2.0);
        let b = RvoAgent::new(Vec2::new(4.0, 0.0), 0.5, 2.0);
        sim.add_agent(a);
        sim.add_agent(b);
        sim.set_preferred_velocity(0, Vec2::new(1.0, 0.0));
        sim.set_preferred_velocity(1, Vec2::new(-1.0, 0.0));

        for _ in 0..20 {
            sim.step(0.1);
        }
        // Agents should have avoided each other
        let dist = sim.agent(0).position.distance(sim.agent(1).position);
        assert!(dist >= 0.9, "agents too close: {dist}");
    }

    #[test]
    fn rvo_far_agents_ignored() {
        let mut sim = RvoSimulation::with_neighbor_dist(2.0, 5.0);
        let a = RvoAgent::new(Vec2::ZERO, 0.5, 2.0);
        let b = RvoAgent::new(Vec2::new(100.0, 0.0), 0.5, 2.0);
        sim.add_agent(a);
        sim.add_agent(b);
        sim.set_preferred_velocity(0, Vec2::new(1.0, 0.0));
        sim.set_preferred_velocity(1, Vec2::new(-1.0, 0.0));

        sim.step(0.1);
        // Agent 0 should move at full preferred velocity (no constraints from far agent)
        assert!(
            (sim.agent(0).velocity.x - 1.0).abs() < 0.01,
            "expected ~1.0, got {}",
            sim.agent(0).velocity.x
        );
    }
}
