//! Crowd simulation — density-aware movement on top of RVO/ORCA.

use hisab::Vec2;

use crate::rvo::{RvoAgent, RvoSimulation};

/// Crowd simulation with density-aware velocity damping.
///
/// Wraps an [`RvoSimulation`] and adds a density grid that tracks agent
/// concentrations. Agents in high-density areas have their preferred
/// velocities scaled down to prevent crushing and create natural flow.
#[derive(Debug, Clone)]
pub struct CrowdSimulation {
    rvo: RvoSimulation,
    /// Density grid cell size in world units.
    density_cell_size: f32,
    /// Maximum comfortable agents per density cell.
    max_density: f32,
    /// Persistent density map — reused each step to avoid per-frame allocation.
    density_cache: std::collections::HashMap<(i32, i32), f32>,
}

impl CrowdSimulation {
    /// Create a new crowd simulation.
    ///
    /// - `time_horizon`: ORCA look-ahead time
    /// - `density_cell_size`: spatial grid cell size for density tracking
    /// - `max_density`: agents per cell before velocity damping kicks in
    #[must_use]
    pub fn new(time_horizon: f32, density_cell_size: f32, max_density: f32) -> Self {
        Self {
            rvo: RvoSimulation::new(time_horizon),
            density_cell_size,
            max_density,
            density_cache: std::collections::HashMap::new(),
        }
    }

    /// Add an agent and return its index.
    pub fn add_agent(&mut self, agent: RvoAgent) -> usize {
        self.rvo.add_agent(agent)
    }

    /// Number of agents.
    #[must_use]
    pub fn agent_count(&self) -> usize {
        self.rvo.agent_count()
    }

    /// Get an agent by index.
    #[must_use]
    pub fn agent(&self, idx: usize) -> &RvoAgent {
        self.rvo.agent(idx)
    }

    /// Mutable access to an agent.
    pub fn agent_mut(&mut self, idx: usize) -> &mut RvoAgent {
        self.rvo.agent_mut(idx)
    }

    /// Set the preferred velocity for an agent.
    pub fn set_preferred_velocity(&mut self, idx: usize, velocity: Vec2) {
        self.rvo.set_preferred_velocity(idx, velocity);
    }

    /// Step the simulation by `dt` seconds.
    ///
    /// Computes a density map, damps preferred velocities in crowded areas,
    /// then runs the RVO step.
    pub fn step(&mut self, dt: f32) {
        let n = self.agent_count();
        if n == 0 {
            return;
        }

        // Rebuild density map in-place (reuse allocation)
        self.rebuild_density();

        // Damp preferred velocities based on local density
        for i in 0..n {
            let cell = self.density_cell(self.rvo.agent(i).position);
            let local_density = self.density_cache.get(&cell).copied().unwrap_or(0.0);
            let damping = if local_density > self.max_density {
                self.max_density / local_density
            } else {
                1.0
            };
            let pref = self.rvo.agent(i).preferred_velocity;
            self.rvo.set_preferred_velocity(i, pref * damping);
        }

        self.rvo.step(dt);
    }

    /// Compute the density at a specific world position.
    ///
    /// Uses the cached density from the last `step()` call. Call `step()`
    /// first to ensure up-to-date values.
    #[must_use]
    pub fn density_at(&self, position: Vec2) -> f32 {
        let cell = self.density_cell(position);
        self.density_cache.get(&cell).copied().unwrap_or(0.0)
    }

    fn density_cell(&self, position: Vec2) -> (i32, i32) {
        (
            (position.x / self.density_cell_size).floor() as i32,
            (position.y / self.density_cell_size).floor() as i32,
        )
    }

    /// Clear and refill the density cache without allocating a new HashMap.
    fn rebuild_density(&mut self) {
        // Clear values but keep allocated buckets
        for v in self.density_cache.values_mut() {
            *v = 0.0;
        }
        for i in 0..self.agent_count() {
            let cell = self.density_cell(self.rvo.agent(i).position);
            *self.density_cache.entry(cell).or_insert(0.0) += 1.0;
        }
        // Remove cells that are now zero (agents moved away)
        self.density_cache.retain(|_, v| *v > 0.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crowd_basic() {
        let mut sim = CrowdSimulation::new(3.0, 2.0, 4.0);
        let a = sim.add_agent(RvoAgent::new(Vec2::ZERO, 0.5, 2.0));
        let b = sim.add_agent(RvoAgent::new(Vec2::new(10.0, 0.0), 0.5, 2.0));

        sim.set_preferred_velocity(a, Vec2::new(1.0, 0.0));
        sim.set_preferred_velocity(b, Vec2::new(-1.0, 0.0));

        assert_eq!(sim.agent_count(), 2);

        for _ in 0..10 {
            sim.step(0.1);
        }

        assert!(sim.agent(a).position.x > 0.0);
    }

    #[test]
    fn density_damping() {
        let mut sim = CrowdSimulation::new(3.0, 2.0, 2.0);

        // Pack 5 agents into a small area (max_density = 2)
        for i in 0..5 {
            let idx = sim.add_agent(RvoAgent::new(Vec2::new(0.1 * i as f32, 0.0), 0.1, 5.0));
            sim.set_preferred_velocity(idx, Vec2::new(5.0, 0.0));
        }

        // Step to populate density cache and apply damping
        sim.step(0.1);

        // Density at origin should be > max_density
        let d = sim.density_at(Vec2::ZERO);
        assert!(d > 2.0);
    }

    #[test]
    fn density_at_empty() {
        let sim = CrowdSimulation::new(3.0, 2.0, 4.0);
        assert!((sim.density_at(Vec2::ZERO) - 0.0).abs() < f32::EPSILON);
    }

    #[test]
    fn crowd_empty_step() {
        let mut sim = CrowdSimulation::new(3.0, 2.0, 4.0);
        sim.step(0.1); // Should not panic
    }

    #[test]
    fn crowd_single_agent() {
        let mut sim = CrowdSimulation::new(3.0, 2.0, 4.0);
        let a = sim.add_agent(RvoAgent::new(Vec2::ZERO, 0.5, 5.0));
        sim.set_preferred_velocity(a, Vec2::new(3.0, 0.0));
        sim.step(1.0);
        assert!((sim.agent(a).position.x - 3.0).abs() < 0.5);
    }

    #[test]
    fn crowd_crossing() {
        let mut sim = CrowdSimulation::new(3.0, 3.0, 5.0);

        // Two groups crossing perpendicular
        for i in 0..5 {
            let idx = sim.add_agent(RvoAgent::new(Vec2::new(-10.0, i as f32 - 2.0), 0.5, 2.0));
            sim.set_preferred_velocity(idx, Vec2::new(1.0, 0.0));
        }
        for i in 0..5 {
            let idx = sim.add_agent(RvoAgent::new(Vec2::new(i as f32 - 2.0, -10.0), 0.5, 2.0));
            sim.set_preferred_velocity(idx, Vec2::new(0.0, 1.0));
        }

        for _ in 0..50 {
            sim.step(0.1);
        }

        // All agents should have moved generally toward their targets
        assert!(sim.agent(0).position.x > -10.0);
        assert!(sim.agent(5).position.y > -10.0);
    }
}
