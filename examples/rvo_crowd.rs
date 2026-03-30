//! RVO crowd simulation example — agents avoid each other while moving to goals.

use raasta::{RvoAgent, RvoSimulation, Vec2};

fn main() {
    let mut sim = RvoSimulation::new(2.0);

    // Place 8 agents in a circle, all heading toward the center
    let n = 8;
    let radius = 10.0;
    for i in 0..n {
        let angle = std::f32::consts::TAU * i as f32 / n as f32;
        let pos = Vec2::new(angle.cos() * radius, angle.sin() * radius);
        let idx = sim.add_agent(RvoAgent::new(pos, 0.5, 2.0));
        let dir = (-pos).normalize_or_zero();
        sim.set_preferred_velocity(idx, dir * 2.0);
    }

    println!("Initial positions:");
    for i in 0..n {
        let a = sim.agent(i);
        println!("  Agent {i}: ({:.2}, {:.2})", a.position.x, a.position.y);
    }

    // Simulate 100 steps
    for _ in 0..100 {
        sim.step(0.05);
    }

    println!("\nAfter 5 seconds of simulation:");
    for i in 0..n {
        let a = sim.agent(i);
        println!("  Agent {i}: ({:.2}, {:.2})", a.position.x, a.position.y);
    }

    // Verify no collisions (agents should have avoided each other)
    let mut min_dist = f32::INFINITY;
    for i in 0..n {
        for j in (i + 1)..n {
            let d = sim.agent(i).position.distance(sim.agent(j).position);
            min_dist = min_dist.min(d);
        }
    }
    println!("\nMinimum inter-agent distance: {min_dist:.2} (radius sum = 1.0)");
}
