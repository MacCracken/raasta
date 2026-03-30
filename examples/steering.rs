//! Steering behaviors example — seek, flee, arrive, and blending.

use raasta::{
    Obstacle, SteerBehavior, Vec2, WeightedSteer, avoid_obstacles, blend_weighted, compute_steer,
};

fn main() {
    let position = Vec2::new(10.0, 10.0);
    let max_speed = 5.0;

    // Seek
    let seek = compute_steer(
        &SteerBehavior::Seek {
            target: Vec2::new(50.0, 50.0),
        },
        position,
        max_speed,
    );
    println!(
        "Seek:   velocity ({:.2}, {:.2})",
        seek.velocity.x, seek.velocity.y
    );

    // Arrive (slows down near target)
    let arrive = compute_steer(
        &SteerBehavior::Arrive {
            target: Vec2::new(12.0, 12.0),
            slow_radius: 5.0,
        },
        position,
        max_speed,
    );
    println!(
        "Arrive: velocity ({:.2}, {:.2}), speed {:.2}",
        arrive.velocity.x,
        arrive.velocity.y,
        arrive.speed()
    );

    // Obstacle avoidance
    let obstacles = vec![Obstacle {
        center: Vec2::new(15.0, 10.0),
        radius: 2.0,
    }];
    let avoid = avoid_obstacles(position, Vec2::new(1.0, 0.0), &obstacles, 10.0, 5.0);
    println!(
        "Avoid:  velocity ({:.2}, {:.2})",
        avoid.velocity.x, avoid.velocity.y
    );

    // Blend seek + avoidance (avoidance weighted higher)
    let blended = blend_weighted(
        &[
            WeightedSteer {
                output: seek,
                weight: 1.0,
            },
            WeightedSteer {
                output: avoid,
                weight: 3.0,
            },
        ],
        max_speed,
    );
    println!(
        "Blend:  velocity ({:.2}, {:.2})",
        blended.velocity.x, blended.velocity.y
    );
}
