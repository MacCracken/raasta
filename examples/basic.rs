//! Basic pathfinding example.

use raasta::{GridPos, NavGrid};

fn main() {
    // Create a 20x20 grid
    let mut grid = NavGrid::new(20, 20, 1.0);

    // Add some walls
    for y in 0..15 {
        grid.set_walkable(10, y, false);
    }
    for x in 5..20 {
        grid.set_walkable(x, 15, false);
    }

    // Find a path
    let start = GridPos::new(0, 0);
    let goal = GridPos::new(19, 19);

    match grid.find_path(start, goal) {
        Some(path) => {
            println!("Path found! {} steps", path.len());
            for (i, pos) in path.iter().enumerate() {
                let w = grid.grid_to_world(*pos);
                println!(
                    "  [{i}] grid({}, {}) -> world({:.1}, {:.1})",
                    pos.x, pos.y, w.x, w.y
                );
            }
        }
        None => println!("No path found!"),
    }
}
