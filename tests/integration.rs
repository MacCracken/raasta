//! Integration tests for raasta.

use raasta::{GridPos, NavGrid, NavMesh, NavPoly, NavPolyId, PathResult, PathStatus};
use raasta::{SteerBehavior, compute_steer, funnel_smooth};

#[test]
fn grid_pathfinding_end_to_end() {
    let mut grid = NavGrid::new(20, 20, 1.0);

    // Create a wall with a gap
    for y in 0..18 {
        grid.set_walkable(10, y, false);
    }

    let path = grid.find_path(GridPos::new(0, 0), GridPos::new(19, 19));
    assert!(path.is_some());
    let path = path.unwrap();
    assert_eq!(*path.first().unwrap(), GridPos::new(0, 0));
    assert_eq!(*path.last().unwrap(), GridPos::new(19, 19));

    // All waypoints should be walkable
    for p in &path {
        assert!(grid.is_walkable(p.x, p.y));
    }
}

#[test]
fn navmesh_pathfinding_end_to_end() {
    let mut mesh = NavMesh::new();

    // Create a corridor: 3 connected squares
    mesh.add_poly(NavPoly {
        id: NavPolyId(0),
        vertices: vec![[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]],
        neighbors: vec![NavPolyId(1)],
    });
    mesh.add_poly(NavPoly {
        id: NavPolyId(1),
        vertices: vec![[2.0, 0.0], [4.0, 0.0], [4.0, 2.0], [2.0, 2.0]],
        neighbors: vec![NavPolyId(0), NavPolyId(2)],
    });
    mesh.add_poly(NavPoly {
        id: NavPolyId(2),
        vertices: vec![[4.0, 0.0], [6.0, 0.0], [6.0, 2.0], [4.0, 2.0]],
        neighbors: vec![NavPolyId(1)],
    });

    let path = mesh.find_path([1.0, 1.0], [5.0, 1.0]);
    assert!(path.is_some());
    let path = path.unwrap();
    assert_eq!(path.len(), 3);
    assert_eq!(path[0], NavPolyId(0));
    assert_eq!(path[2], NavPolyId(2));
}

#[test]
fn grid_flow_field_end_to_end() {
    let grid = NavGrid::new(10, 10, 1.0);
    let field = grid.flow_field(GridPos::new(9, 9));

    // Every walkable cell should have a direction (except goal)
    for y in 0..10i32 {
        for x in 0..10i32 {
            let idx = (y * 10 + x) as usize;
            if x == 9 && y == 9 {
                assert_eq!(field[idx], (0, 0)); // goal
            } else {
                let (dx, dy) = field[idx];
                assert!(dx != 0 || dy != 0, "cell ({x},{y}) has no direction");
            }
        }
    }
}

#[test]
fn path_result_types() {
    let found = PathResult::found(vec![[0.0, 0.0], [3.0, 4.0]]);
    assert_eq!(found.status, PathStatus::Found);
    assert!(found.is_found());
    assert!((found.length - 5.0).abs() < 0.01);

    let not_found = PathResult::not_found();
    assert!(!not_found.is_found());
}

#[test]
fn steering_follow_path() {
    // Simulate an agent following a path using seek
    let mut position = [0.0f32, 0.0f32];
    let waypoints = vec![[5.0, 0.0], [5.0, 5.0], [10.0, 5.0]];
    let max_speed = 1.0;

    for target in &waypoints {
        for _ in 0..100 {
            let out = compute_steer(
                &SteerBehavior::Seek { target: *target },
                position,
                max_speed,
            );
            position[0] += out.velocity[0] * 0.1;
            position[1] += out.velocity[1] * 0.1;

            let dx = target[0] - position[0];
            let dy = target[1] - position[1];
            if (dx * dx + dy * dy).sqrt() < 0.5 {
                break;
            }
        }
    }

    // Should be near the final target
    let dx = 10.0 - position[0];
    let dy = 5.0 - position[1];
    assert!((dx * dx + dy * dy).sqrt() < 1.0);
}

#[test]
fn smooth_grid_path() {
    let grid = NavGrid::new(10, 10, 1.0);
    let path = grid
        .find_path(GridPos::new(0, 0), GridPos::new(9, 9))
        .unwrap();

    // Convert to world positions
    let world_path: Vec<[f32; 2]> = path
        .iter()
        .map(|p| {
            let (wx, wy) = grid.grid_to_world(*p);
            [wx, wy]
        })
        .collect();

    let smoothed = funnel_smooth(&world_path);

    // Smoothed path should be shorter or equal
    assert!(smoothed.len() <= world_path.len());
    // Endpoints preserved
    assert_eq!(smoothed.first().unwrap(), world_path.first().unwrap());
    assert_eq!(smoothed.last().unwrap(), world_path.last().unwrap());
}
