//! Integration tests for raasta.

use raasta::{GridPos, NavGrid, NavMesh, NavPoly, NavPolyId, PathResult, PathStatus, Vec2};
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
        vertices: vec![
            Vec2::ZERO,
            Vec2::new(2.0, 0.0),
            Vec2::new(2.0, 2.0),
            Vec2::new(0.0, 2.0),
        ],
        neighbors: vec![NavPolyId(1)],
        cost: 1.0,
    });
    mesh.add_poly(NavPoly {
        id: NavPolyId(1),
        vertices: vec![
            Vec2::new(2.0, 0.0),
            Vec2::new(4.0, 0.0),
            Vec2::new(4.0, 2.0),
            Vec2::new(2.0, 2.0),
        ],
        neighbors: vec![NavPolyId(0), NavPolyId(2)],
        cost: 1.0,
    });
    mesh.add_poly(NavPoly {
        id: NavPolyId(2),
        vertices: vec![
            Vec2::new(4.0, 0.0),
            Vec2::new(6.0, 0.0),
            Vec2::new(6.0, 2.0),
            Vec2::new(4.0, 2.0),
        ],
        neighbors: vec![NavPolyId(1)],
        cost: 1.0,
    });

    let path = mesh.find_path(Vec2::ONE, Vec2::new(5.0, 1.0));
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
    let found = PathResult::found(vec![Vec2::ZERO, Vec2::new(3.0, 4.0)]);
    assert_eq!(found.status, PathStatus::Found);
    assert!(found.is_found());
    assert!((found.length - 5.0).abs() < 0.01);

    let not_found = PathResult::not_found();
    assert!(!not_found.is_found());
}

#[test]
fn steering_follow_path() {
    // Simulate an agent following a path using seek
    let mut position = Vec2::ZERO;
    let waypoints = vec![
        Vec2::new(5.0, 0.0),
        Vec2::new(5.0, 5.0),
        Vec2::new(10.0, 5.0),
    ];
    let max_speed = 1.0;

    for target in &waypoints {
        for _ in 0..100 {
            let out = compute_steer(
                &SteerBehavior::Seek { target: *target },
                position,
                max_speed,
            );
            position += out.velocity * 0.1;

            if position.distance(*target) < 0.5 {
                break;
            }
        }
    }

    // Should be near the final target
    assert!(position.distance(Vec2::new(10.0, 5.0)) < 1.0);
}

#[test]
fn smooth_grid_path() {
    let grid = NavGrid::new(10, 10, 1.0);
    let path = grid
        .find_path(GridPos::new(0, 0), GridPos::new(9, 9))
        .unwrap();

    // Convert to world positions
    let world_path: Vec<Vec2> = path.iter().map(|p| grid.grid_to_world(*p)).collect();

    let smoothed = funnel_smooth(&world_path);

    // Smoothed path should be shorter or equal
    assert!(smoothed.len() <= world_path.len());
    // Endpoints preserved
    assert_eq!(smoothed[0], world_path[0]);
    assert_eq!(*smoothed.last().unwrap(), *world_path.last().unwrap());
}
