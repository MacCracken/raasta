use criterion::{Criterion, black_box, criterion_group, criterion_main};
use raasta::{
    AbstractGraph, Agent, CrowdSimulation, GridClusters, GridPos, NavGrid, NavMesh, NavPoly,
    NavPolyId, Obstacle, PathFollower, RvoAgent, RvoSimulation, SteerBehavior, Vec2,
    avoid_obstacles, compute_steer, funnel_smooth, triangulate,
};

fn bench_grid_astar_10x10(c: &mut Criterion) {
    let grid = NavGrid::new(10, 10, 1.0);
    c.bench_function("grid_astar_10x10", |b| {
        b.iter(|| {
            black_box(grid.find_path(GridPos::new(0, 0), GridPos::new(9, 9)));
        });
    });
}

fn bench_grid_astar_50x50(c: &mut Criterion) {
    let grid = NavGrid::new(50, 50, 1.0);
    c.bench_function("grid_astar_50x50", |b| {
        b.iter(|| {
            black_box(grid.find_path(GridPos::new(0, 0), GridPos::new(49, 49)));
        });
    });
}

fn bench_grid_astar_100x100(c: &mut Criterion) {
    let grid = NavGrid::new(100, 100, 1.0);
    c.bench_function("grid_astar_100x100", |b| {
        b.iter(|| {
            black_box(grid.find_path(GridPos::new(0, 0), GridPos::new(99, 99)));
        });
    });
}

fn bench_grid_astar_with_obstacles(c: &mut Criterion) {
    let mut grid = NavGrid::new(50, 50, 1.0);
    // Create a maze-like pattern
    for y in (0..50).step_by(4) {
        for x in 0..48 {
            grid.set_walkable(x, y, false);
        }
    }
    for y in (2..50).step_by(4) {
        for x in 2..50 {
            grid.set_walkable(x, y, false);
        }
    }
    c.bench_function("grid_astar_50x50_obstacles", |b| {
        b.iter(|| {
            black_box(grid.find_path(GridPos::new(0, 1), GridPos::new(49, 49)));
        });
    });
}

fn bench_flow_field_20x20(c: &mut Criterion) {
    let grid = NavGrid::new(20, 20, 1.0);
    c.bench_function("flow_field_20x20", |b| {
        b.iter(|| {
            black_box(grid.flow_field(GridPos::new(19, 19)));
        });
    });
}

fn bench_flow_field_50x50(c: &mut Criterion) {
    let grid = NavGrid::new(50, 50, 1.0);
    c.bench_function("flow_field_50x50", |b| {
        b.iter(|| {
            black_box(grid.flow_field(GridPos::new(49, 49)));
        });
    });
}

fn bench_navmesh_path_10_polys(c: &mut Criterion) {
    let mut mesh = NavMesh::new();
    for i in 0..10u32 {
        let x = i as f32 * 2.0;
        let mut neighbors = Vec::new();
        if i > 0 {
            neighbors.push(NavPolyId(i - 1));
        }
        if i < 9 {
            neighbors.push(NavPolyId(i + 1));
        }
        mesh.add_poly(NavPoly {
            id: NavPolyId(i),
            vertices: vec![
                Vec2::new(x, 0.0),
                Vec2::new(x + 2.0, 0.0),
                Vec2::new(x + 2.0, 2.0),
                Vec2::new(x, 2.0),
            ],
            neighbors,
            cost: 1.0,
        });
    }
    c.bench_function("navmesh_path_10_polys", |b| {
        b.iter(|| {
            black_box(mesh.find_path(Vec2::ONE, Vec2::new(19.0, 1.0)));
        });
    });
}

fn bench_steer_seek(c: &mut Criterion) {
    let behavior = SteerBehavior::Seek {
        target: Vec2::new(100.0, 100.0),
    };
    c.bench_function("steer_seek", |b| {
        b.iter(|| {
            black_box(compute_steer(&behavior, Vec2::ZERO, 5.0));
        });
    });
}

fn bench_steer_arrive(c: &mut Criterion) {
    let behavior = SteerBehavior::Arrive {
        target: Vec2::new(10.0, 10.0),
        slow_radius: 5.0,
    };
    c.bench_function("steer_arrive", |b| {
        b.iter(|| {
            black_box(compute_steer(&behavior, Vec2::new(7.0, 7.0), 5.0));
        });
    });
}

fn bench_grid_astar_batch_100(c: &mut Criterion) {
    let grid = NavGrid::new(50, 50, 1.0);
    let requests: Vec<(GridPos, GridPos)> = (0..100)
        .map(|i| {
            let sx = (i * 7) % 50;
            let sy = (i * 13) % 50;
            let gx = (i * 11 + 3) % 50;
            let gy = (i * 17 + 7) % 50;
            (GridPos::new(sx, sy), GridPos::new(gx, gy))
        })
        .collect();

    c.bench_function("grid_astar_batch_100", |b| {
        b.iter(|| {
            for (start, goal) in &requests {
                black_box(grid.find_path(*start, *goal));
            }
        });
    });
}

fn make_linear_mesh(n: u32) -> NavMesh {
    let mut mesh = NavMesh::new();
    for i in 0..n {
        let x = i as f32 * 2.0;
        let mut neighbors = Vec::new();
        if i > 0 {
            neighbors.push(NavPolyId(i - 1));
        }
        if i < n - 1 {
            neighbors.push(NavPolyId(i + 1));
        }
        mesh.add_poly(NavPoly {
            id: NavPolyId(i),
            vertices: vec![
                Vec2::new(x, 0.0),
                Vec2::new(x + 2.0, 0.0),
                Vec2::new(x + 2.0, 2.0),
                Vec2::new(x, 2.0),
            ],
            neighbors,
            cost: 1.0,
        });
    }
    mesh
}

fn bench_navmesh_path_100_polys(c: &mut Criterion) {
    let mesh = make_linear_mesh(100);
    c.bench_function("navmesh_path_100_polys", |b| {
        b.iter(|| {
            black_box(mesh.find_path(Vec2::ONE, Vec2::new(199.0, 1.0)));
        });
    });
}

fn bench_navmesh_path_500_polys(c: &mut Criterion) {
    let mesh = make_linear_mesh(500);
    c.bench_function("navmesh_path_500_polys", |b| {
        b.iter(|| {
            black_box(mesh.find_path(Vec2::ONE, Vec2::new(999.0, 1.0)));
        });
    });
}

fn bench_navmesh_path_1000_polys(c: &mut Criterion) {
    let mesh = make_linear_mesh(1000);
    c.bench_function("navmesh_path_1000_polys", |b| {
        b.iter(|| {
            black_box(mesh.find_path(Vec2::ONE, Vec2::new(1999.0, 1.0)));
        });
    });
}

fn bench_avoid_obstacles_10(c: &mut Criterion) {
    let obstacles: Vec<Obstacle> = (0..10)
        .map(|i| Obstacle {
            center: Vec2::new(i as f32 * 3.0 + 5.0, ((i * 7) % 5) as f32 - 2.0),
            radius: 1.0,
        })
        .collect();
    c.bench_function("avoid_obstacles_10", |b| {
        b.iter(|| {
            black_box(avoid_obstacles(
                Vec2::ZERO,
                Vec2::new(1.0, 0.0),
                &obstacles,
                30.0,
                5.0,
            ));
        });
    });
}

fn bench_agent_tick(c: &mut Criterion) {
    let mut agent = Agent::new(Vec2::ZERO, 10.0, 20.0);
    agent.set_path(PathFollower::new(
        vec![Vec2::new(50.0, 0.0), Vec2::new(50.0, 50.0)],
        1.0,
        5.0,
    ));
    let obstacles: Vec<Obstacle> = (0..5)
        .map(|i| Obstacle {
            center: Vec2::new(10.0 + i as f32 * 8.0, 2.0),
            radius: 1.5,
        })
        .collect();
    c.bench_function("agent_tick", |b| {
        b.iter(|| {
            let mut a = agent.clone();
            black_box(a.update(0.016, &obstacles));
        });
    });
}

fn bench_jps_50x50(c: &mut Criterion) {
    let grid = NavGrid::new(50, 50, 1.0);
    c.bench_function("jps_50x50", |b| {
        b.iter(|| {
            black_box(grid.find_path_jps(GridPos::new(0, 0), GridPos::new(49, 49)));
        });
    });
}

fn bench_jps_100x100(c: &mut Criterion) {
    let grid = NavGrid::new(100, 100, 1.0);
    c.bench_function("jps_100x100", |b| {
        b.iter(|| {
            black_box(grid.find_path_jps(GridPos::new(0, 0), GridPos::new(99, 99)));
        });
    });
}

fn bench_jps_50x50_obstacles(c: &mut Criterion) {
    let mut grid = NavGrid::new(50, 50, 1.0);
    for y in (0..50).step_by(4) {
        for x in 0..48 {
            grid.set_walkable(x, y, false);
        }
    }
    for y in (2..50).step_by(4) {
        for x in 2..50 {
            grid.set_walkable(x, y, false);
        }
    }
    c.bench_function("jps_50x50_obstacles", |b| {
        b.iter(|| {
            black_box(grid.find_path_jps(GridPos::new(0, 1), GridPos::new(49, 49)));
        });
    });
}

fn bench_hpa_200x200_batch(c: &mut Criterion) {
    let mut grid = NavGrid::new(200, 200, 1.0);
    // Add walls to make it interesting
    for y in (0..200).step_by(20) {
        for x in 0..190 {
            grid.set_walkable(x, y, false);
        }
    }
    for y in (10..200).step_by(20) {
        for x in 10..200 {
            grid.set_walkable(x, y, false);
        }
    }
    let clusters = GridClusters::build(&grid, 16);
    let graph = AbstractGraph::build(&grid, &clusters);
    let requests: Vec<(GridPos, GridPos)> = (0..20)
        .map(|i| {
            let sx = ((i * 7 + 1) % 190) + 1;
            let sy = ((i * 13 + 1) % 19) + 1;
            let gx = ((i * 11 + 3) % 190) + 1;
            let gy = 199 - ((i * 3) % 19);
            (GridPos::new(sx, sy), GridPos::new(gx, gy))
        })
        .collect();
    c.bench_function("hpa_200x200_batch20", |b| {
        b.iter(|| {
            for (start, goal) in &requests {
                black_box(graph.find_path(&grid, &clusters, *start, *goal));
            }
        });
    });
}

fn bench_astar_200x200_batch(c: &mut Criterion) {
    let mut grid = NavGrid::new(200, 200, 1.0);
    for y in (0..200).step_by(20) {
        for x in 0..190 {
            grid.set_walkable(x, y, false);
        }
    }
    for y in (10..200).step_by(20) {
        for x in 10..200 {
            grid.set_walkable(x, y, false);
        }
    }
    let requests: Vec<(GridPos, GridPos)> = (0..20)
        .map(|i| {
            let sx = ((i * 7 + 1) % 190) + 1;
            let sy = ((i * 13 + 1) % 19) + 1;
            let gx = ((i * 11 + 3) % 190) + 1;
            let gy = 199 - ((i * 3) % 19);
            (GridPos::new(sx, sy), GridPos::new(gx, gy))
        })
        .collect();
    c.bench_function("astar_200x200_batch20", |b| {
        b.iter(|| {
            for (start, goal) in &requests {
                black_box(grid.find_path(*start, *goal));
            }
        });
    });
}

fn bench_theta_50x50(c: &mut Criterion) {
    let grid = NavGrid::new(50, 50, 1.0);
    c.bench_function("theta_50x50", |b| {
        b.iter(|| {
            black_box(grid.find_path_theta(GridPos::new(0, 0), GridPos::new(49, 49)));
        });
    });
}

fn bench_theta_50x50_obstacles(c: &mut Criterion) {
    let mut grid = NavGrid::new(50, 50, 1.0);
    for y in (0..50).step_by(4) {
        for x in 0..48 {
            grid.set_walkable(x, y, false);
        }
    }
    for y in (2..50).step_by(4) {
        for x in 2..50 {
            grid.set_walkable(x, y, false);
        }
    }
    c.bench_function("theta_50x50_obstacles", |b| {
        b.iter(|| {
            black_box(grid.find_path_theta(GridPos::new(0, 1), GridPos::new(49, 49)));
        });
    });
}

fn bench_rvo_step_20_agents(c: &mut Criterion) {
    let mut sim = RvoSimulation::new(3.0);
    let n = 20;
    let radius = 10.0;
    for i in 0..n {
        let angle = std::f32::consts::TAU * i as f32 / n as f32;
        let pos = Vec2::new(angle.cos() * radius, angle.sin() * radius);
        let idx = sim.add_agent(RvoAgent::new(pos, 0.5, 2.0));
        let dir = (-pos).normalize_or_zero();
        sim.set_preferred_velocity(idx, dir * 2.0);
    }
    c.bench_function("rvo_step_20_agents", |b| {
        b.iter(|| {
            let mut s = sim.clone();
            s.step(0.016);
            black_box(());
        });
    });
}

fn bench_crowd_step_20_agents(c: &mut Criterion) {
    let mut sim = CrowdSimulation::new(3.0, 2.0, 5.0);
    let n = 20;
    let radius = 10.0;
    for i in 0..n {
        let angle = std::f32::consts::TAU * i as f32 / n as f32;
        let pos = Vec2::new(angle.cos() * radius, angle.sin() * radius);
        let idx = sim.add_agent(RvoAgent::new(pos, 0.5, 2.0));
        let dir = (-pos).normalize_or_zero();
        sim.set_preferred_velocity(idx, dir * 2.0);
    }
    c.bench_function("crowd_step_20_agents", |b| {
        b.iter(|| {
            let mut s = sim.clone();
            s.step(0.016);
            black_box(());
        });
    });
}

fn bench_navmesh_bake_square(c: &mut Criterion) {
    let boundary = vec![
        Vec2::ZERO,
        Vec2::new(10.0, 0.0),
        Vec2::new(10.0, 10.0),
        Vec2::new(0.0, 10.0),
    ];
    c.bench_function("navmesh_bake_square", |b| {
        b.iter(|| {
            black_box(NavMesh::bake(&boundary));
        });
    });
}

fn bench_navmesh_bake_l_shape(c: &mut Criterion) {
    let boundary = vec![
        Vec2::new(0.0, 0.0),
        Vec2::new(6.0, 0.0),
        Vec2::new(6.0, 3.0),
        Vec2::new(3.0, 3.0),
        Vec2::new(3.0, 6.0),
        Vec2::new(0.0, 6.0),
    ];
    c.bench_function("navmesh_bake_l_shape", |b| {
        b.iter(|| {
            black_box(NavMesh::bake(&boundary));
        });
    });
}

fn bench_triangulate_20gon(c: &mut Criterion) {
    let verts: Vec<Vec2> = (0..20)
        .map(|i| {
            let angle = std::f32::consts::TAU * i as f32 / 20.0;
            Vec2::new(angle.cos() * 10.0, angle.sin() * 10.0)
        })
        .collect();
    c.bench_function("triangulate_20gon", |b| {
        b.iter(|| {
            black_box(triangulate(&verts));
        });
    });
}

fn bench_funnel_smooth_20_points(c: &mut Criterion) {
    let path: Vec<Vec2> = (0..20)
        .map(|i| {
            let x = i as f32;
            let y = if i % 3 == 0 { 1.0 } else { 0.0 };
            Vec2::new(x, y)
        })
        .collect();
    c.bench_function("funnel_smooth_20_points", |b| {
        b.iter(|| {
            black_box(funnel_smooth(&path));
        });
    });
}

criterion_group!(
    benches,
    bench_grid_astar_10x10,
    bench_grid_astar_50x50,
    bench_grid_astar_100x100,
    bench_grid_astar_with_obstacles,
    bench_jps_50x50,
    bench_jps_100x100,
    bench_jps_50x50_obstacles,
    bench_theta_50x50,
    bench_theta_50x50_obstacles,
    bench_flow_field_20x20,
    bench_flow_field_50x50,
    bench_navmesh_path_10_polys,
    bench_navmesh_path_100_polys,
    bench_navmesh_path_500_polys,
    bench_navmesh_path_1000_polys,
    bench_navmesh_bake_square,
    bench_navmesh_bake_l_shape,
    bench_triangulate_20gon,
    bench_funnel_smooth_20_points,
    bench_steer_seek,
    bench_steer_arrive,
    bench_avoid_obstacles_10,
    bench_agent_tick,
    bench_rvo_step_20_agents,
    bench_crowd_step_20_agents,
    bench_grid_astar_batch_100,
    bench_hpa_200x200_batch,
    bench_astar_200x200_batch,
);
criterion_main!(benches);
