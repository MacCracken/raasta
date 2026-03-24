use criterion::{Criterion, black_box, criterion_group, criterion_main};
use raasta::{GridPos, NavGrid, NavMesh, NavPoly, NavPolyId, SteerBehavior, Vec2, compute_steer};

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

criterion_group!(
    benches,
    bench_grid_astar_10x10,
    bench_grid_astar_50x50,
    bench_grid_astar_100x100,
    bench_grid_astar_with_obstacles,
    bench_flow_field_20x20,
    bench_flow_field_50x50,
    bench_navmesh_path_10_polys,
    bench_steer_seek,
    bench_steer_arrive,
    bench_grid_astar_batch_100,
);
criterion_main!(benches);
