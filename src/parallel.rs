//! Parallel pathfinding — multi-threaded batch operations using rayon.
//!
//! Requires the `parallel` feature flag.

use hisab::Vec2;
use rayon::prelude::*;

use crate::grid::{GridPos, NavGrid};
use crate::mesh::{NavMesh, NavPolyId};
use crate::path::PathResult;

#[cfg(feature = "logging")]
use tracing::instrument;

/// Compute multiple grid paths in parallel.
///
/// Each request is a `(start, goal)` pair. Returns results in the same order.
/// Uses rayon's work-stealing thread pool for automatic load balancing.
///
/// # Example
///
/// ```ignore
/// use raasta::parallel::find_paths_parallel;
///
/// let grid = NavGrid::new(100, 100, 1.0);
/// let requests = vec![
///     (GridPos::new(0, 0), GridPos::new(99, 99)),
///     (GridPos::new(10, 10), GridPos::new(50, 50)),
/// ];
/// let results = find_paths_parallel(&grid, &requests);
/// ```
#[cfg_attr(feature = "logging", instrument(skip(grid, requests), fields(count = requests.len())))]
#[must_use]
pub fn find_paths_parallel(
    grid: &NavGrid,
    requests: &[(GridPos, GridPos)],
) -> Vec<Option<Vec<GridPos>>> {
    requests
        .par_iter()
        .map(|&(start, goal)| grid.find_path(start, goal))
        .collect()
}

/// Compute multiple grid paths in parallel, returning `PathResult`s with world coordinates.
#[cfg_attr(feature = "logging", instrument(skip(grid, requests), fields(count = requests.len())))]
#[must_use]
pub fn find_path_results_parallel(
    grid: &NavGrid,
    requests: &[(GridPos, GridPos)],
) -> Vec<PathResult> {
    requests
        .par_iter()
        .map(|&(start, goal)| match grid.find_path(start, goal) {
            Some(path) => {
                let waypoints: Vec<Vec2> = path.iter().map(|&p| grid.grid_to_world(p)).collect();
                PathResult::found(waypoints)
            }
            None => PathResult::not_found(),
        })
        .collect()
}

/// Compute multiple navmesh paths in parallel.
#[cfg_attr(feature = "logging", instrument(skip(mesh, requests), fields(count = requests.len())))]
#[must_use]
pub fn find_mesh_paths_parallel(
    mesh: &NavMesh,
    requests: &[(Vec2, Vec2)],
) -> Vec<Option<Vec<NavPolyId>>> {
    requests
        .par_iter()
        .map(|&(start, goal)| mesh.find_path(start, goal))
        .collect()
}

/// Compute multiple JPS paths in parallel.
#[cfg_attr(feature = "logging", instrument(skip(grid, requests), fields(count = requests.len())))]
#[must_use]
pub fn find_paths_jps_parallel(
    grid: &NavGrid,
    requests: &[(GridPos, GridPos)],
) -> Vec<Option<Vec<GridPos>>> {
    requests
        .par_iter()
        .map(|&(start, goal)| grid.find_path_jps(start, goal))
        .collect()
}

/// Compute multiple Theta* paths in parallel.
#[cfg_attr(feature = "logging", instrument(skip(grid, requests), fields(count = requests.len())))]
#[must_use]
pub fn find_paths_theta_parallel(
    grid: &NavGrid,
    requests: &[(GridPos, GridPos)],
) -> Vec<Option<Vec<GridPos>>> {
    requests
        .par_iter()
        .map(|&(start, goal)| grid.find_path_theta(start, goal))
        .collect()
}

/// Process a batch of grid paths in parallel with a custom algorithm selector.
///
/// The `algorithm` closure receives `(grid, start, goal)` and returns the path.
/// This allows mixing algorithms per-request.
#[must_use]
pub fn find_paths_custom_parallel<F>(
    grid: &NavGrid,
    requests: &[(GridPos, GridPos)],
    algorithm: F,
) -> Vec<Option<Vec<GridPos>>>
where
    F: Fn(&NavGrid, GridPos, GridPos) -> Option<Vec<GridPos>> + Sync,
{
    requests
        .par_iter()
        .map(|&(start, goal)| algorithm(grid, start, goal))
        .collect()
}

/// Parallel flow field computation for multiple goals.
#[cfg_attr(feature = "logging", instrument(skip(grid, goals), fields(count = goals.len())))]
#[must_use]
pub fn flow_fields_parallel(grid: &NavGrid, goals: &[GridPos]) -> Vec<Vec<(i32, i32)>> {
    goals
        .par_iter()
        .map(|&goal| grid.flow_field(goal))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parallel_grid_paths() {
        let grid = NavGrid::new(20, 20, 1.0);
        let requests = vec![
            (GridPos::new(0, 0), GridPos::new(19, 19)),
            (GridPos::new(0, 0), GridPos::new(10, 10)),
            (GridPos::new(5, 5), GridPos::new(15, 15)),
        ];
        let results = find_paths_parallel(&grid, &requests);
        assert_eq!(results.len(), 3);
        for result in &results {
            assert!(result.is_some());
        }
    }

    #[test]
    fn parallel_grid_paths_mixed() {
        let mut grid = NavGrid::new(20, 20, 1.0);
        for y in 0..20 {
            grid.set_walkable(10, y, false);
        }
        let requests = vec![
            (GridPos::new(0, 0), GridPos::new(5, 5)),   // reachable
            (GridPos::new(0, 0), GridPos::new(15, 15)), // unreachable
        ];
        let results = find_paths_parallel(&grid, &requests);
        assert!(results[0].is_some());
        assert!(results[1].is_none());
    }

    #[test]
    fn parallel_path_results() {
        let grid = NavGrid::new(10, 10, 1.0);
        let requests = vec![(GridPos::new(0, 0), GridPos::new(9, 9))];
        let results = find_path_results_parallel(&grid, &requests);
        assert_eq!(results.len(), 1);
        assert!(results[0].is_found());
        assert!(results[0].length > 0.0);
    }

    #[test]
    fn parallel_mesh_paths() {
        use crate::mesh::{NavMesh, NavPoly, NavPolyId};
        let mut mesh = NavMesh::new();
        mesh.add_poly(NavPoly {
            id: NavPolyId(0),
            vertices: vec![
                Vec2::ZERO,
                Vec2::new(10.0, 0.0),
                Vec2::new(10.0, 10.0),
                Vec2::new(0.0, 10.0),
            ],
            neighbors: vec![NavPolyId(1)],
            cost: 1.0,
            layer: 0,
        });
        mesh.add_poly(NavPoly {
            id: NavPolyId(1),
            vertices: vec![
                Vec2::new(10.0, 0.0),
                Vec2::new(20.0, 0.0),
                Vec2::new(20.0, 10.0),
                Vec2::new(10.0, 10.0),
            ],
            neighbors: vec![NavPolyId(0)],
            cost: 1.0,
            layer: 0,
        });

        let requests = vec![(Vec2::new(5.0, 5.0), Vec2::new(15.0, 5.0))];
        let results = find_mesh_paths_parallel(&mesh, &requests);
        assert!(results[0].is_some());
    }

    #[test]
    fn parallel_jps() {
        let grid = NavGrid::new(20, 20, 1.0);
        let requests = vec![
            (GridPos::new(0, 0), GridPos::new(19, 19)),
            (GridPos::new(0, 0), GridPos::new(10, 10)),
        ];
        let results = find_paths_jps_parallel(&grid, &requests);
        assert_eq!(results.len(), 2);
        for r in &results {
            assert!(r.is_some());
        }
    }

    #[test]
    fn parallel_flow_fields() {
        let grid = NavGrid::new(10, 10, 1.0);
        let goals = vec![GridPos::new(0, 0), GridPos::new(9, 9)];
        let fields = flow_fields_parallel(&grid, &goals);
        assert_eq!(fields.len(), 2);
    }

    #[test]
    fn parallel_empty_requests() {
        let grid = NavGrid::new(10, 10, 1.0);
        let results = find_paths_parallel(&grid, &[]);
        assert!(results.is_empty());
    }

    #[test]
    fn parallel_custom_algorithm() {
        let grid = NavGrid::new(20, 20, 1.0);
        let requests = vec![(GridPos::new(0, 0), GridPos::new(19, 19))];
        let results =
            find_paths_custom_parallel(&grid, &requests, |g, s, goal| g.find_path_jps(s, goal));
        assert!(results[0].is_some());
    }

    #[test]
    fn parallel_many_requests() {
        let grid = NavGrid::new(50, 50, 1.0);
        let requests: Vec<(GridPos, GridPos)> = (0..100)
            .map(|i| (GridPos::new(0, 0), GridPos::new(i % 50, i / 2 % 50)))
            .collect();
        let results = find_paths_parallel(&grid, &requests);
        assert_eq!(results.len(), 100);
    }
}
