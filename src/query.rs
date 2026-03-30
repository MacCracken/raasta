//! Reusable pathfinding query objects — pre-allocated scratch buffers for zero-alloc pathfinding.

use std::collections::BinaryHeap;

use crate::grid::{GridPos, NavGrid};

#[cfg(feature = "logging")]
use tracing::instrument;

/// A reusable A* query object with pre-allocated scratch buffers.
///
/// Eliminates per-path heap allocations by reusing buffers across
/// multiple pathfinding queries. Useful for high-throughput scenarios
/// where many paths are computed per frame.
///
/// # Example
///
/// ```ignore
/// let mut query = GridPathQuery::new(&grid);
///
/// // First query
/// let path1 = query.find_path(&grid, GridPos::new(0, 0), GridPos::new(9, 9));
///
/// // Second query — reuses same buffers
/// let path2 = query.find_path(&grid, GridPos::new(1, 1), GridPos::new(8, 8));
/// ```
pub struct GridPathQuery {
    g_score: Vec<f32>,
    came_from: Vec<Option<usize>>,
    closed: Vec<bool>,
    open: BinaryHeap<QueryNode>,
    grid_width: usize,
    grid_height: usize,
}

#[derive(Clone, Copy)]
struct QueryNode {
    idx: usize,
    f_score: f32,
}

impl PartialEq for QueryNode {
    fn eq(&self, other: &Self) -> bool {
        self.f_score == other.f_score
    }
}

impl Eq for QueryNode {}

impl PartialOrd for QueryNode {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for QueryNode {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        other
            .f_score
            .partial_cmp(&self.f_score)
            .unwrap_or(std::cmp::Ordering::Equal)
    }
}

impl GridPathQuery {
    /// Create a new query object sized for the given grid.
    ///
    /// Pre-allocates all scratch buffers. The query can only be used
    /// with grids of the same dimensions.
    #[cfg_attr(feature = "logging", instrument(skip(grid)))]
    #[must_use]
    pub fn new(grid: &NavGrid) -> Self {
        let len = grid.width() * grid.height();
        Self {
            g_score: vec![f32::INFINITY; len],
            came_from: vec![None; len],
            closed: vec![false; len],
            open: BinaryHeap::with_capacity(len / 4),
            grid_width: grid.width(),
            grid_height: grid.height(),
        }
    }

    /// Reset scratch buffers for a new query.
    fn reset(&mut self) {
        self.g_score.fill(f32::INFINITY);
        self.came_from.fill(None);
        self.closed.fill(false);
        self.open.clear();
    }

    /// Find a path using A* with pre-allocated buffers.
    ///
    /// Returns `None` if no path exists or start/goal is unwalkable.
    /// The grid must have the same dimensions as the grid used in `new()`.
    #[cfg_attr(feature = "logging", instrument(skip(self, grid)))]
    #[must_use]
    pub fn find_path(
        &mut self,
        grid: &NavGrid,
        start: GridPos,
        goal: GridPos,
    ) -> Option<Vec<GridPos>> {
        debug_assert_eq!(grid.width(), self.grid_width);
        debug_assert_eq!(grid.height(), self.grid_height);

        if !grid.is_walkable(start.x, start.y) || !grid.is_walkable(goal.x, goal.y) {
            return None;
        }
        if start == goal {
            return Some(vec![start]);
        }

        self.reset();

        let start_idx = self.index(start.x, start.y)?;
        let goal_idx = self.index(goal.x, goal.y)?;

        self.g_score[start_idx] = 0.0;
        let h = if grid.allow_diagonal {
            start.octile_distance(goal)
        } else {
            start.manhattan_distance(goal) as f32
        };
        self.open.push(QueryNode {
            idx: start_idx,
            f_score: h,
        });

        let mut neighbors_buf = [(0i32, 0i32, 0.0f32); 8];

        while let Some(current) = self.open.pop() {
            if current.idx == goal_idx {
                return Some(self.reconstruct_path(goal_idx));
            }

            if self.closed[current.idx] {
                continue;
            }
            self.closed[current.idx] = true;

            let cx = (current.idx % self.grid_width) as i32;
            let cy = (current.idx / self.grid_width) as i32;
            let count = grid_neighbors(grid, cx, cy, &mut neighbors_buf);

            for &(nx, ny, move_cost) in &neighbors_buf[..count] {
                let n_idx = ny as usize * self.grid_width + nx as usize;
                if self.closed[n_idx] {
                    continue;
                }
                let tentative_g = self.g_score[current.idx] + move_cost * grid.cost(nx, ny);

                if tentative_g < self.g_score[n_idx] {
                    self.came_from[n_idx] = Some(current.idx);
                    self.g_score[n_idx] = tentative_g;
                    let np = GridPos::new(nx, ny);
                    let h = if grid.allow_diagonal {
                        np.octile_distance(goal)
                    } else {
                        np.manhattan_distance(goal) as f32
                    };
                    self.open.push(QueryNode {
                        idx: n_idx,
                        f_score: tentative_g + h,
                    });
                }
            }
        }

        None
    }

    #[inline]
    fn index(&self, x: i32, y: i32) -> Option<usize> {
        if x >= 0 && y >= 0 && (x as usize) < self.grid_width && (y as usize) < self.grid_height {
            Some(y as usize * self.grid_width + x as usize)
        } else {
            None
        }
    }

    fn reconstruct_path(&self, goal_idx: usize) -> Vec<GridPos> {
        let mut path = Vec::new();
        let mut current = goal_idx;
        loop {
            let x = (current % self.grid_width) as i32;
            let y = (current / self.grid_width) as i32;
            path.push(GridPos::new(x, y));
            match self.came_from[current] {
                Some(prev) => current = prev,
                None => break,
            }
        }
        path.reverse();
        path
    }
}

/// Compute walkable neighbors using the grid's public API.
#[inline]
fn grid_neighbors(grid: &NavGrid, x: i32, y: i32, buf: &mut [(i32, i32, f32); 8]) -> usize {
    const CARDINAL: [(i32, i32); 4] = [(0, 1), (0, -1), (1, 0), (-1, 0)];
    const DIAGONAL: [(i32, i32); 4] = [(1, 1), (1, -1), (-1, 1), (-1, -1)];

    let mut count = 0;
    for (dx, dy) in CARDINAL {
        let nx = x + dx;
        let ny = y + dy;
        if grid.is_walkable(nx, ny) {
            buf[count] = (nx, ny, 1.0);
            count += 1;
        }
    }
    if grid.allow_diagonal {
        for (dx, dy) in DIAGONAL {
            let nx = x + dx;
            let ny = y + dy;
            if grid.is_walkable(nx, ny)
                && grid.is_walkable(x + dx, y)
                && grid.is_walkable(x, y + dy)
            {
                buf[count] = (nx, ny, std::f32::consts::SQRT_2);
                count += 1;
            }
        }
    }
    count
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn query_finds_path() {
        let grid = NavGrid::new(10, 10, 1.0);
        let mut query = GridPathQuery::new(&grid);
        let path = query.find_path(&grid, GridPos::new(0, 0), GridPos::new(9, 9));
        assert!(path.is_some());
        let path = path.unwrap();
        assert_eq!(*path.first().unwrap(), GridPos::new(0, 0));
        assert_eq!(*path.last().unwrap(), GridPos::new(9, 9));
    }

    #[test]
    fn query_reusable() {
        let grid = NavGrid::new(10, 10, 1.0);
        let mut query = GridPathQuery::new(&grid);

        let path1 = query.find_path(&grid, GridPos::new(0, 0), GridPos::new(9, 9));
        assert!(path1.is_some());

        // Reuse for different query
        let path2 = query.find_path(&grid, GridPos::new(0, 0), GridPos::new(5, 5));
        assert!(path2.is_some());
        assert_ne!(path1.unwrap().len(), path2.unwrap().len());
    }

    #[test]
    fn query_matches_regular() {
        let mut grid = NavGrid::new(15, 15, 1.0);
        grid.set_walkable(5, 3, false);
        grid.set_walkable(5, 4, false);

        let start = GridPos::new(0, 0);
        let goal = GridPos::new(14, 14);

        let regular = grid.find_path(start, goal).unwrap();

        let mut query = GridPathQuery::new(&grid);
        let queried = query.find_path(&grid, start, goal).unwrap();

        assert_eq!(regular, queried);
    }

    #[test]
    fn query_no_path() {
        let mut grid = NavGrid::new(10, 10, 1.0);
        for y in 0..10 {
            grid.set_walkable(5, y, false);
        }
        let mut query = GridPathQuery::new(&grid);
        assert!(
            query
                .find_path(&grid, GridPos::new(0, 0), GridPos::new(9, 9))
                .is_none()
        );
    }

    #[test]
    fn query_same_start_goal() {
        let grid = NavGrid::new(10, 10, 1.0);
        let mut query = GridPathQuery::new(&grid);
        let path = query.find_path(&grid, GridPos::new(5, 5), GridPos::new(5, 5));
        assert_eq!(path, Some(vec![GridPos::new(5, 5)]));
    }

    #[test]
    fn query_unwalkable() {
        let mut grid = NavGrid::new(10, 10, 1.0);
        grid.set_walkable(0, 0, false);
        let mut query = GridPathQuery::new(&grid);
        assert!(
            query
                .find_path(&grid, GridPos::new(0, 0), GridPos::new(9, 9))
                .is_none()
        );
    }

    #[test]
    fn query_many_sequential() {
        let grid = NavGrid::new(20, 20, 1.0);
        let mut query = GridPathQuery::new(&grid);

        for i in 0..10 {
            let path = query.find_path(&grid, GridPos::new(0, 0), GridPos::new(i, i));
            assert!(path.is_some());
        }
    }
}
