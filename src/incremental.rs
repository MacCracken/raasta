//! Incremental (time-sliced) pathfinding — spread A* computation across frames.

use std::collections::BinaryHeap;

use hisab::Vec2;
use serde::{Deserialize, Serialize};

use crate::grid::{GridPos, NavGrid};
use crate::path::{PathResult, PathStatus};

#[cfg(feature = "logging")]
use tracing::instrument;

/// Status of an incremental pathfinding query.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
pub enum IncrementalStatus {
    /// Still computing — call `step()` again.
    InProgress,
    /// Path found.
    Found,
    /// No path exists.
    NotFound,
}

/// An incremental (time-sliced) A* pathfinding query.
///
/// Instead of computing the entire path in one call, this allows
/// spreading the computation across multiple frames with a per-step
/// iteration budget.
///
/// # Example
///
/// ```ignore
/// let mut query = IncrementalGridPath::new(&grid, start, goal);
/// loop {
///     match query.step(&grid, 100) {
///         IncrementalStatus::InProgress => continue, // next frame
///         IncrementalStatus::Found => break,
///         IncrementalStatus::NotFound => break,
///     }
/// }
/// let result = query.result();
/// ```
#[derive(Debug, Clone)]
pub struct IncrementalGridPath {
    start: GridPos,
    goal: GridPos,
    g_score: Vec<f32>,
    came_from: Vec<Option<usize>>,
    closed: Vec<bool>,
    open: BinaryHeap<IncrementalNode>,
    status: IncrementalStatus,
    goal_idx: usize,
    grid_width: usize,
    allow_diagonal: bool,
}

#[derive(Clone, Copy)]
struct IncrementalNode {
    idx: usize,
    f_score: f32,
}

impl std::fmt::Debug for IncrementalNode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("IncrementalNode")
            .field("idx", &self.idx)
            .field("f_score", &self.f_score)
            .finish()
    }
}

impl PartialEq for IncrementalNode {
    fn eq(&self, other: &Self) -> bool {
        self.f_score == other.f_score
    }
}

impl Eq for IncrementalNode {}

impl PartialOrd for IncrementalNode {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for IncrementalNode {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        other
            .f_score
            .partial_cmp(&self.f_score)
            .unwrap_or(std::cmp::Ordering::Equal)
    }
}

impl IncrementalGridPath {
    /// Create a new incremental pathfinding query.
    ///
    /// Returns `None` if start or goal is outside the grid or unwalkable.
    #[cfg_attr(feature = "logging", instrument(skip(grid)))]
    #[must_use]
    pub fn new(grid: &NavGrid, start: GridPos, goal: GridPos) -> Option<Self> {
        if !grid.is_walkable(start.x, start.y) || !grid.is_walkable(goal.x, goal.y) {
            return None;
        }

        let len = grid.width() * grid.height();
        let grid_width = grid.width();
        let allow_diagonal = grid.allow_diagonal;

        // Compute start/goal indices manually (same as grid.index())
        let start_idx = start.y as usize * grid_width + start.x as usize;
        let goal_idx = goal.y as usize * grid_width + goal.x as usize;

        if start_idx >= len || goal_idx >= len {
            return None;
        }

        let mut g_score = vec![f32::INFINITY; len];
        g_score[start_idx] = 0.0;

        let h = if allow_diagonal {
            start.octile_distance(goal)
        } else {
            start.manhattan_distance(goal) as f32
        };

        let mut open = BinaryHeap::new();
        open.push(IncrementalNode {
            idx: start_idx,
            f_score: h,
        });

        Some(Self {
            start,
            goal,
            g_score,
            came_from: vec![None; len],
            closed: vec![false; len],
            open,
            status: IncrementalStatus::InProgress,
            goal_idx,
            grid_width,
            allow_diagonal,
        })
    }

    /// Advance the pathfinding computation by up to `max_iterations` node expansions.
    ///
    /// Returns the current status. Call repeatedly until not `InProgress`.
    #[cfg_attr(feature = "logging", instrument(skip(self, grid)))]
    pub fn step(&mut self, grid: &NavGrid, max_iterations: u32) -> IncrementalStatus {
        if self.status != IncrementalStatus::InProgress {
            return self.status;
        }

        let mut iterations = 0u32;
        let mut neighbors_buf = [(0i32, 0i32, 0.0f32); 8];

        while let Some(current) = self.open.pop() {
            if current.idx == self.goal_idx {
                self.status = IncrementalStatus::Found;
                return self.status;
            }

            if self.closed[current.idx] {
                continue;
            }
            self.closed[current.idx] = true;

            let cx = (current.idx % self.grid_width) as i32;
            let cy = (current.idx / self.grid_width) as i32;

            let count = grid_neighbors_into(grid, cx, cy, &mut neighbors_buf);

            for &(nx, ny, move_cost) in &neighbors_buf[..count] {
                let n_idx = ny as usize * self.grid_width + nx as usize;
                if self.closed[n_idx] {
                    continue;
                }
                let cell_cost = grid.cost(nx, ny);
                let tentative_g = self.g_score[current.idx] + move_cost * cell_cost;

                if tentative_g < self.g_score[n_idx] {
                    self.came_from[n_idx] = Some(current.idx);
                    self.g_score[n_idx] = tentative_g;
                    let np = GridPos::new(nx, ny);
                    let h = if self.allow_diagonal {
                        np.octile_distance(self.goal)
                    } else {
                        np.manhattan_distance(self.goal) as f32
                    };
                    self.open.push(IncrementalNode {
                        idx: n_idx,
                        f_score: tentative_g + h,
                    });
                }
            }

            iterations += 1;
            if iterations >= max_iterations {
                return self.status; // Still InProgress
            }
        }

        // Open set exhausted, no path found
        self.status = IncrementalStatus::NotFound;
        self.status
    }

    /// The start position of this query.
    #[must_use]
    pub fn start(&self) -> GridPos {
        self.start
    }

    /// The goal position of this query.
    #[must_use]
    pub fn goal(&self) -> GridPos {
        self.goal
    }

    /// Get the current status.
    #[must_use]
    pub fn status(&self) -> IncrementalStatus {
        self.status
    }

    /// Retrieve the computed path (only valid after `Found` status).
    ///
    /// Returns `None` if the path hasn't been found yet.
    #[must_use]
    pub fn path(&self) -> Option<Vec<GridPos>> {
        if self.status != IncrementalStatus::Found {
            return None;
        }

        let mut path = Vec::new();
        let mut current = self.goal_idx;
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
        Some(path)
    }

    /// Convert to a `PathResult`.
    ///
    /// Converts grid positions to world positions using the grid's coordinate system.
    #[must_use]
    pub fn to_path_result(&self, grid: &NavGrid) -> PathResult {
        match self.status {
            IncrementalStatus::Found => {
                if let Some(grid_path) = self.path() {
                    let waypoints: Vec<Vec2> = grid_path
                        .iter()
                        .map(|&pos| grid.grid_to_world(pos))
                        .collect();
                    PathResult::found(waypoints)
                } else {
                    PathResult::not_found()
                }
            }
            IncrementalStatus::NotFound => PathResult::not_found(),
            IncrementalStatus::InProgress => PathResult {
                status: PathStatus::Pending,
                waypoints: Vec::new(),
                length: 0.0,
            },
        }
    }

    /// Number of nodes expanded so far.
    #[must_use]
    pub fn nodes_expanded(&self) -> usize {
        self.closed.iter().filter(|&&c| c).count()
    }
}

/// Compute walkable neighbors for grid pathfinding.
/// Replicates `NavGrid`'s private `neighbors_into` using public API.
#[inline]
fn grid_neighbors_into(grid: &NavGrid, x: i32, y: i32, buf: &mut [(i32, i32, f32); 8]) -> usize {
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
    fn incremental_finds_path() {
        let grid = NavGrid::new(10, 10, 1.0);
        let mut query =
            IncrementalGridPath::new(&grid, GridPos::new(0, 0), GridPos::new(9, 9)).unwrap();

        // Step with large budget — should find immediately
        let status = query.step(&grid, 1000);
        assert_eq!(status, IncrementalStatus::Found);

        let path = query.path().unwrap();
        assert_eq!(*path.first().unwrap(), GridPos::new(0, 0));
        assert_eq!(*path.last().unwrap(), GridPos::new(9, 9));
    }

    #[test]
    fn incremental_multi_step() {
        let grid = NavGrid::new(20, 20, 1.0);
        let mut query =
            IncrementalGridPath::new(&grid, GridPos::new(0, 0), GridPos::new(19, 19)).unwrap();

        // Step with small budget
        let mut steps = 0;
        loop {
            let status = query.step(&grid, 10);
            steps += 1;
            if status != IncrementalStatus::InProgress {
                break;
            }
        }

        assert!(steps > 1); // Should take multiple steps
        assert_eq!(query.status(), IncrementalStatus::Found);
        assert!(query.path().is_some());
    }

    #[test]
    fn incremental_no_path() {
        let mut grid = NavGrid::new(10, 10, 1.0);
        // Wall off the right side
        for y in 0..10 {
            grid.set_walkable(5, y, false);
        }

        let mut query =
            IncrementalGridPath::new(&grid, GridPos::new(0, 0), GridPos::new(9, 9)).unwrap();
        let status = query.step(&grid, 10000);
        assert_eq!(status, IncrementalStatus::NotFound);
        assert!(query.path().is_none());
    }

    #[test]
    fn incremental_unwalkable_start() {
        let mut grid = NavGrid::new(10, 10, 1.0);
        grid.set_walkable(0, 0, false);
        assert!(IncrementalGridPath::new(&grid, GridPos::new(0, 0), GridPos::new(9, 9)).is_none());
    }

    #[test]
    fn incremental_unwalkable_goal() {
        let mut grid = NavGrid::new(10, 10, 1.0);
        grid.set_walkable(9, 9, false);
        assert!(IncrementalGridPath::new(&grid, GridPos::new(0, 0), GridPos::new(9, 9)).is_none());
    }

    #[test]
    fn incremental_same_start_goal() {
        let grid = NavGrid::new(10, 10, 1.0);
        let mut query =
            IncrementalGridPath::new(&grid, GridPos::new(5, 5), GridPos::new(5, 5)).unwrap();
        let status = query.step(&grid, 1);
        assert_eq!(status, IncrementalStatus::Found);
        let path = query.path().unwrap();
        assert_eq!(path.len(), 1);
    }

    #[test]
    fn incremental_to_path_result_found() {
        let grid = NavGrid::new(10, 10, 1.0);
        let mut query =
            IncrementalGridPath::new(&grid, GridPos::new(0, 0), GridPos::new(9, 9)).unwrap();
        query.step(&grid, 10000);

        let result = query.to_path_result(&grid);
        assert_eq!(result.status, PathStatus::Found);
        assert!(!result.waypoints.is_empty());
        assert!(result.length > 0.0);
    }

    #[test]
    fn incremental_to_path_result_pending() {
        let grid = NavGrid::new(50, 50, 1.0);
        let mut query =
            IncrementalGridPath::new(&grid, GridPos::new(0, 0), GridPos::new(49, 49)).unwrap();
        query.step(&grid, 1); // Just 1 iteration — still in progress

        let result = query.to_path_result(&grid);
        assert_eq!(result.status, PathStatus::Pending);
    }

    #[test]
    fn incremental_matches_regular_path() {
        let mut grid = NavGrid::new(15, 15, 1.0);
        // Add some obstacles
        grid.set_walkable(5, 3, false);
        grid.set_walkable(5, 4, false);
        grid.set_walkable(5, 5, false);

        let start = GridPos::new(0, 0);
        let goal = GridPos::new(14, 14);

        let regular = grid.find_path(start, goal).unwrap();

        let mut query = IncrementalGridPath::new(&grid, start, goal).unwrap();
        query.step(&grid, 100_000);
        let incremental = query.path().unwrap();

        // Both should find the same path
        assert_eq!(regular, incremental);
    }

    #[test]
    fn incremental_nodes_expanded() {
        let grid = NavGrid::new(10, 10, 1.0);
        let mut query =
            IncrementalGridPath::new(&grid, GridPos::new(0, 0), GridPos::new(9, 9)).unwrap();
        assert_eq!(query.nodes_expanded(), 0);
        query.step(&grid, 5);
        assert!(query.nodes_expanded() > 0);
    }

    #[test]
    fn incremental_status_serde_roundtrip() {
        let status = IncrementalStatus::InProgress;
        let json = serde_json::to_string(&status).unwrap();
        let deserialized: IncrementalStatus = serde_json::from_str(&json).unwrap();
        assert_eq!(deserialized, status);
    }
}
