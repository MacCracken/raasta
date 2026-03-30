//! D\* Lite — incremental replanning for dynamic environments.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::grid::{GridPos, NavGrid};

#[cfg(feature = "logging")]
use tracing::instrument;

/// D\* Lite incremental pathfinder.
///
/// Searches backwards from goal to start. When the environment changes
/// (cells blocked/unblocked, costs changed), call [`update_cell()`](Self::update_cell)
/// to notify, then [`compute_path()`](Self::compute_path) to incrementally
/// update only the affected nodes.
///
/// More efficient than re-running A\* from scratch when few cells change.
pub struct DStarLite {
    start_idx: usize,
    goal_idx: usize,
    grid_width: usize,
    grid_height: usize,
    allow_diagonal: bool,
    /// g values: cost from node to goal.
    g: Vec<f32>,
    /// rhs values: one-step lookahead.
    rhs: Vec<f32>,
    /// Priority queue.
    open: BinaryHeap<DStarNode>,
    /// Key modifier (increases when start position moves).
    km: f32,
    /// Current start position (can change as agent moves).
    start: GridPos,
    /// Goal position.
    goal: GridPos,
}

#[derive(Clone, Copy)]
struct DStarNode {
    idx: usize,
    key: (f32, f32), // (k1, k2) priority
}

impl PartialEq for DStarNode {
    fn eq(&self, other: &Self) -> bool {
        self.key == other.key
    }
}

impl Eq for DStarNode {}

impl PartialOrd for DStarNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for DStarNode {
    fn cmp(&self, other: &Self) -> Ordering {
        // Min-heap: reverse ordering. Compare k1 first, then k2.
        match other.key.0.partial_cmp(&self.key.0) {
            Some(Ordering::Equal) | None => other
                .key
                .1
                .partial_cmp(&self.key.1)
                .unwrap_or(Ordering::Equal),
            Some(ord) => ord,
        }
    }
}

impl DStarLite {
    /// Initialize D\* Lite for a grid from `start` to `goal`.
    ///
    /// Returns `None` if start or goal is outside the grid.
    #[cfg_attr(feature = "logging", instrument(skip(grid)))]
    #[must_use]
    pub fn new(grid: &NavGrid, start: GridPos, goal: GridPos) -> Option<Self> {
        let w = grid.width();
        let h = grid.height();
        let len = w * h;

        let start_idx = index(start.x, start.y, w, h)?;
        let goal_idx = index(goal.x, goal.y, w, h)?;

        let g = vec![f32::INFINITY; len];
        let rhs = vec![f32::INFINITY; len];

        let mut inst = Self {
            start_idx,
            goal_idx,
            grid_width: w,
            grid_height: h,
            allow_diagonal: grid.allow_diagonal,
            g,
            rhs,
            open: BinaryHeap::new(),
            km: 0.0,
            start,
            goal,
        };
        inst.rhs[goal_idx] = 0.0;

        let key = calculate_key(goal_idx, &inst.g, &inst.rhs, start_idx, 0.0, w);
        inst.open.push(DStarNode { idx: goal_idx, key });

        Some(inst)
    }

    /// Compute (or recompute) the shortest path.
    ///
    /// Processes all inconsistent cells so the g-field is fully consistent,
    /// enabling reliable full-path extraction via [`path()`](Self::path).
    ///
    /// Call after [`new()`](Self::new) for initial computation, or after
    /// [`update_cell()`](Self::update_cell) for incremental replanning.
    #[cfg_attr(feature = "logging", instrument(skip(self, grid)))]
    pub fn compute_path(&mut self, grid: &NavGrid) {
        self.process_queue(grid, false);
    }

    /// Compute the shortest path with early termination.
    ///
    /// Stops as soon as the start node is locally consistent, which is
    /// sufficient for single-step navigation via [`next_step()`](Self::next_step).
    /// For full path extraction, use [`compute_path()`](Self::compute_path)
    /// instead.
    #[cfg_attr(feature = "logging", instrument(skip(self, grid)))]
    pub fn compute_path_lazy(&mut self, grid: &NavGrid) {
        self.process_queue(grid, true);
    }

    /// Return the next cell to move to from the current start.
    ///
    /// This is the most efficient way to use D\* Lite: call
    /// [`compute_path_lazy()`](Self::compute_path_lazy), take one step with
    /// this method, then call [`set_start()`](Self::set_start) and repeat.
    ///
    /// Returns `None` if no path exists.
    #[must_use]
    pub fn next_step(&self, grid: &NavGrid) -> Option<GridPos> {
        if self.g[self.start_idx] == f32::INFINITY {
            return None;
        }
        if self.start_idx == self.goal_idx {
            return Some(self.goal);
        }

        let sx = (self.start_idx % self.grid_width) as i32;
        let sy = (self.start_idx / self.grid_width) as i32;
        let mut neighbors_buf = [(0i32, 0i32, 0.0f32); 8];
        let count = grid_neighbors(grid, sx, sy, self.allow_diagonal, &mut neighbors_buf);

        let mut best_idx = None;
        let mut best_cost = f32::INFINITY;
        for &(nx, ny, move_cost) in &neighbors_buf[..count] {
            if let Some(n_idx) = index(nx, ny, self.grid_width, self.grid_height) {
                let cost = move_cost * grid.cost(nx, ny) + self.g[n_idx];
                if cost < best_cost {
                    best_cost = cost;
                    best_idx = Some(n_idx);
                }
            }
        }

        best_idx.map(|idx| {
            GridPos::new(
                (idx % self.grid_width) as i32,
                (idx / self.grid_width) as i32,
            )
        })
    }

    fn process_queue(&mut self, grid: &NavGrid, early_terminate: bool) {
        let mut neighbors_buf = [(0i32, 0i32, 0.0f32); 8];
        let mut iterations = 0u32;
        let max_iterations = (self.grid_width * self.grid_height * 4) as u32;

        while let Some(top) = self.open.peek() {
            if early_terminate {
                let top_key = top.key;
                let start_key = calculate_key(
                    self.start_idx,
                    &self.g,
                    &self.rhs,
                    self.start_idx,
                    self.km,
                    self.grid_width,
                );
                let g_neq_rhs = self.rhs[self.start_idx] != self.g[self.start_idx];
                if !(key_less(top_key, start_key) || g_neq_rhs) {
                    break;
                }
            }

            iterations += 1;
            if iterations > max_iterations {
                break;
            }

            let u = self.open.pop().expect("peek succeeded so pop must too");
            let u_idx = u.idx;
            let ux = (u_idx % self.grid_width) as i32;
            let uy = (u_idx / self.grid_width) as i32;

            let new_key = calculate_key(
                u_idx,
                &self.g,
                &self.rhs,
                self.start_idx,
                self.km,
                self.grid_width,
            );

            if key_less(u.key, new_key) {
                // Key is outdated: reinsert with updated key.
                self.open.push(DStarNode {
                    idx: u_idx,
                    key: new_key,
                });
            } else if (self.g[u_idx] - self.rhs[u_idx]).abs() <= f32::EPSILON {
                // Locally consistent — stale queue entry, skip.
                continue;
            } else if self.g[u_idx] > self.rhs[u_idx] {
                // Overconsistent.
                self.g[u_idx] = self.rhs[u_idx];
                let count = grid_neighbors(grid, ux, uy, self.allow_diagonal, &mut neighbors_buf);
                for &(nx, ny, _) in &neighbors_buf[..count] {
                    if let Some(n_idx) = index(nx, ny, self.grid_width, self.grid_height) {
                        self.update_vertex(grid, n_idx);
                    }
                }
            } else {
                // Underconsistent.
                self.g[u_idx] = f32::INFINITY;
                // Update u itself.
                self.update_vertex(grid, u_idx);
                let count = grid_neighbors(grid, ux, uy, self.allow_diagonal, &mut neighbors_buf);
                for &(nx, ny, _) in &neighbors_buf[..count] {
                    if let Some(n_idx) = index(nx, ny, self.grid_width, self.grid_height) {
                        self.update_vertex(grid, n_idx);
                    }
                }
            }
        }
    }

    /// Notify that a cell's cost or walkability has changed.
    ///
    /// Call this for each changed cell, then call [`compute_path()`](Self::compute_path)
    /// to replan.
    #[cfg_attr(feature = "logging", instrument(skip(self, grid)))]
    pub fn update_cell(&mut self, grid: &NavGrid, pos: GridPos) {
        if let Some(idx) = index(pos.x, pos.y, self.grid_width, self.grid_height) {
            self.update_vertex(grid, idx);
            // Also update all neighbors since their paths through this cell changed.
            let mut neighbors_buf = [(0i32, 0i32, 0.0f32); 8];
            let count = grid_neighbors(grid, pos.x, pos.y, self.allow_diagonal, &mut neighbors_buf);
            for &(nx, ny, _) in &neighbors_buf[..count] {
                if let Some(n_idx) = index(nx, ny, self.grid_width, self.grid_height) {
                    self.update_vertex(grid, n_idx);
                }
            }
        }
    }

    /// Update the start position (when the agent moves).
    ///
    /// Call [`compute_path()`](Self::compute_path) after to update the path.
    pub fn set_start(&mut self, new_start: GridPos) {
        if let Some(new_idx) = index(new_start.x, new_start.y, self.grid_width, self.grid_height) {
            // Increase km by the heuristic distance between old and new start.
            self.km += heuristic(self.start_idx, new_idx, self.grid_width);
            self.start = new_start;
            self.start_idx = new_idx;
        }
    }

    /// Extract the current path from start to goal.
    ///
    /// Returns `None` if no path exists.
    #[must_use]
    pub fn path(&self, grid: &NavGrid) -> Option<Vec<GridPos>> {
        if self.g[self.start_idx] == f32::INFINITY {
            return None;
        }

        let mut path = Vec::new();
        let mut current = self.start_idx;
        let mut visited = vec![false; self.g.len()];
        let max_steps = self.grid_width * self.grid_height;

        for _ in 0..max_steps {
            let cx = (current % self.grid_width) as i32;
            let cy = (current / self.grid_width) as i32;
            path.push(GridPos::new(cx, cy));

            if current == self.goal_idx {
                return Some(path);
            }

            visited[current] = true;

            // Follow the lowest g-cost neighbor.
            let mut best_idx = current;
            let mut best_cost = f32::INFINITY;
            let mut neighbors_buf = [(0i32, 0i32, 0.0f32); 8];
            let count = grid_neighbors(grid, cx, cy, self.allow_diagonal, &mut neighbors_buf);

            for &(nx, ny, move_cost) in &neighbors_buf[..count] {
                if let Some(n_idx) = index(nx, ny, self.grid_width, self.grid_height) {
                    if visited[n_idx] {
                        continue;
                    }
                    let cost = move_cost * grid.cost(nx, ny) + self.g[n_idx];
                    if cost < best_cost {
                        best_cost = cost;
                        best_idx = n_idx;
                    }
                }
            }

            if best_idx == current {
                return None; // Stuck.
            }
            current = best_idx;
        }

        None
    }

    /// Start position.
    #[must_use]
    pub fn start(&self) -> GridPos {
        self.start
    }

    /// Goal position.
    #[must_use]
    pub fn goal(&self) -> GridPos {
        self.goal
    }

    fn update_vertex(&mut self, grid: &NavGrid, idx: usize) {
        let x = (idx % self.grid_width) as i32;
        let y = (idx / self.grid_width) as i32;

        if idx != self.goal_idx {
            // rhs = min over successors s': c(u,s') + g(s').
            let mut neighbors_buf = [(0i32, 0i32, 0.0f32); 8];
            let count = grid_neighbors(grid, x, y, self.allow_diagonal, &mut neighbors_buf);

            if !grid.is_walkable(x, y) {
                self.rhs[idx] = f32::INFINITY;
            } else {
                let mut min_rhs = f32::INFINITY;
                for &(nx, ny, move_cost) in &neighbors_buf[..count] {
                    if let Some(n_idx) = index(nx, ny, self.grid_width, self.grid_height) {
                        let cost = move_cost * grid.cost(nx, ny) + self.g[n_idx];
                        if cost < min_rhs {
                            min_rhs = cost;
                        }
                    }
                }
                self.rhs[idx] = min_rhs;
            }
        }

        // Lazy deletion: stale entries in the heap are skipped via key comparison
        // in compute_path, so we simply push a new entry if g != rhs.
        if (self.g[idx] - self.rhs[idx]).abs() > f32::EPSILON {
            let key = calculate_key(
                idx,
                &self.g,
                &self.rhs,
                self.start_idx,
                self.km,
                self.grid_width,
            );
            self.open.push(DStarNode { idx, key });
        }
    }
}

/// Convert (x, y) to a flat index, returning `None` if out of bounds.
#[inline]
fn index(x: i32, y: i32, w: usize, h: usize) -> Option<usize> {
    if x >= 0 && y >= 0 && (x as usize) < w && (y as usize) < h {
        Some(y as usize * w + x as usize)
    } else {
        None
    }
}

/// Octile heuristic between two flat indices.
#[inline]
fn heuristic(a: usize, b: usize, width: usize) -> f32 {
    let ax = (a % width) as f32;
    let ay = (a / width) as f32;
    let bx = (b % width) as f32;
    let by = (b / width) as f32;
    let dx = (ax - bx).abs();
    let dy = (ay - by).abs();
    let (min, max) = if dx < dy { (dx, dy) } else { (dy, dx) };
    max + (std::f32::consts::SQRT_2 - 1.0) * min
}

/// Compare two keys lexicographically: (k1, k2) < (k1', k2').
#[inline]
fn key_less(a: (f32, f32), b: (f32, f32)) -> bool {
    a.0 < b.0 || (a.0 == b.0 && a.1 < b.1)
}

/// Compute the priority key for a node.
#[inline]
fn calculate_key(
    idx: usize,
    g: &[f32],
    rhs: &[f32],
    start_idx: usize,
    km: f32,
    width: usize,
) -> (f32, f32) {
    let min_g_rhs = g[idx].min(rhs[idx]);
    let h = heuristic(start_idx, idx, width);
    (min_g_rhs + h + km, min_g_rhs)
}

/// Compute walkable neighbors into a stack buffer, returning the count.
#[inline]
fn grid_neighbors(
    grid: &NavGrid,
    x: i32,
    y: i32,
    allow_diagonal: bool,
    buf: &mut [(i32, i32, f32); 8],
) -> usize {
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
    if allow_diagonal {
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
    fn dstar_basic_path() {
        let grid = NavGrid::new(10, 10, 1.0);
        let mut ds = DStarLite::new(&grid, GridPos::new(0, 0), GridPos::new(9, 9)).unwrap();
        ds.compute_path(&grid);
        let path = ds.path(&grid);
        assert!(path.is_some());
        let path = path.unwrap();
        assert_eq!(*path.first().unwrap(), GridPos::new(0, 0));
        assert_eq!(*path.last().unwrap(), GridPos::new(9, 9));
    }

    #[test]
    fn dstar_blocked_no_path() {
        let mut grid = NavGrid::new(10, 1, 1.0);
        grid.set_walkable(5, 0, false);
        let mut ds = DStarLite::new(&grid, GridPos::new(0, 0), GridPos::new(9, 0)).unwrap();
        ds.compute_path(&grid);
        assert!(ds.path(&grid).is_none());
    }

    #[test]
    fn dstar_replan_after_block() {
        let mut grid = NavGrid::new(10, 10, 1.0);
        let mut ds = DStarLite::new(&grid, GridPos::new(0, 0), GridPos::new(9, 9)).unwrap();
        ds.compute_path(&grid);
        assert!(ds.path(&grid).is_some());

        // Block a cell and replan.
        grid.set_walkable(5, 5, false);
        ds.update_cell(&grid, GridPos::new(5, 5));
        ds.compute_path(&grid);

        let path = ds.path(&grid);
        assert!(path.is_some());
        // Path should avoid (5,5).
        let path = path.unwrap();
        assert!(!path.contains(&GridPos::new(5, 5)));
    }

    #[test]
    fn dstar_replan_after_unblock() {
        let mut grid = NavGrid::new(10, 10, 1.0);
        // Wall.
        for y in 0..9 {
            grid.set_walkable(5, y, false);
        }
        let mut ds = DStarLite::new(&grid, GridPos::new(0, 0), GridPos::new(9, 0)).unwrap();
        ds.compute_path(&grid);
        let path1 = ds.path(&grid).unwrap();

        // Open a gap.
        grid.set_walkable(5, 0, true);
        ds.update_cell(&grid, GridPos::new(5, 0));
        ds.compute_path(&grid);
        let path2 = ds.path(&grid).unwrap();

        // New path should be shorter (goes through the gap).
        assert!(path2.len() <= path1.len());
    }

    #[test]
    fn dstar_same_start_goal() {
        let grid = NavGrid::new(10, 10, 1.0);
        let mut ds = DStarLite::new(&grid, GridPos::new(5, 5), GridPos::new(5, 5)).unwrap();
        ds.compute_path(&grid);
        let path = ds.path(&grid).unwrap();
        assert_eq!(path.len(), 1);
        assert_eq!(path[0], GridPos::new(5, 5));
    }

    #[test]
    fn dstar_set_start() {
        let grid = NavGrid::new(10, 10, 1.0);
        let mut ds = DStarLite::new(&grid, GridPos::new(0, 0), GridPos::new(9, 9)).unwrap();
        ds.compute_path(&grid);

        // Move start.
        ds.set_start(GridPos::new(2, 2));
        ds.compute_path(&grid);
        let path = ds.path(&grid).unwrap();
        assert_eq!(*path.first().unwrap(), GridPos::new(2, 2));
        assert_eq!(*path.last().unwrap(), GridPos::new(9, 9));
    }

    #[test]
    fn dstar_around_obstacle() {
        let mut grid = NavGrid::new(10, 10, 1.0);
        for y in 0..8 {
            grid.set_walkable(5, y, false);
        }
        let mut ds = DStarLite::new(&grid, GridPos::new(0, 0), GridPos::new(9, 0)).unwrap();
        ds.compute_path(&grid);
        let path = ds.path(&grid);
        assert!(path.is_some());
    }
}
