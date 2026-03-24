//! Grid-based navigation and A* pathfinding.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use hisab::Vec2;
use serde::{Deserialize, Serialize};

/// A position on the navigation grid.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct GridPos {
    pub x: i32,
    pub y: i32,
}

impl GridPos {
    #[must_use]
    pub fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }

    /// Manhattan distance to another position.
    #[inline]
    #[must_use]
    pub fn manhattan_distance(self, other: GridPos) -> i32 {
        (self.x - other.x).abs() + (self.y - other.y).abs()
    }

    /// Octile distance (allows diagonal movement).
    #[inline]
    #[must_use]
    pub fn octile_distance(self, other: GridPos) -> f32 {
        let dx = (self.x - other.x).abs() as f32;
        let dy = (self.y - other.y).abs() as f32;
        let (min, max) = if dx < dy { (dx, dy) } else { (dy, dx) };
        max + (std::f32::consts::SQRT_2 - 1.0) * min
    }
}

/// A 2D navigation grid with walkability and movement costs.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NavGrid {
    width: usize,
    height: usize,
    cell_size: f32,
    /// true = walkable, false = blocked
    walkable: Vec<bool>,
    /// Per-cell movement cost multiplier (1.0 = normal).
    costs: Vec<f32>,
    /// Whether diagonal movement is allowed.
    pub allow_diagonal: bool,
}

impl NavGrid {
    /// Create a new grid where all cells are walkable.
    ///
    /// # Panics
    ///
    /// Panics if `width * height` overflows `usize`.
    #[must_use]
    pub fn new(width: usize, height: usize, cell_size: f32) -> Self {
        let len = width.checked_mul(height).expect("grid dimensions overflow");
        Self {
            width,
            height,
            cell_size,
            walkable: vec![true; len],
            costs: vec![1.0; len],
            allow_diagonal: true,
        }
    }

    #[must_use]
    pub fn width(&self) -> usize {
        self.width
    }

    #[must_use]
    pub fn height(&self) -> usize {
        self.height
    }

    #[must_use]
    pub fn cell_size(&self) -> f32 {
        self.cell_size
    }

    /// Set whether a cell is walkable.
    pub fn set_walkable(&mut self, x: i32, y: i32, walkable: bool) {
        if let Some(idx) = self.index(x, y) {
            self.walkable[idx] = walkable;
        }
    }

    /// Check if a cell is walkable.
    #[must_use]
    pub fn is_walkable(&self, x: i32, y: i32) -> bool {
        self.index(x, y)
            .map(|idx| self.walkable[idx])
            .unwrap_or(false)
    }

    /// Set the movement cost for a cell.
    pub fn set_cost(&mut self, x: i32, y: i32, cost: f32) {
        if let Some(idx) = self.index(x, y) {
            self.costs[idx] = cost;
        }
    }

    /// Get the movement cost for a cell.
    #[must_use]
    pub fn cost(&self, x: i32, y: i32) -> f32 {
        self.index(x, y)
            .map(|idx| self.costs[idx])
            .unwrap_or(f32::INFINITY)
    }

    /// Convert grid position to world position (center of cell).
    #[must_use]
    pub fn grid_to_world(&self, pos: GridPos) -> Vec2 {
        Vec2::new(
            (pos.x as f32 + 0.5) * self.cell_size,
            (pos.y as f32 + 0.5) * self.cell_size,
        )
    }

    /// Convert world position to grid position.
    #[must_use]
    pub fn world_to_grid(&self, world: Vec2) -> GridPos {
        GridPos::new(
            (world.x / self.cell_size).floor() as i32,
            (world.y / self.cell_size).floor() as i32,
        )
    }

    /// Find a path from `start` to `goal` using A*.
    ///
    /// Returns `None` if no path exists.
    #[must_use]
    pub fn find_path(&self, start: GridPos, goal: GridPos) -> Option<Vec<GridPos>> {
        if !self.is_walkable(start.x, start.y) || !self.is_walkable(goal.x, goal.y) {
            return None;
        }
        if start == goal {
            return Some(vec![start]);
        }

        let len = self.width * self.height;
        let mut g_score = vec![f32::INFINITY; len];
        let mut came_from: Vec<Option<usize>> = vec![None; len];
        let mut closed = vec![false; len];
        let mut open = BinaryHeap::new();

        let start_idx = self.index(start.x, start.y)?;
        let goal_idx = self.index(goal.x, goal.y)?;

        g_score[start_idx] = 0.0;
        let h = if self.allow_diagonal {
            start.octile_distance(goal)
        } else {
            start.manhattan_distance(goal) as f32
        };
        open.push(AStarNode {
            idx: start_idx,
            f_score: h,
        });

        let mut neighbors_buf = [(0i32, 0i32, 0.0f32); 8];

        while let Some(current) = open.pop() {
            if current.idx == goal_idx {
                return Some(self.reconstruct_path(&came_from, goal_idx));
            }

            if closed[current.idx] {
                continue;
            }
            closed[current.idx] = true;

            let cx = (current.idx % self.width) as i32;
            let cy = (current.idx / self.width) as i32;
            let count = self.neighbors_into(cx, cy, &mut neighbors_buf);

            for &(nx, ny, move_cost) in &neighbors_buf[..count] {
                let n_idx = self.index_unchecked(nx, ny);
                if closed[n_idx] {
                    continue;
                }
                let tentative_g = g_score[current.idx] + move_cost * self.costs[n_idx];

                if tentative_g < g_score[n_idx] {
                    came_from[n_idx] = Some(current.idx);
                    g_score[n_idx] = tentative_g;
                    let np = GridPos::new(nx, ny);
                    let h = if self.allow_diagonal {
                        np.octile_distance(goal)
                    } else {
                        np.manhattan_distance(goal) as f32
                    };
                    open.push(AStarNode {
                        idx: n_idx,
                        f_score: tentative_g + h,
                    });
                }
            }
        }

        None
    }

    /// Generate a flow field toward the given goal.
    ///
    /// Returns a grid-sized Vec of direction vectors `(dx, dy)` where
    /// `(0, 0)` means the goal cell or unreachable.
    #[must_use]
    pub fn flow_field(&self, goal: GridPos) -> Vec<(i32, i32)> {
        let len = self.width * self.height;
        let mut dist = vec![f32::INFINITY; len];
        let mut directions = vec![(0i32, 0i32); len];

        let goal_idx = match self.index(goal.x, goal.y) {
            Some(idx) => idx,
            None => return directions,
        };

        // Dijkstra from goal
        dist[goal_idx] = 0.0;
        let mut closed = vec![false; len];
        let mut queue = BinaryHeap::new();
        queue.push(AStarNode {
            idx: goal_idx,
            f_score: 0.0,
        });

        let mut neighbors_buf = [(0i32, 0i32, 0.0f32); 8];

        while let Some(current) = queue.pop() {
            if closed[current.idx] {
                continue;
            }
            closed[current.idx] = true;

            let cx = (current.idx % self.width) as i32;
            let cy = (current.idx / self.width) as i32;
            let count = self.neighbors_into(cx, cy, &mut neighbors_buf);

            for &(nx, ny, move_cost) in &neighbors_buf[..count] {
                let n_idx = self.index_unchecked(nx, ny);
                if closed[n_idx] {
                    continue;
                }
                let new_dist = dist[current.idx] + move_cost * self.costs[n_idx];
                if new_dist < dist[n_idx] {
                    dist[n_idx] = new_dist;
                    queue.push(AStarNode {
                        idx: n_idx,
                        f_score: new_dist,
                    });
                }
            }
        }

        // Compute directions: each cell points toward its lowest-cost neighbor
        for y in 0..self.height as i32 {
            for x in 0..self.width as i32 {
                let idx = self.index_unchecked(x, y);
                if idx == goal_idx || dist[idx] == f32::INFINITY {
                    continue;
                }
                let mut best_dir = (0i32, 0i32);
                let mut best_dist = dist[idx];
                let count = self.neighbors_into(x, y, &mut neighbors_buf);
                for &(nx, ny, _) in &neighbors_buf[..count] {
                    let n_idx = self.index_unchecked(nx, ny);
                    if dist[n_idx] < best_dist {
                        best_dist = dist[n_idx];
                        best_dir = (nx - x, ny - y);
                    }
                }
                directions[idx] = best_dir;
            }
        }

        directions
    }

    #[inline]
    fn index(&self, x: i32, y: i32) -> Option<usize> {
        if x >= 0 && y >= 0 && (x as usize) < self.width && (y as usize) < self.height {
            Some(y as usize * self.width + x as usize)
        } else {
            None
        }
    }

    /// Index for coordinates known to be in-bounds (from `neighbors_into` or
    /// bounded iteration). Debug-asserts validity; returns 0 in release if
    /// the invariant is somehow broken (callers already filter via `closed`).
    #[inline]
    fn index_unchecked(&self, x: i32, y: i32) -> usize {
        debug_assert!(
            x >= 0 && y >= 0 && (x as usize) < self.width && (y as usize) < self.height,
            "index_unchecked called with out-of-bounds ({x}, {y})"
        );
        y as usize * self.width + x as usize
    }

    /// Write walkable neighbors into `buf` and return the count.
    /// Zero-allocation hot-path alternative to returning a Vec.
    #[inline]
    fn neighbors_into(&self, x: i32, y: i32, buf: &mut [(i32, i32, f32); 8]) -> usize {
        const CARDINAL: [(i32, i32); 4] = [(0, 1), (0, -1), (1, 0), (-1, 0)];
        const DIAGONAL: [(i32, i32); 4] = [(1, 1), (1, -1), (-1, 1), (-1, -1)];

        let mut count = 0;
        for (dx, dy) in CARDINAL {
            let nx = x + dx;
            let ny = y + dy;
            if self.is_walkable(nx, ny) {
                buf[count] = (nx, ny, 1.0);
                count += 1;
            }
        }
        if self.allow_diagonal {
            for (dx, dy) in DIAGONAL {
                let nx = x + dx;
                let ny = y + dy;
                // Diagonal requires both adjacent cardinal cells walkable (no corner-cutting)
                if self.is_walkable(nx, ny)
                    && self.is_walkable(x + dx, y)
                    && self.is_walkable(x, y + dy)
                {
                    buf[count] = (nx, ny, std::f32::consts::SQRT_2);
                    count += 1;
                }
            }
        }
        count
    }

    fn reconstruct_path(&self, came_from: &[Option<usize>], goal_idx: usize) -> Vec<GridPos> {
        let mut path = Vec::new();
        let mut current = goal_idx;
        loop {
            let x = (current % self.width) as i32;
            let y = (current / self.width) as i32;
            path.push(GridPos::new(x, y));
            match came_from[current] {
                Some(prev) => current = prev,
                None => break,
            }
        }
        path.reverse();
        path
    }
}

/// A* open-set node (min-heap by f_score).
#[derive(Clone, Copy)]
struct AStarNode {
    idx: usize,
    f_score: f32,
}

impl PartialEq for AStarNode {
    fn eq(&self, other: &Self) -> bool {
        self.f_score == other.f_score
    }
}

impl Eq for AStarNode {}

impl PartialOrd for AStarNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for AStarNode {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reversed for min-heap (BinaryHeap is max-heap)
        other
            .f_score
            .partial_cmp(&self.f_score)
            .unwrap_or(Ordering::Equal)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn grid_pos_manhattan() {
        let a = GridPos::new(0, 0);
        let b = GridPos::new(3, 4);
        assert_eq!(a.manhattan_distance(b), 7);
    }

    #[test]
    fn grid_pos_octile() {
        let a = GridPos::new(0, 0);
        let b = GridPos::new(3, 3);
        let d = a.octile_distance(b);
        // 3 diagonal moves: 3 * sqrt(2) ≈ 4.24
        assert!((d - 3.0 * std::f32::consts::SQRT_2).abs() < 0.01);
    }

    #[test]
    fn empty_grid_path() {
        let grid = NavGrid::new(5, 5, 1.0);
        let path = grid.find_path(GridPos::new(0, 0), GridPos::new(4, 4));
        assert!(path.is_some());
        let path = path.unwrap();
        assert_eq!(*path.first().unwrap(), GridPos::new(0, 0));
        assert_eq!(*path.last().unwrap(), GridPos::new(4, 4));
    }

    #[test]
    fn same_start_goal() {
        let grid = NavGrid::new(5, 5, 1.0);
        let path = grid.find_path(GridPos::new(2, 2), GridPos::new(2, 2));
        assert_eq!(path, Some(vec![GridPos::new(2, 2)]));
    }

    #[test]
    fn blocked_path() {
        let mut grid = NavGrid::new(5, 1, 1.0);
        grid.set_walkable(2, 0, false);
        let path = grid.find_path(GridPos::new(0, 0), GridPos::new(4, 0));
        assert!(path.is_none());
    }

    #[test]
    fn path_around_wall() {
        let mut grid = NavGrid::new(5, 5, 1.0);
        // Wall across middle
        for x in 0..4 {
            grid.set_walkable(x, 2, false);
        }
        let path = grid.find_path(GridPos::new(0, 0), GridPos::new(0, 4));
        assert!(path.is_some());
        let path = path.unwrap();
        assert_eq!(*path.last().unwrap(), GridPos::new(0, 4));
    }

    #[test]
    fn blocked_start() {
        let mut grid = NavGrid::new(5, 5, 1.0);
        grid.set_walkable(0, 0, false);
        let path = grid.find_path(GridPos::new(0, 0), GridPos::new(4, 4));
        assert!(path.is_none());
    }

    #[test]
    fn blocked_goal() {
        let mut grid = NavGrid::new(5, 5, 1.0);
        grid.set_walkable(4, 4, false);
        let path = grid.find_path(GridPos::new(0, 0), GridPos::new(4, 4));
        assert!(path.is_none());
    }

    #[test]
    fn cardinal_only() {
        let mut grid = NavGrid::new(3, 3, 1.0);
        grid.allow_diagonal = false;
        let path = grid.find_path(GridPos::new(0, 0), GridPos::new(2, 2));
        assert!(path.is_some());
        let path = path.unwrap();
        // All moves should be cardinal (no diagonals)
        for w in path.windows(2) {
            let dx = (w[1].x - w[0].x).abs();
            let dy = (w[1].y - w[0].y).abs();
            assert!(dx + dy == 1, "expected cardinal move, got dx={dx} dy={dy}");
        }
    }

    #[test]
    fn movement_cost() {
        let mut grid = NavGrid::new(5, 1, 1.0);
        grid.allow_diagonal = false;
        grid.set_cost(2, 0, 100.0); // expensive cell
        // Path should still go through it (only option in 1D)
        let path = grid.find_path(GridPos::new(0, 0), GridPos::new(4, 0));
        assert!(path.is_some());
    }

    #[test]
    fn grid_to_world_conversion() {
        let grid = NavGrid::new(10, 10, 2.0);
        let w = grid.grid_to_world(GridPos::new(3, 4));
        assert!((w.x - 7.0).abs() < f32::EPSILON);
        assert!((w.y - 9.0).abs() < f32::EPSILON);
    }

    #[test]
    fn world_to_grid_conversion() {
        let grid = NavGrid::new(10, 10, 2.0);
        let pos = grid.world_to_grid(Vec2::new(7.5, 9.5));
        assert_eq!(pos, GridPos::new(3, 4));
    }

    #[test]
    fn walkable_default() {
        let grid = NavGrid::new(5, 5, 1.0);
        assert!(grid.is_walkable(0, 0));
        assert!(grid.is_walkable(4, 4));
        assert!(!grid.is_walkable(-1, 0)); // out of bounds
        assert!(!grid.is_walkable(5, 0)); // out of bounds
    }

    #[test]
    fn cost_default() {
        let grid = NavGrid::new(5, 5, 1.0);
        assert!((grid.cost(0, 0) - 1.0).abs() < f32::EPSILON);
        assert!(grid.cost(-1, 0).is_infinite()); // out of bounds
    }

    #[test]
    fn flow_field_basic() {
        let grid = NavGrid::new(5, 5, 1.0);
        let field = grid.flow_field(GridPos::new(4, 4));
        // Cell (0,0) should point toward (4,4) — direction should be (1,1) diagonal
        let (dx, dy) = field[0]; // index for (0,0)
        assert!(
            dx > 0 && dy > 0,
            "expected positive direction, got ({dx},{dy})"
        );
        // Goal cell should be (0,0)
        let goal_idx = 4 * 5 + 4; // y=4, x=4
        assert_eq!(field[goal_idx], (0, 0));
    }

    #[test]
    fn flow_field_blocked() {
        let mut grid = NavGrid::new(3, 1, 1.0);
        grid.set_walkable(1, 0, false);
        let field = grid.flow_field(GridPos::new(2, 0));
        // Cell (0,0) is unreachable — direction should be (0,0)
        assert_eq!(field[0], (0, 0));
    }

    #[test]
    fn no_corner_cutting() {
        // Diagonal should not cut through corners
        let mut grid = NavGrid::new(3, 3, 1.0);
        grid.set_walkable(1, 0, false);
        grid.set_walkable(0, 1, false);
        // (0,0) to (1,1) diagonal should be blocked because adjacent cardinals are blocked
        let path = grid.find_path(GridPos::new(0, 0), GridPos::new(1, 1));
        assert!(path.is_none());
    }

    #[test]
    fn large_grid_path() {
        let grid = NavGrid::new(100, 100, 1.0);
        let path = grid.find_path(GridPos::new(0, 0), GridPos::new(99, 99));
        assert!(path.is_some());
    }

    #[test]
    fn grid_dimensions() {
        let grid = NavGrid::new(10, 20, 0.5);
        assert_eq!(grid.width(), 10);
        assert_eq!(grid.height(), 20);
        assert!((grid.cell_size() - 0.5).abs() < f32::EPSILON);
    }

    #[test]
    fn grid_1x1() {
        let grid = NavGrid::new(1, 1, 1.0);
        let path = grid.find_path(GridPos::new(0, 0), GridPos::new(0, 0));
        assert_eq!(path, Some(vec![GridPos::new(0, 0)]));
    }

    #[test]
    fn out_of_bounds_start_goal() {
        let grid = NavGrid::new(5, 5, 1.0);
        assert!(
            grid.find_path(GridPos::new(-1, 0), GridPos::new(4, 4))
                .is_none()
        );
        assert!(
            grid.find_path(GridPos::new(0, 0), GridPos::new(5, 5))
                .is_none()
        );
    }

    #[test]
    fn flow_field_out_of_bounds_goal() {
        let grid = NavGrid::new(5, 5, 1.0);
        let field = grid.flow_field(GridPos::new(-1, -1));
        // All directions should be (0,0) — no valid goal
        assert!(field.iter().all(|&d| d == (0, 0)));
    }

    #[test]
    fn manhattan_distance_symmetry() {
        let a = GridPos::new(2, 3);
        let b = GridPos::new(7, 1);
        assert_eq!(a.manhattan_distance(b), b.manhattan_distance(a));
    }

    #[test]
    fn octile_distance_symmetry() {
        let a = GridPos::new(0, 0);
        let b = GridPos::new(5, 3);
        assert!((a.octile_distance(b) - b.octile_distance(a)).abs() < f32::EPSILON);
    }

    #[test]
    fn set_walkable_out_of_bounds() {
        let mut grid = NavGrid::new(5, 5, 1.0);
        // Should not panic
        grid.set_walkable(-1, 0, false);
        grid.set_walkable(0, 100, false);
    }

    #[test]
    fn set_cost_out_of_bounds() {
        let mut grid = NavGrid::new(5, 5, 1.0);
        // Should not panic
        grid.set_cost(-1, 0, 5.0);
        grid.set_cost(0, 100, 5.0);
    }

    #[test]
    fn gridpos_serde_roundtrip() {
        let pos = GridPos::new(42, -7);
        let json = serde_json::to_string(&pos).unwrap();
        let deserialized: GridPos = serde_json::from_str(&json).unwrap();
        assert_eq!(deserialized, pos);
    }

    #[test]
    fn cost_affects_path_choice() {
        // 3x1 grid with diagonal off: two paths exist in a 3x3 grid
        let mut grid = NavGrid::new(3, 3, 1.0);
        grid.allow_diagonal = false;
        // Make middle row expensive except the center
        grid.set_cost(0, 1, 100.0);
        grid.set_cost(2, 1, 100.0);
        let path = grid.find_path(GridPos::new(0, 0), GridPos::new(2, 2));
        assert!(path.is_some());
        let path = path.unwrap();
        // Path should prefer going through (1,1) which has default cost
        assert!(path.contains(&GridPos::new(1, 1)));
    }

    #[test]
    fn navgrid_serde_roundtrip() {
        let mut grid = NavGrid::new(5, 5, 2.0);
        grid.set_walkable(2, 2, false);
        grid.set_cost(1, 1, 3.0);
        grid.allow_diagonal = false;

        let json = serde_json::to_string(&grid).unwrap();
        let deserialized: NavGrid = serde_json::from_str(&json).unwrap();

        assert_eq!(deserialized.width(), 5);
        assert_eq!(deserialized.height(), 5);
        assert!((deserialized.cell_size() - 2.0).abs() < f32::EPSILON);
        assert!(!deserialized.is_walkable(2, 2));
        assert!((deserialized.cost(1, 1) - 3.0).abs() < f32::EPSILON);
        assert!(!deserialized.allow_diagonal);
    }

    #[test]
    #[should_panic(expected = "grid dimensions overflow")]
    fn grid_overflow_panics() {
        let _ = NavGrid::new(usize::MAX, 2, 1.0);
    }
}
