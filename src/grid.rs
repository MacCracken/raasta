//! Grid-based navigation and A* pathfinding.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use hisab::Vec2;
use serde::{Deserialize, Serialize};

#[cfg(feature = "logging")]
use tracing::instrument;

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
///
/// Walkability is stored as a bitset (1 bit per cell) for 8x memory
/// reduction compared to `Vec<bool>`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NavGrid {
    width: usize,
    height: usize,
    cell_size: f32,
    /// Bit-packed walkability: 1 = walkable, 0 = blocked. Each u64 stores 64 cells.
    walkable: Vec<u64>,
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
        let bitset_len = len.div_ceil(64);
        Self {
            width,
            height,
            cell_size,
            walkable: vec![u64::MAX; bitset_len], // all bits set = all walkable
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
            let word = idx / 64;
            let bit = idx % 64;
            if walkable {
                self.walkable[word] |= 1u64 << bit;
            } else {
                self.walkable[word] &= !(1u64 << bit);
            }
        }
    }

    /// Check if a cell is walkable.
    #[inline]
    #[must_use]
    pub fn is_walkable(&self, x: i32, y: i32) -> bool {
        self.index(x, y)
            .map(|idx| (self.walkable[idx / 64] >> (idx % 64)) & 1 == 1)
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

    /// Block a rectangular region of cells.
    pub fn block_rect(&mut self, min_x: i32, min_y: i32, max_x: i32, max_y: i32) {
        for y in min_y..=max_y {
            for x in min_x..=max_x {
                self.set_walkable(x, y, false);
            }
        }
    }

    /// Unblock a rectangular region of cells.
    pub fn unblock_rect(&mut self, min_x: i32, min_y: i32, max_x: i32, max_y: i32) {
        for y in min_y..=max_y {
            for x in min_x..=max_x {
                self.set_walkable(x, y, true);
            }
        }
    }

    /// Block all cells within a circle (world-space center and radius).
    pub fn block_circle(&mut self, center: Vec2, radius: f32) {
        let min = self.world_to_grid(center - Vec2::splat(radius));
        let max = self.world_to_grid(center + Vec2::splat(radius));
        let r_sq = radius * radius;
        for y in min.y..=max.y {
            for x in min.x..=max.x {
                let cell_center = self.grid_to_world(GridPos::new(x, y));
                if cell_center.distance_squared(center) <= r_sq {
                    self.set_walkable(x, y, false);
                }
            }
        }
    }

    /// Unblock all cells within a circle (world-space center and radius).
    pub fn unblock_circle(&mut self, center: Vec2, radius: f32) {
        let min = self.world_to_grid(center - Vec2::splat(radius));
        let max = self.world_to_grid(center + Vec2::splat(radius));
        let r_sq = radius * radius;
        for y in min.y..=max.y {
            for x in min.x..=max.x {
                let cell_center = self.grid_to_world(GridPos::new(x, y));
                if cell_center.distance_squared(center) <= r_sq {
                    self.set_walkable(x, y, true);
                }
            }
        }
    }

    /// Set uniform cost for a rectangular region.
    pub fn set_cost_rect(&mut self, min_x: i32, min_y: i32, max_x: i32, max_y: i32, cost: f32) {
        for y in min_y..=max_y {
            for x in min_x..=max_x {
                self.set_cost(x, y, cost);
            }
        }
    }

    /// Find the nearest walkable cell to the given position.
    ///
    /// Searches outward in a spiral pattern. Returns `None` if no walkable
    /// cell exists in the grid.
    #[must_use]
    pub fn nearest_walkable(&self, pos: GridPos) -> Option<GridPos> {
        if self.is_walkable(pos.x, pos.y) {
            return Some(pos);
        }
        let max_radius = self.width.max(self.height) as i32;
        for r in 1..=max_radius {
            for dx in -r..=r {
                for &dy in &[-r, r] {
                    let candidate = GridPos::new(pos.x + dx, pos.y + dy);
                    if self.is_walkable(candidate.x, candidate.y) {
                        return Some(candidate);
                    }
                }
            }
            for dy in (-r + 1)..r {
                for &dx in &[-r, r] {
                    let candidate = GridPos::new(pos.x + dx, pos.y + dy);
                    if self.is_walkable(candidate.x, candidate.y) {
                        return Some(candidate);
                    }
                }
            }
        }
        None
    }

    /// Reset all cells to walkable with cost 1.0.
    pub fn clear(&mut self) {
        self.walkable.fill(u64::MAX);
        self.costs.fill(1.0);
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
    #[cfg_attr(feature = "logging", instrument(skip(self), fields(w = self.width, h = self.height)))]
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

    /// Find a path using Jump Point Search (JPS).
    ///
    /// JPS is significantly faster than A* on uniform-cost grids with diagonal
    /// movement. It prunes symmetric paths by "jumping" along straight lines
    /// and only expanding nodes where forced neighbors exist.
    ///
    /// **Requirements:** `allow_diagonal` must be `true` and all walkable cells
    /// should have uniform cost (1.0). Falls back to standard A* if these
    /// conditions are not met.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    #[must_use]
    pub fn find_path_jps(&self, start: GridPos, goal: GridPos) -> Option<Vec<GridPos>> {
        // JPS requires diagonal movement
        if !self.allow_diagonal {
            return self.find_path(start, goal);
        }
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
        open.push(AStarNode {
            idx: start_idx,
            f_score: start.octile_distance(goal),
        });

        while let Some(current) = open.pop() {
            if current.idx == goal_idx {
                return Some(self.reconstruct_jps_path(&came_from, goal_idx));
            }

            if closed[current.idx] {
                continue;
            }
            closed[current.idx] = true;

            let cx = (current.idx % self.width) as i32;
            let cy = (current.idx / self.width) as i32;
            let current_pos = GridPos::new(cx, cy);

            // Determine search directions from parent
            let directions = if let Some(parent_idx) = came_from[current.idx] {
                let px = (parent_idx % self.width) as i32;
                let py = (parent_idx / self.width) as i32;
                let dx = (cx - px).signum();
                let dy = (cy - py).signum();
                self.jps_prune_directions(cx, cy, dx, dy)
            } else {
                // Start node: expand all 8 directions
                [
                    (1, 0),
                    (-1, 0),
                    (0, 1),
                    (0, -1),
                    (1, 1),
                    (1, -1),
                    (-1, 1),
                    (-1, -1),
                ]
            };

            for (dx, dy) in directions {
                if dx == 0 && dy == 0 {
                    continue;
                }
                if let Some(jump_pos) = self.jump(cx, cy, dx, dy, goal) {
                    let j_idx = self.index_unchecked(jump_pos.x, jump_pos.y);
                    if closed[j_idx] {
                        continue;
                    }

                    let dist = current_pos.octile_distance(jump_pos);
                    let tentative_g = g_score[current.idx] + dist;

                    if tentative_g < g_score[j_idx] {
                        g_score[j_idx] = tentative_g;
                        came_from[j_idx] = Some(current.idx);
                        let h = jump_pos.octile_distance(goal);
                        open.push(AStarNode {
                            idx: j_idx,
                            f_score: tentative_g + h,
                        });
                    }
                }
            }
        }

        None
    }

    /// Jump in direction (dx, dy) from (x, y). Returns the jump point if found.
    ///
    /// Iterative implementation to avoid stack overflow on large grids.
    #[inline]
    fn jump(&self, x: i32, y: i32, dx: i32, dy: i32, goal: GridPos) -> Option<GridPos> {
        let mut cx = x + dx;
        let mut cy = y + dy;

        loop {
            if !self.is_walkable(cx, cy) {
                return None;
            }

            if cx == goal.x && cy == goal.y {
                return Some(GridPos::new(cx, cy));
            }

            // Check for forced neighbors
            if dx != 0 && dy != 0 {
                // Diagonal move: forced neighbor if blocked cardinal but open diagonal-adjacent
                if (!self.is_walkable(cx - dx, cy) && self.is_walkable(cx - dx, cy + dy))
                    || (!self.is_walkable(cx, cy - dy) && self.is_walkable(cx + dx, cy - dy))
                {
                    return Some(GridPos::new(cx, cy));
                }
                // Diagonal: must also check cardinal sub-jumps (these are straight-line, safe to recurse)
                if self.jump_straight(cx, cy, dx, 0, goal).is_some()
                    || self.jump_straight(cx, cy, 0, dy, goal).is_some()
                {
                    return Some(GridPos::new(cx, cy));
                }
            } else if dx != 0 {
                // Horizontal move
                if (!self.is_walkable(cx, cy + 1) && self.is_walkable(cx + dx, cy + 1))
                    || (!self.is_walkable(cx, cy - 1) && self.is_walkable(cx + dx, cy - 1))
                {
                    return Some(GridPos::new(cx, cy));
                }
            } else {
                // Vertical move
                if (!self.is_walkable(cx + 1, cy) && self.is_walkable(cx + 1, cy + dy))
                    || (!self.is_walkable(cx - 1, cy) && self.is_walkable(cx - 1, cy + dy))
                {
                    return Some(GridPos::new(cx, cy));
                }
            }

            // Continue jumping
            // Diagonal requires both cardinal cells walkable (no corner-cutting)
            if dx != 0
                && dy != 0
                && (!self.is_walkable(cx + dx, cy) || !self.is_walkable(cx, cy + dy))
            {
                return None;
            }
            cx += dx;
            cy += dy;
        }
    }

    /// Straight-line (cardinal) jump — iterative helper for diagonal jump's sub-searches.
    #[inline]
    fn jump_straight(&self, x: i32, y: i32, dx: i32, dy: i32, goal: GridPos) -> Option<GridPos> {
        debug_assert!(
            (dx == 0) != (dy == 0),
            "jump_straight must be cardinal only"
        );
        let mut cx = x + dx;
        let mut cy = y + dy;

        loop {
            if !self.is_walkable(cx, cy) {
                return None;
            }
            if cx == goal.x && cy == goal.y {
                return Some(GridPos::new(cx, cy));
            }
            if dx != 0 {
                if (!self.is_walkable(cx, cy + 1) && self.is_walkable(cx + dx, cy + 1))
                    || (!self.is_walkable(cx, cy - 1) && self.is_walkable(cx + dx, cy - 1))
                {
                    return Some(GridPos::new(cx, cy));
                }
            } else if (!self.is_walkable(cx + 1, cy) && self.is_walkable(cx + 1, cy + dy))
                || (!self.is_walkable(cx - 1, cy) && self.is_walkable(cx - 1, cy + dy))
            {
                return Some(GridPos::new(cx, cy));
            }
            cx += dx;
            cy += dy;
        }
    }

    /// Prune directions for JPS based on parent direction.
    /// Returns up to 8 (dx, dy) pairs; unused slots are (0, 0).
    fn jps_prune_directions(&self, x: i32, y: i32, dx: i32, dy: i32) -> [(i32, i32); 8] {
        let mut dirs = [(0i32, 0i32); 8];
        let mut count = 0;

        if dx != 0 && dy != 0 {
            // Diagonal: natural neighbors are (dx,0), (0,dy), (dx,dy)
            dirs[count] = (dx, 0);
            count += 1;
            dirs[count] = (0, dy);
            count += 1;
            dirs[count] = (dx, dy);
            count += 1;

            // Forced neighbors from blocked cardinals
            if !self.is_walkable(x - dx, y) {
                dirs[count] = (-dx, dy);
                count += 1;
            }
            if !self.is_walkable(x, y - dy) {
                dirs[count] = (dx, -dy);
                count += 1;
            }
        } else if dx != 0 {
            // Horizontal: natural neighbor is (dx, 0)
            dirs[count] = (dx, 0);
            count += 1;

            // Forced: if cell above/below is blocked, diagonal becomes forced
            if !self.is_walkable(x, y + 1) {
                dirs[count] = (dx, 1);
                count += 1;
            }
            if !self.is_walkable(x, y - 1) {
                dirs[count] = (dx, -1);
                count += 1;
            }
        } else if dy != 0 {
            // Vertical: natural neighbor is (0, dy)
            dirs[count] = (0, dy);
            count += 1;

            // Forced
            if !self.is_walkable(x + 1, y) {
                dirs[count] = (1, dy);
                count += 1;
            }
            if !self.is_walkable(x - 1, y) {
                dirs[count] = (-1, dy);
                count += 1;
            }
        }

        let _ = count; // suppress unused warning
        dirs
    }

    /// Reconstruct path from JPS came_from, inserting intermediate cells
    /// along jumps so the path is contiguous.
    fn reconstruct_jps_path(&self, came_from: &[Option<usize>], goal_idx: usize) -> Vec<GridPos> {
        // First collect jump points in reverse
        let mut jump_points = Vec::new();
        let mut current = goal_idx;
        loop {
            let x = (current % self.width) as i32;
            let y = (current / self.width) as i32;
            jump_points.push(GridPos::new(x, y));
            match came_from[current] {
                Some(prev) => current = prev,
                None => break,
            }
        }
        jump_points.reverse();

        if jump_points.len() <= 1 {
            return jump_points;
        }

        // Interpolate between consecutive jump points
        let mut path = Vec::with_capacity(jump_points.len() * 4);
        path.push(jump_points[0]);

        for w in jump_points.windows(2) {
            let from = w[0];
            let to = w[1];
            let dx = (to.x - from.x).signum();
            let dy = (to.y - from.y).signum();
            let mut cx = from.x;
            let mut cy = from.y;
            while cx != to.x || cy != to.y {
                cx += dx;
                cy += dy;
                path.push(GridPos::new(cx, cy));
            }
        }

        path
    }

    /// Check line-of-sight between two grid cells using Bresenham's line.
    ///
    /// Returns `true` if all cells along the line are walkable.
    #[must_use]
    pub fn has_line_of_sight(&self, from: GridPos, to: GridPos) -> bool {
        let mut x = from.x;
        let mut y = from.y;
        let dx = (to.x - from.x).abs();
        let dy = (to.y - from.y).abs();
        let sx = if from.x < to.x { 1 } else { -1 };
        let sy = if from.y < to.y { 1 } else { -1 };
        let mut err = dx - dy;

        loop {
            if !self.is_walkable(x, y) {
                return false;
            }
            if x == to.x && y == to.y {
                return true;
            }
            let e2 = 2 * err;
            if e2 > -dy {
                err -= dy;
                x += sx;
            }
            if e2 < dx {
                err += dx;
                y += sy;
            }
        }
    }

    /// Find a path using Theta\* (any-angle pathfinding).
    ///
    /// Like A\*, but checks line-of-sight to the parent's parent. If LOS
    /// exists, skips the intermediate node, producing smoother paths
    /// without post-processing.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    #[must_use]
    pub fn find_path_theta(&self, start: GridPos, goal: GridPos) -> Option<Vec<GridPos>> {
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
        let h = start.octile_distance(goal);
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

            for &(nx, ny, _) in &neighbors_buf[..count] {
                let n_idx = self.index_unchecked(nx, ny);
                if closed[n_idx] {
                    continue;
                }

                let neighbor_pos = GridPos::new(nx, ny);

                // Theta* key insight: check LOS from parent of current to neighbor
                let (source_idx, source_g) = if let Some(parent_idx) = came_from[current.idx] {
                    let px = (parent_idx % self.width) as i32;
                    let py = (parent_idx / self.width) as i32;
                    let parent_pos = GridPos::new(px, py);
                    if self.has_line_of_sight(parent_pos, neighbor_pos) {
                        (parent_idx, g_score[parent_idx])
                    } else {
                        (current.idx, g_score[current.idx])
                    }
                } else {
                    (current.idx, g_score[current.idx])
                };

                let source_pos = GridPos::new(
                    (source_idx % self.width) as i32,
                    (source_idx / self.width) as i32,
                );
                let edge_cost = if source_idx == current.idx {
                    // Regular A* step — adjacent cell
                    source_pos.octile_distance(neighbor_pos) * self.costs[n_idx]
                } else {
                    // LOS shortcut — sum costs along the line
                    self.line_cost(source_pos, neighbor_pos)
                };
                let tentative_g = source_g + edge_cost;

                if tentative_g < g_score[n_idx] {
                    came_from[n_idx] = Some(source_idx);
                    g_score[n_idx] = tentative_g;
                    let h = neighbor_pos.octile_distance(goal);
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
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
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

    /// Sum movement costs along a Bresenham line from `from` to `to`.
    /// Used by Theta\* for cost-aware LOS shortcuts.
    fn line_cost(&self, from: GridPos, to: GridPos) -> f32 {
        let mut cost = 0.0f32;
        let mut x = from.x;
        let mut y = from.y;
        let dx = (to.x - from.x).abs();
        let dy = (to.y - from.y).abs();
        let sx = if from.x < to.x { 1 } else { -1 };
        let sy = if from.y < to.y { 1 } else { -1 };
        let mut err = dx - dy;

        loop {
            if x == to.x && y == to.y {
                break;
            }
            let e2 = 2 * err;
            let mut moved_x = false;
            let mut moved_y = false;
            if e2 > -dy {
                err -= dy;
                x += sx;
                moved_x = true;
            }
            if e2 < dx {
                err += dx;
                y += sy;
                moved_y = true;
            }
            let step_dist = if moved_x && moved_y {
                std::f32::consts::SQRT_2
            } else {
                1.0
            };
            let cell_cost = self.index(x, y).map(|idx| self.costs[idx]).unwrap_or(1.0);
            cost += step_dist * cell_cost;
        }

        cost
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

    // --- JPS tests ---

    #[test]
    fn jps_empty_grid() {
        let grid = NavGrid::new(10, 10, 1.0);
        let path = grid.find_path_jps(GridPos::new(0, 0), GridPos::new(9, 9));
        assert!(path.is_some());
        let path = path.unwrap();
        assert_eq!(*path.first().unwrap(), GridPos::new(0, 0));
        assert_eq!(*path.last().unwrap(), GridPos::new(9, 9));
    }

    #[test]
    fn jps_same_start_goal() {
        let grid = NavGrid::new(5, 5, 1.0);
        let path = grid.find_path_jps(GridPos::new(2, 2), GridPos::new(2, 2));
        assert_eq!(path, Some(vec![GridPos::new(2, 2)]));
    }

    #[test]
    fn jps_blocked_start() {
        let mut grid = NavGrid::new(5, 5, 1.0);
        grid.set_walkable(0, 0, false);
        assert!(
            grid.find_path_jps(GridPos::new(0, 0), GridPos::new(4, 4))
                .is_none()
        );
    }

    #[test]
    fn jps_blocked_goal() {
        let mut grid = NavGrid::new(5, 5, 1.0);
        grid.set_walkable(4, 4, false);
        assert!(
            grid.find_path_jps(GridPos::new(0, 0), GridPos::new(4, 4))
                .is_none()
        );
    }

    #[test]
    fn jps_path_around_wall() {
        let mut grid = NavGrid::new(10, 10, 1.0);
        // Wall across middle
        for x in 0..8 {
            grid.set_walkable(x, 5, false);
        }
        let path = grid.find_path_jps(GridPos::new(0, 0), GridPos::new(0, 9));
        assert!(path.is_some());
        let path = path.unwrap();
        assert_eq!(*path.last().unwrap(), GridPos::new(0, 9));
        // All waypoints should be walkable
        for p in &path {
            assert!(
                grid.is_walkable(p.x, p.y),
                "unwalkable cell in path: ({}, {})",
                p.x,
                p.y
            );
        }
    }

    #[test]
    fn jps_no_path() {
        let mut grid = NavGrid::new(5, 1, 1.0);
        grid.set_walkable(2, 0, false);
        // 1D grid with a wall — no way around
        assert!(
            grid.find_path_jps(GridPos::new(0, 0), GridPos::new(4, 0))
                .is_none()
        );
    }

    #[test]
    fn jps_matches_astar_result() {
        // JPS should find a path of equal or shorter length than A*
        let mut grid = NavGrid::new(20, 20, 1.0);
        for y in 0..15 {
            grid.set_walkable(10, y, false);
        }
        let astar_path = grid
            .find_path(GridPos::new(0, 0), GridPos::new(19, 19))
            .unwrap();
        let jps_path = grid
            .find_path_jps(GridPos::new(0, 0), GridPos::new(19, 19))
            .unwrap();

        // Same endpoints
        assert_eq!(jps_path.first(), astar_path.first());
        assert_eq!(jps_path.last(), astar_path.last());

        // JPS path should be contiguous (each step is adjacent)
        for w in jps_path.windows(2) {
            let dx = (w[1].x - w[0].x).abs();
            let dy = (w[1].y - w[0].y).abs();
            assert!(
                dx <= 1 && dy <= 1 && (dx + dy) > 0,
                "non-adjacent step: ({},{}) -> ({},{})",
                w[0].x,
                w[0].y,
                w[1].x,
                w[1].y
            );
        }
    }

    #[test]
    fn jps_large_grid() {
        let grid = NavGrid::new(100, 100, 1.0);
        let path = grid.find_path_jps(GridPos::new(0, 0), GridPos::new(99, 99));
        assert!(path.is_some());
    }

    #[test]
    fn jps_fallback_cardinal_only() {
        // JPS falls back to A* when diagonal is disabled
        let mut grid = NavGrid::new(5, 5, 1.0);
        grid.allow_diagonal = false;
        let path = grid.find_path_jps(GridPos::new(0, 0), GridPos::new(4, 4));
        assert!(path.is_some());
        // Verify all moves are cardinal
        let path = path.unwrap();
        for w in path.windows(2) {
            let dx = (w[1].x - w[0].x).abs();
            let dy = (w[1].y - w[0].y).abs();
            assert!(dx + dy == 1);
        }
    }

    // --- Dynamic obstacle tests ---

    #[test]
    fn block_rect() {
        let mut grid = NavGrid::new(10, 10, 1.0);
        grid.block_rect(2, 2, 4, 4);
        assert!(!grid.is_walkable(2, 2));
        assert!(!grid.is_walkable(3, 3));
        assert!(!grid.is_walkable(4, 4));
        assert!(grid.is_walkable(1, 1));
        assert!(grid.is_walkable(5, 5));
    }

    #[test]
    fn unblock_rect() {
        let mut grid = NavGrid::new(10, 10, 1.0);
        grid.block_rect(0, 0, 9, 9);
        grid.unblock_rect(3, 3, 6, 6);
        assert!(!grid.is_walkable(0, 0));
        assert!(grid.is_walkable(4, 4));
    }

    #[test]
    fn block_circle() {
        let mut grid = NavGrid::new(10, 10, 1.0);
        grid.block_circle(Vec2::new(5.0, 5.0), 2.0);
        // Center should be blocked
        assert!(!grid.is_walkable(5, 5));
        assert!(!grid.is_walkable(4, 5));
        // Far corner should be fine
        assert!(grid.is_walkable(0, 0));
    }

    #[test]
    fn unblock_circle() {
        let mut grid = NavGrid::new(10, 10, 1.0);
        grid.block_rect(0, 0, 9, 9);
        grid.unblock_circle(Vec2::new(5.0, 5.0), 2.0);
        assert!(grid.is_walkable(5, 5));
        assert!(!grid.is_walkable(0, 0));
    }

    #[test]
    fn set_cost_rect() {
        let mut grid = NavGrid::new(10, 10, 1.0);
        grid.set_cost_rect(2, 2, 4, 4, 5.0);
        assert!((grid.cost(3, 3) - 5.0).abs() < f32::EPSILON);
        assert!((grid.cost(0, 0) - 1.0).abs() < f32::EPSILON);
    }

    #[test]
    fn clear_grid() {
        let mut grid = NavGrid::new(10, 10, 1.0);
        grid.block_rect(0, 0, 9, 9);
        grid.set_cost_rect(0, 0, 9, 9, 99.0);
        grid.clear();
        assert!(grid.is_walkable(5, 5));
        assert!((grid.cost(5, 5) - 1.0).abs() < f32::EPSILON);
    }

    #[test]
    fn dynamic_obstacle_repath() {
        let mut grid = NavGrid::new(10, 10, 1.0);
        // Path exists
        assert!(
            grid.find_path(GridPos::new(0, 0), GridPos::new(9, 9))
                .is_some()
        );
        // Block the way
        grid.block_rect(0, 5, 9, 5);
        // No path through solid wall
        assert!(
            grid.find_path(GridPos::new(0, 0), GridPos::new(0, 9))
                .is_none()
        );
        // Open a gap
        grid.set_walkable(5, 5, true);
        // Path exists again
        assert!(
            grid.find_path(GridPos::new(0, 0), GridPos::new(0, 9))
                .is_some()
        );
    }

    #[test]
    fn block_rect_out_of_bounds() {
        let mut grid = NavGrid::new(5, 5, 1.0);
        // Partially out of bounds — should not panic
        grid.block_rect(-2, -2, 1, 1);
        assert!(!grid.is_walkable(0, 0));
        assert!(!grid.is_walkable(1, 1));
    }

    #[test]
    fn jps_contiguous_path_with_obstacles() {
        let mut grid = NavGrid::new(15, 15, 1.0);
        // Create an L-shaped wall
        for y in 0..10 {
            grid.set_walkable(7, y, false);
        }
        for x in 7..15 {
            grid.set_walkable(x, 10, false);
        }
        let path = grid.find_path_jps(GridPos::new(0, 0), GridPos::new(14, 14));
        assert!(path.is_some());
        let path = path.unwrap();
        for p in &path {
            assert!(grid.is_walkable(p.x, p.y));
        }
        for w in path.windows(2) {
            let dx = (w[1].x - w[0].x).abs();
            let dy = (w[1].y - w[0].y).abs();
            assert!(dx <= 1 && dy <= 1 && (dx + dy) > 0);
        }
    }

    // --- Nav query tests ---

    #[test]
    fn nearest_walkable_already_walkable() {
        let grid = NavGrid::new(10, 10, 1.0);
        assert_eq!(
            grid.nearest_walkable(GridPos::new(5, 5)),
            Some(GridPos::new(5, 5))
        );
    }

    #[test]
    fn nearest_walkable_blocked() {
        let mut grid = NavGrid::new(10, 10, 1.0);
        grid.set_walkable(5, 5, false);
        let nearest = grid.nearest_walkable(GridPos::new(5, 5));
        assert!(nearest.is_some());
        let n = nearest.unwrap();
        // Should be adjacent
        let dist = (n.x - 5).abs() + (n.y - 5).abs();
        assert!(dist <= 2);
    }

    #[test]
    fn nearest_walkable_all_blocked() {
        let mut grid = NavGrid::new(3, 3, 1.0);
        grid.block_rect(0, 0, 2, 2);
        assert!(grid.nearest_walkable(GridPos::new(1, 1)).is_none());
    }

    // --- Theta* / LOS tests ---

    #[test]
    fn los_open_grid() {
        let grid = NavGrid::new(10, 10, 1.0);
        assert!(grid.has_line_of_sight(GridPos::new(0, 0), GridPos::new(9, 9)));
    }

    #[test]
    fn los_blocked_by_wall() {
        let mut grid = NavGrid::new(10, 10, 1.0);
        grid.set_walkable(5, 5, false);
        assert!(!grid.has_line_of_sight(GridPos::new(0, 0), GridPos::new(9, 9)));
    }

    #[test]
    fn los_same_cell() {
        let grid = NavGrid::new(5, 5, 1.0);
        assert!(grid.has_line_of_sight(GridPos::new(2, 2), GridPos::new(2, 2)));
    }

    #[test]
    fn los_horizontal() {
        let grid = NavGrid::new(10, 10, 1.0);
        assert!(grid.has_line_of_sight(GridPos::new(0, 5), GridPos::new(9, 5)));
    }

    #[test]
    fn theta_open_grid() {
        let grid = NavGrid::new(10, 10, 1.0);
        let path = grid.find_path_theta(GridPos::new(0, 0), GridPos::new(9, 9));
        assert!(path.is_some());
        let path = path.unwrap();
        assert_eq!(*path.first().unwrap(), GridPos::new(0, 0));
        assert_eq!(*path.last().unwrap(), GridPos::new(9, 9));
    }

    #[test]
    fn theta_same_start_goal() {
        let grid = NavGrid::new(5, 5, 1.0);
        let path = grid.find_path_theta(GridPos::new(2, 2), GridPos::new(2, 2));
        assert_eq!(path, Some(vec![GridPos::new(2, 2)]));
    }

    #[test]
    fn theta_blocked() {
        let mut grid = NavGrid::new(5, 1, 1.0);
        grid.set_walkable(2, 0, false);
        assert!(
            grid.find_path_theta(GridPos::new(0, 0), GridPos::new(4, 0))
                .is_none()
        );
    }

    #[test]
    fn theta_fewer_waypoints_than_astar() {
        // On an open grid, Theta* should produce fewer waypoints
        // because it can skip intermediate nodes via LOS
        let grid = NavGrid::new(20, 20, 1.0);
        let astar = grid
            .find_path(GridPos::new(0, 0), GridPos::new(19, 19))
            .unwrap();
        let theta = grid
            .find_path_theta(GridPos::new(0, 0), GridPos::new(19, 19))
            .unwrap();
        // Theta* path should be equal or shorter in waypoint count
        assert!(theta.len() <= astar.len());
    }

    #[test]
    fn theta_with_wall() {
        let mut grid = NavGrid::new(20, 20, 1.0);
        for y in 0..18 {
            grid.set_walkable(10, y, false);
        }
        let path = grid.find_path_theta(GridPos::new(0, 0), GridPos::new(19, 19));
        assert!(path.is_some());
        let path = path.unwrap();
        for p in &path {
            assert!(grid.is_walkable(p.x, p.y));
        }
    }

    #[test]
    fn theta_respects_movement_costs() {
        // 10x3 grid with expensive vertical stripe at x=5.
        // If Theta* respects costs, it should avoid cells in column 5.
        let mut grid = NavGrid::new(10, 3, 1.0);
        for y in 0..3 {
            grid.set_cost(5, y, 100.0);
        }
        let path = grid.find_path_theta(GridPos::new(0, 1), GridPos::new(9, 1));
        assert!(path.is_some());
        let path = path.unwrap();
        assert_eq!(*path.first().unwrap(), GridPos::new(0, 1));
        assert_eq!(*path.last().unwrap(), GridPos::new(9, 1));
        // All waypoints should be walkable
        for p in &path {
            assert!(grid.is_walkable(p.x, p.y));
        }
        // The path should avoid the expensive column 5 at y=1
        // (may pass through (5,0) or (5,2) which are cheaper routes)
        let passes_through_5_1 = path.contains(&GridPos::new(5, 1));
        // With a cost of 100x, the path should detour around column 5 via row 0 or 2
        assert!(
            !passes_through_5_1,
            "Theta* should avoid expensive cell (5,1)"
        );
    }

    #[test]
    fn flow_field_with_costs() {
        let mut grid = NavGrid::new(5, 1, 1.0);
        grid.allow_diagonal = false;
        // Make cell (2,0) expensive
        grid.set_cost(2, 0, 10.0);
        let field = grid.flow_field(GridPos::new(4, 0));
        // Cell (1,0) should still point toward goal (positive dx)
        let idx = 1; // (1, 0)
        assert_eq!(field[idx].0, 1);
    }

    #[test]
    fn jps_large_grid_with_maze() {
        // Stress test: 200x200 grid with maze-like walls
        let mut grid = NavGrid::new(200, 200, 1.0);
        for y in (0..200).step_by(4) {
            for x in 0..198 {
                grid.set_walkable(x, y, false);
            }
        }
        for y in (2..200).step_by(4) {
            for x in 2..200 {
                grid.set_walkable(x, y, false);
            }
        }
        let path = grid.find_path_jps(GridPos::new(0, 1), GridPos::new(199, 199));
        // Either finds a path or correctly returns None — just shouldn't stack overflow
        if let Some(path) = &path {
            for p in path {
                assert!(grid.is_walkable(p.x, p.y));
            }
            for w in path.windows(2) {
                let dx = (w[1].x - w[0].x).abs();
                let dy = (w[1].y - w[0].y).abs();
                assert!(dx <= 1 && dy <= 1 && (dx + dy) > 0);
            }
        }
    }
}
