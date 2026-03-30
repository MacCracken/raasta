//! 3D navigation volumes — voxel-based pathfinding for flying/swimming agents.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use hisab::Vec3;
use serde::{Deserialize, Serialize};

#[cfg(feature = "logging")]
use tracing::instrument;

/// A position in the voxel grid.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct VoxelPos {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

impl VoxelPos {
    #[must_use]
    pub fn new(x: i32, y: i32, z: i32) -> Self {
        Self { x, y, z }
    }

    /// Chebyshev distance (diagonal movement in 3D).
    #[inline]
    #[must_use]
    pub fn chebyshev_distance(self, other: VoxelPos) -> i32 {
        let dx = (self.x - other.x).abs();
        let dy = (self.y - other.y).abs();
        let dz = (self.z - other.z).abs();
        dx.max(dy).max(dz)
    }

    /// Octile-like 3D distance heuristic.
    #[inline]
    #[must_use]
    pub fn distance_3d(self, other: VoxelPos) -> f32 {
        let dx = (self.x - other.x).abs() as f32;
        let dy = (self.y - other.y).abs() as f32;
        let dz = (self.z - other.z).abs() as f32;
        let mut sorted = [dx, dy, dz];
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(Ordering::Equal));
        // smallest + (sqrt(2)-1)*middle + (sqrt(3)-sqrt(2))*largest... simplified:
        sorted[2]
            + (std::f32::consts::SQRT_2 - 1.0) * sorted[1]
            + (3.0f32.sqrt() - std::f32::consts::SQRT_2) * sorted[0]
    }
}

/// A 3D navigation volume (voxel grid).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NavVolume {
    width: usize,
    height: usize,
    depth: usize,
    cell_size: f32,
    /// Bit-packed navigability (1 = navigable).
    navigable: Vec<u64>,
    /// Per-cell cost multiplier.
    costs: Vec<f32>,
}

impl NavVolume {
    /// Create a new volume where all cells are navigable.
    #[must_use]
    pub fn new(width: usize, height: usize, depth: usize, cell_size: f32) -> Self {
        let cell_size = cell_size.max(f32::EPSILON);
        let len = width * height * depth;
        let bitset_len = len.div_ceil(64);
        Self {
            width,
            height,
            depth,
            cell_size,
            navigable: vec![u64::MAX; bitset_len],
            costs: vec![1.0; len],
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
    pub fn depth(&self) -> usize {
        self.depth
    }
    #[must_use]
    pub fn cell_size(&self) -> f32 {
        self.cell_size
    }

    /// Set whether a cell is navigable.
    pub fn set_navigable(&mut self, x: i32, y: i32, z: i32, nav: bool) {
        if let Some(idx) = self.index(x, y, z) {
            let word = idx / 64;
            let bit = idx % 64;
            if nav {
                self.navigable[word] |= 1u64 << bit;
            } else {
                self.navigable[word] &= !(1u64 << bit);
            }
        }
    }

    /// Check if a cell is navigable.
    #[inline]
    #[must_use]
    pub fn is_navigable(&self, x: i32, y: i32, z: i32) -> bool {
        self.index(x, y, z)
            .map(|idx| (self.navigable[idx / 64] >> (idx % 64)) & 1 == 1)
            .unwrap_or(false)
    }

    /// Set the traversal cost for a cell.
    pub fn set_cost(&mut self, x: i32, y: i32, z: i32, cost: f32) {
        if let Some(idx) = self.index(x, y, z) {
            self.costs[idx] = cost;
        }
    }

    /// Get the traversal cost of a cell.
    #[must_use]
    pub fn cost(&self, x: i32, y: i32, z: i32) -> f32 {
        self.index(x, y, z)
            .map(|i| self.costs[i])
            .unwrap_or(f32::INFINITY)
    }

    /// Convert voxel position to world position (center of cell).
    #[must_use]
    pub fn voxel_to_world(&self, pos: VoxelPos) -> Vec3 {
        Vec3::new(
            (pos.x as f32 + 0.5) * self.cell_size,
            (pos.y as f32 + 0.5) * self.cell_size,
            (pos.z as f32 + 0.5) * self.cell_size,
        )
    }

    /// Convert world position to voxel position.
    #[must_use]
    pub fn world_to_voxel(&self, world: Vec3) -> VoxelPos {
        VoxelPos::new(
            (world.x / self.cell_size).floor() as i32,
            (world.y / self.cell_size).floor() as i32,
            (world.z / self.cell_size).floor() as i32,
        )
    }

    /// Find a path using 3D A*.
    ///
    /// Supports 26-connected neighbors (cardinal + diagonal + vertical).
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    #[must_use]
    pub fn find_path(&self, start: VoxelPos, goal: VoxelPos) -> Option<Vec<VoxelPos>> {
        if !self.is_navigable(start.x, start.y, start.z)
            || !self.is_navigable(goal.x, goal.y, goal.z)
        {
            return None;
        }
        if start == goal {
            return Some(vec![start]);
        }

        let len = self.width * self.height * self.depth;
        let mut g_score = vec![f32::INFINITY; len];
        let mut came_from: Vec<Option<usize>> = vec![None; len];
        let mut closed = vec![false; len];
        let mut open = BinaryHeap::new();

        let start_idx = self.index(start.x, start.y, start.z)?;
        let goal_idx = self.index(goal.x, goal.y, goal.z)?;

        g_score[start_idx] = 0.0;
        open.push(VoxelNode {
            idx: start_idx,
            f_score: start.distance_3d(goal),
        });

        while let Some(current) = open.pop() {
            if current.idx == goal_idx {
                return Some(self.reconstruct_path(&came_from, goal_idx));
            }
            if closed[current.idx] {
                continue;
            }
            closed[current.idx] = true;

            let cx = (current.idx % self.width) as i32;
            let cy = ((current.idx / self.width) % self.height) as i32;
            let cz = (current.idx / (self.width * self.height)) as i32;

            // 26-connected neighbors
            for dz in -1..=1i32 {
                for dy in -1..=1i32 {
                    for dx in -1..=1i32 {
                        if dx == 0 && dy == 0 && dz == 0 {
                            continue;
                        }
                        let nx = cx + dx;
                        let ny = cy + dy;
                        let nz = cz + dz;
                        if !self.is_navigable(nx, ny, nz) {
                            continue;
                        }

                        let n_idx = match self.index(nx, ny, nz) {
                            Some(i) => i,
                            None => continue,
                        };
                        if closed[n_idx] {
                            continue;
                        }

                        let move_dist = match dx.abs() + dy.abs() + dz.abs() {
                            1 => 1.0,
                            2 => std::f32::consts::SQRT_2,
                            _ => 3.0f32.sqrt(), // 3D diagonal
                        };
                        let tentative_g = g_score[current.idx] + move_dist * self.costs[n_idx];

                        if tentative_g < g_score[n_idx] {
                            came_from[n_idx] = Some(current.idx);
                            g_score[n_idx] = tentative_g;
                            let np = VoxelPos::new(nx, ny, nz);
                            let h = np.distance_3d(goal);
                            open.push(VoxelNode {
                                idx: n_idx,
                                f_score: tentative_g + h,
                            });
                        }
                    }
                }
            }
        }

        None
    }

    #[inline]
    fn index(&self, x: i32, y: i32, z: i32) -> Option<usize> {
        if x >= 0
            && y >= 0
            && z >= 0
            && (x as usize) < self.width
            && (y as usize) < self.height
            && (z as usize) < self.depth
        {
            Some(z as usize * self.width * self.height + y as usize * self.width + x as usize)
        } else {
            None
        }
    }

    fn reconstruct_path(&self, came_from: &[Option<usize>], goal_idx: usize) -> Vec<VoxelPos> {
        let mut path = Vec::new();
        let mut current = goal_idx;
        let plane = self.width * self.height;
        loop {
            let x = (current % self.width) as i32;
            let y = ((current / self.width) % self.height) as i32;
            let z = (current / plane) as i32;
            path.push(VoxelPos::new(x, y, z));
            match came_from[current] {
                Some(p) => current = p,
                None => break,
            }
        }
        path.reverse();
        path
    }
}

#[derive(Clone, Copy)]
struct VoxelNode {
    idx: usize,
    f_score: f32,
}

impl PartialEq for VoxelNode {
    fn eq(&self, o: &Self) -> bool {
        self.f_score == o.f_score
    }
}

impl Eq for VoxelNode {}

impl PartialOrd for VoxelNode {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}

impl Ord for VoxelNode {
    fn cmp(&self, o: &Self) -> Ordering {
        o.f_score
            .partial_cmp(&self.f_score)
            .unwrap_or(Ordering::Equal)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn basic_3d_path() {
        let vol = NavVolume::new(10, 10, 10, 1.0);
        let path = vol.find_path(VoxelPos::new(0, 0, 0), VoxelPos::new(5, 5, 5));
        assert!(path.is_some());
        let path = path.unwrap();
        assert_eq!(*path.first().unwrap(), VoxelPos::new(0, 0, 0));
        assert_eq!(*path.last().unwrap(), VoxelPos::new(5, 5, 5));
    }

    #[test]
    fn same_start_goal() {
        let vol = NavVolume::new(5, 5, 5, 1.0);
        let path = vol.find_path(VoxelPos::new(2, 2, 2), VoxelPos::new(2, 2, 2));
        assert_eq!(path, Some(vec![VoxelPos::new(2, 2, 2)]));
    }

    #[test]
    fn blocked_path() {
        let mut vol = NavVolume::new(3, 1, 1, 1.0);
        // Block the middle cell
        vol.set_navigable(1, 0, 0, false);
        let path = vol.find_path(VoxelPos::new(0, 0, 0), VoxelPos::new(2, 0, 0));
        // 1D grid with blocked middle: no path
        assert!(path.is_none());
    }

    #[test]
    fn vertical_movement() {
        let vol = NavVolume::new(1, 1, 5, 1.0);
        let path = vol.find_path(VoxelPos::new(0, 0, 0), VoxelPos::new(0, 0, 4));
        assert!(path.is_some());
        let path = path.unwrap();
        assert_eq!(path.len(), 5); // straight line through z
        for (i, pos) in path.iter().enumerate() {
            assert_eq!(pos.x, 0);
            assert_eq!(pos.y, 0);
            assert_eq!(pos.z, i as i32);
        }
    }

    #[test]
    fn not_navigable_start_goal() {
        let mut vol = NavVolume::new(5, 5, 5, 1.0);
        vol.set_navigable(0, 0, 0, false);
        // Start not navigable
        assert!(
            vol.find_path(VoxelPos::new(0, 0, 0), VoxelPos::new(4, 4, 4))
                .is_none()
        );

        let mut vol2 = NavVolume::new(5, 5, 5, 1.0);
        vol2.set_navigable(4, 4, 4, false);
        // Goal not navigable
        assert!(
            vol2.find_path(VoxelPos::new(0, 0, 0), VoxelPos::new(4, 4, 4))
                .is_none()
        );
    }

    #[test]
    fn world_voxel_conversion_roundtrip() {
        let vol = NavVolume::new(10, 10, 10, 2.0);
        let pos = VoxelPos::new(3, 5, 7);
        let world = vol.voxel_to_world(pos);
        let back = vol.world_to_voxel(world);
        assert_eq!(back, pos);
    }

    #[test]
    fn serde_roundtrip() {
        let mut vol = NavVolume::new(4, 4, 4, 0.5);
        vol.set_navigable(1, 1, 1, false);
        vol.set_cost(2, 2, 2, 3.0);

        let json = serde_json::to_string(&vol).unwrap();
        let deserialized: NavVolume = serde_json::from_str(&json).unwrap();
        assert_eq!(deserialized.width(), 4);
        assert_eq!(deserialized.height(), 4);
        assert_eq!(deserialized.depth(), 4);
        assert!(!deserialized.is_navigable(1, 1, 1));
        assert!(deserialized.is_navigable(0, 0, 0));
        assert!((deserialized.cost(2, 2, 2) - 3.0).abs() < f32::EPSILON);
    }

    #[test]
    fn voxel_1x1x1() {
        let vol = NavVolume::new(1, 1, 1, 1.0);
        let path = vol.find_path(VoxelPos::new(0, 0, 0), VoxelPos::new(0, 0, 0));
        assert_eq!(path, Some(vec![VoxelPos::new(0, 0, 0)]));
    }

    #[test]
    fn voxel_cost_aware() {
        let mut vol = NavVolume::new(10, 1, 10, 1.0);
        // Make a wall of expensive cells at x=5, except leave z=0 cheap
        for z in 1..10 {
            vol.set_cost(5, 0, z, 1000.0);
        }
        // Path from (0,0,5) to (9,0,5) should go around via z=0
        let path = vol.find_path(VoxelPos::new(0, 0, 5), VoxelPos::new(9, 0, 5));
        assert!(path.is_some());
        let path = path.unwrap();
        // Cells at x=5 with z>0 should be avoided (only x=5,z=0 is cheap)
        for p in &path {
            if p.x == 5 {
                assert!(
                    p.z == 0,
                    "Path should avoid expensive cells at x=5, z>0, but went through ({},{},{})",
                    p.x,
                    p.y,
                    p.z
                );
            }
        }
    }

    #[test]
    fn voxel_all_blocked() {
        let mut vol = NavVolume::new(3, 3, 3, 1.0);
        // Block everything except start and goal
        for z in 0..3 {
            for y in 0..3 {
                for x in 0..3 {
                    let is_start = x == 0 && y == 0 && z == 0;
                    let is_goal = x == 2 && y == 2 && z == 2;
                    if !is_start && !is_goal {
                        vol.set_navigable(x, y, z, false);
                    }
                }
            }
        }
        // No path possible (no connected neighbors)
        assert!(
            vol.find_path(VoxelPos::new(0, 0, 0), VoxelPos::new(2, 2, 2))
                .is_none()
        );
    }

    #[test]
    fn voxel_large_volume() {
        let vol = NavVolume::new(20, 20, 20, 1.0);
        let path = vol.find_path(VoxelPos::new(0, 0, 0), VoxelPos::new(19, 19, 19));
        assert!(path.is_some());
        let path = path.unwrap();
        assert_eq!(*path.first().unwrap(), VoxelPos::new(0, 0, 0));
        assert_eq!(*path.last().unwrap(), VoxelPos::new(19, 19, 19));
    }

    #[test]
    fn voxel_cell_size_clamped() {
        // cell_size should be clamped to positive
        let vol = NavVolume::new(5, 5, 5, 0.0);
        // Should not panic on coordinate conversion
        let _pos = vol.world_to_voxel(Vec3::new(1.0, 1.0, 1.0));
    }
}
