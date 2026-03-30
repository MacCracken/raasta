//! Influence maps — overlay grids for danger zones, strategic value, and cost annotations.

use hisab::Vec2;
use serde::{Deserialize, Serialize};

#[cfg(feature = "logging")]
use tracing::instrument;

/// A 2D influence map grid for annotating areas with scalar values.
///
/// Useful for danger zones, strategic value overlays, scent trails,
/// or any spatial cost annotation that affects pathfinding or AI decisions.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InfluenceMap {
    width: usize,
    height: usize,
    cell_size: f32,
    values: Vec<f32>,
}

impl InfluenceMap {
    /// Create a new influence map initialized to zero.
    #[must_use]
    pub fn new(width: usize, height: usize, cell_size: f32) -> Self {
        Self {
            width,
            height,
            cell_size,
            values: vec![0.0; width * height],
        }
    }

    /// Grid width in cells.
    #[must_use]
    pub fn width(&self) -> usize {
        self.width
    }

    /// Grid height in cells.
    #[must_use]
    pub fn height(&self) -> usize {
        self.height
    }

    /// Cell size in world units.
    #[must_use]
    pub fn cell_size(&self) -> f32 {
        self.cell_size
    }

    /// Get the influence value at grid coordinates.
    #[inline]
    #[must_use]
    pub fn get(&self, x: usize, y: usize) -> f32 {
        self.index(x, y).map(|i| self.values[i]).unwrap_or(0.0)
    }

    /// Set the influence value at grid coordinates.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    pub fn set(&mut self, x: usize, y: usize, value: f32) {
        if let Some(i) = self.index(x, y) {
            self.values[i] = value;
        }
    }

    /// Add to the influence value at grid coordinates.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    pub fn add(&mut self, x: usize, y: usize, delta: f32) {
        if let Some(i) = self.index(x, y) {
            self.values[i] += delta;
        }
    }

    /// Sample the influence at a world position (nearest cell).
    #[must_use]
    pub fn sample(&self, pos: Vec2) -> f32 {
        let x = (pos.x / self.cell_size) as usize;
        let y = (pos.y / self.cell_size) as usize;
        self.get(x, y)
    }

    /// Apply a circular stamp — add `value` to all cells within `radius` of `center`.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    pub fn stamp_circle(&mut self, center: Vec2, radius: f32, value: f32) {
        let min_x = ((center.x - radius) / self.cell_size).floor().max(0.0) as usize;
        let max_x = ((center.x + radius) / self.cell_size)
            .ceil()
            .min(self.width as f32 - 1.0) as usize;
        let min_y = ((center.y - radius) / self.cell_size).floor().max(0.0) as usize;
        let max_y = ((center.y + radius) / self.cell_size)
            .ceil()
            .min(self.height as f32 - 1.0) as usize;
        let r_sq = radius * radius;

        for y in min_y..=max_y {
            for x in min_x..=max_x {
                let cx = (x as f32 + 0.5) * self.cell_size;
                let cy = (y as f32 + 0.5) * self.cell_size;
                let dx = cx - center.x;
                let dy = cy - center.y;
                if dx * dx + dy * dy <= r_sq {
                    self.add(x, y, value);
                }
            }
        }
    }

    /// Decay all values toward zero by `factor` (0.0..1.0).
    /// Each value becomes `value * factor`.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    pub fn decay(&mut self, factor: f32) {
        for v in &mut self.values {
            *v *= factor;
        }
    }

    /// Reset all values to zero.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    pub fn clear(&mut self) {
        self.values.fill(0.0);
    }

    /// Get the raw values slice.
    #[must_use]
    pub fn values(&self) -> &[f32] {
        &self.values
    }

    /// Maximum value in the map.
    #[must_use]
    pub fn max_value(&self) -> f32 {
        self.values
            .iter()
            .cloned()
            .fold(f32::NEG_INFINITY, f32::max)
    }

    /// Minimum value in the map.
    #[must_use]
    pub fn min_value(&self) -> f32 {
        self.values.iter().cloned().fold(f32::INFINITY, f32::min)
    }

    #[inline]
    fn index(&self, x: usize, y: usize) -> Option<usize> {
        if x < self.width && y < self.height {
            Some(y * self.width + x)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_initializes_to_zero() {
        let map = InfluenceMap::new(4, 4, 1.0);
        for &v in map.values() {
            assert!((v).abs() < f32::EPSILON);
        }
    }

    #[test]
    fn get_set() {
        let mut map = InfluenceMap::new(4, 4, 1.0);
        map.set(2, 3, 5.0);
        assert!((map.get(2, 3) - 5.0).abs() < f32::EPSILON);
        assert!((map.get(0, 0)).abs() < f32::EPSILON);
    }

    #[test]
    fn add_accumulates() {
        let mut map = InfluenceMap::new(4, 4, 1.0);
        map.add(1, 1, 3.0);
        map.add(1, 1, 2.0);
        assert!((map.get(1, 1) - 5.0).abs() < f32::EPSILON);
    }

    #[test]
    fn sample_world_position() {
        let mut map = InfluenceMap::new(10, 10, 2.0);
        map.set(1, 1, 7.0);
        // World position (2.5, 2.5) / cell_size 2.0 = cell (1, 1)
        let v = map.sample(Vec2::new(2.5, 2.5));
        assert!((v - 7.0).abs() < f32::EPSILON);
    }

    #[test]
    fn stamp_circle_affects_nearby_cells() {
        let mut map = InfluenceMap::new(10, 10, 1.0);
        map.stamp_circle(Vec2::new(5.0, 5.0), 1.5, 1.0);

        // Center cell should be affected
        assert!(map.get(5, 5) > 0.0);
        // Far corner should not be affected
        assert!((map.get(0, 0)).abs() < f32::EPSILON);
    }

    #[test]
    fn decay_reduces_values() {
        let mut map = InfluenceMap::new(4, 4, 1.0);
        map.set(0, 0, 10.0);
        map.decay(0.5);
        assert!((map.get(0, 0) - 5.0).abs() < f32::EPSILON);
    }

    #[test]
    fn clear_resets_all() {
        let mut map = InfluenceMap::new(4, 4, 1.0);
        map.set(0, 0, 10.0);
        map.set(3, 3, 20.0);
        map.clear();
        for &v in map.values() {
            assert!((v).abs() < f32::EPSILON);
        }
    }

    #[test]
    fn max_min_values() {
        let mut map = InfluenceMap::new(4, 4, 1.0);
        map.set(0, 0, -5.0);
        map.set(1, 1, 10.0);
        map.set(2, 2, 3.0);
        assert!((map.max_value() - 10.0).abs() < f32::EPSILON);
        assert!((map.min_value() - (-5.0)).abs() < f32::EPSILON);
    }

    #[test]
    fn out_of_bounds_returns_zero() {
        let map = InfluenceMap::new(4, 4, 1.0);
        assert!((map.get(10, 10)).abs() < f32::EPSILON);
        assert!((map.get(4, 0)).abs() < f32::EPSILON);
        assert!((map.get(0, 4)).abs() < f32::EPSILON);
    }

    #[test]
    fn serde_roundtrip() {
        let mut map = InfluenceMap::new(3, 3, 2.0);
        map.set(1, 1, 42.0);

        let json = serde_json::to_string(&map).expect("serialize");
        let map2: InfluenceMap = serde_json::from_str(&json).expect("deserialize");

        assert_eq!(map2.width(), 3);
        assert_eq!(map2.height(), 3);
        assert!((map2.cell_size() - 2.0).abs() < f32::EPSILON);
        assert!((map2.get(1, 1) - 42.0).abs() < f32::EPSILON);
    }
}
