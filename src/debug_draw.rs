//! Debug visualization — geometry output for rendering navmeshes, paths, and flow fields.
//!
//! Produces line segments and points that consumers can render with their
//! own graphics backend. No rendering dependency.

use hisab::Vec2;

use crate::grid::{GridPos, NavGrid};
use crate::mesh::NavMesh;

/// A colored line segment for debug rendering.
#[derive(Debug, Clone, Copy)]
pub struct DebugLine {
    pub start: Vec2,
    pub end: Vec2,
    pub color: [f32; 4],
}

/// A colored point for debug rendering.
#[derive(Debug, Clone, Copy)]
pub struct DebugPoint {
    pub position: Vec2,
    pub color: [f32; 4],
}

/// Collects debug geometry for visualization.
#[derive(Debug, Clone, Default)]
pub struct DebugDraw {
    pub lines: Vec<DebugLine>,
    pub points: Vec<DebugPoint>,
}

impl DebugDraw {
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Clear all collected geometry.
    pub fn clear(&mut self) {
        self.lines.clear();
        self.points.clear();
    }

    /// Draw a NavMesh as polygon outlines.
    pub fn draw_navmesh(&mut self, mesh: &NavMesh, color: [f32; 4]) {
        for poly in mesh.polys() {
            let n = poly.vertices.len();
            for i in 0..n {
                let j = (i + 1) % n;
                self.lines.push(DebugLine {
                    start: poly.vertices[i],
                    end: poly.vertices[j],
                    color,
                });
            }
            // Mark centroid
            self.points.push(DebugPoint {
                position: poly.centroid(),
                color,
            });
        }
    }

    /// Draw a path as connected line segments.
    pub fn draw_path(&mut self, waypoints: &[Vec2], color: [f32; 4]) {
        for w in waypoints.windows(2) {
            self.lines.push(DebugLine {
                start: w[0],
                end: w[1],
                color,
            });
        }
        for &p in waypoints {
            self.points.push(DebugPoint { position: p, color });
        }
    }

    /// Draw a grid path (converts grid positions to world positions).
    pub fn draw_grid_path(&mut self, grid: &NavGrid, path: &[GridPos], color: [f32; 4]) {
        let world: Vec<Vec2> = path.iter().map(|p| grid.grid_to_world(*p)).collect();
        self.draw_path(&world, color);
    }

    /// Draw the flow field as arrows from each cell center.
    pub fn draw_flow_field(&mut self, grid: &NavGrid, field: &[(i32, i32)], color: [f32; 4]) {
        let arrow_scale = grid.cell_size() * 0.4;
        for y in 0..grid.height() as i32 {
            for x in 0..grid.width() as i32 {
                let idx = (y as usize) * grid.width() + (x as usize);
                let (dx, dy) = field[idx];
                if dx == 0 && dy == 0 {
                    continue;
                }
                let center = grid.grid_to_world(GridPos::new(x, y));
                let dir = Vec2::new(dx as f32, dy as f32).normalize_or_zero();
                let tip = center + dir * arrow_scale;
                self.lines.push(DebugLine {
                    start: center,
                    end: tip,
                    color,
                });
            }
        }
    }

    /// Draw grid walkability — blocked cells as filled squares.
    pub fn draw_grid_walkability(
        &mut self,
        grid: &NavGrid,
        walkable_color: [f32; 4],
        blocked_color: [f32; 4],
    ) {
        let half = grid.cell_size() * 0.4;
        for y in 0..grid.height() as i32 {
            for x in 0..grid.width() as i32 {
                let center = grid.grid_to_world(GridPos::new(x, y));
                let color = if grid.is_walkable(x, y) {
                    walkable_color
                } else {
                    blocked_color
                };
                // Draw cell as a small square (4 lines)
                let corners = [
                    Vec2::new(center.x - half, center.y - half),
                    Vec2::new(center.x + half, center.y - half),
                    Vec2::new(center.x + half, center.y + half),
                    Vec2::new(center.x - half, center.y + half),
                ];
                for i in 0..4 {
                    self.lines.push(DebugLine {
                        start: corners[i],
                        end: corners[(i + 1) % 4],
                        color,
                    });
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::{NavPoly, NavPolyId};

    #[test]
    fn draw_navmesh_produces_lines() {
        let mut mesh = NavMesh::new();
        mesh.add_poly(NavPoly {
            id: NavPolyId(0),
            vertices: vec![Vec2::ZERO, Vec2::new(1.0, 0.0), Vec2::new(0.5, 1.0)],
            neighbors: vec![],
        });

        let mut dd = DebugDraw::new();
        dd.draw_navmesh(&mesh, [1.0, 0.0, 0.0, 1.0]);

        assert_eq!(dd.lines.len(), 3); // triangle = 3 edges
        assert_eq!(dd.points.len(), 1); // 1 centroid
    }

    #[test]
    fn draw_path_produces_segments() {
        let mut dd = DebugDraw::new();
        let path = vec![Vec2::ZERO, Vec2::new(1.0, 0.0), Vec2::new(2.0, 1.0)];
        dd.draw_path(&path, [0.0, 1.0, 0.0, 1.0]);

        assert_eq!(dd.lines.len(), 2); // 3 points = 2 segments
        assert_eq!(dd.points.len(), 3); // 3 waypoints
    }

    #[test]
    fn draw_grid_path() {
        let grid = NavGrid::new(5, 5, 1.0);
        let path = vec![GridPos::new(0, 0), GridPos::new(1, 1), GridPos::new(2, 2)];

        let mut dd = DebugDraw::new();
        dd.draw_grid_path(&grid, &path, [0.0, 0.0, 1.0, 1.0]);

        assert_eq!(dd.lines.len(), 2);
        assert_eq!(dd.points.len(), 3);
    }

    #[test]
    fn draw_flow_field_produces_arrows() {
        let grid = NavGrid::new(3, 3, 1.0);
        let field = grid.flow_field(GridPos::new(2, 2));

        let mut dd = DebugDraw::new();
        dd.draw_flow_field(&grid, &field, [1.0, 1.0, 0.0, 1.0]);

        // 8 cells with directions (all except goal (2,2))
        assert_eq!(dd.lines.len(), 8);
    }

    #[test]
    fn draw_grid_walkability() {
        let mut grid = NavGrid::new(3, 3, 1.0);
        grid.set_walkable(1, 1, false);

        let mut dd = DebugDraw::new();
        dd.draw_grid_walkability(&grid, [0.0, 1.0, 0.0, 0.3], [1.0, 0.0, 0.0, 0.5]);

        // 9 cells × 4 lines each = 36 lines
        assert_eq!(dd.lines.len(), 36);
    }

    #[test]
    fn clear_resets() {
        let mut dd = DebugDraw::new();
        dd.lines.push(DebugLine {
            start: Vec2::ZERO,
            end: Vec2::ONE,
            color: [1.0; 4],
        });
        dd.points.push(DebugPoint {
            position: Vec2::ZERO,
            color: [1.0; 4],
        });
        dd.clear();
        assert!(dd.lines.is_empty());
        assert!(dd.points.is_empty());
    }

    #[test]
    fn empty_mesh_no_output() {
        let mesh = NavMesh::new();
        let mut dd = DebugDraw::new();
        dd.draw_navmesh(&mesh, [1.0; 4]);
        assert!(dd.lines.is_empty());
        assert!(dd.points.is_empty());
    }
}
