//! Tiled NavMesh — streaming and localized re-baking for open worlds.

use std::collections::HashMap;

use hisab::Vec2;
use serde::{Deserialize, Serialize};

use crate::mesh::{NavMesh, NavPolyId};

#[cfg(feature = "logging")]
use tracing::instrument;

/// Identifies a tile by its grid coordinates.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct TileCoord {
    pub x: i32,
    pub y: i32,
}

impl TileCoord {
    #[must_use]
    pub fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }
}

/// A globally unique polygon ID combining tile coordinate and local polygon index.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct GlobalPolyId {
    pub tile: TileCoord,
    pub local: NavPolyId,
}

/// A tile containing its own local navmesh.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NavTile {
    /// The tile's local navmesh.
    pub mesh: NavMesh,
    /// World-space origin of this tile (bottom-left corner).
    pub origin: Vec2,
}

/// Connection between polygons across tile boundaries.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
struct TileConnection {
    from: GlobalPolyId,
    to: GlobalPolyId,
    cost: f32,
}

/// A tiled navmesh for open-world navigation.
///
/// Divides the world into rectangular tiles, each with its own `NavMesh`.
/// Supports streaming (load/unload tiles at runtime) and localized
/// re-baking (only rebuild the affected tile).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TiledNavMesh {
    /// Size of each tile in world units.
    tile_size: f32,
    /// Loaded tiles.
    #[serde(
        serialize_with = "serialize_tile_map",
        deserialize_with = "deserialize_tile_map"
    )]
    tiles: HashMap<TileCoord, NavTile>,
    /// Cross-tile connections (border polygon pairs).
    connections: Vec<TileConnection>,
}

fn serialize_tile_map<S>(
    map: &HashMap<TileCoord, NavTile>,
    serializer: S,
) -> Result<S::Ok, S::Error>
where
    S: serde::Serializer,
{
    use serde::ser::SerializeSeq;
    let mut seq = serializer.serialize_seq(Some(map.len()))?;
    for (k, v) in map {
        seq.serialize_element(&(*k, v))?;
    }
    seq.end()
}

fn deserialize_tile_map<'de, D>(deserializer: D) -> Result<HashMap<TileCoord, NavTile>, D::Error>
where
    D: serde::Deserializer<'de>,
{
    let entries: Vec<(TileCoord, NavTile)> = Vec::deserialize(deserializer)?;
    Ok(entries.into_iter().collect())
}

impl TiledNavMesh {
    /// Create a new tiled navmesh with the given tile size.
    #[cfg_attr(feature = "logging", instrument)]
    #[must_use]
    pub fn new(tile_size: f32) -> Self {
        Self {
            tile_size,
            tiles: HashMap::new(),
            connections: Vec::new(),
        }
    }

    /// Tile size in world units.
    #[must_use]
    pub fn tile_size(&self) -> f32 {
        self.tile_size
    }

    /// Number of loaded tiles.
    #[must_use]
    pub fn tile_count(&self) -> usize {
        self.tiles.len()
    }

    /// Get the tile coordinate for a world position.
    #[must_use]
    pub fn world_to_tile(&self, pos: Vec2) -> TileCoord {
        TileCoord {
            x: (pos.x / self.tile_size).floor() as i32,
            y: (pos.y / self.tile_size).floor() as i32,
        }
    }

    /// Load (insert) a tile. If a tile already exists at this coordinate, it is replaced.
    ///
    /// After loading, call `rebuild_connections()` to update cross-tile connectivity.
    #[cfg_attr(feature = "logging", instrument(skip(self, mesh)))]
    pub fn load_tile(&mut self, coord: TileCoord, mesh: NavMesh) {
        let origin = Vec2::new(
            coord.x as f32 * self.tile_size,
            coord.y as f32 * self.tile_size,
        );
        self.tiles.insert(coord, NavTile { mesh, origin });
    }

    /// Unload (remove) a tile.
    ///
    /// After unloading, call `rebuild_connections()` to update connectivity.
    pub fn unload_tile(&mut self, coord: TileCoord) -> bool {
        let removed = self.tiles.remove(&coord).is_some();
        if removed {
            // Remove connections involving this tile
            self.connections
                .retain(|c| c.from.tile != coord && c.to.tile != coord);
        }
        removed
    }

    /// Check if a tile is loaded.
    #[must_use]
    pub fn has_tile(&self, coord: TileCoord) -> bool {
        self.tiles.contains_key(&coord)
    }

    /// Get a tile's navmesh.
    #[must_use]
    pub fn get_tile(&self, coord: TileCoord) -> Option<&NavTile> {
        self.tiles.get(&coord)
    }

    /// Re-bake a single tile from new boundary geometry.
    ///
    /// Replaces the tile's navmesh with a freshly baked one and rebuilds
    /// only the connections involving this tile and its neighbors.
    /// Much cheaper than `rebuild_connections()` for the full mesh.
    ///
    /// `boundary` is in tile-local coordinates (origin at 0,0).
    #[cfg_attr(feature = "logging", instrument(skip(self, boundary)))]
    pub fn rebake_tile(&mut self, coord: TileCoord, boundary: &[Vec2]) {
        let mesh = NavMesh::bake(boundary);
        self.load_tile(coord, mesh);
        self.rebuild_tile_connections(coord);
    }

    /// Rebuild connections for a single tile and its neighbors.
    ///
    /// Cheaper than full `rebuild_connections()` when only one tile changed.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    pub fn rebuild_tile_connections(&mut self, coord: TileCoord) {
        // Remove old connections involving this tile
        self.connections
            .retain(|c| c.from.tile != coord && c.to.tile != coord);

        // Reconnect with all 4 neighbors
        let neighbors = [
            (TileCoord::new(coord.x + 1, coord.y), Border::Right),
            (TileCoord::new(coord.x - 1, coord.y), Border::Left),
            (TileCoord::new(coord.x, coord.y + 1), Border::Top),
            (TileCoord::new(coord.x, coord.y - 1), Border::Bottom),
        ];

        for (neighbor, border) in neighbors {
            if !self.tiles.contains_key(&neighbor) {
                continue;
            }
            match border {
                Border::Right => self.connect_tiles(coord, neighbor, Border::Right),
                Border::Left => self.connect_tiles(neighbor, coord, Border::Right),
                Border::Top => self.connect_tiles(coord, neighbor, Border::Top),
                Border::Bottom => self.connect_tiles(neighbor, coord, Border::Top),
            }
        }
    }

    /// Rebuild cross-tile connections for all loaded tiles.
    ///
    /// Scans border polygons of adjacent tiles and creates connections
    /// between polygons that share vertices near tile boundaries.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    pub fn rebuild_connections(&mut self) {
        self.connections.clear();

        let coords: Vec<TileCoord> = self.tiles.keys().copied().collect();

        for &coord in &coords {
            // Check right neighbor
            let right = TileCoord::new(coord.x + 1, coord.y);
            if self.tiles.contains_key(&right) {
                self.connect_tiles(coord, right, Border::Right);
            }
            // Check top neighbor
            let top = TileCoord::new(coord.x, coord.y + 1);
            if self.tiles.contains_key(&top) {
                self.connect_tiles(coord, top, Border::Top);
            }
        }
    }

    /// Find a path across tiles from start to goal (world positions).
    ///
    /// Performs a two-level search:
    /// 1. Find start/goal tiles and local polygons
    /// 2. A* across tiles using connections
    ///
    /// Returns the path as a sequence of world-space waypoints (polygon centroids).
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    #[must_use]
    pub fn find_path(&self, start: Vec2, goal: Vec2) -> Option<Vec<Vec2>> {
        let start_tile_coord = self.world_to_tile(start);
        let goal_tile_coord = self.world_to_tile(goal);

        let start_tile = self.tiles.get(&start_tile_coord)?;
        let _goal_tile = self.tiles.get(&goal_tile_coord)?;

        // If same tile, use local pathfinding
        if start_tile_coord == goal_tile_coord {
            let local_start = start - start_tile.origin;
            let local_goal = goal - start_tile.origin;
            let poly_path = start_tile.mesh.find_path(local_start, local_goal)?;
            return Some(
                poly_path
                    .iter()
                    .filter_map(|&id| start_tile.mesh.get_poly(id))
                    .map(|p| p.centroid() + start_tile.origin)
                    .collect(),
            );
        }

        // Cross-tile: simple A* on the global graph
        self.cross_tile_path(start, goal, start_tile_coord, goal_tile_coord)
    }

    /// Cross-tile A* pathfinding on the global polygon graph.
    fn cross_tile_path(
        &self,
        start: Vec2,
        goal: Vec2,
        start_tile: TileCoord,
        goal_tile: TileCoord,
    ) -> Option<Vec<Vec2>> {
        use std::cmp::Ordering;
        use std::collections::BinaryHeap;

        // Build a simple global node list: (tile, local_poly_id) -> global index
        let mut nodes: Vec<GlobalPolyId> = Vec::new();
        let mut node_positions: Vec<Vec2> = Vec::new();
        let mut tile_node_map: HashMap<GlobalPolyId, usize> = HashMap::new();

        for (&coord, tile) in &self.tiles {
            for poly in tile.mesh.polys() {
                let gid = GlobalPolyId {
                    tile: coord,
                    local: poly.id,
                };
                let idx = nodes.len();
                tile_node_map.insert(gid, idx);
                nodes.push(gid);
                node_positions.push(poly.centroid() + tile.origin);
            }
        }

        if nodes.is_empty() {
            return None;
        }

        // Find start and goal nodes
        let st = self.tiles.get(&start_tile)?;
        let local_start = start - st.origin;
        let start_poly = st.mesh.find_poly_at(local_start)?;
        let start_gid = GlobalPolyId {
            tile: start_tile,
            local: start_poly,
        };
        let start_idx = *tile_node_map.get(&start_gid)?;

        let gt = self.tiles.get(&goal_tile)?;
        let local_goal = goal - gt.origin;
        let goal_poly = gt.mesh.find_poly_at(local_goal)?;
        let goal_gid = GlobalPolyId {
            tile: goal_tile,
            local: goal_poly,
        };
        let goal_idx = *tile_node_map.get(&goal_gid)?;

        // Build adjacency: local neighbors + cross-tile connections
        let n = nodes.len();
        let mut adj: Vec<Vec<(usize, f32)>> = vec![Vec::new(); n];

        // Local neighbors
        for (&coord, tile) in &self.tiles {
            for poly in tile.mesh.polys() {
                let gid = GlobalPolyId {
                    tile: coord,
                    local: poly.id,
                };
                let from_idx = tile_node_map[&gid];
                for &neighbor_id in &poly.neighbors {
                    let ngid = GlobalPolyId {
                        tile: coord,
                        local: neighbor_id,
                    };
                    if let Some(&to_idx) = tile_node_map.get(&ngid) {
                        let cost = node_positions[from_idx].distance(node_positions[to_idx]);
                        adj[from_idx].push((to_idx, cost));
                    }
                }
            }
        }

        // Cross-tile connections
        for conn in &self.connections {
            if let (Some(&from_idx), Some(&to_idx)) =
                (tile_node_map.get(&conn.from), tile_node_map.get(&conn.to))
            {
                adj[from_idx].push((to_idx, conn.cost));
                adj[to_idx].push((from_idx, conn.cost));
            }
        }

        // A*
        #[derive(Clone, Copy)]
        struct AStarNode {
            idx: usize,
            f: f32,
        }

        impl PartialEq for AStarNode {
            fn eq(&self, other: &Self) -> bool {
                self.f == other.f
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
                other.f.partial_cmp(&self.f).unwrap_or(Ordering::Equal)
            }
        }

        let mut g = vec![f32::INFINITY; n];
        let mut came_from: Vec<Option<usize>> = vec![None; n];
        let mut closed = vec![false; n];
        let mut open = BinaryHeap::new();

        g[start_idx] = 0.0;
        let h = node_positions[start_idx].distance(node_positions[goal_idx]);
        open.push(AStarNode {
            idx: start_idx,
            f: h,
        });

        while let Some(current) = open.pop() {
            if current.idx == goal_idx {
                // Reconstruct path
                let mut path = Vec::new();
                let mut cur = goal_idx;
                loop {
                    path.push(node_positions[cur]);
                    match came_from[cur] {
                        Some(prev) => cur = prev,
                        None => break,
                    }
                }
                path.reverse();
                return Some(path);
            }

            if closed[current.idx] {
                continue;
            }
            closed[current.idx] = true;

            for &(ni, cost) in &adj[current.idx] {
                if closed[ni] {
                    continue;
                }
                let tg = g[current.idx] + cost;
                if tg < g[ni] {
                    g[ni] = tg;
                    came_from[ni] = Some(current.idx);
                    let h = node_positions[ni].distance(node_positions[goal_idx]);
                    open.push(AStarNode { idx: ni, f: tg + h });
                }
            }
        }

        None
    }

    /// Connect border polygons between two adjacent tiles.
    fn connect_tiles(&mut self, a: TileCoord, b: TileCoord, border: Border) {
        let tile_a = match self.tiles.get(&a) {
            Some(t) => t,
            None => return,
        };
        let tile_b = match self.tiles.get(&b) {
            Some(t) => t,
            None => return,
        };

        let threshold = self.tile_size * 0.05; // 5% tolerance for border matching

        // Collect polygon data to avoid borrow conflicts
        let polys_a: Vec<(NavPolyId, Vec<Vec2>, Vec2)> = tile_a
            .mesh
            .polys()
            .iter()
            .map(|p| (p.id, p.vertices.clone(), p.centroid() + tile_a.origin))
            .collect();
        let polys_b: Vec<(NavPolyId, Vec<Vec2>, Vec2)> = tile_b
            .mesh
            .polys()
            .iter()
            .map(|p| (p.id, p.vertices.clone(), p.centroid() + tile_b.origin))
            .collect();

        let origin_a = tile_a.origin;
        let origin_b = tile_b.origin;
        let tile_size = self.tile_size;

        let opposite = match border {
            Border::Right => Border::Left,
            Border::Top => Border::Bottom,
            _ => return,
        };

        for (id_a, verts_a, centroid_a) in &polys_a {
            if !is_near_border(verts_a, tile_size, border, threshold) {
                continue;
            }

            for (id_b, verts_b, centroid_b) in &polys_b {
                if !is_near_border(verts_b, tile_size, opposite, threshold) {
                    continue;
                }

                if has_shared_border_vertices(verts_a, origin_a, verts_b, origin_b, threshold) {
                    let cost = centroid_a.distance(*centroid_b);
                    self.connections.push(TileConnection {
                        from: GlobalPolyId {
                            tile: a,
                            local: *id_a,
                        },
                        to: GlobalPolyId {
                            tile: b,
                            local: *id_b,
                        },
                        cost,
                    });
                }
            }
        }
    }
}

#[derive(Debug, Clone, Copy)]
enum Border {
    Right,
    Top,
    Left,
    Bottom,
}

/// Check if any vertex of a polygon is near a tile border.
fn is_near_border(vertices: &[Vec2], tile_size: f32, border: Border, threshold: f32) -> bool {
    for v in vertices {
        match border {
            Border::Right => {
                if (v.x - tile_size).abs() < threshold {
                    return true;
                }
            }
            Border::Left => {
                if v.x.abs() < threshold {
                    return true;
                }
            }
            Border::Top => {
                if (v.y - tile_size).abs() < threshold {
                    return true;
                }
            }
            Border::Bottom => {
                if v.y.abs() < threshold {
                    return true;
                }
            }
        }
    }
    false
}

/// Check if two polygons share vertices near the tile boundary.
fn has_shared_border_vertices(
    verts_a: &[Vec2],
    origin_a: Vec2,
    verts_b: &[Vec2],
    origin_b: Vec2,
    threshold: f32,
) -> bool {
    for va in verts_a {
        let world_a = *va + origin_a;
        for vb in verts_b {
            let world_b = *vb + origin_b;
            if world_a.distance(world_b) < threshold {
                return true;
            }
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::NavPoly;

    fn make_tile_mesh() -> NavMesh {
        // Simple square polygon from (0,0) to (10,10)
        let mut mesh = NavMesh::new();
        mesh.add_poly(NavPoly {
            id: NavPolyId(0),
            vertices: vec![
                Vec2::ZERO,
                Vec2::new(10.0, 0.0),
                Vec2::new(10.0, 10.0),
                Vec2::new(0.0, 10.0),
            ],
            neighbors: vec![],
            cost: 1.0,
            layer: 0,
        });
        mesh
    }

    #[test]
    fn tiled_basic() {
        let mut tiled = TiledNavMesh::new(10.0);
        tiled.load_tile(TileCoord::new(0, 0), make_tile_mesh());
        assert_eq!(tiled.tile_count(), 1);
        assert!(tiled.has_tile(TileCoord::new(0, 0)));
    }

    #[test]
    fn tiled_world_to_tile() {
        let tiled = TiledNavMesh::new(10.0);
        assert_eq!(
            tiled.world_to_tile(Vec2::new(5.0, 5.0)),
            TileCoord::new(0, 0)
        );
        assert_eq!(
            tiled.world_to_tile(Vec2::new(15.0, 5.0)),
            TileCoord::new(1, 0)
        );
        assert_eq!(
            tiled.world_to_tile(Vec2::new(-5.0, -5.0)),
            TileCoord::new(-1, -1)
        );
    }

    #[test]
    fn tiled_same_tile_path() {
        let mut tiled = TiledNavMesh::new(10.0);
        tiled.load_tile(TileCoord::new(0, 0), make_tile_mesh());
        let path = tiled.find_path(Vec2::new(2.0, 2.0), Vec2::new(8.0, 8.0));
        assert!(path.is_some());
    }

    #[test]
    fn tiled_unload() {
        let mut tiled = TiledNavMesh::new(10.0);
        tiled.load_tile(TileCoord::new(0, 0), make_tile_mesh());
        assert!(tiled.unload_tile(TileCoord::new(0, 0)));
        assert!(!tiled.has_tile(TileCoord::new(0, 0)));
        assert_eq!(tiled.tile_count(), 0);
    }

    #[test]
    fn tiled_unload_nonexistent() {
        let mut tiled = TiledNavMesh::new(10.0);
        assert!(!tiled.unload_tile(TileCoord::new(0, 0)));
    }

    #[test]
    fn tiled_cross_tile_path() {
        let mut tiled = TiledNavMesh::new(10.0);

        // Two tiles side by side, each with a square polygon
        // Tile (0,0): polygon with vertex at (10,0)-(10,10) on right border
        // Tile (1,0): polygon with vertex at (0,0)-(0,10) on left border (= world 10,0-10,10)
        tiled.load_tile(TileCoord::new(0, 0), make_tile_mesh());
        tiled.load_tile(TileCoord::new(1, 0), make_tile_mesh());
        tiled.rebuild_connections();

        let path = tiled.find_path(Vec2::new(5.0, 5.0), Vec2::new(15.0, 5.0));
        assert!(path.is_some());
        let path = path.unwrap();
        assert!(path.len() >= 2);
    }

    #[test]
    fn tiled_no_path_unloaded_tile() {
        let mut tiled = TiledNavMesh::new(10.0);
        tiled.load_tile(TileCoord::new(0, 0), make_tile_mesh());
        // Goal is in unloaded tile
        assert!(
            tiled
                .find_path(Vec2::new(5.0, 5.0), Vec2::new(25.0, 5.0))
                .is_none()
        );
    }

    #[test]
    fn tiled_serde_roundtrip() {
        let mut tiled = TiledNavMesh::new(10.0);
        tiled.load_tile(TileCoord::new(0, 0), make_tile_mesh());
        let json = serde_json::to_string(&tiled).unwrap();
        let deserialized: TiledNavMesh = serde_json::from_str(&json).unwrap();
        assert_eq!(deserialized.tile_count(), 1);
    }

    #[test]
    fn tile_coord_new() {
        let tc = TileCoord::new(3, -2);
        assert_eq!(tc.x, 3);
        assert_eq!(tc.y, -2);
    }

    #[test]
    fn rebake_tile() {
        let mut tiled = TiledNavMesh::new(10.0);
        tiled.load_tile(TileCoord::new(0, 0), make_tile_mesh());
        assert_eq!(
            tiled
                .get_tile(TileCoord::new(0, 0))
                .unwrap()
                .mesh
                .poly_count(),
            1
        );

        // Rebake with a triangle
        tiled.rebake_tile(
            TileCoord::new(0, 0),
            &[Vec2::ZERO, Vec2::new(10.0, 0.0), Vec2::new(5.0, 10.0)],
        );
        assert!(tiled.has_tile(TileCoord::new(0, 0)));
        // Poly count may differ after rebake
        assert!(
            tiled
                .get_tile(TileCoord::new(0, 0))
                .unwrap()
                .mesh
                .poly_count()
                >= 1
        );
    }

    #[test]
    fn rebuild_tile_connections_local() {
        let mut tiled = TiledNavMesh::new(10.0);
        tiled.load_tile(TileCoord::new(0, 0), make_tile_mesh());
        tiled.load_tile(TileCoord::new(1, 0), make_tile_mesh());
        tiled.rebuild_connections();

        // Rebake one tile — connections should still work
        tiled.rebake_tile(
            TileCoord::new(0, 0),
            &[
                Vec2::ZERO,
                Vec2::new(10.0, 0.0),
                Vec2::new(10.0, 10.0),
                Vec2::new(0.0, 10.0),
            ],
        );

        let path = tiled.find_path(Vec2::new(5.0, 5.0), Vec2::new(15.0, 5.0));
        assert!(path.is_some());
    }
}
