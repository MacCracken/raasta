//! Hierarchical Pathfinding A* (HPA*).
//!
//! Divides a [`NavGrid`] into rectangular clusters, identifies entrances
//! between adjacent clusters, and builds an abstract graph for fast
//! long-range pathfinding.

use serde::{Deserialize, Serialize};

use crate::grid::{GridPos, NavGrid};

/// Identifier for a cluster in the grid hierarchy.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct ClusterId {
    /// Cluster column (x / cluster_size).
    pub cx: u32,
    /// Cluster row (y / cluster_size).
    pub cy: u32,
}

/// An entrance between two adjacent clusters.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Entrance {
    /// Cells on the border — each pair is (cell_in_cluster_a, cell_in_cluster_b).
    pub cells: Vec<(GridPos, GridPos)>,
    /// The two clusters this entrance connects.
    pub cluster_a: ClusterId,
    pub cluster_b: ClusterId,
}

/// Grid clustering for HPA*.
#[derive(Debug, Clone)]
pub struct GridClusters {
    cluster_size: u32,
    clusters_wide: u32,
    clusters_high: u32,
    entrances: Vec<Entrance>,
}

impl GridClusters {
    /// Build clusters and identify entrances for the given grid.
    ///
    /// `cluster_size` is the width/height of each cluster in cells (e.g., 8 or 16).
    #[must_use]
    pub fn build(grid: &NavGrid, cluster_size: u32) -> Self {
        let clusters_wide = (grid.width() as u32).div_ceil(cluster_size);
        let clusters_high = (grid.height() as u32).div_ceil(cluster_size);

        let mut gc = Self {
            cluster_size,
            clusters_wide,
            clusters_high,
            entrances: Vec::new(),
        };

        gc.find_entrances(grid);
        gc
    }

    /// Which cluster a grid position belongs to.
    #[must_use]
    pub fn cluster_of(&self, pos: GridPos) -> ClusterId {
        ClusterId {
            cx: (pos.x as u32) / self.cluster_size,
            cy: (pos.y as u32) / self.cluster_size,
        }
    }

    /// Cluster size in cells.
    #[must_use]
    pub fn cluster_size(&self) -> u32 {
        self.cluster_size
    }

    /// Number of clusters wide.
    #[must_use]
    pub fn clusters_wide(&self) -> u32 {
        self.clusters_wide
    }

    /// Number of clusters high.
    #[must_use]
    pub fn clusters_high(&self) -> u32 {
        self.clusters_high
    }

    /// All identified entrances.
    #[must_use]
    pub fn entrances(&self) -> &[Entrance] {
        &self.entrances
    }

    /// Bounding box of a cluster in grid coordinates (inclusive).
    #[must_use]
    pub fn cluster_bounds(&self, id: ClusterId, grid: &NavGrid) -> (GridPos, GridPos) {
        let min_x = (id.cx * self.cluster_size) as i32;
        let min_y = (id.cy * self.cluster_size) as i32;
        let max_x = ((id.cx + 1) * self.cluster_size) as i32 - 1;
        let max_y = ((id.cy + 1) * self.cluster_size) as i32 - 1;
        (
            GridPos::new(min_x, min_y),
            GridPos::new(
                max_x.min(grid.width() as i32 - 1),
                max_y.min(grid.height() as i32 - 1),
            ),
        )
    }

    /// Scan all adjacent cluster pairs and identify entrances.
    fn find_entrances(&mut self, grid: &NavGrid) {
        // Horizontal borders (between cluster (cx, cy) and (cx+1, cy))
        for cy in 0..self.clusters_high {
            for cx in 0..(self.clusters_wide - 1) {
                let border_x = ((cx + 1) * self.cluster_size) as i32;
                let y_start = (cy * self.cluster_size) as i32;
                let y_end =
                    (((cy + 1) * self.cluster_size) as i32 - 1).min(grid.height() as i32 - 1);

                let cluster_a = ClusterId { cx, cy };
                let cluster_b = ClusterId { cx: cx + 1, cy };

                self.scan_border(
                    grid,
                    border_x - 1,
                    border_x,
                    y_start,
                    y_end,
                    true,
                    cluster_a,
                    cluster_b,
                );
            }
        }

        // Vertical borders (between cluster (cx, cy) and (cx, cy+1))
        for cx in 0..self.clusters_wide {
            for cy in 0..(self.clusters_high - 1) {
                let border_y = ((cy + 1) * self.cluster_size) as i32;
                let x_start = (cx * self.cluster_size) as i32;
                let x_end =
                    (((cx + 1) * self.cluster_size) as i32 - 1).min(grid.width() as i32 - 1);

                let cluster_a = ClusterId { cx, cy };
                let cluster_b = ClusterId { cx, cy: cy + 1 };

                self.scan_border(
                    grid,
                    border_y - 1,
                    border_y,
                    x_start,
                    x_end,
                    false,
                    cluster_a,
                    cluster_b,
                );
            }
        }
    }

    /// Scan a border between two clusters and create entrances from contiguous
    /// runs of walkable cell pairs.
    #[allow(clippy::too_many_arguments)]
    fn scan_border(
        &mut self,
        grid: &NavGrid,
        a_coord: i32,
        b_coord: i32,
        range_start: i32,
        range_end: i32,
        horizontal: bool,
        cluster_a: ClusterId,
        cluster_b: ClusterId,
    ) {
        let mut current_entrance: Vec<(GridPos, GridPos)> = Vec::new();

        for i in range_start..=range_end {
            let (ax, ay, bx, by) = if horizontal {
                (a_coord, i, b_coord, i)
            } else {
                (i, a_coord, i, b_coord)
            };

            if grid.is_walkable(ax, ay) && grid.is_walkable(bx, by) {
                current_entrance.push((GridPos::new(ax, ay), GridPos::new(bx, by)));
            } else if !current_entrance.is_empty() {
                self.entrances.push(Entrance {
                    cells: std::mem::take(&mut current_entrance),
                    cluster_a,
                    cluster_b,
                });
            }
        }

        if !current_entrance.is_empty() {
            self.entrances.push(Entrance {
                cells: current_entrance,
                cluster_a,
                cluster_b,
            });
        }
    }
}

/// A node in the abstract graph — represents one side of an entrance.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct AbstractNodeId(pub u32);

/// An edge in the abstract graph.
#[derive(Debug, Clone)]
struct AbstractEdge {
    to: AbstractNodeId,
    cost: f32,
}

/// Abstract graph for hierarchical pathfinding.
///
/// Nodes are representative cells at cluster entrances.
/// Edges connect entrance pairs (inter-cluster, cost = cell distance)
/// and entrances within the same cluster (intra-cluster, cost = A* path cost).
#[derive(Debug, Clone)]
pub struct AbstractGraph {
    /// Position of each node.
    node_positions: Vec<GridPos>,
    /// Cluster each node belongs to.
    node_clusters: Vec<ClusterId>,
    /// Adjacency list.
    edges: Vec<Vec<AbstractEdge>>,
    /// Cached intra-cluster paths: (node_a, node_b) → grid path.
    /// Keyed with (min_id, max_id) for symmetry.
    path_cache: std::collections::HashMap<(u32, u32), Vec<GridPos>>,
}

impl AbstractGraph {
    /// Build the abstract graph from grid clusters.
    #[must_use]
    pub fn build(grid: &NavGrid, clusters: &GridClusters) -> Self {
        let mut graph = Self {
            node_positions: Vec::new(),
            node_clusters: Vec::new(),
            edges: Vec::new(),
            path_cache: std::collections::HashMap::new(),
        };

        // For each entrance, pick a representative cell on each side
        // and create two nodes + an inter-cluster edge.
        // We use the middle cell pair of each entrance.
        let mut cluster_nodes: std::collections::HashMap<ClusterId, Vec<AbstractNodeId>> =
            std::collections::HashMap::new();

        for entrance in clusters.entrances() {
            let mid = entrance.cells.len() / 2;
            let (cell_a, cell_b) = entrance.cells[mid];

            let id_a = graph.add_node(cell_a, entrance.cluster_a);
            let id_b = graph.add_node(cell_b, entrance.cluster_b);

            // Inter-cluster edge (adjacent cells, cost = 1.0)
            graph.add_edge(id_a, id_b, 1.0);
            graph.add_edge(id_b, id_a, 1.0);

            cluster_nodes
                .entry(entrance.cluster_a)
                .or_default()
                .push(id_a);
            cluster_nodes
                .entry(entrance.cluster_b)
                .or_default()
                .push(id_b);
        }

        // Intra-cluster edges: for each cluster, connect all its entrance nodes
        // via A* within the cluster bounds. Cache the paths for O(1) refinement.
        for nodes in cluster_nodes.values() {
            for i in 0..nodes.len() {
                for j in (i + 1)..nodes.len() {
                    let pos_a = graph.node_positions[nodes[i].0 as usize];
                    let pos_b = graph.node_positions[nodes[j].0 as usize];

                    if let Some(path) = grid.find_path(pos_a, pos_b) {
                        let cost: f32 = path.windows(2).map(|w| w[0].octile_distance(w[1])).sum();
                        graph.add_edge(nodes[i], nodes[j], cost);
                        graph.add_edge(nodes[j], nodes[i], cost);

                        let key = Self::cache_key(nodes[i], nodes[j]);
                        graph.path_cache.insert(key, path);
                    }
                }
            }
        }

        graph
    }

    /// Number of nodes.
    #[must_use]
    pub fn node_count(&self) -> usize {
        self.node_positions.len()
    }

    /// Number of edges (directed).
    #[must_use]
    pub fn edge_count(&self) -> usize {
        self.edges.iter().map(|e| e.len()).sum()
    }

    /// Position of a node.
    #[must_use]
    pub fn node_position(&self, id: AbstractNodeId) -> GridPos {
        self.node_positions[id.0 as usize]
    }

    /// Cluster of a node.
    #[must_use]
    pub fn node_cluster(&self, id: AbstractNodeId) -> ClusterId {
        self.node_clusters[id.0 as usize]
    }

    fn add_node(&mut self, pos: GridPos, cluster: ClusterId) -> AbstractNodeId {
        let id = AbstractNodeId(self.node_positions.len() as u32);
        self.node_positions.push(pos);
        self.node_clusters.push(cluster);
        self.edges.push(Vec::new());
        id
    }

    fn add_edge(&mut self, from: AbstractNodeId, to: AbstractNodeId, cost: f32) {
        self.edges[from.0 as usize].push(AbstractEdge { to, cost });
    }

    /// Symmetric cache key — always (min, max).
    fn cache_key(a: AbstractNodeId, b: AbstractNodeId) -> (u32, u32) {
        if a.0 <= b.0 { (a.0, b.0) } else { (b.0, a.0) }
    }

    /// Find a path on the abstract graph using A*.
    ///
    /// Temporarily inserts `start` and `goal` as nodes, connects them to
    /// entrance nodes in their respective clusters, runs A* on the abstract
    /// graph, then refines through intra-cluster A* paths on the base grid.
    ///
    /// Returns `None` if no path exists.
    #[must_use]
    pub fn find_path(
        &self,
        grid: &NavGrid,
        clusters: &GridClusters,
        start: GridPos,
        goal: GridPos,
    ) -> Option<Vec<GridPos>> {
        if !grid.is_walkable(start.x, start.y) || !grid.is_walkable(goal.x, goal.y) {
            return None;
        }
        if start == goal {
            return Some(vec![start]);
        }

        let start_cluster = clusters.cluster_of(start);
        let goal_cluster = clusters.cluster_of(goal);

        // Same cluster — just use base A*
        if start_cluster == goal_cluster {
            return grid.find_path(start, goal);
        }

        // Build temporary extended graph with start/goal inserted
        let n = self.node_positions.len();
        let start_id = AbstractNodeId(n as u32);
        let goal_id = AbstractNodeId((n + 1) as u32);

        // Collect edges for temporary nodes
        let mut start_edges: Vec<AbstractEdge> = Vec::new();
        let mut goal_edges_incoming: Vec<(AbstractNodeId, f32)> = Vec::new();

        for i in 0..n {
            let node_id = AbstractNodeId(i as u32);
            let node_cluster = self.node_clusters[i];
            let node_pos = self.node_positions[i];

            // Connect start to nodes in its cluster
            if node_cluster == start_cluster
                && let Some(path) = grid.find_path(start, node_pos)
            {
                let cost: f32 = path.windows(2).map(|w| w[0].octile_distance(w[1])).sum();
                start_edges.push(AbstractEdge { to: node_id, cost });
            }

            // Connect nodes in goal's cluster to goal
            if node_cluster == goal_cluster
                && let Some(path) = grid.find_path(node_pos, goal)
            {
                let cost: f32 = path.windows(2).map(|w| w[0].octile_distance(w[1])).sum();
                goal_edges_incoming.push((node_id, cost));
            }
        }

        // A* on abstract graph with temporary nodes
        let total_nodes = n + 2;
        let mut g_score = vec![f32::INFINITY; total_nodes];
        let mut came_from: Vec<Option<u32>> = vec![None; total_nodes];
        let mut closed = vec![false; total_nodes];

        g_score[start_id.0 as usize] = 0.0;
        let mut open = std::collections::BinaryHeap::new();

        let h = start.octile_distance(goal);
        open.push(std::cmp::Reverse((OrderedFloat(h), start_id.0)));

        while let Some(std::cmp::Reverse((OrderedFloat(_f), current_raw))) = open.pop() {
            let current = current_raw as usize;

            if current_raw == goal_id.0 {
                // Reconstruct abstract path
                let abstract_path = self.reconstruct_abstract_path(&came_from, start_id, goal_id);
                return self.refine_path(grid, &abstract_path, start, goal);
            }

            if closed[current] {
                continue;
            }
            closed[current] = true;

            // Get edges for current node
            let edges: &[AbstractEdge] = if current_raw == start_id.0 {
                &start_edges
            } else if current_raw == goal_id.0 {
                &[]
            } else {
                &self.edges[current]
            };

            for edge in edges {
                let next = edge.to.0 as usize;
                if closed[next] {
                    continue;
                }
                let tentative_g = g_score[current] + edge.cost;
                if tentative_g < g_score[next] {
                    g_score[next] = tentative_g;
                    came_from[next] = Some(current_raw);
                    let next_pos = if edge.to == goal_id {
                        goal
                    } else {
                        self.node_positions[next]
                    };
                    let h = next_pos.octile_distance(goal);
                    open.push(std::cmp::Reverse((
                        OrderedFloat(tentative_g + h),
                        edge.to.0,
                    )));
                }
            }

            // Also check goal edges from current node
            if current_raw != start_id.0 && current_raw != goal_id.0 {
                for &(node_id, cost) in &goal_edges_incoming {
                    if node_id.0 as usize == current {
                        let next = goal_id.0 as usize;
                        if !closed[next] {
                            let tentative_g = g_score[current] + cost;
                            if tentative_g < g_score[next] {
                                g_score[next] = tentative_g;
                                came_from[next] = Some(current_raw);
                                let h = goal.octile_distance(goal); // 0
                                open.push(std::cmp::Reverse((
                                    OrderedFloat(tentative_g + h),
                                    goal_id.0,
                                )));
                            }
                        }
                    }
                }
            }
        }

        None
    }

    fn reconstruct_abstract_path(
        &self,
        came_from: &[Option<u32>],
        start_id: AbstractNodeId,
        goal_id: AbstractNodeId,
    ) -> Vec<AbstractNodeId> {
        let mut path = Vec::new();
        let mut current = goal_id.0;
        loop {
            path.push(AbstractNodeId(current));
            if current == start_id.0 {
                break;
            }
            match came_from[current as usize] {
                Some(prev) => current = prev,
                None => break,
            }
        }
        path.reverse();
        path
    }

    /// Refine an abstract path into a full grid path.
    ///
    /// Uses cached intra-cluster paths for known node pairs (O(1) lookup).
    /// Only calls A* for start/goal connections to entrance nodes.
    fn refine_path(
        &self,
        grid: &NavGrid,
        abstract_path: &[AbstractNodeId],
        start: GridPos,
        goal: GridPos,
    ) -> Option<Vec<GridPos>> {
        if abstract_path.len() < 2 {
            return grid.find_path(start, goal);
        }

        let n = self.node_positions.len() as u32;
        let mut full_path: Vec<GridPos> = Vec::new();

        for w in abstract_path.windows(2) {
            let is_temp_a = w[0].0 >= n;
            let is_temp_b = w[1].0 >= n;

            let segment = if !is_temp_a && !is_temp_b {
                // Both are permanent nodes — check cache first
                let key = Self::cache_key(w[0], w[1]);
                if let Some(cached) = self.path_cache.get(&key) {
                    // Cache stores A→B; if we need B→A, reverse it
                    if key.0 == w[0].0 {
                        cached.clone()
                    } else {
                        let mut rev = cached.clone();
                        rev.reverse();
                        rev
                    }
                } else {
                    // Inter-cluster edge (adjacent cells) — direct step
                    let from_pos = self.node_positions[w[0].0 as usize];
                    let to_pos = self.node_positions[w[1].0 as usize];
                    vec![from_pos, to_pos]
                }
            } else {
                // Temporary node (start or goal) — A* on base grid
                let from_pos = if is_temp_a {
                    start
                } else {
                    self.node_positions[w[0].0 as usize]
                };
                let to_pos = if is_temp_b {
                    goal
                } else {
                    self.node_positions[w[1].0 as usize]
                };
                grid.find_path(from_pos, to_pos)?
            };

            if full_path.is_empty() {
                full_path.extend_from_slice(&segment);
            } else if segment.len() > 1 {
                full_path.extend_from_slice(&segment[1..]);
            }
        }

        Some(full_path)
    }
}

/// Wrapper for f32 ordering in BinaryHeap (min-heap via Reverse).
#[derive(Debug, Clone, Copy, PartialEq)]
struct OrderedFloat(f32);

impl Eq for OrderedFloat {}

impl PartialOrd for OrderedFloat {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for OrderedFloat {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.0
            .partial_cmp(&other.0)
            .unwrap_or(std::cmp::Ordering::Equal)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cluster_of_basic() {
        let grid = NavGrid::new(20, 20, 1.0);
        let gc = GridClusters::build(&grid, 8);

        assert_eq!(
            gc.cluster_of(GridPos::new(0, 0)),
            ClusterId { cx: 0, cy: 0 }
        );
        assert_eq!(
            gc.cluster_of(GridPos::new(7, 7)),
            ClusterId { cx: 0, cy: 0 }
        );
        assert_eq!(
            gc.cluster_of(GridPos::new(8, 0)),
            ClusterId { cx: 1, cy: 0 }
        );
        assert_eq!(
            gc.cluster_of(GridPos::new(16, 16)),
            ClusterId { cx: 2, cy: 2 }
        );
    }

    #[test]
    fn cluster_dimensions() {
        let grid = NavGrid::new(20, 20, 1.0);
        let gc = GridClusters::build(&grid, 8);

        assert_eq!(gc.cluster_size(), 8);
        assert_eq!(gc.clusters_wide(), 3); // ceil(20/8) = 3
        assert_eq!(gc.clusters_high(), 3);
    }

    #[test]
    fn entrances_open_grid() {
        let grid = NavGrid::new(16, 16, 1.0);
        let gc = GridClusters::build(&grid, 8);

        // 2x2 clusters, open grid: should have entrances on all internal borders
        assert!(!gc.entrances().is_empty());

        // Each border should produce exactly 1 entrance (fully open = 1 contiguous run)
        // 1 horizontal border + 1 vertical border = 2 borders total for 2x2
        // Horizontal: cluster(0,0)↔(1,0) and cluster(0,1)↔(1,1) = 2
        // Vertical: cluster(0,0)↔(0,1) and cluster(1,0)↔(1,1) = 2
        assert_eq!(gc.entrances().len(), 4);
    }

    #[test]
    fn entrances_with_wall() {
        let mut grid = NavGrid::new(16, 16, 1.0);
        // Block half the horizontal border between cluster (0,0) and (1,0)
        for y in 0..4 {
            grid.set_walkable(7, y, false); // left side of border
        }

        let gc = GridClusters::build(&grid, 8);

        // The horizontal border between (0,0)↔(1,0) should be split:
        // y=0..3 blocked, y=4..7 open → 1 entrance (the open part)
        let border_entrances: Vec<_> = gc
            .entrances()
            .iter()
            .filter(|e| {
                e.cluster_a == ClusterId { cx: 0, cy: 0 }
                    && e.cluster_b == ClusterId { cx: 1, cy: 0 }
            })
            .collect();

        assert_eq!(border_entrances.len(), 1);
        // Should have 4 cell pairs (y=4,5,6,7)
        assert_eq!(border_entrances[0].cells.len(), 4);
    }

    #[test]
    fn entrances_fully_blocked_border() {
        let mut grid = NavGrid::new(16, 16, 1.0);
        // Block entire horizontal border between cluster (0,0) and (1,0)
        for y in 0..8 {
            grid.set_walkable(7, y, false);
        }

        let gc = GridClusters::build(&grid, 8);

        let border_entrances: Vec<_> = gc
            .entrances()
            .iter()
            .filter(|e| {
                e.cluster_a == ClusterId { cx: 0, cy: 0 }
                    && e.cluster_b == ClusterId { cx: 1, cy: 0 }
            })
            .collect();

        // Fully blocked — no entrances
        assert!(border_entrances.is_empty());
    }

    #[test]
    fn cluster_bounds() {
        let grid = NavGrid::new(20, 20, 1.0);
        let gc = GridClusters::build(&grid, 8);

        let (min, max) = gc.cluster_bounds(ClusterId { cx: 0, cy: 0 }, &grid);
        assert_eq!(min, GridPos::new(0, 0));
        assert_eq!(max, GridPos::new(7, 7));

        // Last cluster may be smaller
        let (min, max) = gc.cluster_bounds(ClusterId { cx: 2, cy: 2 }, &grid);
        assert_eq!(min, GridPos::new(16, 16));
        assert_eq!(max, GridPos::new(19, 19));
    }

    #[test]
    fn entrance_cells_are_adjacent() {
        let grid = NavGrid::new(16, 16, 1.0);
        let gc = GridClusters::build(&grid, 8);

        for entrance in gc.entrances() {
            for (a, b) in &entrance.cells {
                let dx = (a.x - b.x).abs();
                let dy = (a.y - b.y).abs();
                assert_eq!(
                    dx + dy,
                    1,
                    "entrance cells must be adjacent: ({},{}) ({},{})",
                    a.x,
                    a.y,
                    b.x,
                    b.y
                );
            }
        }
    }

    #[test]
    fn single_cluster_no_entrances() {
        let grid = NavGrid::new(5, 5, 1.0);
        let gc = GridClusters::build(&grid, 8);

        assert_eq!(gc.clusters_wide(), 1);
        assert_eq!(gc.clusters_high(), 1);
        assert!(gc.entrances().is_empty());
    }

    // --- Abstract graph tests ---

    #[test]
    fn abstract_graph_open_grid() {
        let grid = NavGrid::new(16, 16, 1.0);
        let clusters = GridClusters::build(&grid, 8);
        let graph = AbstractGraph::build(&grid, &clusters);

        // 4 entrances, each produces 2 nodes = 8 nodes
        assert_eq!(graph.node_count(), 8);
        // Each entrance has 2 inter-cluster edges (bidirectional) = 8
        // Plus intra-cluster edges between nodes in the same cluster
        assert!(graph.edge_count() > 8);
    }

    #[test]
    fn abstract_graph_nodes_are_walkable() {
        let grid = NavGrid::new(16, 16, 1.0);
        let clusters = GridClusters::build(&grid, 8);
        let graph = AbstractGraph::build(&grid, &clusters);

        for i in 0..graph.node_count() {
            let pos = graph.node_position(AbstractNodeId(i as u32));
            assert!(
                grid.is_walkable(pos.x, pos.y),
                "abstract node at ({},{}) is not walkable",
                pos.x,
                pos.y
            );
        }
    }

    #[test]
    fn abstract_graph_with_wall() {
        let mut grid = NavGrid::new(16, 16, 1.0);
        // Block entire horizontal border between (0,0)↔(1,0)
        for y in 0..8 {
            grid.set_walkable(7, y, false);
        }
        let clusters = GridClusters::build(&grid, 8);
        let graph = AbstractGraph::build(&grid, &clusters);

        // Fewer nodes because one border is fully blocked
        assert!(graph.node_count() < 8);
    }

    #[test]
    fn abstract_graph_intra_cluster_edges() {
        let grid = NavGrid::new(24, 8, 1.0);
        let clusters = GridClusters::build(&grid, 8);
        let graph = AbstractGraph::build(&grid, &clusters);

        // 3x1 clusters: 2 horizontal borders, each with 1 entrance
        // Cluster (1,0) has 2 entrance nodes (left and right border)
        // They should be connected with an intra-cluster edge
        assert!(graph.edge_count() > 4); // more than just inter-cluster
    }

    #[test]
    fn abstract_graph_empty_grid() {
        let grid = NavGrid::new(5, 5, 1.0);
        let clusters = GridClusters::build(&grid, 8);
        let graph = AbstractGraph::build(&grid, &clusters);

        // Single cluster — no entrances, no nodes
        assert_eq!(graph.node_count(), 0);
        assert_eq!(graph.edge_count(), 0);
    }

    // --- HPA* search tests ---

    #[test]
    fn hpa_cross_cluster_path() {
        let grid = NavGrid::new(32, 32, 1.0);
        let clusters = GridClusters::build(&grid, 8);
        let graph = AbstractGraph::build(&grid, &clusters);

        let path = graph.find_path(&grid, &clusters, GridPos::new(0, 0), GridPos::new(31, 31));
        assert!(path.is_some());
        let path = path.unwrap();
        assert_eq!(*path.first().unwrap(), GridPos::new(0, 0));
        assert_eq!(*path.last().unwrap(), GridPos::new(31, 31));

        // Path should be contiguous
        for w in path.windows(2) {
            let dx = (w[1].x - w[0].x).abs();
            let dy = (w[1].y - w[0].y).abs();
            assert!(dx <= 1 && dy <= 1 && (dx + dy) > 0);
        }
    }

    #[test]
    fn hpa_same_cluster_path() {
        let grid = NavGrid::new(32, 32, 1.0);
        let clusters = GridClusters::build(&grid, 8);
        let graph = AbstractGraph::build(&grid, &clusters);

        // Both in cluster (0,0)
        let path = graph.find_path(&grid, &clusters, GridPos::new(1, 1), GridPos::new(5, 5));
        assert!(path.is_some());
    }

    #[test]
    fn hpa_same_cell() {
        let grid = NavGrid::new(16, 16, 1.0);
        let clusters = GridClusters::build(&grid, 8);
        let graph = AbstractGraph::build(&grid, &clusters);

        let path = graph.find_path(&grid, &clusters, GridPos::new(3, 3), GridPos::new(3, 3));
        assert_eq!(path, Some(vec![GridPos::new(3, 3)]));
    }

    #[test]
    fn hpa_blocked_start() {
        let mut grid = NavGrid::new(16, 16, 1.0);
        grid.set_walkable(0, 0, false);
        let clusters = GridClusters::build(&grid, 8);
        let graph = AbstractGraph::build(&grid, &clusters);

        assert!(
            graph
                .find_path(&grid, &clusters, GridPos::new(0, 0), GridPos::new(15, 15))
                .is_none()
        );
    }

    #[test]
    fn hpa_with_wall() {
        let mut grid = NavGrid::new(32, 32, 1.0);
        // Wall across the middle with a gap
        for y in 0..30 {
            grid.set_walkable(16, y, false);
        }
        let clusters = GridClusters::build(&grid, 8);
        let graph = AbstractGraph::build(&grid, &clusters);

        let path = graph.find_path(&grid, &clusters, GridPos::new(0, 0), GridPos::new(31, 31));
        assert!(path.is_some());
        let path = path.unwrap();
        // All cells should be walkable
        for p in &path {
            assert!(grid.is_walkable(p.x, p.y));
        }
    }

    #[test]
    fn hpa_matches_astar_reachability() {
        let mut grid = NavGrid::new(32, 32, 1.0);
        // Complete wall — no path
        for y in 0..32 {
            grid.set_walkable(16, y, false);
        }
        let clusters = GridClusters::build(&grid, 8);
        let graph = AbstractGraph::build(&grid, &clusters);

        let astar = grid.find_path(GridPos::new(0, 0), GridPos::new(31, 31));
        let hpa = graph.find_path(&grid, &clusters, GridPos::new(0, 0), GridPos::new(31, 31));
        assert_eq!(astar.is_some(), hpa.is_some());
    }
}
