//! Multi-layer NavMesh — overlapping navigation surfaces for bridges and buildings.

use std::collections::HashMap;

use hisab::Vec2;
use serde::{Deserialize, Serialize};

use crate::mesh::{NavMesh, NavPolyId};

#[cfg(feature = "logging")]
use tracing::instrument;

/// Identifies a layer in the multi-layer navmesh.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct LayerId(pub u32);

/// A polygon ID scoped to a specific layer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct LayeredPolyId {
    pub layer: LayerId,
    pub poly: NavPolyId,
}

/// Connection between layers (stairs, elevators, ramps).
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct LayerConnection {
    pub from: LayeredPolyId,
    pub to: LayeredPolyId,
    pub cost: f32,
    pub bidirectional: bool,
}

/// Multi-layer navmesh — multiple overlapping navigation surfaces.
///
/// Each layer is an independent `NavMesh` (e.g., ground floor, second floor,
/// bridge surface). Layers are connected by explicit connections (stairs, etc.).
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct MultiLayerNavMesh {
    layers: HashMap<LayerId, NavMesh>,
    connections: Vec<LayerConnection>,
}

impl MultiLayerNavMesh {
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a navigation layer.
    pub fn add_layer(&mut self, id: LayerId, mesh: NavMesh) {
        self.layers.insert(id, mesh);
    }

    /// Remove a layer.
    pub fn remove_layer(&mut self, id: LayerId) -> bool {
        if self.layers.remove(&id).is_some() {
            self.connections
                .retain(|c| c.from.layer != id && c.to.layer != id);
            true
        } else {
            false
        }
    }

    /// Get a layer's navmesh.
    #[must_use]
    pub fn get_layer(&self, id: LayerId) -> Option<&NavMesh> {
        self.layers.get(&id)
    }

    /// Number of layers.
    #[must_use]
    pub fn layer_count(&self) -> usize {
        self.layers.len()
    }

    /// Add a connection between layers.
    pub fn add_connection(&mut self, conn: LayerConnection) {
        self.connections.push(conn);
    }

    /// Get all connections.
    #[must_use]
    pub fn connections(&self) -> &[LayerConnection] {
        &self.connections
    }

    /// Find a path across layers.
    ///
    /// Returns a sequence of `LayeredPolyId`s from start to goal.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    #[must_use]
    pub fn find_path(
        &self,
        start_layer: LayerId,
        start_pos: Vec2,
        goal_layer: LayerId,
        goal_pos: Vec2,
    ) -> Option<Vec<LayeredPolyId>> {
        // Same layer — use local pathfinding
        if start_layer == goal_layer {
            let mesh = self.layers.get(&start_layer)?;
            let poly_path = mesh.find_path(start_pos, goal_pos)?;
            return Some(
                poly_path
                    .into_iter()
                    .map(|p| LayeredPolyId {
                        layer: start_layer,
                        poly: p,
                    })
                    .collect(),
            );
        }

        // Cross-layer A*
        // Build node list
        use std::cmp::Ordering;
        use std::collections::BinaryHeap;

        let mut nodes: Vec<LayeredPolyId> = Vec::new();
        let mut positions: Vec<Vec2> = Vec::new();
        let mut node_map: HashMap<LayeredPolyId, usize> = HashMap::new();

        for (&lid, mesh) in &self.layers {
            for poly in mesh.polys() {
                let lpid = LayeredPolyId {
                    layer: lid,
                    poly: poly.id,
                };
                let idx = nodes.len();
                node_map.insert(lpid, idx);
                nodes.push(lpid);
                positions.push(poly.centroid());
            }
        }

        let n = nodes.len();
        if n == 0 {
            return None;
        }

        // Find start/goal nodes
        let start_mesh = self.layers.get(&start_layer)?;
        let start_poly = start_mesh.find_poly_at(start_pos)?;
        let start_id = LayeredPolyId {
            layer: start_layer,
            poly: start_poly,
        };
        let start_idx = *node_map.get(&start_id)?;

        let goal_mesh = self.layers.get(&goal_layer)?;
        let goal_poly = goal_mesh.find_poly_at(goal_pos)?;
        let goal_id = LayeredPolyId {
            layer: goal_layer,
            poly: goal_poly,
        };
        let goal_idx = *node_map.get(&goal_id)?;

        // Build adjacency
        let mut adj: Vec<Vec<(usize, f32)>> = vec![Vec::new(); n];

        // Local neighbors
        for (&lid, mesh) in &self.layers {
            for poly in mesh.polys() {
                let from = LayeredPolyId {
                    layer: lid,
                    poly: poly.id,
                };
                let fi = node_map[&from];
                for &nid in &poly.neighbors {
                    let to = LayeredPolyId {
                        layer: lid,
                        poly: nid,
                    };
                    if let Some(&ti) = node_map.get(&to) {
                        let cost = positions[fi].distance(positions[ti]) * poly.cost;
                        adj[fi].push((ti, cost));
                    }
                }
            }
        }

        // Layer connections
        for conn in &self.connections {
            if let (Some(&fi), Some(&ti)) = (node_map.get(&conn.from), node_map.get(&conn.to)) {
                adj[fi].push((ti, conn.cost));
                if conn.bidirectional {
                    adj[ti].push((fi, conn.cost));
                }
            }
        }

        // A*
        #[derive(Clone, Copy)]
        struct N {
            idx: usize,
            f: f32,
        }
        impl PartialEq for N {
            fn eq(&self, o: &Self) -> bool {
                self.f == o.f
            }
        }
        impl Eq for N {}
        impl PartialOrd for N {
            fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
                Some(self.cmp(o))
            }
        }
        impl Ord for N {
            fn cmp(&self, o: &Self) -> Ordering {
                o.f.partial_cmp(&self.f).unwrap_or(Ordering::Equal)
            }
        }

        let mut g = vec![f32::INFINITY; n];
        let mut came: Vec<Option<usize>> = vec![None; n];
        let mut closed = vec![false; n];
        let mut open = BinaryHeap::new();

        g[start_idx] = 0.0;
        open.push(N {
            idx: start_idx,
            f: positions[start_idx].distance(positions[goal_idx]),
        });

        while let Some(cur) = open.pop() {
            if cur.idx == goal_idx {
                let mut path = Vec::new();
                let mut c = goal_idx;
                loop {
                    path.push(nodes[c]);
                    match came[c] {
                        Some(p) => c = p,
                        None => break,
                    }
                }
                path.reverse();
                return Some(path);
            }
            if closed[cur.idx] {
                continue;
            }
            closed[cur.idx] = true;

            for &(ni, cost) in &adj[cur.idx] {
                if closed[ni] {
                    continue;
                }
                let tg = g[cur.idx] + cost;
                if tg < g[ni] {
                    g[ni] = tg;
                    came[ni] = Some(cur.idx);
                    open.push(N {
                        idx: ni,
                        f: tg + positions[ni].distance(positions[goal_idx]),
                    });
                }
            }
        }

        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::{NavMesh, NavPoly, NavPolyId};

    /// Build a simple 2-polygon navmesh for testing.
    fn make_simple_mesh() -> NavMesh {
        let mut mesh = NavMesh::new();
        mesh.add_poly(NavPoly {
            id: NavPolyId(0),
            vertices: vec![
                Vec2::new(0.0, 0.0),
                Vec2::new(5.0, 0.0),
                Vec2::new(5.0, 5.0),
                Vec2::new(0.0, 5.0),
            ],
            neighbors: vec![NavPolyId(1)],
            cost: 1.0,
            layer: 0,
        });
        mesh.add_poly(NavPoly {
            id: NavPolyId(1),
            vertices: vec![
                Vec2::new(5.0, 0.0),
                Vec2::new(10.0, 0.0),
                Vec2::new(10.0, 5.0),
                Vec2::new(5.0, 5.0),
            ],
            neighbors: vec![NavPolyId(0)],
            cost: 1.0,
            layer: 0,
        });
        mesh
    }

    #[test]
    fn same_layer_path() {
        let mut ml = MultiLayerNavMesh::new();
        ml.add_layer(LayerId(0), make_simple_mesh());

        let path = ml.find_path(
            LayerId(0),
            Vec2::new(1.0, 1.0),
            LayerId(0),
            Vec2::new(9.0, 1.0),
        );
        assert!(path.is_some());
        let path = path.unwrap();
        assert_eq!(path.len(), 2);
        assert!(path.iter().all(|lp| lp.layer == LayerId(0)));
    }

    #[test]
    fn cross_layer_path_via_connection() {
        let mut ml = MultiLayerNavMesh::new();
        ml.add_layer(LayerId(0), make_simple_mesh());
        ml.add_layer(LayerId(1), make_simple_mesh());

        // Connect layer 0 poly 1 to layer 1 poly 0 (like stairs)
        ml.add_connection(LayerConnection {
            from: LayeredPolyId {
                layer: LayerId(0),
                poly: NavPolyId(1),
            },
            to: LayeredPolyId {
                layer: LayerId(1),
                poly: NavPolyId(0),
            },
            cost: 2.0,
            bidirectional: false,
        });

        let path = ml.find_path(
            LayerId(0),
            Vec2::new(1.0, 1.0),
            LayerId(1),
            Vec2::new(9.0, 1.0),
        );
        assert!(path.is_some());
        let path = path.unwrap();
        // Should traverse: layer0/poly0 -> layer0/poly1 -> layer1/poly0 -> layer1/poly1
        assert!(path.len() >= 3);
        // First node on layer 0, last node on layer 1
        assert_eq!(path.first().unwrap().layer, LayerId(0));
        assert_eq!(path.last().unwrap().layer, LayerId(1));
    }

    #[test]
    fn no_path_without_connection() {
        let mut ml = MultiLayerNavMesh::new();
        ml.add_layer(LayerId(0), make_simple_mesh());
        ml.add_layer(LayerId(1), make_simple_mesh());

        // No connection between layers — path should fail
        let path = ml.find_path(
            LayerId(0),
            Vec2::new(1.0, 1.0),
            LayerId(1),
            Vec2::new(9.0, 1.0),
        );
        assert!(path.is_none());
    }

    #[test]
    fn bidirectional_connection() {
        let mut ml = MultiLayerNavMesh::new();
        ml.add_layer(LayerId(0), make_simple_mesh());
        ml.add_layer(LayerId(1), make_simple_mesh());

        ml.add_connection(LayerConnection {
            from: LayeredPolyId {
                layer: LayerId(0),
                poly: NavPolyId(1),
            },
            to: LayeredPolyId {
                layer: LayerId(1),
                poly: NavPolyId(0),
            },
            cost: 2.0,
            bidirectional: true,
        });

        // Forward: layer 0 -> layer 1
        let fwd = ml.find_path(
            LayerId(0),
            Vec2::new(1.0, 1.0),
            LayerId(1),
            Vec2::new(9.0, 1.0),
        );
        assert!(fwd.is_some());

        // Reverse: layer 1 -> layer 0
        let rev = ml.find_path(
            LayerId(1),
            Vec2::new(1.0, 1.0),
            LayerId(0),
            Vec2::new(9.0, 1.0),
        );
        assert!(rev.is_some());
    }

    #[test]
    fn remove_layer() {
        let mut ml = MultiLayerNavMesh::new();
        ml.add_layer(LayerId(0), make_simple_mesh());
        ml.add_layer(LayerId(1), make_simple_mesh());
        ml.add_connection(LayerConnection {
            from: LayeredPolyId {
                layer: LayerId(0),
                poly: NavPolyId(0),
            },
            to: LayeredPolyId {
                layer: LayerId(1),
                poly: NavPolyId(0),
            },
            cost: 1.0,
            bidirectional: false,
        });

        assert_eq!(ml.layer_count(), 2);
        assert_eq!(ml.connections().len(), 1);

        assert!(ml.remove_layer(LayerId(1)));
        assert_eq!(ml.layer_count(), 1);
        // Connection referencing removed layer should be cleaned up
        assert_eq!(ml.connections().len(), 0);

        // Removing non-existent layer returns false
        assert!(!ml.remove_layer(LayerId(99)));
    }

    #[test]
    fn serde_roundtrip() {
        let mut ml = MultiLayerNavMesh::new();
        ml.add_layer(LayerId(0), make_simple_mesh());
        ml.add_layer(LayerId(1), make_simple_mesh());
        ml.add_connection(LayerConnection {
            from: LayeredPolyId {
                layer: LayerId(0),
                poly: NavPolyId(1),
            },
            to: LayeredPolyId {
                layer: LayerId(1),
                poly: NavPolyId(0),
            },
            cost: 3.0,
            bidirectional: true,
        });

        let json = serde_json::to_string(&ml).unwrap();
        let deserialized: MultiLayerNavMesh = serde_json::from_str(&json).unwrap();
        assert_eq!(deserialized.layer_count(), 2);
        assert_eq!(deserialized.connections().len(), 1);
        assert!((deserialized.connections()[0].cost - 3.0).abs() < f32::EPSILON);
    }
}
