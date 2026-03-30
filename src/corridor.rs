//! Path corridor — sliding window over navmesh polygon paths with local replanning.

use hisab::Vec2;
use serde::{Deserialize, Serialize};

use crate::mesh::{NavMesh, NavPolyId};
use crate::smooth::{extract_portals, funnel_portals};

#[cfg(feature = "logging")]
use tracing::instrument;

/// A path corridor — a sequence of navmesh polygons from start to goal
/// with position tracking and local replanning support.
///
/// The corridor maintains the current polygon sequence and the agent's
/// position within it. When the world changes, `replan_local()` can
/// update just the affected section without recomputing the entire path.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PathCorridor {
    /// The polygon sequence from current position to goal.
    polys: Vec<NavPolyId>,
    /// Current agent position.
    position: Vec2,
    /// Goal position.
    goal: Vec2,
    /// Index of the polygon the agent is currently in.
    current_idx: usize,
}

impl PathCorridor {
    /// Create a new corridor from a polygon path.
    ///
    /// `position` should be inside `polys[0]`, `goal` inside the last polygon.
    /// Returns `None` if the polygon list is empty.
    #[cfg_attr(feature = "logging", instrument)]
    #[must_use]
    pub fn new(polys: Vec<NavPolyId>, position: Vec2, goal: Vec2) -> Option<Self> {
        if polys.is_empty() {
            return None;
        }
        Some(Self {
            polys,
            position,
            goal,
            current_idx: 0,
        })
    }

    /// Create a corridor by pathfinding on the navmesh.
    ///
    /// Returns `None` if no path exists or positions are off-mesh.
    #[cfg_attr(feature = "logging", instrument(skip(mesh)))]
    #[must_use]
    pub fn find(mesh: &NavMesh, position: Vec2, goal: Vec2) -> Option<Self> {
        let polys = mesh.find_path(position, goal)?;
        Self::new(polys, position, goal)
    }

    /// The polygon sequence.
    #[must_use]
    pub fn polys(&self) -> &[NavPolyId] {
        &self.polys
    }

    /// Current agent position.
    #[must_use]
    pub fn position(&self) -> Vec2 {
        self.position
    }

    /// Goal position.
    #[must_use]
    pub fn goal(&self) -> Vec2 {
        self.goal
    }

    /// Index of the current polygon in the corridor.
    #[must_use]
    pub fn current_index(&self) -> usize {
        self.current_idx
    }

    /// Number of polygons remaining in the corridor (including current).
    #[must_use]
    pub fn remaining_polys(&self) -> usize {
        self.polys.len().saturating_sub(self.current_idx)
    }

    /// Whether the agent has reached the final polygon.
    #[must_use]
    pub fn is_at_end(&self) -> bool {
        self.current_idx >= self.polys.len().saturating_sub(1)
    }

    /// Move the agent to a new position, advancing through the corridor.
    ///
    /// Checks if the new position is in a later polygon in the corridor
    /// and advances `current_idx` accordingly.
    #[cfg_attr(feature = "logging", instrument(skip(self, mesh)))]
    pub fn move_position(&mut self, mesh: &NavMesh, new_position: Vec2) {
        self.position = new_position;

        // Check if we've moved to a later polygon in the corridor
        // Search forward from current position (most likely case)
        for i in self.current_idx..self.polys.len() {
            if let Some(poly) = mesh.get_poly(self.polys[i])
                && poly.contains_point(new_position)
            {
                self.current_idx = i;
                return;
            }
        }

        // If not found forward, check all polys (agent might have been pushed back)
        for i in 0..self.current_idx {
            if let Some(poly) = mesh.get_poly(self.polys[i])
                && poly.contains_point(new_position)
            {
                self.current_idx = i;
                return;
            }
        }
        // If position isn't in any corridor poly, keep current_idx unchanged
    }

    /// Trim the corridor — remove polygons the agent has already passed.
    ///
    /// Reduces memory and keeps the corridor focused on remaining path.
    pub fn trim_passed(&mut self) {
        if self.current_idx > 0 {
            self.polys.drain(..self.current_idx);
            self.current_idx = 0;
        }
    }

    /// Locally replan the corridor from the current position.
    ///
    /// Use when the world has changed (obstacles added/removed) and the
    /// existing corridor may be invalid. Replans from current position
    /// to the goal, replacing the remaining corridor.
    ///
    /// Returns `true` if replanning succeeded, `false` if no path exists.
    #[cfg_attr(feature = "logging", instrument(skip(self, mesh)))]
    pub fn replan(&mut self, mesh: &NavMesh) -> bool {
        if let Some(new_polys) = mesh.find_path(self.position, self.goal) {
            self.polys = new_polys;
            self.current_idx = 0;
            true
        } else {
            false
        }
    }

    /// Replan only the section from the current position to a specific
    /// polygon further along the corridor.
    ///
    /// This is cheaper than a full replan when only a local section is
    /// invalidated. Falls back to full replan if local replanning fails.
    ///
    /// Returns `true` if replanning succeeded.
    #[cfg_attr(feature = "logging", instrument(skip(self, mesh)))]
    pub fn replan_local(&mut self, mesh: &NavMesh, rejoin_idx: usize) -> bool {
        let rejoin_idx = rejoin_idx.min(self.polys.len().saturating_sub(1));
        if rejoin_idx <= self.current_idx {
            return self.replan(mesh);
        }

        // Find the centroid of the rejoin polygon as a local goal
        let rejoin_poly_id = self.polys[rejoin_idx];
        let rejoin_target = match mesh.get_poly(rejoin_poly_id) {
            Some(p) => p.centroid(),
            None => return self.replan(mesh),
        };

        // Pathfind from current position to rejoin point
        if let Some(local_path) = mesh.find_path(self.position, rejoin_target) {
            // Splice: replace polys[current_idx..rejoin_idx] with local_path
            let tail: Vec<NavPolyId> = self.polys[rejoin_idx..].to_vec();
            self.polys.truncate(self.current_idx);
            self.polys.extend(local_path);
            // Avoid duplicate if local_path ends at rejoin_poly
            if self.polys.last() == Some(&rejoin_poly_id) && tail.first() == Some(&rejoin_poly_id) {
                self.polys.pop();
            }
            self.polys.extend(tail);
            true
        } else {
            self.replan(mesh)
        }
    }

    /// Compute a smooth path through the corridor using the funnel algorithm.
    ///
    /// Returns world-space waypoints with agent radius clearance.
    #[cfg_attr(feature = "logging", instrument(skip(self, mesh)))]
    #[must_use]
    pub fn smooth_path(&self, mesh: &NavMesh, agent_radius: f32) -> Vec<Vec2> {
        let remaining = &self.polys[self.current_idx..];
        if remaining.is_empty() {
            return vec![self.position];
        }
        if remaining.len() == 1 {
            return vec![self.position, self.goal];
        }

        match extract_portals(mesh, remaining, self.position, self.goal) {
            Some(portals) if !portals.is_empty() => funnel_portals(&portals, agent_radius),
            _ => vec![self.position, self.goal],
        }
    }

    /// Change the goal. Triggers a full replan.
    ///
    /// Returns `true` if a path to the new goal was found.
    #[cfg_attr(feature = "logging", instrument(skip(self, mesh)))]
    pub fn set_goal(&mut self, mesh: &NavMesh, new_goal: Vec2) -> bool {
        self.goal = new_goal;
        self.replan(mesh)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::{NavMesh, NavPoly, NavPolyId};

    fn make_three_poly_mesh() -> NavMesh {
        let mut mesh = NavMesh::new();
        // Three squares in a row sharing edges
        mesh.add_poly(NavPoly {
            id: NavPolyId(0),
            vertices: vec![
                Vec2::new(0.0, 0.0),
                Vec2::new(10.0, 0.0),
                Vec2::new(10.0, 10.0),
                Vec2::new(0.0, 10.0),
            ],
            neighbors: vec![NavPolyId(1)],
            cost: 1.0,
            layer: 0,
        });
        mesh.add_poly(NavPoly {
            id: NavPolyId(1),
            vertices: vec![
                Vec2::new(10.0, 0.0),
                Vec2::new(20.0, 0.0),
                Vec2::new(20.0, 10.0),
                Vec2::new(10.0, 10.0),
            ],
            neighbors: vec![NavPolyId(0), NavPolyId(2)],
            cost: 1.0,
            layer: 0,
        });
        mesh.add_poly(NavPoly {
            id: NavPolyId(2),
            vertices: vec![
                Vec2::new(20.0, 0.0),
                Vec2::new(30.0, 0.0),
                Vec2::new(30.0, 10.0),
                Vec2::new(20.0, 10.0),
            ],
            neighbors: vec![NavPolyId(1)],
            cost: 1.0,
            layer: 0,
        });
        mesh
    }

    #[test]
    fn corridor_create() {
        let corridor = PathCorridor::new(
            vec![NavPolyId(0), NavPolyId(1), NavPolyId(2)],
            Vec2::new(5.0, 5.0),
            Vec2::new(25.0, 5.0),
        );
        assert!(corridor.is_some());
        let c = corridor.unwrap();
        assert_eq!(c.polys().len(), 3);
        assert_eq!(c.current_index(), 0);
        assert!(!c.is_at_end());
    }

    #[test]
    fn corridor_empty() {
        assert!(PathCorridor::new(vec![], Vec2::ZERO, Vec2::ONE).is_none());
    }

    #[test]
    fn corridor_find() {
        let mesh = make_three_poly_mesh();
        let corridor = PathCorridor::find(&mesh, Vec2::new(5.0, 5.0), Vec2::new(25.0, 5.0));
        assert!(corridor.is_some());
        let c = corridor.unwrap();
        assert_eq!(c.polys().len(), 3);
    }

    #[test]
    fn corridor_move_position() {
        let mesh = make_three_poly_mesh();
        let mut c = PathCorridor::new(
            vec![NavPolyId(0), NavPolyId(1), NavPolyId(2)],
            Vec2::new(5.0, 5.0),
            Vec2::new(25.0, 5.0),
        )
        .unwrap();

        assert_eq!(c.current_index(), 0);

        // Move to middle polygon
        c.move_position(&mesh, Vec2::new(15.0, 5.0));
        assert_eq!(c.current_index(), 1);

        // Move to last polygon
        c.move_position(&mesh, Vec2::new(25.0, 5.0));
        assert_eq!(c.current_index(), 2);
        assert!(c.is_at_end());
    }

    #[test]
    fn corridor_trim() {
        let mut c = PathCorridor::new(
            vec![NavPolyId(0), NavPolyId(1), NavPolyId(2)],
            Vec2::new(15.0, 5.0),
            Vec2::new(25.0, 5.0),
        )
        .unwrap();
        c.current_idx = 1;

        c.trim_passed();
        assert_eq!(c.polys().len(), 2);
        assert_eq!(c.current_index(), 0);
        assert_eq!(c.polys()[0], NavPolyId(1));
    }

    #[test]
    fn corridor_replan() {
        let mesh = make_three_poly_mesh();
        let mut c = PathCorridor::find(&mesh, Vec2::new(5.0, 5.0), Vec2::new(25.0, 5.0)).unwrap();
        assert!(c.replan(&mesh));
        assert_eq!(c.polys().len(), 3);
    }

    #[test]
    fn corridor_set_goal() {
        let mesh = make_three_poly_mesh();
        let mut c = PathCorridor::find(&mesh, Vec2::new(5.0, 5.0), Vec2::new(25.0, 5.0)).unwrap();
        // Change goal to poly 1
        assert!(c.set_goal(&mesh, Vec2::new(15.0, 5.0)));
        assert_eq!(c.polys().len(), 2);
    }

    #[test]
    fn corridor_remaining() {
        let c = PathCorridor::new(
            vec![NavPolyId(0), NavPolyId(1), NavPolyId(2)],
            Vec2::new(5.0, 5.0),
            Vec2::new(25.0, 5.0),
        )
        .unwrap();
        assert_eq!(c.remaining_polys(), 3);
    }

    #[test]
    fn corridor_serde_roundtrip() {
        let c = PathCorridor::new(
            vec![NavPolyId(0), NavPolyId(1)],
            Vec2::new(5.0, 5.0),
            Vec2::new(15.0, 5.0),
        )
        .unwrap();
        let json = serde_json::to_string(&c).unwrap();
        let deserialized: PathCorridor = serde_json::from_str(&json).unwrap();
        assert_eq!(deserialized.polys().len(), 2);
    }

    #[test]
    fn corridor_single_poly() {
        let c = PathCorridor::new(vec![NavPolyId(0)], Vec2::new(5.0, 5.0), Vec2::new(8.0, 8.0))
            .unwrap();
        assert!(c.is_at_end());
        assert_eq!(c.remaining_polys(), 1);
    }
}
