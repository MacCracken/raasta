//! Formation movement — slot-based group movement patterns.

use hisab::Vec2;
use serde::{Deserialize, Serialize};

use crate::steer::SteerOutput;

#[cfg(feature = "logging")]
use tracing::instrument;

/// A formation shape defining slot positions relative to the leader.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[non_exhaustive]
pub enum FormationShape {
    /// Line formation perpendicular to movement direction.
    Line { spacing: f32 },
    /// V-shape (wedge) formation.
    Wedge { spacing: f32, angle: f32 },
    /// Circle around leader.
    Circle { radius: f32 },
    /// Grid formation (rows x columns).
    Grid { spacing: f32, columns: usize },
    /// Custom slot offsets relative to leader.
    Custom { offsets: Vec<Vec2> },
}

/// A formation manager for group movement.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Formation {
    /// Formation shape.
    shape: FormationShape,
    /// Leader position.
    leader_position: Vec2,
    /// Leader forward direction (normalized).
    leader_forward: Vec2,
}

impl Formation {
    /// Create a new formation.
    #[must_use]
    pub fn new(shape: FormationShape) -> Self {
        Self {
            shape,
            leader_position: Vec2::ZERO,
            leader_forward: Vec2::new(1.0, 0.0),
        }
    }

    /// Update the leader's position and facing direction.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    pub fn update_leader(&mut self, position: Vec2, forward: Vec2) {
        self.leader_position = position;
        let len = forward.length();
        self.leader_forward = if len > f32::EPSILON {
            forward / len
        } else {
            Vec2::new(1.0, 0.0)
        };
    }

    /// Get the world-space target position for slot `index`.
    ///
    /// Slot 0 is the leader position. Subsequent slots are offset
    /// based on the formation shape and leader orientation.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    #[must_use]
    pub fn slot_position(&self, index: usize) -> Vec2 {
        if index == 0 {
            return self.leader_position;
        }

        let offset = self.local_offset(index);
        // Rotate offset by leader forward direction
        let right = Vec2::new(self.leader_forward.y, -self.leader_forward.x);
        self.leader_position + self.leader_forward * offset.y + right * offset.x
    }

    /// Compute the number of slots for a given agent count.
    #[must_use]
    pub fn slot_count(&self, agent_count: usize) -> usize {
        agent_count // 1:1 mapping
    }

    /// Compute steering to move an agent toward its formation slot.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    #[must_use]
    pub fn steer_to_slot(
        &self,
        agent_position: Vec2,
        slot_index: usize,
        max_speed: f32,
    ) -> SteerOutput {
        let target = self.slot_position(slot_index);
        let desired = target - agent_position;
        let dist = desired.length();
        if dist < f32::EPSILON {
            return SteerOutput::default();
        }
        // Arrive behavior — slow down near target
        let slow_radius = max_speed * 0.5;
        let speed = if dist < slow_radius {
            max_speed * (dist / slow_radius)
        } else {
            max_speed
        };
        SteerOutput::from_vec2(desired / dist * speed)
    }

    /// Leader position.
    #[must_use]
    pub fn leader_position(&self) -> Vec2 {
        self.leader_position
    }

    /// Leader forward direction.
    #[must_use]
    pub fn leader_forward(&self) -> Vec2 {
        self.leader_forward
    }

    fn local_offset(&self, index: usize) -> Vec2 {
        let i = index as f32;
        match &self.shape {
            FormationShape::Line { spacing } => {
                // Alternate left/right
                let side = if index % 2 == 1 { 1.0 } else { -1.0 };
                let rank = index.div_ceil(2) as f32;
                Vec2::new(side * rank * spacing, 0.0)
            }
            FormationShape::Wedge { spacing, angle } => {
                let side = if index % 2 == 1 { 1.0 } else { -1.0 };
                let rank = index.div_ceil(2) as f32;
                let x = side * rank * spacing;
                let y = -rank * spacing * angle.cos();
                Vec2::new(x, y)
            }
            FormationShape::Circle { radius } => {
                let angle = (i - 1.0) / (index.max(1)) as f32 * std::f32::consts::TAU;
                Vec2::new(angle.cos() * radius, angle.sin() * radius)
            }
            FormationShape::Grid { spacing, columns } => {
                let cols = *columns.max(&1);
                let col = (index - 1) % cols;
                let row = (index - 1) / cols;
                let x = (col as f32 - (cols as f32 - 1.0) * 0.5) * spacing;
                let y = -(row as f32 + 1.0) * spacing;
                Vec2::new(x, y)
            }
            FormationShape::Custom { offsets } => {
                offsets.get(index - 1).copied().unwrap_or(Vec2::ZERO)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn leader_slot_is_leader_position() {
        let mut f = Formation::new(FormationShape::Line { spacing: 2.0 });
        f.update_leader(Vec2::new(5.0, 3.0), Vec2::new(1.0, 0.0));
        let pos = f.slot_position(0);
        assert!((pos.x - 5.0).abs() < f32::EPSILON);
        assert!((pos.y - 3.0).abs() < f32::EPSILON);
    }

    #[test]
    fn line_formation_slot_positions() {
        let mut f = Formation::new(FormationShape::Line { spacing: 2.0 });
        f.update_leader(Vec2::ZERO, Vec2::new(1.0, 0.0));

        // Slot 1: right side, rank 1
        let p1 = f.slot_position(1);
        // Slot 2: left side, rank 1
        let p2 = f.slot_position(2);

        // They should be on opposite sides
        assert!(
            (p1.y + p2.y).abs() < 0.01,
            "slots should mirror: {p1:?} vs {p2:?}"
        );
        // Both at same forward position (offset.y == 0 for line)
        assert!((p1.x - p2.x).abs() < 0.01);
    }

    #[test]
    fn wedge_formation() {
        let mut f = Formation::new(FormationShape::Wedge {
            spacing: 2.0,
            angle: 0.0,
        });
        f.update_leader(Vec2::ZERO, Vec2::new(1.0, 0.0));

        let p1 = f.slot_position(1);
        let p2 = f.slot_position(2);
        // Wedge slots should be behind leader (negative x when forward is +x)
        assert!(p1.x < 0.01);
        assert!(p2.x < 0.01);
    }

    #[test]
    fn circle_formation() {
        let mut f = Formation::new(FormationShape::Circle { radius: 5.0 });
        f.update_leader(Vec2::ZERO, Vec2::new(1.0, 0.0));

        let p1 = f.slot_position(1);
        let dist = p1.length();
        assert!(
            (dist - 5.0).abs() < 0.01,
            "circle slot should be at radius distance: {dist}"
        );
    }

    #[test]
    fn grid_formation() {
        let mut f = Formation::new(FormationShape::Grid {
            spacing: 2.0,
            columns: 3,
        });
        f.update_leader(Vec2::ZERO, Vec2::new(1.0, 0.0));

        // Slot 1 is first column of first row behind leader
        let p1 = f.slot_position(1);
        // Should be behind leader (negative x when forward = +x)
        assert!(p1.x < 0.01, "grid slot should be behind leader: {p1:?}");
    }

    #[test]
    fn custom_offsets() {
        let offsets = vec![Vec2::new(1.0, 2.0), Vec2::new(-3.0, 4.0)];
        let mut f = Formation::new(FormationShape::Custom { offsets });
        f.update_leader(Vec2::ZERO, Vec2::new(1.0, 0.0));

        // Slot 1 uses first offset
        let p1 = f.slot_position(1);
        // With forward = (1,0), right = (0,-1)
        // world = leader + forward * offset.y + right * offset.x
        // = (0,0) + (1,0)*2 + (0,-1)*1 = (2, -1)
        assert!((p1.x - 2.0).abs() < 0.01);
        assert!((p1.y - (-1.0)).abs() < 0.01);

        // Slot 3 is out of range — should return leader position
        let p3 = f.slot_position(3);
        assert!((p3.x).abs() < 0.01);
        assert!((p3.y).abs() < 0.01);
    }

    #[test]
    fn steer_to_slot_produces_movement() {
        let mut f = Formation::new(FormationShape::Line { spacing: 2.0 });
        f.update_leader(Vec2::new(10.0, 0.0), Vec2::new(1.0, 0.0));

        let out = f.steer_to_slot(Vec2::ZERO, 0, 5.0);
        assert!(out.speed() > 0.0);
        assert!(out.velocity.x > 0.0);
    }

    #[test]
    fn update_leader_changes_positions() {
        let mut f = Formation::new(FormationShape::Line { spacing: 2.0 });
        f.update_leader(Vec2::ZERO, Vec2::new(1.0, 0.0));
        let before = f.slot_position(1);

        f.update_leader(Vec2::new(10.0, 10.0), Vec2::new(0.0, 1.0));
        let after = f.slot_position(1);

        assert!(
            (before.x - after.x).abs() > 0.1 || (before.y - after.y).abs() > 0.1,
            "slot position should change after leader update"
        );
    }

    #[test]
    fn serde_roundtrip() {
        let mut f = Formation::new(FormationShape::Wedge {
            spacing: 3.0,
            angle: 0.5,
        });
        f.update_leader(Vec2::new(1.0, 2.0), Vec2::new(0.0, 1.0));

        let json = serde_json::to_string(&f).expect("serialize");
        let f2: Formation = serde_json::from_str(&json).expect("deserialize");

        assert!((f2.leader_position().x - 1.0).abs() < f32::EPSILON);
        assert!((f2.leader_position().y - 2.0).abs() < f32::EPSILON);
    }
}
