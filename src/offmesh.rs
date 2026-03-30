//! Off-mesh links — custom traversal edges for jumps, ladders, teleporters, doors.

use hisab::Vec2;
use serde::{Deserialize, Serialize};

use crate::mesh::NavPolyId;

#[cfg(feature = "logging")]
use tracing::instrument;

/// The type of traversal an off-mesh link represents.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[non_exhaustive]
pub enum LinkType {
    /// A jump between two points (one-way or bidirectional).
    Jump,
    /// A ladder or climbable surface.
    Ladder,
    /// An instant teleporter.
    Teleport,
    /// A door that may be open or closed.
    Door,
    /// Generic custom link.
    Custom,
}

/// Unique identifier for an off-mesh link.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct OffMeshLinkId(pub u32);

/// An off-mesh link connecting two positions on the navmesh.
///
/// Links create additional edges in the navigation graph beyond the
/// polygon adjacency. They can be bidirectional or one-way, and have
/// a traversal cost.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OffMeshLink {
    /// Unique identifier.
    pub id: OffMeshLinkId,
    /// Start position (world space).
    pub start: Vec2,
    /// End position (world space).
    pub end: Vec2,
    /// Polygon containing the start position.
    pub start_poly: NavPolyId,
    /// Polygon containing the end position.
    pub end_poly: NavPolyId,
    /// Traversal cost (added to pathfinding cost).
    pub cost: f32,
    /// Whether this link can be traversed in both directions.
    pub bidirectional: bool,
    /// Type of traversal.
    pub link_type: LinkType,
    /// Whether this link is currently enabled (e.g., door open/closed).
    pub enabled: bool,
}

/// Description of an off-mesh link to add to a registry.
///
/// Groups the parameters for [`OffMeshLinkRegistry::add_link`] into a
/// single struct to keep the API ergonomic.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OffMeshLinkDesc {
    /// Start position (world space).
    pub start: Vec2,
    /// End position (world space).
    pub end: Vec2,
    /// Polygon containing the start position.
    pub start_poly: NavPolyId,
    /// Polygon containing the end position.
    pub end_poly: NavPolyId,
    /// Traversal cost (added to pathfinding cost). Clamped to >= 0.
    pub cost: f32,
    /// Whether this link can be traversed in both directions.
    pub bidirectional: bool,
    /// Type of traversal.
    pub link_type: LinkType,
}

/// Registry of off-mesh links for a navmesh.
///
/// Manages links and provides query methods for pathfinding integration.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct OffMeshLinkRegistry {
    links: Vec<OffMeshLink>,
    next_id: u32,
}

impl OffMeshLinkRegistry {
    /// Create an empty registry.
    #[must_use]
    pub fn new() -> Self {
        Self {
            links: Vec::new(),
            next_id: 0,
        }
    }

    /// Add an off-mesh link and return its ID.
    ///
    /// The caller is responsible for ensuring `start_poly` and `end_poly`
    /// are valid polygon IDs in the associated navmesh.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    pub fn add_link(&mut self, desc: OffMeshLinkDesc) -> OffMeshLinkId {
        let id = OffMeshLinkId(self.next_id);
        self.next_id += 1;
        self.links.push(OffMeshLink {
            id,
            start: desc.start,
            end: desc.end,
            start_poly: desc.start_poly,
            end_poly: desc.end_poly,
            cost: desc.cost.max(0.0),
            bidirectional: desc.bidirectional,
            link_type: desc.link_type,
            enabled: true,
        });
        id
    }

    /// Remove a link by ID. Returns the removed link, or `None`.
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    #[must_use]
    pub fn remove_link(&mut self, id: OffMeshLinkId) -> Option<OffMeshLink> {
        if let Some(pos) = self.links.iter().position(|l| l.id == id) {
            Some(self.links.swap_remove(pos))
        } else {
            None
        }
    }

    /// Enable or disable a link (e.g., open/close a door).
    #[cfg_attr(feature = "logging", instrument(skip(self)))]
    pub fn set_enabled(&mut self, id: OffMeshLinkId, enabled: bool) {
        if let Some(link) = self.links.iter_mut().find(|l| l.id == id) {
            link.enabled = enabled;
        }
    }

    /// Get a link by ID.
    #[must_use]
    pub fn get_link(&self, id: OffMeshLinkId) -> Option<&OffMeshLink> {
        self.links.iter().find(|l| l.id == id)
    }

    /// Get all links.
    #[must_use]
    pub fn links(&self) -> &[OffMeshLink] {
        &self.links
    }

    /// Get all enabled links originating from (or connected to) a given polygon.
    ///
    /// Returns links where `start_poly == poly_id`, plus bidirectional links
    /// where `end_poly == poly_id`.
    #[must_use]
    pub fn links_from_poly(&self, poly_id: NavPolyId) -> Vec<&OffMeshLink> {
        self.links
            .iter()
            .filter(|l| {
                l.enabled && (l.start_poly == poly_id || (l.bidirectional && l.end_poly == poly_id))
            })
            .collect()
    }

    /// Number of links.
    #[must_use]
    #[inline]
    pub fn link_count(&self) -> usize {
        self.links.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn desc(
        start: Vec2,
        end: Vec2,
        start_poly: NavPolyId,
        end_poly: NavPolyId,
        cost: f32,
        bidirectional: bool,
        link_type: LinkType,
    ) -> OffMeshLinkDesc {
        OffMeshLinkDesc {
            start,
            end,
            start_poly,
            end_poly,
            cost,
            bidirectional,
            link_type,
        }
    }

    #[test]
    fn registry_add_remove() {
        let mut reg = OffMeshLinkRegistry::new();
        let id = reg.add_link(desc(
            Vec2::ZERO,
            Vec2::new(10.0, 0.0),
            NavPolyId(0),
            NavPolyId(1),
            5.0,
            true,
            LinkType::Jump,
        ));
        assert_eq!(reg.link_count(), 1);
        assert!(reg.get_link(id).is_some());

        let removed = reg.remove_link(id);
        assert!(removed.is_some());
        assert_eq!(reg.link_count(), 0);
    }

    #[test]
    fn registry_enable_disable() {
        let mut reg = OffMeshLinkRegistry::new();
        let id = reg.add_link(desc(
            Vec2::ZERO,
            Vec2::new(10.0, 0.0),
            NavPolyId(0),
            NavPolyId(1),
            5.0,
            false,
            LinkType::Door,
        ));
        assert!(reg.get_link(id).unwrap().enabled);

        reg.set_enabled(id, false);
        assert!(!reg.get_link(id).unwrap().enabled);

        // Disabled links shouldn't appear in links_from_poly
        assert!(reg.links_from_poly(NavPolyId(0)).is_empty());

        reg.set_enabled(id, true);
        assert_eq!(reg.links_from_poly(NavPolyId(0)).len(), 1);
    }

    #[test]
    fn bidirectional_links() {
        let mut reg = OffMeshLinkRegistry::new();
        reg.add_link(desc(
            Vec2::ZERO,
            Vec2::new(10.0, 0.0),
            NavPolyId(0),
            NavPolyId(1),
            5.0,
            true,
            LinkType::Teleport,
        ));
        // Should appear from both sides
        assert_eq!(reg.links_from_poly(NavPolyId(0)).len(), 1);
        assert_eq!(reg.links_from_poly(NavPolyId(1)).len(), 1);
    }

    #[test]
    fn one_way_links() {
        let mut reg = OffMeshLinkRegistry::new();
        reg.add_link(desc(
            Vec2::ZERO,
            Vec2::new(10.0, 0.0),
            NavPolyId(0),
            NavPolyId(1),
            5.0,
            false,
            LinkType::Jump,
        ));
        // Only from start poly
        assert_eq!(reg.links_from_poly(NavPolyId(0)).len(), 1);
        assert_eq!(reg.links_from_poly(NavPolyId(1)).len(), 0);
    }

    #[test]
    fn link_serde_roundtrip() {
        let mut reg = OffMeshLinkRegistry::new();
        reg.add_link(desc(
            Vec2::new(1.0, 2.0),
            Vec2::new(3.0, 4.0),
            NavPolyId(0),
            NavPolyId(1),
            2.5,
            true,
            LinkType::Ladder,
        ));
        let json = serde_json::to_string(&reg).unwrap();
        let deserialized: OffMeshLinkRegistry = serde_json::from_str(&json).unwrap();
        assert_eq!(deserialized.link_count(), 1);
    }

    #[test]
    fn remove_nonexistent_link() {
        let mut reg = OffMeshLinkRegistry::new();
        assert!(reg.remove_link(OffMeshLinkId(99)).is_none());
    }

    #[test]
    fn cost_clamped_to_zero() {
        let mut reg = OffMeshLinkRegistry::new();
        let id = reg.add_link(desc(
            Vec2::ZERO,
            Vec2::new(10.0, 0.0),
            NavPolyId(0),
            NavPolyId(1),
            -5.0,
            false,
            LinkType::Custom,
        ));
        assert!(reg.get_link(id).unwrap().cost >= 0.0);
    }
}
