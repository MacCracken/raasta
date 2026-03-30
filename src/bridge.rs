//! Cross-crate bridges — convert primitive values from other AGNOS science crates
//! into raasta navigation parameters and vice versa.

use hisab::Vec2;

use crate::steer::Obstacle;

// ── Impetus bridges (physics) ──────────────────────────────────────────────

/// Convert collider position and radius to a navmesh [`Obstacle`].
///
/// Projects the 3D position onto the XZ ground plane.
#[must_use]
#[inline]
#[cfg_attr(feature = "logging", tracing::instrument)]
pub fn collider_to_obstacle(position: [f32; 3], radius: f32) -> Obstacle {
    Obstacle {
        center: Vec2::new(position[0], position[2]),
        radius,
    }
}

/// Convert rigid body velocity `[vx, vy, vz]` to a 2D movement [`Vec2`] `(vx, vz)`.
///
/// Drops the Y component for ground-plane navigation.
#[must_use]
#[inline]
#[cfg_attr(feature = "logging", tracing::instrument)]
pub fn velocity_3d_to_2d(velocity: [f32; 3]) -> Vec2 {
    Vec2::new(velocity[0], velocity[2])
}

// ── Jantu bridges (creature behavior) ──────────────────────────────────────

/// Convert group target position to a crowd destination [`Vec2`] `(x, z)`.
#[must_use]
#[inline]
#[cfg_attr(feature = "logging", tracing::instrument)]
pub fn group_target_to_destination(target: [f32; 3]) -> Vec2 {
    Vec2::new(target[0], target[2])
}

/// Convert flee point to a repulsion field center with strength.
///
/// Returns `(center, repulsion_strength)` where center is `(x, z)`.
/// Strength decays with distance: 1.0 at 0, 0.0 at `max_range`.
#[must_use]
#[cfg_attr(feature = "logging", tracing::instrument)]
pub fn flee_point_to_repulsion(
    flee_from: [f32; 3],
    agent_pos: [f32; 3],
    max_range: f32,
) -> (Vec2, f32) {
    let dx = agent_pos[0] - flee_from[0];
    let dz = agent_pos[2] - flee_from[2];
    let dist = (dx * dx + dz * dz).sqrt();
    let strength = if max_range > 0.0 {
        (1.0 - dist / max_range).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (Vec2::new(flee_from[0], flee_from[2]), strength)
}

// ── Pavan bridges (aerodynamics/wind) ──────────────────────────────────────

/// Convert wind velocity `[wx, wz]` (m/s) to movement cost modifier.
///
/// Headwind increases cost, tailwind decreases it.
/// `movement_direction`: unit vector `[dx, dz]`.
/// Returns cost multiplier (>1 = harder, <1 = easier, min 0.5).
///
/// The wind vector is the direction the wind blows. A positive dot with
/// movement direction means tailwind (cheaper), negative means headwind
/// (costlier).
#[must_use]
#[cfg_attr(feature = "logging", tracing::instrument)]
pub fn wind_to_movement_cost(wind: [f32; 2], movement_direction: [f32; 2]) -> f32 {
    // Project wind onto movement direction: positive = tailwind, negative = headwind
    let dot = wind[0] * movement_direction[0] + wind[1] * movement_direction[1];
    // Scale: 10 m/s headwind → 2× cost, 10 m/s tailwind → 0.5× (clamped)
    (1.0 - dot / 10.0).clamp(0.5, 2.0)
}

/// Convert terrain slope (radians) to traversal speed scaling factor.
///
/// Flat = 1.0, uphill slows down, steep = impassable.
#[must_use]
#[inline]
#[cfg_attr(feature = "logging", tracing::instrument)]
pub fn slope_to_speed_scale(slope_rad: f32) -> f32 {
    let slope_deg = slope_rad.abs() * 180.0 / core::f32::consts::PI;
    if slope_deg > 45.0 {
        return 0.0; // impassable
    }
    (1.0 - slope_deg / 60.0).clamp(0.1, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn collider_to_obstacle_basic() {
        let obs = collider_to_obstacle([1.0, 2.0, 3.0], 0.5);
        assert!((obs.center.x - 1.0).abs() < 0.001);
        assert!((obs.center.y - 3.0).abs() < 0.001);
        assert!((obs.radius - 0.5).abs() < 0.001);
    }

    #[test]
    fn velocity_drop_y() {
        let v = velocity_3d_to_2d([1.0, 5.0, 3.0]);
        assert!((v.x - 1.0).abs() < 0.001);
        assert!((v.y - 3.0).abs() < 0.001);
    }

    #[test]
    fn group_target_drops_y() {
        let d = group_target_to_destination([10.0, 5.0, 20.0]);
        assert!((d.x - 10.0).abs() < 0.001);
        assert!((d.y - 20.0).abs() < 0.001);
    }

    #[test]
    fn flee_repulsion_close() {
        let (_, strength) = flee_point_to_repulsion([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 10.0);
        assert!(strength > 0.5);
    }

    #[test]
    fn flee_repulsion_far() {
        let (_, strength) = flee_point_to_repulsion([0.0, 0.0, 0.0], [15.0, 0.0, 0.0], 10.0);
        assert!(strength.abs() < 0.001);
    }

    #[test]
    fn flee_repulsion_returns_center() {
        let (center, _) = flee_point_to_repulsion([3.0, 1.0, 7.0], [0.0, 0.0, 0.0], 10.0);
        assert!((center.x - 3.0).abs() < 0.001);
        assert!((center.y - 7.0).abs() < 0.001);
    }

    #[test]
    fn wind_tailwind_cheap() {
        // Wind blowing east, moving east → tailwind → cheaper
        let cost = wind_to_movement_cost([5.0, 0.0], [1.0, 0.0]);
        assert!(cost < 1.0);
    }

    #[test]
    fn wind_headwind_costly() {
        // Wind blowing west, moving east → headwind → costlier
        let cost = wind_to_movement_cost([-5.0, 0.0], [1.0, 0.0]);
        assert!(cost > 1.0);
    }

    #[test]
    fn wind_no_wind() {
        let cost = wind_to_movement_cost([0.0, 0.0], [1.0, 0.0]);
        assert!((cost - 1.0).abs() < 0.01);
    }

    #[test]
    fn wind_perpendicular_neutral() {
        // Wind blowing north, moving east → crosswind → neutral
        let cost = wind_to_movement_cost([0.0, 5.0], [1.0, 0.0]);
        assert!((cost - 1.0).abs() < 0.01);
    }

    #[test]
    fn wind_clamps_to_bounds() {
        // Extreme headwind clamps to 2.0
        let cost = wind_to_movement_cost([-100.0, 0.0], [1.0, 0.0]);
        assert!((cost - 2.0).abs() < 0.01);
        // Extreme tailwind clamps to 0.5
        let cost = wind_to_movement_cost([100.0, 0.0], [1.0, 0.0]);
        assert!((cost - 0.5).abs() < 0.01);
    }

    #[test]
    fn slope_flat() {
        assert!((slope_to_speed_scale(0.0) - 1.0).abs() < 0.01);
    }

    #[test]
    fn slope_steep_impassable() {
        let scale = slope_to_speed_scale(std::f32::consts::FRAC_PI_2); // 90°
        assert!(scale.abs() < 0.001);
    }

    #[test]
    fn slope_moderate() {
        let scale = slope_to_speed_scale(0.26); // ~15°
        assert!(scale > 0.5 && scale < 1.0);
    }
}
