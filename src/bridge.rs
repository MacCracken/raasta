//! Cross-crate bridges — convert primitive values from other AGNOS science crates
//! into raasta navigation parameters and vice versa.

// ── Impetus bridges (physics) ──────────────────────────────────────────────

/// Convert collider position and radius to a navmesh obstacle.
///
/// Returns `(center_x, center_z, radius)` for obstacle avoidance.
#[must_use]
#[inline]
pub fn collider_to_obstacle(position: [f32; 3], radius: f32) -> (f32, f32, f32) {
    (position[0], position[2], radius)
}

/// Convert rigid body velocity `[vx, vy, vz]` to a 2D movement vector `[vx, vz]`.
///
/// Drops the Y component for ground-plane navigation.
#[must_use]
#[inline]
pub fn velocity_3d_to_2d(velocity: [f32; 3]) -> [f32; 2] {
    [velocity[0], velocity[2]]
}

// ── Jantu bridges (creature behavior) ──────────────────────────────────────

/// Convert group target position to a crowd destination point `[x, z]`.
#[must_use]
#[inline]
pub fn group_target_to_destination(target: [f32; 3]) -> [f32; 2] {
    [target[0], target[2]]
}

/// Convert flee point to a repulsion field center with strength.
///
/// Returns `(center_x, center_z, repulsion_strength)`.
/// Strength decays with distance: 1.0 at 0, 0.0 at `max_range`.
#[must_use]
pub fn flee_point_to_repulsion(
    flee_from: [f32; 3],
    agent_pos: [f32; 3],
    max_range: f32,
) -> (f32, f32, f32) {
    let dx = agent_pos[0] - flee_from[0];
    let dz = agent_pos[2] - flee_from[2];
    let dist = (dx * dx + dz * dz).sqrt();
    let strength = if max_range > 0.0 {
        (1.0 - dist / max_range).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (flee_from[0], flee_from[2], strength)
}

// ── Pavan bridges (aerodynamics/wind) ──────────────────────────────────────

/// Convert wind velocity `[wx, wz]` (m/s) to movement cost modifier.
///
/// Headwind increases cost, tailwind decreases it.
/// `movement_direction`: unit vector `[dx, dz]`.
/// Returns cost multiplier (>1 = harder, <1 = easier, min 0.5).
#[must_use]
pub fn wind_to_movement_cost(wind: [f32; 2], movement_direction: [f32; 2]) -> f32 {
    // Dot product: positive = headwind, negative = tailwind
    let dot = wind[0] * movement_direction[0] + wind[1] * movement_direction[1];
    let wind_speed = (wind[0] * wind[0] + wind[1] * wind[1]).sqrt();
    if wind_speed < 0.01 {
        return 1.0;
    }
    // Scale: 10 m/s headwind → 2× cost, 10 m/s tailwind → 0.5× cost
    (1.0 + dot / (wind_speed * 10.0)).clamp(0.5, 2.0)
}

/// Convert terrain slope (radians) to traversal speed scaling factor.
///
/// Flat = 1.0, uphill slows down, steep = impassable.
#[must_use]
#[inline]
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
        let (x, z, r) = collider_to_obstacle([1.0, 2.0, 3.0], 0.5);
        assert!((x - 1.0).abs() < 0.001);
        assert!((z - 3.0).abs() < 0.001);
        assert!((r - 0.5).abs() < 0.001);
    }

    #[test]
    fn velocity_drop_y() {
        let v = velocity_3d_to_2d([1.0, 5.0, 3.0]);
        assert!((v[0] - 1.0).abs() < 0.001);
        assert!((v[1] - 3.0).abs() < 0.001);
    }

    #[test]
    fn group_target_drops_y() {
        let d = group_target_to_destination([10.0, 5.0, 20.0]);
        assert!((d[0] - 10.0).abs() < 0.001);
        assert!((d[1] - 20.0).abs() < 0.001);
    }

    #[test]
    fn flee_repulsion_close() {
        let (_, _, strength) = flee_point_to_repulsion([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 10.0);
        assert!(strength > 0.5);
    }

    #[test]
    fn flee_repulsion_far() {
        let (_, _, strength) = flee_point_to_repulsion([0.0, 0.0, 0.0], [15.0, 0.0, 0.0], 10.0);
        assert!((strength).abs() < 0.001);
    }

    #[test]
    fn wind_headwind_costly() {
        let cost = wind_to_movement_cost([5.0, 0.0], [1.0, 0.0]); // headwind
        assert!(cost > 1.0);
    }

    #[test]
    fn wind_tailwind_cheap() {
        let cost = wind_to_movement_cost([-5.0, 0.0], [1.0, 0.0]); // tailwind
        assert!(cost < 1.0);
    }

    #[test]
    fn wind_no_wind() {
        let cost = wind_to_movement_cost([0.0, 0.0], [1.0, 0.0]);
        assert!((cost - 1.0).abs() < 0.01);
    }

    #[test]
    fn slope_flat() {
        assert!((slope_to_speed_scale(0.0) - 1.0).abs() < 0.01);
    }

    #[test]
    fn slope_steep_impassable() {
        let scale = slope_to_speed_scale(std::f32::consts::FRAC_PI_2); // 90°
        assert!((scale).abs() < 0.001);
    }

    #[test]
    fn slope_moderate() {
        let scale = slope_to_speed_scale(0.26); // ~15°
        assert!(scale > 0.5 && scale < 1.0);
    }
}
