//! 3D steering behaviors — seek, flee, arrive in 3D space.

use hisab::Vec3;
use serde::{Deserialize, Serialize};

/// A 3D steering behavior.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[non_exhaustive]
pub enum SteerBehavior3D {
    /// Move toward a target position in 3D.
    Seek { target: Vec3 },
    /// Move away from a target position in 3D.
    Flee { target: Vec3 },
    /// Move toward a target, slowing down as it approaches.
    Arrive { target: Vec3, slow_radius: f32 },
}

/// Output of a 3D steering computation.
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct SteerOutput3D {
    pub velocity: Vec3,
}

impl SteerOutput3D {
    #[must_use]
    pub fn new(vx: f32, vy: f32, vz: f32) -> Self {
        Self {
            velocity: Vec3::new(vx, vy, vz),
        }
    }

    #[must_use]
    pub fn from_vec3(velocity: Vec3) -> Self {
        Self { velocity }
    }

    #[inline]
    #[must_use]
    pub fn speed(&self) -> f32 {
        self.velocity.length()
    }
}

/// Compute 3D steering output for the given behavior.
#[inline]
#[must_use]
pub fn compute_steer_3d(
    behavior: &SteerBehavior3D,
    position: Vec3,
    max_speed: f32,
) -> SteerOutput3D {
    match behavior {
        SteerBehavior3D::Seek { target } => {
            let desired = *target - position;
            let len = desired.length();
            if len < f32::EPSILON {
                return SteerOutput3D::default();
            }
            SteerOutput3D::from_vec3(desired / len * max_speed)
        }
        SteerBehavior3D::Flee { target } => {
            let desired = position - *target;
            let len = desired.length();
            if len < f32::EPSILON {
                return SteerOutput3D::default();
            }
            SteerOutput3D::from_vec3(desired / len * max_speed)
        }
        SteerBehavior3D::Arrive {
            target,
            slow_radius,
        } => {
            let desired = *target - position;
            let dist = desired.length();
            if dist < f32::EPSILON {
                return SteerOutput3D::default();
            }
            let speed = if dist < *slow_radius {
                max_speed * (dist / slow_radius)
            } else {
                max_speed
            };
            SteerOutput3D::from_vec3(desired / dist * speed)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn seek_3d() {
        let out = compute_steer_3d(
            &SteerBehavior3D::Seek {
                target: Vec3::new(10.0, 0.0, 0.0),
            },
            Vec3::ZERO,
            5.0,
        );
        assert!((out.velocity.x - 5.0).abs() < 0.01);
        assert!(out.velocity.y.abs() < 0.01);
        assert!(out.velocity.z.abs() < 0.01);
    }

    #[test]
    fn flee_3d() {
        let out = compute_steer_3d(
            &SteerBehavior3D::Flee {
                target: Vec3::new(10.0, 0.0, 0.0),
            },
            Vec3::ZERO,
            5.0,
        );
        assert!(out.velocity.x < 0.0);
    }

    #[test]
    fn arrive_3d_full_speed() {
        let out = compute_steer_3d(
            &SteerBehavior3D::Arrive {
                target: Vec3::new(100.0, 0.0, 0.0),
                slow_radius: 10.0,
            },
            Vec3::ZERO,
            5.0,
        );
        assert!((out.speed() - 5.0).abs() < 0.01);
    }

    #[test]
    fn arrive_3d_slow_down() {
        let out = compute_steer_3d(
            &SteerBehavior3D::Arrive {
                target: Vec3::new(5.0, 0.0, 0.0),
                slow_radius: 10.0,
            },
            Vec3::ZERO,
            10.0,
        );
        assert!((out.speed() - 5.0).abs() < 0.1);
    }

    #[test]
    fn arrive_3d_at_target() {
        let out = compute_steer_3d(
            &SteerBehavior3D::Arrive {
                target: Vec3::new(1.0, 1.0, 1.0),
                slow_radius: 5.0,
            },
            Vec3::new(1.0, 1.0, 1.0),
            10.0,
        );
        assert!(out.speed() < f32::EPSILON);
    }

    #[test]
    fn seek_3d_diagonal() {
        let out = compute_steer_3d(
            &SteerBehavior3D::Seek {
                target: Vec3::new(1.0, 1.0, 1.0),
            },
            Vec3::ZERO,
            1.0,
        );
        assert!((out.speed() - 1.0).abs() < 0.01);
        // All components should be equal
        assert!((out.velocity.x - out.velocity.y).abs() < 0.01);
        assert!((out.velocity.y - out.velocity.z).abs() < 0.01);
    }

    #[test]
    fn steer_3d_serde_roundtrip() {
        let b = SteerBehavior3D::Arrive {
            target: Vec3::new(1.0, 2.0, 3.0),
            slow_radius: 5.0,
        };
        let json = serde_json::to_string(&b).unwrap();
        let deserialized: SteerBehavior3D = serde_json::from_str(&json).unwrap();
        assert_eq!(deserialized, b);
    }

    #[test]
    fn steer_output_3d_speed() {
        let out = SteerOutput3D::new(3.0, 4.0, 0.0);
        assert!((out.speed() - 5.0).abs() < 0.01);
    }

    #[test]
    fn steer_output_3d_serde_roundtrip() {
        let out = SteerOutput3D::new(1.0, 2.0, 3.0);
        let json = serde_json::to_string(&out).unwrap();
        let deserialized: SteerOutput3D = serde_json::from_str(&json).unwrap();
        assert!((deserialized.velocity - out.velocity).length() < f32::EPSILON);
    }
}
