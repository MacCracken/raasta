//! Steering behavior blending and priority systems.

use hisab::Vec2;
use serde::{Deserialize, Serialize};

use crate::steer::SteerOutput;

#[cfg(feature = "logging")]
use tracing::instrument;

/// A weighted steering behavior entry for blending.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct WeightedSteer {
    /// The steering output to blend.
    pub output: SteerOutput,
    /// Weight for blending (higher = more influence).
    pub weight: f32,
}

/// Blend multiple steering outputs by weighted sum.
///
/// Each output is scaled by its weight, then the sum is clamped to `max_speed`.
/// Weights do not need to sum to 1.0 — they are relative.
///
/// Returns zero velocity if inputs are empty or all weights are zero.
#[cfg_attr(feature = "logging", instrument)]
#[must_use]
pub fn blend_weighted(inputs: &[WeightedSteer], max_speed: f32) -> SteerOutput {
    if inputs.is_empty() {
        return SteerOutput::default();
    }

    let mut total = Vec2::ZERO;
    let mut weight_sum = 0.0f32;

    for entry in inputs {
        total += entry.output.velocity * entry.weight;
        weight_sum += entry.weight;
    }

    if weight_sum < f32::EPSILON {
        return SteerOutput::default();
    }

    // Normalize by total weight
    let blended = total / weight_sum;

    // Clamp to max speed
    let len = blended.length();
    if len > max_speed && len > f32::EPSILON {
        SteerOutput::from_vec2(blended / len * max_speed)
    } else {
        SteerOutput::from_vec2(blended)
    }
}

/// A prioritized steering behavior entry.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PrioritizedSteer {
    /// The steering output.
    pub output: SteerOutput,
    /// Priority level (lower number = higher priority, evaluated first).
    pub priority: u32,
    /// Weight for this behavior when active.
    pub weight: f32,
}

/// Select steering output by priority groups.
///
/// Behaviors are grouped by priority level (lower = higher priority).
/// The highest-priority group whose weighted blend exceeds `threshold`
/// speed is used. If no group exceeds the threshold, the lowest-priority
/// group's output is returned as a fallback.
///
/// Within each priority group, behaviors are blended by weight.
#[cfg_attr(feature = "logging", instrument)]
#[must_use]
pub fn blend_priority(inputs: &[PrioritizedSteer], max_speed: f32, threshold: f32) -> SteerOutput {
    if inputs.is_empty() {
        return SteerOutput::default();
    }

    // Sort by priority (will work on a local copy of indices)
    let mut sorted: Vec<usize> = (0..inputs.len()).collect();
    sorted.sort_by_key(|&i| inputs[i].priority);

    let mut current_priority = inputs[sorted[0]].priority;
    let mut group_start = 0;

    for i in 0..=sorted.len() {
        let new_group = i == sorted.len() || inputs[sorted[i]].priority != current_priority;

        if new_group {
            // Evaluate this priority group
            let group: Vec<WeightedSteer> = sorted[group_start..i]
                .iter()
                .map(|&idx| WeightedSteer {
                    output: inputs[idx].output,
                    weight: inputs[idx].weight,
                })
                .collect();

            let result = blend_weighted(&group, max_speed);

            // If this group produces significant output, use it
            if result.speed() >= threshold || i == sorted.len() {
                return result;
            }

            // Move to next group
            if i < sorted.len() {
                current_priority = inputs[sorted[i]].priority;
                group_start = i;
            }
        }
    }

    SteerOutput::default()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn blend_empty() {
        let result = blend_weighted(&[], 5.0);
        assert!(result.speed() < f32::EPSILON);
    }

    #[test]
    fn blend_single() {
        let inputs = [WeightedSteer {
            output: SteerOutput::new(3.0, 4.0),
            weight: 1.0,
        }];
        let result = blend_weighted(&inputs, 10.0);
        assert!((result.speed() - 5.0).abs() < 0.01);
    }

    #[test]
    fn blend_equal_weights() {
        let inputs = [
            WeightedSteer {
                output: SteerOutput::new(4.0, 0.0),
                weight: 1.0,
            },
            WeightedSteer {
                output: SteerOutput::new(0.0, 4.0),
                weight: 1.0,
            },
        ];
        let result = blend_weighted(&inputs, 10.0);
        // Average of (4,0) and (0,4) = (2,2)
        assert!((result.velocity.x - 2.0).abs() < 0.01);
        assert!((result.velocity.y - 2.0).abs() < 0.01);
    }

    #[test]
    fn blend_unequal_weights() {
        let inputs = [
            WeightedSteer {
                output: SteerOutput::new(10.0, 0.0),
                weight: 3.0,
            },
            WeightedSteer {
                output: SteerOutput::new(0.0, 10.0),
                weight: 1.0,
            },
        ];
        let result = blend_weighted(&inputs, 100.0);
        // Weighted average: (30 + 0) / 4 = 7.5, (0 + 10) / 4 = 2.5
        assert!((result.velocity.x - 7.5).abs() < 0.01);
        assert!((result.velocity.y - 2.5).abs() < 0.01);
    }

    #[test]
    fn blend_clamps_to_max_speed() {
        let inputs = [WeightedSteer {
            output: SteerOutput::new(100.0, 0.0),
            weight: 1.0,
        }];
        let result = blend_weighted(&inputs, 5.0);
        assert!((result.speed() - 5.0).abs() < 0.01);
    }

    #[test]
    fn blend_zero_weights() {
        let inputs = [WeightedSteer {
            output: SteerOutput::new(10.0, 0.0),
            weight: 0.0,
        }];
        let result = blend_weighted(&inputs, 5.0);
        assert!(result.speed() < f32::EPSILON);
    }

    #[test]
    fn priority_empty() {
        let result = blend_priority(&[], 5.0, 0.1);
        assert!(result.speed() < f32::EPSILON);
    }

    #[test]
    fn priority_high_overrides_low() {
        let inputs = [
            PrioritizedSteer {
                output: SteerOutput::new(5.0, 0.0), // obstacle avoidance
                priority: 0,
                weight: 1.0,
            },
            PrioritizedSteer {
                output: SteerOutput::new(0.0, 5.0), // seek
                priority: 1,
                weight: 1.0,
            },
        ];
        let result = blend_priority(&inputs, 10.0, 0.1);
        // High priority (0) should win
        assert!((result.velocity.x - 5.0).abs() < 0.01);
        assert!(result.velocity.y.abs() < 0.01);
    }

    #[test]
    fn priority_falls_through() {
        let inputs = [
            PrioritizedSteer {
                output: SteerOutput::default(), // avoidance produces nothing
                priority: 0,
                weight: 1.0,
            },
            PrioritizedSteer {
                output: SteerOutput::new(0.0, 5.0), // seek
                priority: 1,
                weight: 1.0,
            },
        ];
        let result = blend_priority(&inputs, 10.0, 0.1);
        // Priority 0 produces nothing, falls through to priority 1
        assert!((result.velocity.y - 5.0).abs() < 0.01);
    }

    #[test]
    fn priority_same_level_blended() {
        let inputs = [
            PrioritizedSteer {
                output: SteerOutput::new(4.0, 0.0),
                priority: 0,
                weight: 1.0,
            },
            PrioritizedSteer {
                output: SteerOutput::new(0.0, 4.0),
                priority: 0,
                weight: 1.0,
            },
        ];
        let result = blend_priority(&inputs, 10.0, 0.1);
        // Same priority — blended
        assert!((result.velocity.x - 2.0).abs() < 0.01);
        assert!((result.velocity.y - 2.0).abs() < 0.01);
    }

    #[test]
    fn blend_weighted_serde_roundtrip() {
        let ws = WeightedSteer {
            output: SteerOutput::new(1.0, 2.0),
            weight: 0.5,
        };
        let json = serde_json::to_string(&ws).unwrap();
        let deserialized: WeightedSteer = serde_json::from_str(&json).unwrap();
        assert!((deserialized.weight - 0.5).abs() < f32::EPSILON);
    }

    #[test]
    fn blend_priority_serde_roundtrip() {
        let ps = PrioritizedSteer {
            output: SteerOutput::new(1.0, 2.0),
            priority: 3,
            weight: 0.5,
        };
        let json = serde_json::to_string(&ps).unwrap();
        let deserialized: PrioritizedSteer = serde_json::from_str(&json).unwrap();
        assert_eq!(deserialized.priority, 3);
    }

    #[test]
    fn blend_opposing_forces_cancel() {
        let inputs = [
            WeightedSteer {
                output: SteerOutput::new(5.0, 0.0),
                weight: 1.0,
            },
            WeightedSteer {
                output: SteerOutput::new(-5.0, 0.0),
                weight: 1.0,
            },
        ];
        let result = blend_weighted(&inputs, 10.0);
        assert!(result.speed() < f32::EPSILON);
    }
}
