//! Error types for raasta navigation operations.

use core::fmt;

use serde::{Deserialize, Serialize};

/// Errors that can occur during navigation operations.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[non_exhaustive]
pub enum NavError {
    /// Grid dimensions overflow `usize`.
    GridOverflow { width: usize, height: usize },
    /// An empty mesh was provided where at least one polygon is required.
    EmptyMesh,
    /// Cell size must be positive.
    InvalidCellSize { value: f32 },
}

impl fmt::Display for NavError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::GridOverflow { width, height } => {
                write!(f, "grid dimensions overflow: {width} x {height}")
            }
            Self::EmptyMesh => write!(f, "operation requires a non-empty mesh"),
            Self::InvalidCellSize { value } => {
                write!(f, "invalid cell size: {value} (must be positive)")
            }
        }
    }
}

impl std::error::Error for NavError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn display_grid_overflow() {
        let e = NavError::GridOverflow {
            width: 10,
            height: 20,
        };
        assert_eq!(e.to_string(), "grid dimensions overflow: 10 x 20");
    }

    #[test]
    fn display_empty_mesh() {
        let e = NavError::EmptyMesh;
        assert_eq!(e.to_string(), "operation requires a non-empty mesh");
    }

    #[test]
    fn display_invalid_cell_size() {
        let e = NavError::InvalidCellSize { value: -1.0 };
        assert_eq!(e.to_string(), "invalid cell size: -1 (must be positive)");
    }

    #[test]
    fn serde_roundtrip() {
        let variants = [
            NavError::GridOverflow {
                width: 5,
                height: 10,
            },
            NavError::EmptyMesh,
            NavError::InvalidCellSize { value: 0.0 },
        ];
        for variant in &variants {
            let json = serde_json::to_string(variant).unwrap();
            let deserialized: NavError = serde_json::from_str(&json).unwrap();
            assert_eq!(&deserialized, variant);
        }
    }
}
