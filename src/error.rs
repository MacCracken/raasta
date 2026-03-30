//! Error types for raasta navigation operations.

use core::fmt;

/// Errors that can occur during navigation operations.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum NavError {
    /// Grid dimensions overflow `usize`.
    GridOverflow { width: usize, height: usize },
    /// An empty mesh was provided where at least one polygon is required.
    EmptyMesh,
}

impl fmt::Display for NavError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::GridOverflow { width, height } => {
                write!(f, "grid dimensions overflow: {width} x {height}")
            }
            Self::EmptyMesh => write!(f, "operation requires a non-empty mesh"),
        }
    }
}

impl std::error::Error for NavError {}
