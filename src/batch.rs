//! Path request batching — queue and process pathfinding requests with per-frame budgets.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use serde::{Deserialize, Serialize};

use crate::grid::{GridPos, NavGrid};
use crate::incremental::{IncrementalGridPath, IncrementalStatus};
use crate::path::PathResult;

#[cfg(feature = "logging")]
use tracing::instrument;

/// Unique identifier for a batched path request.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct PathRequestId(pub u32);

/// Priority level for a path request (lower number = higher priority).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct RequestPriority(pub u32);

/// A queued path request with priority.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QueuedRequest {
    /// Unique request ID.
    pub id: PathRequestId,
    /// Start grid position.
    pub start: GridPos,
    /// Goal grid position.
    pub goal: GridPos,
    /// Priority (lower = processed first).
    pub priority: RequestPriority,
}

/// Internal entry for the priority queue.
struct PendingEntry {
    id: PathRequestId,
    priority: RequestPriority,
}

impl PartialEq for PendingEntry {
    fn eq(&self, other: &Self) -> bool {
        self.priority == other.priority
    }
}

impl Eq for PendingEntry {}

impl PartialOrd for PendingEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for PendingEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reversed: lower priority number = higher heap priority
        other.priority.cmp(&self.priority)
    }
}

/// A completed path result with its request ID.
#[derive(Debug, Clone)]
pub struct BatchedResult {
    /// The request ID this result belongs to.
    pub id: PathRequestId,
    /// The computed path result.
    pub result: PathResult,
}

/// Batched path request processor.
///
/// Queues pathfinding requests with priorities and processes them
/// within a per-frame iteration budget. Higher-priority requests
/// are processed first. Each request uses incremental (time-sliced)
/// A* so it can be spread across multiple frames.
///
/// # Example
///
/// ```ignore
/// let mut batcher = PathBatcher::new();
///
/// // Queue requests
/// batcher.enqueue(GridPos::new(0, 0), GridPos::new(99, 99), RequestPriority(0), &grid);
/// batcher.enqueue(GridPos::new(10, 10), GridPos::new(50, 50), RequestPriority(1), &grid);
///
/// // Each frame, process with budget
/// let completed = batcher.process(&grid, 200);
/// for result in &completed {
///     println!("Request {} completed: {:?}", result.id.0, result.result.status);
/// }
/// ```
#[derive(Debug)]
pub struct PathBatcher {
    /// Priority queue of pending request IDs.
    pending: BinaryHeap<PendingEntry>,
    /// Active queries being computed (request_id -> incremental query).
    active: Vec<(PathRequestId, IncrementalGridPath)>,
    /// Queued requests waiting to become active.
    queued: Vec<QueuedRequest>,
    /// Maximum concurrent active queries.
    max_active: usize,
    /// Next request ID.
    next_id: u32,
}

// Manual Debug for PendingEntry since it's not derived
impl std::fmt::Debug for PendingEntry {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PendingEntry")
            .field("id", &self.id)
            .field("priority", &self.priority)
            .finish()
    }
}

impl PathBatcher {
    /// Create a new batcher with default settings.
    ///
    /// `max_active` defaults to 4, limiting how many queries can be computed concurrently.
    /// Remaining queued requests wait until an active slot opens.
    #[cfg_attr(feature = "logging", instrument)]
    #[must_use]
    pub fn new() -> Self {
        Self {
            pending: BinaryHeap::new(),
            active: Vec::new(),
            queued: Vec::new(),
            max_active: 4,
            next_id: 0,
        }
    }

    /// Create a batcher with a custom concurrent query limit.
    #[cfg_attr(feature = "logging", instrument)]
    #[must_use]
    pub fn with_max_active(max_active: usize) -> Self {
        Self {
            pending: BinaryHeap::new(),
            active: Vec::new(),
            queued: Vec::new(),
            max_active: max_active.max(1),
            next_id: 0,
        }
    }

    /// Queue a new pathfinding request.
    ///
    /// Returns the request ID. The request will be processed when
    /// `process()` is called, in priority order.
    ///
    /// Returns `None` if start or goal is unwalkable.
    #[cfg_attr(feature = "logging", instrument(skip(self, grid)))]
    pub fn enqueue(
        &mut self,
        start: GridPos,
        goal: GridPos,
        priority: RequestPriority,
        grid: &NavGrid,
    ) -> Option<PathRequestId> {
        // Validate start/goal
        if !grid.is_walkable(start.x, start.y) || !grid.is_walkable(goal.x, goal.y) {
            return None;
        }

        let id = PathRequestId(self.next_id);
        self.next_id += 1;

        self.queued.push(QueuedRequest {
            id,
            start,
            goal,
            priority,
        });

        self.pending.push(PendingEntry { id, priority });

        Some(id)
    }

    /// Cancel a pending request. Returns `true` if the request was found and removed.
    pub fn cancel(&mut self, id: PathRequestId) -> bool {
        // Remove from queued
        if let Some(pos) = self.queued.iter().position(|r| r.id == id) {
            self.queued.swap_remove(pos);
            return true;
        }
        // Remove from active
        if let Some(pos) = self.active.iter().position(|(rid, _)| *rid == id) {
            self.active.swap_remove(pos);
            return true;
        }
        false
    }

    /// Number of pending + active requests.
    #[must_use]
    pub fn request_count(&self) -> usize {
        self.queued.len() + self.active.len()
    }

    /// Number of currently active (being computed) queries.
    #[must_use]
    pub fn active_count(&self) -> usize {
        self.active.len()
    }

    /// Number of queued (waiting) requests.
    #[must_use]
    pub fn queued_count(&self) -> usize {
        self.queued.len()
    }

    /// Process queued requests within an iteration budget.
    ///
    /// `max_iterations` is the total A* node expansion budget for this frame,
    /// distributed among active queries. Higher-priority queries get budget first.
    ///
    /// Returns a list of completed results from this frame.
    #[cfg_attr(feature = "logging", instrument(skip(self, grid)))]
    #[must_use]
    pub fn process(&mut self, grid: &NavGrid, max_iterations: u32) -> Vec<BatchedResult> {
        // Promote queued requests to active
        self.promote_queued(grid);

        if self.active.is_empty() {
            return Vec::new();
        }

        let mut completed = Vec::new();
        let mut remaining_budget = max_iterations;

        // Distribute budget: give each active query a fair share
        let per_query = (remaining_budget / self.active.len() as u32).max(1);

        let mut i = 0;
        while i < self.active.len() {
            if remaining_budget == 0 {
                break;
            }

            let budget = per_query.min(remaining_budget);
            let (id, ref mut query) = self.active[i];
            let status = query.step(grid, budget);
            remaining_budget = remaining_budget.saturating_sub(budget);

            match status {
                IncrementalStatus::Found | IncrementalStatus::NotFound => {
                    let result = query.to_path_result(grid);
                    completed.push(BatchedResult { id, result });
                    self.active.swap_remove(i);
                    // Don't increment i — swap_remove moved the last element here
                }
                IncrementalStatus::InProgress => {
                    i += 1;
                }
            }
        }

        // Try to promote more queued requests if slots opened
        self.promote_queued(grid);

        completed
    }

    /// Promote highest-priority queued requests to active.
    fn promote_queued(&mut self, grid: &NavGrid) {
        while self.active.len() < self.max_active {
            // Find highest priority queued request
            let entry = match self.pending.pop() {
                Some(e) => e,
                None => break,
            };

            // Find and remove from queued list
            let pos = match self.queued.iter().position(|r| r.id == entry.id) {
                Some(p) => p,
                None => continue, // Was cancelled
            };
            let request = self.queued.swap_remove(pos);

            // Create incremental query
            if let Some(query) = IncrementalGridPath::new(grid, request.start, request.goal) {
                self.active.push((request.id, query));
            }
            // If query creation fails (shouldn't happen since we validated), skip silently
        }
    }

    /// Clear all pending and active requests.
    pub fn clear(&mut self) {
        self.pending.clear();
        self.active.clear();
        self.queued.clear();
    }
}

impl Default for PathBatcher {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::path::PathStatus;

    #[test]
    fn batcher_enqueue_process() {
        let grid = NavGrid::new(20, 20, 1.0);
        let mut batcher = PathBatcher::new();

        let id = batcher.enqueue(
            GridPos::new(0, 0),
            GridPos::new(19, 19),
            RequestPriority(0),
            &grid,
        );
        assert!(id.is_some());
        assert_eq!(batcher.request_count(), 1);

        // Process with large budget — should complete
        let completed = batcher.process(&grid, 10000);
        assert_eq!(completed.len(), 1);
        assert_eq!(completed[0].result.status, PathStatus::Found);
        assert_eq!(batcher.request_count(), 0);
    }

    #[test]
    fn batcher_priority_order() {
        let grid = NavGrid::new(10, 10, 1.0);
        let mut batcher = PathBatcher::with_max_active(1); // Only 1 active at a time

        // Enqueue low priority first, then high
        let _low = batcher
            .enqueue(
                GridPos::new(0, 0),
                GridPos::new(9, 0),
                RequestPriority(10),
                &grid,
            )
            .unwrap();
        let high = batcher
            .enqueue(
                GridPos::new(0, 0),
                GridPos::new(0, 9),
                RequestPriority(0),
                &grid,
            )
            .unwrap();

        // Process — high priority should complete first
        let completed = batcher.process(&grid, 10000);
        assert!(!completed.is_empty());
        assert_eq!(completed[0].id, high);
    }

    #[test]
    fn batcher_multi_frame() {
        let grid = NavGrid::new(50, 50, 1.0);
        let mut batcher = PathBatcher::new();

        batcher.enqueue(
            GridPos::new(0, 0),
            GridPos::new(49, 49),
            RequestPriority(0),
            &grid,
        );

        // Process with tiny budget — should take multiple frames
        let mut total_completed = Vec::new();
        for _ in 0..100 {
            let completed = batcher.process(&grid, 10);
            total_completed.extend(completed);
            if !total_completed.is_empty() {
                break;
            }
        }

        assert_eq!(total_completed.len(), 1);
        assert_eq!(total_completed[0].result.status, PathStatus::Found);
    }

    #[test]
    fn batcher_cancel() {
        let grid = NavGrid::new(10, 10, 1.0);
        let mut batcher = PathBatcher::new();

        let id = batcher
            .enqueue(
                GridPos::new(0, 0),
                GridPos::new(9, 9),
                RequestPriority(0),
                &grid,
            )
            .unwrap();

        assert!(batcher.cancel(id));
        assert_eq!(batcher.request_count(), 0);
    }

    #[test]
    fn batcher_unwalkable_rejected() {
        let mut grid = NavGrid::new(10, 10, 1.0);
        grid.set_walkable(0, 0, false);

        let mut batcher = PathBatcher::new();
        let id = batcher.enqueue(
            GridPos::new(0, 0),
            GridPos::new(9, 9),
            RequestPriority(0),
            &grid,
        );
        assert!(id.is_none());
    }

    #[test]
    fn batcher_no_path() {
        let mut grid = NavGrid::new(10, 10, 1.0);
        for y in 0..10 {
            grid.set_walkable(5, y, false);
        }

        let mut batcher = PathBatcher::new();
        batcher.enqueue(
            GridPos::new(0, 0),
            GridPos::new(9, 9),
            RequestPriority(0),
            &grid,
        );

        let completed = batcher.process(&grid, 100_000);
        assert_eq!(completed.len(), 1);
        assert_eq!(completed[0].result.status, PathStatus::NotFound);
    }

    #[test]
    fn batcher_multiple_requests() {
        let grid = NavGrid::new(10, 10, 1.0);
        let mut batcher = PathBatcher::new();

        for i in 0..5 {
            batcher.enqueue(
                GridPos::new(0, 0),
                GridPos::new(9, i),
                RequestPriority(i as u32),
                &grid,
            );
        }

        assert_eq!(batcher.request_count(), 5);

        // Process all
        let mut total = Vec::new();
        for _ in 0..100 {
            total.extend(batcher.process(&grid, 1000));
            if batcher.request_count() == 0 {
                break;
            }
        }

        assert_eq!(total.len(), 5);
        for result in &total {
            assert_eq!(result.result.status, PathStatus::Found);
        }
    }

    #[test]
    fn batcher_clear() {
        let grid = NavGrid::new(10, 10, 1.0);
        let mut batcher = PathBatcher::new();

        batcher.enqueue(
            GridPos::new(0, 0),
            GridPos::new(9, 9),
            RequestPriority(0),
            &grid,
        );
        batcher.enqueue(
            GridPos::new(1, 1),
            GridPos::new(8, 8),
            RequestPriority(1),
            &grid,
        );

        batcher.clear();
        assert_eq!(batcher.request_count(), 0);
    }

    #[test]
    fn batcher_max_active_limit() {
        let grid = NavGrid::new(50, 50, 1.0);
        let mut batcher = PathBatcher::with_max_active(2);

        for i in 0..5 {
            batcher.enqueue(
                GridPos::new(0, 0),
                GridPos::new(49, i),
                RequestPriority(i as u32),
                &grid,
            );
        }

        // After first process, should have at most 2 active
        let _ = batcher.process(&grid, 1);
        assert!(batcher.active_count() <= 2);
    }

    #[test]
    fn request_priority_serde_roundtrip() {
        let p = RequestPriority(5);
        let json = serde_json::to_string(&p).unwrap();
        let deserialized: RequestPriority = serde_json::from_str(&json).unwrap();
        assert_eq!(deserialized.0, 5);
    }

    #[test]
    fn path_request_id_serde_roundtrip() {
        let id = PathRequestId(42);
        let json = serde_json::to_string(&id).unwrap();
        let deserialized: PathRequestId = serde_json::from_str(&json).unwrap();
        assert_eq!(deserialized.0, 42);
    }
}
