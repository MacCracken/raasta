# Testing Guide

## Running Tests

```bash
# All tests (default features)
cargo test

# With parallel feature
cargo test --features parallel

# All features
cargo test --all-features

# Specific module
cargo test grid
cargo test mesh
cargo test rvo
```

## Coverage

```bash
# Check coverage meets 80% threshold
./scripts/coverage-check.sh

# Full coverage report
cargo tarpaulin --all-features
```

## Benchmarks

```bash
# Run all 51 benchmarks with history tracking
./scripts/bench-history.sh

# Run specific benchmark
cargo bench -- grid_astar

# Results saved to bench-history.csv and benchmarks.md
```

## Test Counts

- 536 unit tests (without parallel feature)
- 6 integration tests
- 2 doc tests
- 9 parallel tests (with `--features parallel`)
- **544 total** (all features)
