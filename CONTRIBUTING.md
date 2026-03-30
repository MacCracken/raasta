# Contributing to Raasta

## Getting Started

```bash
git clone https://github.com/MacCracken/raasta
cd raasta
cargo test
```

## Development Workflow

1. Fork and create a feature branch
2. Write code + tests
3. Run checks:

```bash
cargo fmt --check
cargo clippy --all-features --all-targets -- -D warnings
cargo test --all-features
RUSTDOCFLAGS="-D warnings" cargo doc --all-features --no-deps
./scripts/coverage-check.sh  # must be >= 80%
```

4. Run benchmarks if touching performance-critical code:

```bash
./scripts/bench-history.sh
```

5. Update CHANGELOG.md
6. Open a PR

## Code Standards

- `#[must_use]` on all pure functions
- `#[inline]` on hot-path functions
- `#[non_exhaustive]` on all public enums
- `#[cfg_attr(feature = "logging", instrument)]` on public operations
- No `unwrap()` or `panic!()` in library code
- `write!` over `format!` to avoid temporary allocations
- Vec arena over HashMap where indices are sequential
- All public types must be serializable (serde)

## License

By contributing, you agree that your contributions will be licensed under GPL-3.0-only.
