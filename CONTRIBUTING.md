# Contributing to Raasta

Thank you for your interest in contributing to Raasta!

## Development

```bash
make check    # fmt + clippy + test + audit
make bench    # run benchmarks
make coverage # generate coverage report
```

## Guidelines

- Follow [First-Party Standards](https://github.com/MacCracken/agnosticos/blob/main/docs/development/applications/first-party-standards.md)
- All public types must be serializable (serde)
- Add tests for new functionality
- Add benchmarks for performance-critical paths
- Use `tracing` for structured logging

## License

By contributing, you agree that your contributions will be licensed under GPL-3.0.
