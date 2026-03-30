# Raasta Roadmap

> **Raasta** (راستہ) — navigation and pathfinding engine for AGNOS

## v1.0 — Released 2026-03-29

All planned features shipped. See [CHANGELOG.md](../../CHANGELOG.md) for details.

- 34 source modules, 544 tests, 51 benchmarks
- 90%+ test coverage
- Full clippy + fmt + doc clean

## Future

Demand-gated — features added when consumers need them.

- [ ] SIMD-vectorized RVO inner loop (explicit SIMD intrinsics beyond auto-vectorization)
- [ ] NavMesh auto-LOD (simplify distant tiles)
- [ ] Predictive avoidance (velocity obstacles with time-to-collision ranking)
- [ ] Navigation event system (path invalidated, waypoint reached, corridor breach)
- [ ] WebAssembly target validation + optimization
- [ ] C FFI bindings for non-Rust consumers

## Dependency Map

```
raasta (navigation/pathfinding)
  └── hisab (math — Vec2, Vec3, geometry)
        └── glam (SIMD linear algebra)
```
