# 002 — Use hisab Types in Public API

## Status
Accepted

## Context
raasta needs 2D/3D vector types. Options: raw `[f32; 2]` arrays, glam types directly, or hisab wrapper types.

## Decision
All public APIs use `hisab::Vec2` and `hisab::Vec3`. Re-exported as `raasta::Vec2` and `raasta::Vec3`.

## Consequences
- Consistent types across the AGNOS ecosystem
- Consumers don't need a direct glam dependency
- SIMD performance via glam under the hood
- Type conversions needed at boundaries with non-AGNOS code
