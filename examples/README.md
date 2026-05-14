# csgrs Examples

These examples are intentionally split by topic so each one can be read and run
without digging through a large demo binary.

## Geometry Examples

- `basic_shapes.rs`: cube, sphere, cylinder, ellipsoid, and torus.
- `basic2d_shapes_and_offsetting.rs`: 2D sketches and offset variants.
- `boolean_operations.rs`: union, difference, intersection, and xor.
- `transformations.rs`: translate, rotate, scale, mirror, and matrix transforms.
- `convex_hull.rs`: convex hull over a boolean model.
- `minkowski_sum.rs`: Minkowski sum for a rounded-cube style result.
- `extrude.rs`: extrude, vector extrude, revolve, sweep, and loft.

## Utility Examples

- `adjacency_demo.rs`: mesh connectivity, smoothing, and quality analysis.
- `multi_format_export.rs`: OBJ, PLY, AMF, GLTF, and related export checks.

## Running

```bash
cargo run --example basic_shapes
cargo run --example boolean_operations
cargo run --example extrude
```

Most examples write output under `stl/examples/`. Check that every maintained
example still compiles with:

```bash
cargo check --examples
```
