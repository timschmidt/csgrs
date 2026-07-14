# Voxelis Math

`voxelis-math` provides the floating-point intersection helpers used by the
legacy Voxelis OBJ voxelizer.

## API

- `triangle_cube_intersection` tests a triangle against an axis-aligned cube.
- `point_in_or_on_cube` and `point_in_or_on_triangle` provide containment
  helpers.
- `edge_quad_intersection` and `point_in_quad` support the triangle/cube test.

All inputs use `glam::DVec3`. These routines use explicit floating tolerances
and are suitable for sampled voxelization and previews, not proof-producing
topology. Use `hypervoxel` with `hyperlimit` predicates when exact or certified
classification is required.

```rust
use glam::DVec3;
use voxelis_math::triangle_cube_intersection;

let triangle = (
    DVec3::new(0.0, 0.0, 0.5),
    DVec3::new(1.0, 0.0, 0.5),
    DVec3::new(0.0, 1.0, 0.5),
);
let cube = (DVec3::ZERO, DVec3::ONE);
assert!(triangle_cube_intersection(triangle, cube));
```

## Development

```sh
cargo test -p voxelis-math --all-features --locked
cargo clippy -p voxelis-math --all-targets --all-features --locked -- -D warnings
```

## References

- Tomas Akenine-Möller, [“Fast 3D Triangle-Box Overlap Testing”](https://doi.org/10.1145/1198555.1198748), 2005.
- Philippe Guigue and Olivier Devillers, [“Fast and Robust Triangle-Triangle Overlap Test Using Orientation Predicates”](https://doi.org/10.1080/10867651.2003.10487580), 2003.

See [HyperVoxel](../README.md) for the exact-aware layer and the wider Hyper
ecosystem.
