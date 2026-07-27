# hypergraphics

`hypergraphics` provides exact scene geometry and a small unlit OpenGL/WebGL
renderer for the Hyper geometry stack. Positions remain `hyperreal::Real`
values until an explicit export or projection converts them to `f64`; GPU
uploads then narrow them to `f32`.

## Quick start

Build and triangulate an exact square, then export it for a renderer:

```rust
use hypergraphics::{Color3, Point2, Real, polygon_surface_mesh};

fn main() -> hypergraphics::Result<()> {
    let square = vec![
        Point2::new(Real::from(0), Real::from(0)),
        Point2::new(Real::from(2), Real::from(0)),
        Point2::new(Real::from(2), Real::from(2)),
        Point2::new(Real::from(0), Real::from(2)),
    ];
    let orange = Color3::new(0.8, 0.4, 0.1)?;
    let mesh = polygon_surface_mesh(&square, &[], Real::zero(), orange)?;

    assert_eq!(mesh.triangle_count(), 2);
    let render_vertices = mesh.to_render_vertices64()?;
    assert_eq!(render_vertices.len(), 6);
    Ok(())
}
```

## Core API

- `ExactMesh`, `ExactVertex`, and `Primitive` own colored line or triangle
  geometry without converting its positions to primitive floats.
- `axes_mesh`, `grid_mesh`, and `polygon_surface_mesh` construct common scene
  geometry. Polygon triangulation is delegated to `hypertri`.
- `ExactMesh::triangle_orientation_against` evaluates a robust `hyperlimit`
  orientation predicate against one triangle and retains repeated triangle
  queries internally.
- `ExactCamera`, `Viewport`, and `Projection64` provide orbit-camera projection
  and screen/world conversion.
- `GpuColoredMesh` and `UnlitProgram` upload and draw the exported `xyz rgb`
  stream through `glow`.

Topology and geometric decisions must use the exact scene values and predicate
APIs. `Projection64`, `RenderVertex64`, and the flat-buffer export methods are
lossy presentation boundaries and must not be used as topology certificates.
The backend's methods are `unsafe` because callers must keep the owning graphics
context current and observe its thread-affinity rules.
GPU meshes and programs expose consuming, context-bound `destroy` methods so GL
objects can be released while their owning context is current.

Applications that only need the primitive-float renderer can disable the exact
geometry stack:

```toml
hypergraphics = { version = "0.1", default-features = false }
```

That configuration retains `Projection64`, `RenderVertex64`, `GpuColoredMesh`,
and `UnlitProgram`. `Projection64::try_from_column_major_f32` accepts external
camera matrices, while `GpuColoredMesh::upload_xyz_rgb_f32` uploads existing
interleaved buffers without an intermediate repack.

Criterion results and the reference-by-reference implementation audit are in
[`PERFORMANCE.md`](PERFORMANCE.md).

The renderer originated in CopperForge's `render3d` module, which credits the
MIT-licensed `alumina-interface` project by Timothy Schmidt. This crate retains
the compact renderer architecture while moving geometry ownership and robust
decisions into the Hyper stack; Alumina now consumes the shared backend directly.

## References

- [OpenGL 3.3 Core Specification](https://registry.khronos.org/OpenGL/specs/gl/glspec33.core.pdf)
- [WebGL specifications](https://registry.khronos.org/webgl/specs/latest/)
- [`glow`: cross-platform GL bindings](https://github.com/grovesNL/glow)
- [`nalgebra` documentation](https://nalgebra.rs/docs/)
- Jonathan Richard Shewchuk, [*Adaptive Precision Floating-Point Arithmetic and
  Fast Robust Geometric Predicates*](https://people.eecs.berkeley.edu/~jrs/papers/robustr.pdf)

Hyper stack: [hyperreal](https://github.com/timschmidt/hyperreal) ·
[hyperlattice](https://github.com/timschmidt/hyperlattice) ·
[hyperlimit](https://github.com/timschmidt/hyperlimit) ·
[hypertri](https://github.com/timschmidt/hypertri)

## Development

```sh
cargo fmt --all -- --check
cargo test --all-targets --all-features
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --all-features
cargo bench --bench graphics
```
