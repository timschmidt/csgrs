# hypergraphics

`hypergraphics` connects exact Hyper geometry to a small unlit OpenGL/WebGL
renderer. Scene positions remain `hyperreal::Real` values while geometry is
constructed, triangulated, and queried. Conversion to `f64` and then `f32`
happens only through named camera, render-export, and GPU-upload APIs.

The crate owns colored scene meshes, camera projection, and the graphics
backend. It does not own windows, event loops, materials, lighting, or general
mesh topology. Applications provide a current [`glow`](https://github.com/grovesNL/glow)
context when they use the GPU layer.

## Why an exact scene layer?

Rendering ultimately needs fixed-size floating-point values, but construction
and topology often need stronger guarantees. If geometry is rounded before a
polygon is triangulated or a point is classified against a triangle, a display
approximation can accidentally become a geometric decision.

`hypergraphics` keeps those responsibilities separate:

```text
Real geometry
    │  construct, triangulate, classify
    ▼
ExactMesh
    │  explicit lossy export
    ▼
RenderVertex64 / Projection64
    │  checked narrowing at the graphics boundary
    ▼
GpuColoredMesh + UnlitProgram
```

Use the exact layer when the scene is produced by CAD, geometry, or simulation
code. Use the backend-only configuration when the application already owns
finite interleaved vertex data and projection matrices.

## Primary types

| Type | Purpose |
| --- | --- |
| `ExactMesh` | Colored line or triangle vertices whose positions remain exact. |
| `ExactVertex` | One `Point3` position and one finite linear-RGB `Color3`. |
| `TriangleOrientation` | Exact/certified classification of a point against an oriented triangle. |
| `ExactCamera` | Orbit-camera state stored as exact values until projection. |
| `Viewport`, `ScreenPoint` | Checked screen-space dimensions and coordinates. |
| `ApproxPoint3` | Finite approximate world point produced by `f64` camera math. |
| `Projection64` | Explicit `f64` view-projection matrix at the rendering boundary. |
| `RenderVertex64` | One exported `f64` position with an `f32` color. |
| `GpuColoredMesh` | Context-owned interleaved `xyz rgb` vertex buffer. |
| `UnlitProgram` | OpenGL, OpenGL ES, or WebGL-compatible unlit shader program. |
| `Error`, `Result<T>` | Checked construction, conversion, triangulation, camera, and backend failures. |

`Real`, `Rational`, `Point2`, `Point3`, `Vector2`, and `Vector3` are re-exported
with the default `exact` feature.

## Quick start

Create a project and add the crate:

```sh
cargo new exact-scene
cd exact-scene
cargo add hypergraphics
```

Replace `src/main.rs` with this complete example:

<!-- quickstart:start -->
```rust
use hypergraphics::{
    Color3, Point2, PredicatePolicy, Real, TriangulationContext, polygon_surface_mesh,
};

fn main() -> hypergraphics::Result<()> {
    let square = [
        Point2::new(Real::from(0), Real::from(0)),
        Point2::new(Real::from(2), Real::from(0)),
        Point2::new(Real::from(2), Real::from(2)),
        Point2::new(Real::from(0), Real::from(2)),
    ];
    let orange = Color3::new(0.8, 0.4, 0.1)?;
    let context = TriangulationContext::new(PredicatePolicy::STRICT);
    let mesh = polygon_surface_mesh(&context, &square, &[], Real::zero(), orange)?.into_value();
    let render_vertices = mesh.to_render_vertices64()?;

    println!(
        "{} exact triangles; {} render vertices",
        mesh.triangle_count(),
        render_vertices.len()
    );
    Ok(())
}
```
<!-- quickstart:end -->

Run it with:

```sh
cargo run
```

It prints:

```text
2 exact triangles; 6 render vertices
```

The same source is checked in as
[`examples/readme_quickstart.rs`](examples/readme_quickstart.rs) and is compiled
by the crate test suite.

## API guide

### Build exact scene geometry

| Task | API |
| --- | --- |
| Construct vertices and meshes | `ExactVertex::new`, `ExactMesh::new`, `ExactMesh::empty` |
| Inspect or edit a mesh | `primitive`, `vertices`, `vertices_mut`, `push`, `vertex_count`, `triangle_count` |
| Build scene helpers | `axes_mesh`, `grid_mesh`, `polygon_surface_mesh`, `triangle_mesh` |
| Select draw topology | `Primitive::Lines`, `Primitive::Triangles` |
| Construct colors | `Color3::new`, `Color3::{RED, GREEN, BLUE}`, `Color3::to_array` |

`polygon_surface_mesh` accepts an explicit immutable triangulation context, one
exact outer polygon plus optional hole-start indices, and lifts Hypertri's
triangles to a caller-selected exact Z plane. Its outcome retains whether the
selected policy consumed approximate-512 equality.

`grid_mesh` takes an unsigned half-step count and returns an error if its exact
coordinate and vertex storage cannot be allocated safely.

### Query exact geometry

| Task | API |
| --- | --- |
| Classify a point against a triangle | `ExactMesh::triangle_orientation_against` |
| Select a predicate policy | Required `PredicatePolicy` argument |
| Read the result | `TriangleOrientation::{Negative, Coplanar, Positive, Unknown}` |

Triangle queries require `Primitive::Triangles` and a valid triangle index.
Repeated queries may reuse retained oriented-plane evidence, but the returned
classification remains exact, certified, or explicitly unknown.

### Cross the rendering boundary

| Task | API |
| --- | --- |
| Export structured vertices | `ExactMesh::to_render_vertices64` |
| Export an interleaved CPU buffer | `ExactMesh::to_xyz_rgb_f64` |
| Flatten one render vertex | `RenderVertex64::to_xyz_rgb` |
| Import an external projection | `Projection64::try_from_column_major_f32` |
| Export an OpenGL uniform matrix | `Projection64::to_f32_array` |

Exports fail rather than emitting `NaN`, infinity, or an overflowing `f32`.
The resulting values are presentation data and must not be fed back into
topology decisions.

### Project and unproject

| Task | API |
| --- | --- |
| Construct checked screen values | `Viewport::new`, `ScreenPoint::new` |
| Read screen values | Viewport and screen-point accessors, `Viewport::aspect`, `Viewport::center` |
| Create and edit an exact camera | `ExactCamera::default`, `orbit`, `zoom_by`, `reset_top_down` |
| Build a projection | `ExactCamera::projection64` |
| Select a camera predicate policy | Required `PredicatePolicy` argument |
| Project a world point | `ExactCamera::project_point`, `camera::project_point` |
| Approximately unproject onto the XY plane | `camera::unproject_to_z0` → `ApproxPoint3` |
| Access the native matrix | `Projection64::new`, `Projection64::matrix` |

Camera projection is intentionally lossy: exact camera parameters and world
points are converted to `f64` when `Projection64` is built. Domain decisions
are made first by Hyperlimit and then revalidated after lowering so a valid
exact interval cannot collapse into an invalid primitive-float projection. An
undecided explicit policy returns `Error::IndeterminatePredicate`.

Screen coordinates use a top-left origin with Y increasing downward.
Unprojection inverts the already-lossy `f64` matrix, rejects ill-conditioned
matrices and nearly parallel rays, and returns `ApproxPoint3` so the result
cannot be mistaken for exact geometric evidence.

### Upload and draw

The backend APIs live in `hypergraphics::backend`:

| Task | API |
| --- | --- |
| Allocate a GPU mesh | `GpuColoredMesh::new` |
| Upload an exact mesh | `upload_exact_mesh` |
| Upload structured render vertices | `upload_render_vertices64` |
| Upload an existing interleaved buffer | `upload_xyz_rgb_f32` |
| Inspect or draw | `is_empty`, `draw` |
| Compile and bind the shader | `UnlitProgram::new`, `bind`, `set_alpha` |
| Release GL objects | `GpuColoredMesh::destroy`, `UnlitProgram::destroy` |

Allocation, upload, draw, bind, and destruction are `unsafe` because
`hypergraphics` cannot prove that a `glow::Context` is current, belongs to the
object, or is being used on its required thread. The caller must maintain those
invariants.

GLSL ES 1.00/WebGL 1 additionally requires `OES_vertex_array_object`.
`GpuColoredMesh::new` reports `Error::UnsupportedBackend` when that extension
is unavailable. ES 1.00 attribute locations are bound explicitly to the same
indices used by the mesh layout.

## Features

| Feature | Default | Effect |
| --- | --- | --- |
| `exact` | yes | Enables Hyperreal geometry, exact queries, triangulation, scene helpers, and `ExactCamera`. |
| `dispatch-trace` | no | Enables lower-stack predicate/scalar tracing for development and benchmarks; also enables `exact`. |

Renderer-only applications can avoid the exact geometry dependencies:

```toml
[dependencies]
hypergraphics = { version = "0.1.0", default-features = false }
```

That configuration retains `Primitive`, `Color3`, `RenderVertex64`, `Viewport`,
`ScreenPoint`, `ApproxPoint3`, `Projection64`, `GpuColoredMesh`, and
`UnlitProgram`. Supply camera matrices through
`Projection64::try_from_column_major_f32` and vertex data through
`GpuColoredMesh::upload_xyz_rgb_f32`.

## Errors and guarantees

- Primitive color, viewport, screen-point, render-vertex, matrix, and alpha
  constructors reject non-finite values; their validated fields are private.
- Camera zoom, field of view, and clipping planes are decided through the
  centralized Hyperlimit policy and revalidated at the `f64` boundary.
- Primitive-float unprojection uses explicit numerical conditioning and never
  promotes its result to an exact point implicitly.
- Exact exports fail when a coordinate cannot be represented as a finite
  primitive float.
- GPU uploads reject partial vertices and counts that exceed OpenGL's signed
  draw-count range.
- `hypergraphics` does not create a window or graphics context.
- `ExactMesh` is an unindexed draw stream. General indexed mesh ownership and
  Boolean topology belong to Hypermesh and CSGRS.
- The unlit backend supports desktop GLSL 3.30, GLSL ES 3.00/WebGL 2, and GLSL
  ES 1.00/WebGL 1 with `OES_vertex_array_object`.

## Ecosystem and further documentation

- [Hyperreal](https://github.com/timschmidt/hyperreal) supplies exact scalar
  values.
- [Hyperlattice](https://github.com/timschmidt/hyperlattice) supplies points,
  vectors, and exact camera parameters.
- [Hyperlimit](https://github.com/timschmidt/hyperlimit) supplies robust
  orientation predicates.
- [Hypertri](https://github.com/timschmidt/hypertri) triangulates exact planar
  polygons.

API details are available through
[`cargo doc --open`](https://doc.rust-lang.org/cargo/commands/cargo-doc.html).
Benchmark methodology and implementation experiments are recorded in
[`PERFORMANCE.md`](PERFORMANCE.md).

## References

### Graphics standards

- Khronos Group. [*OpenGL 3.3 Core Profile
  Specification*](https://registry.khronos.org/OpenGL/specs/gl/glspec33.core.pdf).
- Khronos WebGL Working Group. [*WebGL 1.0
  Specification*](https://registry.khronos.org/webgl/specs/latest/1.0/) and
  [*WebGL 2.0 Specification*](https://registry.khronos.org/webgl/specs/latest/2.0/).

### Robust geometric computation

- Shewchuk, Jonathan Richard. “Adaptive Precision Floating-Point Arithmetic
  and Fast Robust Geometric Predicates.” *Discrete & Computational Geometry*,
  vol. 18, no. 3, 1997, pp. 305–363.
  [doi:10.1007/PL00009321](https://doi.org/10.1007/PL00009321).
- Yap, Chee K. “Towards Exact Geometric Computation.” *Computational
  Geometry*, vol. 7, nos. 1–2, 1997, pp. 3–23.
  [doi:10.1016/0925-7721(95)00040-2](https://doi.org/10.1016/0925-7721(95)00040-2).

The Khronos specifications define the backend's object, buffer, shader, and
draw contracts. Shewchuk and Yap provide the robust-predicate and
exact-computation background for keeping geometric decisions on the exact side
of the rendering boundary.

## Acknowledgements

The renderer originated in CopperForge's `render3d` module, which credited the
MIT-licensed `alumina-interface` project by Timothy Schmidt. Hypergraphics uses
the [`glow`](https://github.com/grovesNL/glow) bindings for its common
OpenGL/WebGL context API and [`nalgebra`](https://nalgebra.org/) for finite
camera projection at the explicit rendering boundary.

## License and contributing

Hypergraphics is available under either the [MIT License](LICENSE-MIT) or the
[Apache License 2.0](LICENSE-APACHE). Rust 1.88 is the declared minimum
supported version. Contributions should preserve the separation between exact
geometry and lossy presentation data. CI checks that MSRV plus the full stable
feature matrix. Before submitting a change, run:

```sh
cargo fmt --all -- --check
cargo test --all-targets --all-features
cargo test --all-targets --no-default-features
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --all-features
```
