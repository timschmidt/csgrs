# CSGRS

CSGRS is the constructive-solid-geometry grammar for the Hyper geometry
workspace. It provides an OpenSCAD-like vocabulary for primitives, Boolean
composition, transforms, extrusion, revolution, sweep, loft, conversion, and
CAD interchange without introducing another geometry kernel.

- [Hypercurve](https://github.com/timschmidt/hypercurve) owns curves, paths, filled regions, and
  certified planar operations.
- [Hypermesh](https://github.com/timschmidt/hypermesh) owns indexed triangle geometry, certified
  solid Booleans, topology facts, and mesh queries.
- [Hypertri](https://github.com/timschmidt/hypertri) owns triangulation.
- [Hyperphysics](https://github.com/timschmidt/hyperphysics) owns physical-property analysis.
- CSGRS supplies the modeling language that composes those native types and
  connects them to file formats and application boundaries.

Core coordinates use `hyperreal::Real`. Primitive floats occur only at
explicit rendering, serialization, adapter, FFI, and WebAssembly boundaries.

![A composed CSG model](docs/csg.png)

## Primary types

CSGRS operates directly on its dependencies' native geometry:

```text
CurveRegion2 ── extrude / revolve / sweep ──► TriangleMesh
     │                                             │
     ├─ CurveString2 and CurvePath2 for open work  ├─ Hypermesh Booleans
     └─ Hypercurve topology and Booleans           └─ queries and interchange
```

- `hypercurve::CurveRegion2` is filled planar material, including holes.
- `hypercurve::CurveString2` is an open or closed curve string without implied
  fill.
- `hypercurve::CurvePath2` preserves higher-order path topology.
- `hypermesh::TriangleMesh`, re-exported as `csgrs::TriangleMesh`, is reusable
  indexed triangle geometry and the input/output type for solid modeling.
- `csgrs::AttributedMesh<M>` is an optional, feature-gated boundary sidecar
  with one metadata value per triangle and optional authored normals. Modeling
  operations still consume its `TriangleMesh` geometry.
- `Real` is CSGRS's re-export of the exact-aware Hyperreal scalar.

Use Hypercurve or Hypermesh directly for kernel-level work. Use CSGRS when the
task is to describe a model, cross between curve and solid representations, or
exchange geometry with another format.

## Quick start

Install Rust with [rustup](https://rustup.rs/), then:

```sh
cargo new drilled-cube
cd drilled-cube
cargo add csgrs
```

Replace `src/main.rs` with:

The equivalent dependency entry is:

```toml
[dependencies]
csgrs = "0.23.0"
```

<!-- quickstart:start -->
```rust
use csgrs::{
    Real, curve,
    io::stl::to_stl_binary,
    solid::{self, SolidExt},
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let body = solid::cube(Real::from(12));
    let opening = curve::extrude(&curve::square(Real::from(4)), Real::from(16)).translated(
        Real::from(4),
        Real::from(4),
        Real::from(-2),
    );

    let part = body.try_difference(&opening)?;
    std::fs::write("drilled_cube.stl", to_stl_binary(&part, "drilled_cube")?)?;
    Ok(())
}
```
<!-- quickstart:end -->

Run it with `cargo run`. It writes `drilled_cube.stl`. The same program is
included as `examples/basic.rs` and runs with:

```sh
cargo run --example basic
```

## Modeling API

The following is the useful public modeling surface. Generated API
documentation contains complete signatures, feature conditions, and errors.

### Filled regions and open curves

Functions in `csgrs::curve` return native Hypercurve types.

Filled constructors:

- basic: `empty`, `rectangle`, `square`, `circle`, `right_triangle`,
  `polygon`, `polygon_points`, `ellipse`, and `regular_ngon`;
- decorative: `arrow`, `trapezoid`, `star`, `teardrop`, `egg`,
  `rounded_rectangle`, `squircle`, `keyhole`, `reuleaux`, `ring`,
  `pie_slice`, `heart`, `crescent`, and `supershape`;
- mechanical: `circle_with_keyway`, `circle_with_flat`,
  `circle_with_two_flats`, `involute_gear`, `cycloidal_gear`,
  `involute_rack`, `cycloidal_rack`, and `airfoil_naca4`;
- sampled/imported: `bezier_region`, `metaballs`, `from_image`, and
  `truetype_text`.

Open constructors return `CurvePath2`, `CurveString2`, or collections of those
types: `bezier_path`, `bspline_path`, `hilbert_strings`, and
`hershey_strings`. The Hershey catalog is compiled into Hypercurve and exposed
as `curve::hershey::fonts`; no font files or runtime parser are required.

Planar operations:

- `CurveRegionExt::{try_union, try_difference, try_intersection, try_xor}`;
- `offset` and `offset_rounded`;
- `transformed`, `translated`, `rotated`, and `scaled`;
- `contains_xy`, `bounding_box`, `finite_profiles`, and `triangulate`.

`finite_profiles` is an explicit display/interchange projection. It is not a
replacement for the native curve topology.

### Curves to solids

These functions consume a `CurveRegion2` and return a native `TriangleMesh`:

- `extrude` and `extrude_vector`;
- `extrude_twisted`;
- `revolve`;
- `sweep`.

`solid::loft` joins corresponding 3D sections and emits native indexed
triangles directly.

### Solid construction

Functions in `csgrs::solid` return `TriangleMesh`:

- basic: `empty`, `cube`, `cuboid`, `sphere`, `cylinder`, `ellipsoid`,
  `frustum`, `frustum_between`, `polyhedron`, `octahedron`, and `icosahedron`;
- modeled parts: `arrow`, `torus`, `teardrop_cylinder`,
  `spur_gear_involute`, `spur_gear_cycloid`, and
  `helical_involute_gear`;
- implicit geometry: `metaballs`, `metaballs_with_diagnostics`, `sdf`,
  `sdf_expr`, `gyroid_solid`, `schwarz_p_solid`, and `schwarz_d_solid`.

Sampling arguments such as segments, stacks, slices, and resolution are
explicit tessellation choices; CSGRS has no global tolerance setting.

### Solid operations and queries

`solid::SolidExt` supplies fluent native operations:

- `try_union`, `try_difference`, `try_intersection`, and `try_xor`;
- `transformed` and `translated`;
- `exact_bounds`;
- `visit_native_triangles`.

The corresponding free functions and additional operations are:

- Booleans and transforms: `boolean`, `transform`, `rotate`, `scale`,
  `mirror`, `inverse`, `translation`, and `translation_to`;
- positioning and repetition: `bounding_box`, `center`, `float`, `merge`,
  `distribute_linear`, `distribute_grid`, and `distribute_arc`;
- topology/refinement: `subdivide`, `renormalized`, `materialize_finite`,
  `is_closed_manifold`, `convex_hull`, and `minkowski_sum`;
- geometric queries: `contains_point`, `ray_intersections`,
  `polyline_intersections`, and `dihedral_angle`;
- projections: `flatten` and `slice_z`;
- analysis: `exact_mass_properties`;
- application conversion: `to_bevy_mesh` when `bevymesh` is enabled;
- surface forms of implicit fields: `gyroid`, `schwarz_p`, and `schwarz_d`.

Hypermesh retains reusable bounds, topology, transform, convexity, and
Boolean facts on `TriangleMesh`. Cloning a mesh shares its immutable geometry
and retained facts.

### Metadata

Enable `attributed` when a renderer or interchange boundary needs aligned
face information:

```rust
use csgrs::{AttributedMesh, Real, solid};

let geometry = solid::cube(Real::from(2));
let attributed = AttributedMesh::from_uniform(geometry, "housing");
assert_eq!(
    attributed.face_metadata().len(),
    attributed.geometry().triangles.len()
);
```

`AttributedMesh::new` validates one metadata row per triangle.
`with_authored_normals` additionally validates one normal per position.
`geometry`, `face_metadata`, `authored_normals`, `into_parts`, and
`map_metadata` expose the sidecar without duplicating modeling behavior.

## File formats

Feature-gated functions live under `csgrs::io`:

| Format | Import | Export |
|---|---|---|
| STL | `stl::from_stl` | `to_stl_ascii`, `to_stl_binary` |
| OBJ | `obj::from_obj` | `to_obj`, `write_obj` |
| PLY | — | `to_ply`, `write_ply` |
| AMF | — | `to_amf`, `to_amf_with_color`, `write_amf` |
| glTF/GLB | `gltf::from_gltf` | `to_gltf`, `write_gltf`, `to_gltf_scene`, `write_gltf_scene` |
| VRML 2.0 | `vrml::from_vrml` | — |
| DXF | `dxf::from_dxf` | `dxf::to_dxf` |
| SVG | `svg::import_svg` | `svg::export_svg` |
| Gerber | `gerber::import_gerber` | `gerber::export_gerber` |

Triangle importers return `TriangleMesh`. Curve importers return distinct
filled, string, and path carriers where the source format can contain more
than one topology kind. Mesh exporters accept `TriangleMesh` directly, and
planar exporters accept native `CurveRegion2` values.

## Boundary adapters

`csgrs::adapter` provides opt-in primitive-scalar wrappers over native
geometry:

- scalar policies: `RawReal`, `F32`, `F64`, and `I128`;
- solid boundary: `ScalarMesh<A>`;
- curve boundary: `ScalarCurve<A>`;
- result carriers: `GraphicsMesh`, `IndexedMeshBuffers`, and `Aabb3`.

Conversions happen only at ingress and egress. Float output is fallible when
coordinates are not finite; integer output requires an exact, in-range
integer. The adapters do not implement another geometry kernel.

The C ABI is the sibling
[`csgrs-ffi`](https://github.com/timschmidt/csgrs-ffi) crate. C++, Go, Python,
and TypeScript wrappers are documented in [bindings](bindings/README.md).
The `wasm` feature provides browser-oriented wrappers over the same core.

## Features

Default features provide the common modeling and interchange surface. For
smaller builds use `default-features = false`.

| Feature | Adds |
|---|---|
| `curve` | native curve grammar and curve-to-solid operations |
| `attributed` | aligned metadata/authored-normal boundary carrier |
| `stl-io`, `obj-io`, `ply-io`, `amf-io` | triangle interchange |
| `gltf-io`, `vrml-io`, `dxf-io` | scene and CAD interchange |
| `svg-io`, `gerber-io` | curve interchange |
| `offset` | regularized planar offsets |
| `truetype-text`, `hershey-text`, `image-io` | text and raster curve construction |
| `metaballs`, `sdf` | sampled implicit geometry |
| `bevymesh` | Bevy conversion |
| `wasm` | WebAssembly/browser wrappers |
| `dispatch-trace` | computation-path evidence for tests and benchmarks |

A minimal curve-and-solid build is:

```toml
[dependencies]
csgrs = { version = "0.23.0", default-features = false, features = ["curve", "stl-io"] }
```

For WebAssembly:

```sh
cargo install wasm-pack
wasm-pack build --release --target bundler --out-dir pkg -- --features wasm
```

## Correctness boundaries

- Topology-sensitive decisions use `Real` and the Hyper predicate stack.
- Boolean inputs must meet Hypermesh's documented finite, closed, oriented
  solid preconditions. Keep structured `try_*` errors.
- Sampled circles, text, raster contours, implicit fields, and graphics
  buffers remain explicit approximations at caller-chosen resolutions.
- `CurveRegion2`, `CurveString2`, and `CurvePath2` have different meanings;
  CSGRS does not hide them behind a combined 2D facade.
- `TriangleMesh` is authoritative geometry. `AttributedMesh` stores only
  aligned boundary attributes.
- Bevy, WebAssembly, scalar adapters, FFI, and serialization are application
  boundaries; their finite projections never become topology truth.

## Performance and hard tests

The reproducible harness, CSV schema, dependency policy, and interpretation
rules are in [benchmarks/README.md](benchmarks/README.md). Retained
measurements and optimization notes are in [PERFORMANCE.md](PERFORMANCE.md).

The serialized suite compares CSGRS, CGAL EPECK, and tight-tolerance
OpenCascade across 66 shared construction, transform, Boolean, analysis, and
I/O workloads. That includes deterministic 8,020-triangle concave-labyrinth
and 36,096-triangle level-3 Sierpiński-foam meshes. Rust comparisons add
`boolmesh`, `manifold-rust`, and `tri-mesh` where their representations
overlap.

Set `YEAHRIGHT_BENCH=1` for benchmark runs that should download the optional
public-domain YeahRight corpus into `target/benchmark-fixtures/yeahright`.

## Examples

The `examples` directory covers primitives, native curve extrusion, Booleans,
transforms, hulls, Minkowski sums, metadata, adapters, and interchange. The
Nuxt WebAssembly demo is under
[examples/wasm-nuxt4](examples/wasm-nuxt4/README.md).

Community integrations:

- [csgrs-bevy-example](https://github.com/timschmidt/csgrs-bevy-example)
- [csgrs-egui-example](https://github.com/timschmidt/csgrs-egui-example)
- [csgrs-egui-wasm-example](https://github.com/timschmidt/csgrs-egui-wasm-example)
- [csgrs-druid-example](https://github.com/timschmidt/csgrs-druid-example)

## References

These are the authoritative mathematical and interchange references for the
crate:

- Nicholas M. Patrikalakis, Takashi Maekawa, and Wonjoon Cho,
  [*Shape Interrogation for Computer Aided Design and Manufacturing*](https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/),
  MIT Press. The curve/surface, intersection, robustness, and offset chapters
  motivate the separation of representation, interrogation, and presentation.
- Jonathan Richard Shewchuk,
  [“Adaptive Precision Floating-Point Arithmetic and Fast Robust Geometric Predicates”](https://www.cs.cmu.edu/~quake/robust.html),
  *Discrete & Computational Geometry* 18, 1997, pp. 305–363. This is the
  primary reference for sign-correct adaptive predicates.
- Chee K. Yap,
  [“Towards Exact Geometric Computation”](https://doi.org/10.1016/0925-7721(95)00040-2),
  *Computational Geometry* 7, 1997, pp. 3–23. This informs the separation
  between exact decisions and explicit approximation boundaries.
- Ucamco,
  [Gerber Layer Format specifications](https://www.ucamco.com/en/gerber/downloads),
  the normative source for the Gerber adapter.
- W3C, [Scalable Vector Graphics (SVG) 2](https://www.w3.org/TR/SVG2/),
  the normative source format adapted through Hypercurve.
- Khronos Group,
  [glTF 2.0 specification](https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html),
  the normative scene and buffer format for the glTF adapter.
## Acknowledgements

CSGRS began as a Rust translation of Evan Wallace's
[CSG.js](https://github.com/evanw/csg.js), released under the MIT license.
That lineage and copyright notice remain in [LICENSE](LICENSE).

The crate builds on work by the Hyperreal, Hyperlattice, Hyperlimit,
Hypercurve, Hypertri, Hypermesh, Hyperphysics, and Hypersdf contributors and
the upstream format/rendering crates named in `Cargo.toml`.

The compiled single-stroke font catalog was created by Dr. A. V. Hershey at
the U.S. National Bureau of Standards. Its source distribution format was
created by James Hurt of Cognition, Inc.; the integrated Rust representation
is not the U.S. NTIS distribution format.

## Contributing and license

Run the release checks before submitting changes:

```sh
cargo fmt --all -- --check
cargo test --all-features
cargo clippy --all-features --all-targets -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --all-features --no-deps
cargo check --all-features --benches
```

Discussion is available in the
[CSGRS Discord community](https://discord.gg/9WkD3WFxMC). CSGRS is available
under the [MIT License](LICENSE).
