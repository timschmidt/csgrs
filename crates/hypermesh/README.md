# hypermesh

`hypermesh` provides exact 3D triangle-mesh Boolean operations for the Hyper geometry
stack. It accepts finite, closed, piecewise-winding-number (PWN) meshes, constructs
local exact arrangements using an EMBER-style subdivision and BSP pipeline, propagates
winding-number evidence, and returns a certified polygon arrangement or an explicit
error.

The crate owns mesh validation, intersection, classification, and certified output
triangulation. File-format IO, including OBJ parsing and export, belongs in adapter
crates such as [`csgrs`](https://github.com/timschmidt/csgrs).

## WASM Demo

The WASM demo is configured for <https://timschmidt.github.io/hypermesh/>. Its source
and Trunk configuration live in [`examples/hypermesh_ui`](./examples/hypermesh_ui).

## Typical Mesh Problems

Mesh Booleans make topology depend on exact geometric branch decisions: sidedness,
coplanarity, segment incidence, polygon overlap, winding transitions, and output edge
cancellation. Floating-point disagreement at any of those branches can create cracks,
drop faces, invert output, or make a repair loop depend on tolerance choices.

`hypermesh` routes topology-changing decisions through `hyperreal`, `hyperlattice`,
and `hyperlimit`. It does not repair an uncertified result into apparent success. When
a required sign, incidence, reference-propagation step, leaf classification, or output
closure fact cannot be certified, the operation returns `HypermeshError`.

## Main Types

- `InputMesh`, `MeshRef`, and `Triangle` are the owned and borrowed indexed-triangle
  input types. `prepare_input` validates inputs and builds a combined `PolygonSoup`.
- `Point3`, `Vector3`, and `Real` are re-exported from `hyperlattice` for exact mesh
  coordinates.
- `Plane`, `Aabb`, `Classification`, `ConvexPolygon`, `ExactBvh`, and `LocalBsp`
  expose the principal exact geometry and local-arrangement building blocks.
- `PairwiseIntersection`, `PairwiseIntersectionType`, `IntersectionSegment`, and
  `OverlapInfo` describe exact polygon intersection results.
- `BooleanOp` selects `Union`, `Intersection`, `Difference`, or
  `SymmetricDifference`. `EmberConfig` controls the optional subdivision-depth
  budget.
- `BooleanResult` contains the certified classified polygon arrangement. Its
  `output`, `classifications`, and `winding_pairs` accessors expose geometry and
  classification evidence.
- `BooleanArrangement` retains certified winding evidence for its requested
  operation set. `prepare_boolean_operations` can preserve a one-operation
  pruning schedule or retain several results for extraction from one build;
  `build_boolean_arrangement` requests all four operations.
- `OutputPolygon` is an extracted polygon with explicit exact vertices.
  `TriangleSoup` is the indexed triangle output produced by
  `triangulate_and_resolve_certified`.
- `ExactGpuMeshBuffers` preserves exact position/normal rows with `u32` indices;
  `GpuMeshBuffersF32` and `GpuMeshBuffersF64` are explicit finite approximations
  for graphics APIs. `TriangleSoup::try_to_gpu_mesh_f32` and
  `TriangleSoup::try_to_gpu_mesh_f64` build flat-shaded buffers directly.
- `TriangleSoupClosureReport`, `triangle_soup_closure_report`, and
  `triangle_soup_is_closed` expose exact output closure diagnostics.

## Precision Model

Native coordinates are `hyperreal::Real` values carried by `hyperlattice::Point3`.
Planes, intersections, winding transitions, and closure checks retain exact values;
primitive floats should remain at explicit import, rendering, or export boundaries.

The supported input model is a non-empty collection of finite, closed, consistently
oriented PWN triangle meshes. Disconnected and nested closed components are supported.
Empty meshes, invalid indices, degenerate triangles, open surfaces, directed edge
imbalances, and arbitrary non-PWN surface collections are rejected.

Completeness applies when every strict predicate required by the finite closed-PWN
operation is decidable under the bounded refinement policy. Computable `Real` values
whose required signs cannot be certified remain outside that boundary and produce an
error rather than an approximate topology decision.

## Algorithm

The Boolean path follows the EMBER architecture:

1. `prepare_input` validates each source mesh and converts triangles to exact planar
   polygons with winding-number transitions.
2. Adaptive axis-aligned subdivision isolates local arrangements while propagating an
   outside reference point and its winding vector.
3. Local BSP trees split intersecting polygons into disjoint fragments.
4. Exact segment traces classify fragments from front/back winding vectors.
5. `certify_output_polygon_closure` verifies singleton-edge absence and exact directed
   edge cancellation before the operation succeeds.
6. `triangulate_and_resolve_certified` triangulates the arrangement, resolves output
   T-junctions and crossings, and rejects open or zero-volume output.

`EmberConfig::default()` uses `DEFAULT_MAX_DEPTH`, currently `usize::MAX`: there is no
caller-selected arbitrary depth cap. A finite `max_depth` is a certification budget;
reaching it before a leaf is certified returns `SubdivisionDepthLimit`.

## Numerical Explosion

`hypermesh` limits exact-expression growth structurally. Axis-aligned subdivision
reduces candidate sets, exact BVH bounds avoid unnecessary polygon tests, plane-based
polygons defer affine division through homogeneous intersections, and local BSP trees
avoid constructing one global arrangement. Cached plane and edge profiles also reduce
repeated exact comparisons during output assembly.

## Performance Model

The implementation prioritizes pruning work before invoking expensive exact
predicates: AABB and BVH rejection, adaptive subdivision, leaf-level pairwise tests,
local winding traces, and retained arrangement evidence all constrain the exact work.
All public Boolean operations use one prepared pipeline. `boolean_operation` requests a
single operation and retains operation-specific winding-reachability pruning;
`prepare_boolean_operations` requests an explicit reusable subset; and
`build_boolean_arrangement` retains all four operations. Multi-operation preparation
avoids repeating intersection, BSP, and winding-classification work.
The current implementation is single-process Rust and does not claim the parallel
throughput numbers of the EMBER reference implementation.

## Current Status

Implemented today:

- exact union, intersection, difference, and symmetric difference over multiple
  finite closed-PWN triangle meshes;
- reusable certified arrangement construction for extracting several operations
  over the same inputs;
- one unified scoped-preparation path for single and multi-operation Booleans;
- input validation for indices, degeneracy, closure, and directed edge balance;
- exact polygon intersection, adaptive subdivision, local BSP splitting, and winding
  classification;
- certified polygon output and indexed triangle-soup extraction;
- backend-neutral exact, binary32, and binary64 GPU position, normal, and `u32`
  index buffers;
- closure diagnostics and explicit errors for uncertified or unsupported cases;
- a browser demo built with Rust, Yew, WebAssembly, and Trunk.

Current boundaries:

- `hypermesh` does not parse or write OBJ, STL, or other file formats;
- open surfaces and arbitrary polygon soups are not Boolean inputs;
- a successful Boolean result is exact; the optional GPU adapters make the
  `Real`-to-`f32` or `Real`-to-`f64` approximation explicit and offer strict or
  documented zero-fallback conversion policies;
- explicitly bounded subdivision can fail when the selected depth is insufficient.

## Installation

The Hyper crates are currently developed as sibling repositories:

```toml
[dependencies]
hypermesh = { path = "../hypermesh" }
hyperlattice = { path = "../hyperlattice" }
hyperreal = { path = "../hyperreal" }
```

`hypermesh` currently has no optional Cargo features.

## Usage

```rust
use hypermesh::{
    BooleanOp, EmberConfig, InputMesh, Point3, Real, Triangle, boolean_operation,
    triangulate_and_resolve_certified,
};

fn tetrahedron(offset: i64) -> InputMesh {
    let p = |x, y, z| {
        Point3::new(
            Real::from(x + offset),
            Real::from(y),
            Real::from(z),
        )
    };

    InputMesh::new(
        vec![p(0, 0, 0), p(2, 0, 0), p(0, 2, 0), p(0, 0, 2)],
        vec![
            Triangle::new(0, 2, 1),
            Triangle::new(0, 1, 3),
            Triangle::new(1, 2, 3),
            Triangle::new(2, 0, 3),
        ],
    )
}

fn main() -> hypermesh::HypermeshResult<()> {
    let a = tetrahedron(0);
    let b = tetrahedron(3);
    let meshes = [a.as_ref(), b.as_ref()];

    let result = boolean_operation(
        &meshes,
        BooleanOp::Union,
        EmberConfig::default(),
    )?;
    let triangles = triangulate_and_resolve_certified(&result)?;
    let gpu = triangles
        .try_to_gpu_mesh_f32()
        .expect("finite exact output should approximate for the renderer");

    println!("{} exact output triangles", triangles.triangles.len());
    println!("{} GPU indices", gpu.indices.len());
    Ok(())
}
```

For two meshes, `boolean_union`, `boolean_intersection`, and `boolean_difference` are
convenience wrappers. Use `boolean_operation` for multi-mesh operations or symmetric
difference. Use `extract_output` when polygon loops are preferable to indexed
triangles.

## Development

Paper-derived optimization experiments and their benchmark, trace, and test
evidence are recorded in [PERFORMANCE.md](PERFORMANCE.md).

Run the crate checks from this directory:

```bash
cargo fmt --check
cargo test --all-targets
cargo clippy --all-targets -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps
cargo check --examples --benches
cargo run --example basic
```

Build the browser demo with Trunk and the WebAssembly target installed:

```bash
rustup target add wasm32-unknown-unknown
cd examples/hypermesh_ui
trunk build --release --locked
```

GitHub Actions runs the native crate checks and the locked WASM demo build. The Pages
workflow publishes the Trunk output from `main` when GitHub Pages is configured to use
GitHub Actions as its source.

## References

- Philip Trettner, Julius Nehring-Wirxel, and Leif Kobbelt. [EMBER: Exact Mesh
  Booleans via Efficient & Robust Local
  Arrangements](https://doi.org/10.1145/3528223.3530181). ACM Transactions on
  Graphics 41(4), 2022.
- Qingnan Zhou, Eitan Grinspun, Denis Zorin, and Alec Jacobson. [Mesh
  Arrangements for Solid Geometry](https://doi.org/10.1145/2897824.2925901).
  ACM Transactions on Graphics 35(4), 2016.
- Alec Jacobson, Ladislav Kavan, and Olga Sorkine-Hornung. [Robust
  Inside-Outside Segmentation Using Generalized Winding
  Numbers](https://doi.org/10.1145/2461912.2461916). ACM Transactions on
  Graphics 32(4), 2013.

## Hyper Ecosystem

`hypermesh` builds on [hyperreal](https://github.com/timschmidt/hyperreal),
[hyperlimit](https://github.com/timschmidt/hyperlimit), and
[hyperlattice](https://github.com/timschmidt/hyperlattice). It supplies exact
triangle-mesh Booleans to [hyperbrep](https://github.com/timschmidt/hyperbrep),
[csgrs](https://github.com/timschmidt/csgrs), and the other [Hyper geometry
crates](https://github.com/timschmidt?tab=repositories&q=hyper&type=source).
