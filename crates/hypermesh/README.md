# hypermesh

`hypermesh` provides exact 3D triangle-mesh Boolean operations for the Hyper
geometry stack. It validates finite closed piecewise-winding-number (PWN)
meshes, builds local exact arrangements with an EMBER-style subdivision and
BSP pipeline, propagates winding-number evidence, and returns certified
polygon or triangle output.

The crate owns indexed triangle input, mesh validation, intersection,
classification, winding propagation, and output closure. File formats,
parametric solid grammar, extrusion/revolution/sweep/loft construction, and
cross-representation conversion belong in adapter and CSG crates such as
CSGRS.

This README describes crate version `0.1.0`.

## Why exact mesh Booleans?

A mesh Boolean changes topology based on sidedness, coplanarity, segment
incidence, polygon overlap, and winding transitions. If different stages round
the same configuration differently, the output can crack, lose faces, or
require tolerance-driven repair.

Hypermesh keeps that decision chain explicit:

```text
closed PWN TriangleMesh values
           │  validate indices, degeneracy, closure, orientation
           ▼
       PolygonSoup
           │  exact subdivision + local BSP arrangements
           ▼
   classified fragments + winding evidence
           │  certify directed edge cancellation
           ▼
      BooleanResult
           │  certified triangulation and T-junction resolution
           ▼
      BooleanMesh
```

An uncertified sign, incidence, reference-propagation step, leaf
classification, subdivision budget, or closure fact is a `HypermeshError`.
The crate does not repair an unresolved result into apparent success.

## Primary types

| Type | Purpose |
| --- | --- |
| `TriangleMesh`, `TriangleMeshRef`, `Triangle` | Owned and borrowed indexed-triangle input. |
| `PolygonSoup` | Validated combined exact polygon input for one or more meshes. |
| `Point3`, `Vector3`, `Real` | Re-exported exact coordinate carriers. |
| `Plane`, `Aabb`, `Classification` | Exact plane, bounds, and sidedness primitives. |
| `ConvexPolygon`, `ExactBvh` | Principal arrangement and acceleration structures. |
| `MeshContext`, `PredicatePolicy` | Immutable per-operation choice of `STRICT` or `APPROXIMATE_512`. |
| `MeshOutcome<T>`, `MeshCertainty` | Result and aggregate predicate certainty consumed to produce it. |
| `BooleanOp`, `EmberConfig` | Operation selection and optional subdivision-depth budget. |
| `BooleanResult` | Certified polygon arrangement with classifications and winding evidence. |
| `OutputPolygon`, `BooleanMesh` | Exact polygon and indexed triangle output. |
| `BooleanMeshClosureEvidence` | Exact boundary, imbalance, and non-manifold diagnostics. |
| `ExactGpuMeshBuffers`, `GpuMeshBuffersF32`, `GpuMeshBuffersF64` | Exact and explicitly approximate renderer-neutral buffers. |
| `HypermeshError`, `HypermeshResult<T>` | Invalid input, unresolved predicate, budget, and output-certification failures. |

## Quick start

For sibling Hyper checkouts:

```toml
[dependencies]
hypermesh = { path = "../hypermesh" }
```

Replace `src/main.rs` with:

<!-- quickstart:start -->
```rust
use hypermesh::{
    BooleanOp, EmberConfig, MeshContext, Point3, PredicatePolicy, Real, Triangle, TriangleMesh,
    boolean_operation, triangulate_and_resolve_certified,
};

const CONTEXT: MeshContext = MeshContext::new(PredicatePolicy::APPROXIMATE_512);

fn tetrahedron(offset: i64) -> TriangleMesh {
    let p = |x, y, z| Point3::new(Real::from(x + offset), Real::from(y), Real::from(z));
    TriangleMesh::new(
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
    let first = tetrahedron(0);
    let second = tetrahedron(3);
    let result = boolean_operation(
        &CONTEXT,
        &[first.as_ref(), second.as_ref()],
        BooleanOp::Union,
        EmberConfig::default(),
    )?
    .into_value();
    let triangles = triangulate_and_resolve_certified(&CONTEXT, &result)?.into_value();
    println!("{} exact output triangles", triangles.triangles.len());
    Ok(())
}
```
<!-- quickstart:end -->

Run it with:

```sh
cargo run --example basic
```

The same source is [`examples/basic.rs`](examples/basic.rs); the test suite
compiles it and checks that it remains identical to this README block.

## Supported input model

Boolean input must be a non-empty collection of finite, closed, consistently
oriented PWN triangle meshes. Disconnected and nested closed components are
supported.

The following are rejected before or during the certified Boolean path:

- empty meshes and invalid triangle indices;
- degenerate source triangles;
- open triangle soups or directed edge imbalance;
- arbitrary non-PWN surface collections;
- a predicate that the selected policy cannot decide;
- a caller-selected subdivision limit reached before certification.

Every predicate-bearing API requires a `MeshContext`; there is no implicit
policy. `STRICT` consumes only certified decisions and returns
`PredicateUndecided` when bounded certification is exhausted.
`APPROXIMATE_512` uses the same certified cascade, then permits Hyperlimit's
deterministic 512-bit terminal equality/sign interpretation. A successful
`MeshOutcome` reports `Approximate512Consumed` if any required decision used
that terminal. Coordinates and constructions remain `Real` under both
policies.

Completeness is scoped to the finite closed-PWN model under the selected
policy. Arbitrary computable `Real` coordinates can fall outside the strict
boundary if a needed sign cannot be certified.

## API guide

### Build and validate input

| Task | API |
| --- | --- |
| Select predicate semantics | `MeshContext::new(PredicatePolicy::{STRICT, APPROXIMATE_512})` |
| Construct triangles and meshes | `Triangle::new`, `TriangleMesh::new` |
| Borrow without copying | `TriangleMesh::as_ref`, `TriangleMeshRef` |
| Validate and combine | `polygon_soup` |
| Certify reusable convexity | `certify_convex_mesh` |
| Construct a convex face | `convex_triangle`, `convex_quad`, `ConvexPolygon::from_points` |
| Construct planes and bounds | `Plane::from_coefficients`, `from_points`, `axis_aligned`; `Aabb` helpers |

`polygon_soup` is the public input-contract check. Use
`TriangleMesh::try_certify_convex` when the owner wants to retain a proven
convexity fact. `TriangleMesh::as_ref` preserves access to immutable native
facts, while `TriangleMeshRef::new` deliberately creates a fact-free borrowed
view.

### Run Boolean operations

| Task | API |
| --- | --- |
| Multi-mesh polygon output | `boolean_operation` |
| Immediate indexed output | `boolean_mesh` |
| Reusable native two-input output | `boolean_triangle_meshes` |
| Select the operation | `BooleanOp::{Union, Intersection, Difference, SymmetricDifference}` |
| Set a certification budget | `EmberConfig { max_depth }` |

The canonical Boolean functions automatically consume compatible immutable
facts retained by native `TriangleMesh` inputs. There are no policyless or
fact-forwarding Boolean variants.

`EmberConfig::default()` uses `DEFAULT_MAX_DEPTH`, currently `usize::MAX`.
A finite `max_depth` is a caller-selected certification budget, not a license
to return a partial result.

### Inspect and materialize output

| Task | API |
| --- | --- |
| Read certified polygons | `BooleanResult::output` |
| Read classification evidence | `classifications`, `winding_pairs` |
| Extract output polygons | `extract_output` |
| Triangulate certified polygons | `triangulate_and_resolve_certified` |
| Certify polygon closure | `certify_output_polygon_closure` |
| Check triangle closure | `boolean_mesh_closure_evidence`, `boolean_mesh_is_closed` |

Triangulation resolves output T-junctions and crossings, then rejects open or
zero-volume output. Closure is a precondition for success, not a repair
performed after the fact.

### Export graphics buffers

| Task | API |
| --- | --- |
| Preserve exact vertices | `BooleanMesh::to_exact_gpu_mesh_buffers`, `ExactGpuMeshBuffers::from_triangles` |
| Strict finite export | `BooleanMesh::try_to_gpu_mesh_f32`, `try_to_gpu_mesh_f64` |
| Documented zero fallback | `to_gpu_mesh_f32_or_zero`, `to_gpu_mesh_f64_or_zero` |
| Convert exact buffers | `approximate_gpu_mesh_f32`, `approximate_gpu_mesh_f64` and `_or_zero` variants |
| Build interleaved buffers | `approximate_interleaved_gpu_mesh_f32`, `approximate_interleaved_gpu_mesh_f64` |

The primitive-float types are presentation data. They must not be fed back into
mesh topology decisions.

### Use lower-level geometry

| Task | API |
| --- | --- |
| Classify points | `classify_point`, `classify_projective_point` |
| Intersect polygons | `intersect_polygons` and intersection result types |
| Build/query acceleration | `ExactBvh::build`, `ExactPointBvh::build`, query methods |
| Clip | `clip::clip_polygon`, `clip::clip_polygon_to_aabb` |
| Build convex hulls | `convex_hull`, `convex_hull_with_coplanar_groups`, `convex_hull_with_retained_facts` |
| Trace classifications | `trace_segment`, `trace_axis_segment`, `classify_leaf_polygon` |
| Drive subdivision | `subdivide`, `subdivide_into`, `process_leaf`, `process_leaf_into` |
| Propagate winding | `propagate_wnv`, `classify_polygon_output` |

These surfaces support mesh-kernel authors. Most applications should use the
operation and output APIs above.

## Algorithm and guarantees

The Boolean path follows the EMBER architecture:

1. Validate source meshes and construct exact planar polygons with winding
   transitions.
2. Use axis-aligned subdivision and exact BVHs to isolate local arrangements.
3. Split intersecting polygons in local BSP trees.
4. Trace exact segments to classify front/back winding vectors.
5. Verify singleton-edge absence and exact directed edge cancellation.
6. Triangulate, resolve junctions, and certify indexed output closure.

Bounds, BVHs, cached plane/edge profiles, retained source identities, and
certified-convex facts reduce predicate work. Lossy scheduling evidence never
certifies topology. Every required decision uses the operation's immutable
policy, and output closure remains a success condition.

## Features

| Feature | Default | Effect |
| --- | --- | --- |
| `fuzz-bounded-campaign` | no | Enables explicitly bounded fuzz campaign behavior. |
| `dispatch-trace` | no | Enables correlated Hyperreal, Hyperlattice, and Hyperlimit dispatch tracing. |

Hypermesh has no default features.

## Optional benchmark fixture

Set `YEAHRIGHT_BENCH=1` when running a competitive benchmark to download the
public-domain YeahRight corpus into `target/benchmark-fixtures/yeahright`.

## Ecosystem and further documentation

- [Hyperreal](https://github.com/timschmidt/hyperreal) supplies exact-aware
  scalars.
- [Hyperlattice](https://github.com/timschmidt/hyperlattice) supplies points
  and projective carriers.
- [Hyperlimit](https://github.com/timschmidt/hyperlimit) supplies certified
  predicates.
- [CSGRS](https://github.com/timschmidt/csgrs) owns CSG grammar, parametric
  construction, file IO, and conversions into this mesh kernel.

[`PERFORMANCE.md`](PERFORMANCE.md) contains benchmark methodology, competitive
results, and retained/rejected optimization evidence. Generate complete
signatures with `cargo doc --open`.

The browser demo is deployed at <https://timschmidt.github.io/hypermesh/> and
its source lives in [`examples/hypermesh_ui`](examples/hypermesh_ui).

## References

- Trettner, Philip, Julius Nehring-Wirxel, and Leif Kobbelt. “EMBER: Exact
  Mesh Booleans via Efficient & Robust Local Arrangements.” *ACM Transactions
  on Graphics*, vol. 41, no. 4, 2022.
  [doi:10.1145/3528223.3530181](https://doi.org/10.1145/3528223.3530181).
- Zhou, Qingnan, Eitan Grinspun, Denis Zorin, and Alec Jacobson. “Mesh
  Arrangements for Solid Geometry.” *ACM Transactions on Graphics*, vol. 35,
  no. 4, 2016.
  [doi:10.1145/2897824.2925901](https://doi.org/10.1145/2897824.2925901).
- Jacobson, Alec, Ladislav Kavan, and Olga Sorkine-Hornung. “Robust
  Inside-Outside Segmentation Using Generalized Winding Numbers.” *ACM
  Transactions on Graphics*, vol. 32, no. 4, 2013.
  [doi:10.1145/2461912.2461916](https://doi.org/10.1145/2461912.2461916).
- Shewchuk, Jonathan Richard. “Adaptive Precision Floating-Point Arithmetic
  and Fast Robust Geometric Predicates.” *Discrete & Computational Geometry*,
  vol. 18, 1997, pp. 305–363.
  [doi:10.1007/PL00009321](https://doi.org/10.1007/PL00009321).
- Yap, Chee K. “Towards Exact Geometric Computation.” *Computational
  Geometry*, vol. 7, 1997, pp. 3–23.
  [doi:10.1016/0925-7721(95)00040-2](https://doi.org/10.1016/0925-7721%2895%2900040-2).

EMBER defines the local-arrangement architecture; Zhou et al. and Jacobson et
al. cover arrangement and winding classification; Shewchuk and Yap establish
the robust/exact predicate boundary.

## Acknowledgements

Hypermesh is developed by Timothy Schmidt, with repository-history
contributions from sakikomikado. The architecture is directly informed by the
EMBER paper and the mesh-arrangement literature above.

## License and contributing

Hypermesh is distributed under the MIT License; see [`LICENSE`](LICENSE).
Changes must preserve the closed-PWN input contract, explicit uncertainty, and
output-closure certification. Before submitting a change, run:

```sh
cargo fmt --all -- --check
cargo test --all-features
cargo clippy --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --all-features --no-deps
cargo check --manifest-path fuzz/Cargo.toml --bins
```
