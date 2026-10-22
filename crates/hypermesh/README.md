# hypermesh

`hypermesh` provides exact 3D triangle-mesh Boolean operations for the Hyper
geometry stack. It validates finite closed piecewise-winding-number (PWN)
meshes, constructs one exact global surface arrangement, classifies its cells,
and selects closure-certified triangle boundaries for one or many expressions.

Coordinates remain canonical `hyperreal::Real` values throughout topology.
File formats and solid-modeling grammar belong in adapters such as CSGRS.

This README describes crate version `0.1.0`.

Exact GPU export now distinguishes proven degenerate triangles
(`GpuMeshError::TriangleNormalUnavailable`) from normals whose nonzero length
is undecided (`TriangleNormalUndecided`). `TriangleNormalFailed` retains a
scalar arithmetic error, available through `std::error::Error::source`.
Update exhaustive matches for these new variants. Successful exact rows and
the strict certification policy are unchanged; a failed export publishes no
partial buffer. `GpuMeshError` retains `Eq`, using the additive `Eq` implementation
on `hyperreal::Problem`.

## Exactness and policy

Extensions follow the stack-wide [admission protocol](../../docs/admission.md),
including closed PWN validation, exact coincidence, arrangement reuse,
unbounded-output rejection, and replayable boundary evidence.

Every predicate-bearing operation receives a `MeshContext` selecting one
Hyperlimit policy:

- `PredicatePolicy::STRICT` consumes only exact or certified decisions. If a
  required decision cannot be certified, the operation returns a typed error.
- `PredicatePolicy::APPROXIMATE_512` uses the same cascade and may terminate in
  Hyperlimit's deterministic 512-bit equality/sign decision.

Successful operations return `MeshOutcome<T>`. Its `MeshCertainty` is
`Approximate512Consumed` if any required decision used that terminal; it is
otherwise `Certified`. No internal default changes predicate semantics.

## Boolean pipeline

```text
closed PWN TriangleMesh views
          │ exact validation and source-face construction
          ▼
 shared face corefinement + compact radial cell complex
          │ absolute winding vectors and Boolean truth DAG
          ▼
 oriented boundary selection + exact output certification
          │ coplanar merge: drop vertices inside one plane or on a
          │ straight crease, retriangulate with Hypertri
          ▼
 BooleanMeshBatch
   ├─ one shared Point3<Real> vertex arena
   └─ compact u32 triangles/provenance per requested result
```

There is one public Boolean entry point: `boolean`. Additional expression
roots reuse the same intersections, corefinement, cells, windings, and exact
vertex arena.

Each result keeps only the vertices its set needs. Coplanar facets are
grouped by plane from coplanarity the arrangement already decided or that an
earlier result retained; vertices inside one plane, or on a straight crease
between two, are removed. Corners, crease ends and non-manifold contacts
stay, and no vertex is added, so results fed into further Booleans stay
proportional to the represented set.

## Quick start

Add hypermesh 0.1.0 to a project with `cargo add hypermesh`; it requires Rust
1.95 or newer. The examples below are checked in; from the root of a
[CSGRS source checkout](../../readme.md), run:

```sh
cargo run -p hypermesh --example basic
```

The checked-in example is:

<!-- quickstart:start -->
```rust
use hypermesh::{
    BooleanOp, BooleanProgram, MeshContext, Point3, PredicatePolicy, Real, Triangle, TriangleMesh,
    boolean,
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
    let result = boolean(
        &CONTEXT,
        &[first.as_ref(), second.as_ref()],
        BooleanProgram::Operation(BooleanOp::Union),
    )?
    .into_value();
    println!(
        "{} exact output triangles",
        result.results[0].triangles.len()
    );
    Ok(())
}
```
<!-- quickstart:end -->

The same source is [`examples/basic.rs`](examples/basic.rs); the test suite
keeps it byte-for-byte synchronized with this block.

## API guide

### Minkowski sums

`minkowski_sum(context, left, right)` computes the regularized sum of bounded
closed-PWN solids. Nonzero winding defines material, so disconnected components,
concavity, cavities, reversed shells and winding multiplicity keep their set
meaning. Only referenced positions participate. Both operands are validated
before canonical-empty identities; malformed meshes remain typed failures.

General inputs are refined into convex cells by their source supporting planes.
Exact signed ray predicates select interior cells. Each convex pair sum uses
the native hull, and their union accumulates through the native Boolean kernel.
The output is closed-PWN geometry or canonical empty. This does not certify
fidelity to an analytic surface or the intent of an earlier approximate operation.
The receipt reports the decisions consumed by this operation.

`minkowski_sum_with_options` accepts `MinkowskiOptions`: maximum live decomposition
cells per operand, maximum convex pair sums, maximum vertex sums per pair, and an
optional cooperative cancellation flag. Defaults add no count limit. Exhaustion
and cancellation return distinct typed causes before output publication. These
counts do not bound total work, heap bytes or scalar growth; validation, hull and
Boolean calls cannot be interrupted internally. A supporting-plane arrangement
can be much larger than its source boundary.

`minkowski_hull(context, left_points, right_points)` explicitly computes the
convex hull of pairwise stored-point sums. It fills disconnected gaps and
concavities and includes every supplied point. It is the migration path for
CSGRS's former vertex-hull interpretation of `minkowski_sum`; point clouds and
open surfaces belong to this explicit hull operation, not the solid-sum API.

### Multi-expression programs

`BooleanProgram::Operation` evaluates one variadic built-in operation:

- `Union`
- `Intersection`
- `Difference` (operand zero minus all later operands)
- `SymmetricDifference`

`BooleanProgram::Expressions` accepts a topologically ordered slice of
`BooleanExpression` nodes and a slice of output roots. Nodes include constants,
operands, `Not`, `And`, `Or`, `Xor`, and built-in operations. References must
name earlier nodes and operand indices must be in range; malformed programs are
rejected as `InvalidBooleanProgram` before geometry work begins.

Each `BooleanMeshResult` records `exterior_inside`. Negation and constant-true
expressions can describe unbounded regularized sets with finite oriented
boundaries. `BooleanMeshBatch::into_triangle_meshes` shares the exact position
allocation among bounded results and returns `UnboundedBooleanOutput` rather
than misrepresenting an exterior-containing result as a finite solid.

### Supported input and output

Boolean input is a non-empty slice of non-empty, finite, closed, consistently
oriented PWN triangle meshes. Disconnected, nested, coincident, balanced
nonmanifold, and winding-multiplicity components are supported. The following
are typed failures:

- invalid indices or degenerate source triangles;
- empty or open meshes;
- directed edge imbalance and non-PWN input;
- invalid expression programs or address-space overflow;
- policy-undecidable predicates;
- malformed, degenerate, duplicate, open, or directionally unbalanced output.

Boundary-only and lower-dimensional contacts are regularized from exact cell
truth. Empty Boolean results are valid and contain no vertices or triangles.
Balanced nonmanifold PWN output is accepted and reported by its topology.

## Primary types

`TriangleMesh::slice_z(context, z)` returns an oriented horizontal section
with source triangle indices, source vertex/edge identities and exact edge
parameters. Closed directed chains retain winding multiplicity. Separate
limits from above and below prevent double counting edges lying in the plane;
coplanar faces and remaining wires are retained separately. A planar consumer
owns filling and regularization. Open sheets are accepted, isolated point
contacts are omitted, and invalid indices or unresolved decisions return
typed errors. Certainty records the decisions consumed by this operation;
it does not certify input manifoldness or inherited source intent. No internal
total-work, retained-byte or cancellation bound is claimed.

| Purpose | API |
| --- | --- |
| Exact Boolean evaluation | `boolean` |
| Built-in or arbitrary truth request | `BooleanProgram`, `BooleanExpression`, `BooleanOp` |
| Shared exact output | `BooleanMeshBatch`, `BooleanMeshResult`, `TriangleSource` |
| Reusable indexed input | `TriangleMesh`, `TriangleMeshRef`, `Triangle` |
| Policy and aggregate certainty | `MeshContext`, `PredicatePolicy`, `MeshOutcome`, `MeshCertainty` |
| Explicit native conversion | `BooleanMeshBatch::into_triangle_meshes` |
| Input validation | `TriangleMesh::validate_closed_pwn`, `polygon_soup` |
| Structural concatenation | `TriangleMesh::try_concatenate` |
| Exact pairwise geometry | `intersect_polygons`, `PairwiseIntersection` |
| Acceleration | `ExactBvh`, `ExactPointBvh` |
| Convex hull | `convex_hull`, `convex_hull_with_coplanar_groups` |
| Mesh queries and editing | methods on `TriangleMesh` |
| Errors | `HypermeshError`, `HypermeshResult<T>` |

`validate_closed_pwn` checks the same nonempty input contract as Boolean soup
preparation without constructing polygons or computing bounds. It accepts
balanced nonmanifold surfaces and retains the combined certainty of face and
edge validation. Facts remain bound to matching source buffers and admissible
policies; this validates topology, not inherited construction intent.

`try_concatenate` preserves position/triangle order, open surfaces, unused
vertices and winding multiplicity. It validates source indices and checks
count/byte capacities before rebasing, with typed reservation failures. It
does not compute a Boolean union. Existing closed-PWN facts retain their weakest
certainty when all nonempty inputs supply them. These vector checks do not bound
total work, scalar/shared-buffer allocation, retained heap bytes or cancellation.

Primitive-float GPU buffers are explicit presentation boundaries. Do not feed
their approximated coordinates back into topology decisions.

## Native-real Surface Nets

`surface_nets` constructs a regular-grid Surface Nets proposal directly from
`Real` scalar samples. Sign classification, edge interpolation, cell centroids,
quad diagonal selection, and triangle-degeneracy checks remain in the exact
geometry boundary, and the returned `SurfaceNetsOutput` carries the result as
a `TriangleMesh`.

This removes primitive-float coordinate quantization but does not certify that
the finite sample grid captures the topology of an underlying continuous
field. `MeshOutcome::certainty` describes the predicates consumed while
constructing the sampled proposal only. Callers needing a solid must also keep
the level set away from the grid boundary and validate closed PWN intake.

## Features

| Feature | Default | Effect |
| --- | --- | --- |
| `fuzz-bounded-campaign` | no | Enabled by the fuzz and size harnesses; adds one internal predicate test and does not change library behavior. |
| `dispatch-trace` | no | Enables correlated Hyperreal, Hyperlattice, and Hyperlimit dispatch tracing. |

Hypermesh has no default features.

## Performance and corpus

The permanent corpus covers exhaustive incidence microcases, historical
regressions, exact-coordinate metamorphics, scaling families, fuzz reducers,
large-mesh heap probes, and pinned competitive cases. Large fixture input
storage is measured separately from kernel and process peaks.

[`PERFORMANCE.md`](PERFORMANCE.md) describes benchmark methodology and retained
optimization evidence. Competitive runs use a pinned CGAL EPECK adapter as a
performance and differential signal; competitor output is never the sole
correctness oracle.

Set `YEAHRIGHT_BENCH=1` for the optional public-domain YeahRight benchmark
fixture. The browser demo source is in
[`examples/hypermesh_ui`](examples/hypermesh_ui).

## Optional benchmark fixture

The YeahRight mesh is not required for normal builds or tests. Opt into its
downloaded fixture and the ignored competitive cases with `YEAHRIGHT_BENCH=1`;
large generated box fixtures remain available without external data.

## References

- [Hyperreal](https://github.com/timschmidt/csgrs/tree/main/crates/hyperreal) supplies canonical exact
  scalars.
- [Hyperlimit](https://github.com/timschmidt/csgrs/tree/main/crates/hyperlimit) supplies policy-aware
  certified predicates.
- [Hyperlattice](https://github.com/timschmidt/csgrs/tree/main/crates/hyperlattice) supplies exact
  point, vector, matrix, and projective carriers.
- [Hypertri](https://github.com/timschmidt/csgrs/tree/main/crates/hypertri) supplies policy-aware exact
  constrained triangulation.
- [CSGRS](https://github.com/timschmidt/csgrs) owns CSG grammar, construction,
  file IO, and mesh-kernel adapters.

## Acknowledgements

The permanent differential corpus and benchmark methodology draw on the CGAL
EPECK, Manifold, and Boolmesh ecosystems, and on the public YeahRight mesh
fixture. Their results are comparison signals; Hypermesh independently checks
its exact topology and output closure.

## License and contributing

Hypermesh is MIT licensed. Changes must preserve the closed-PWN contract,
selected Hyperlimit policy, aggregate certainty, deterministic exact output,
and path-complete error handling. Before submitting a change, run:

```sh
cargo fmt --all -- --check
cargo test --locked --all-features
cargo clippy --locked --all-targets --all-features -- -D warnings
RUSTDOCFLAGS="-Dwarnings" cargo doc --locked --all-features --no-deps
cargo check --manifest-path fuzz/Cargo.toml --bins
```
