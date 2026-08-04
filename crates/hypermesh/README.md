# hypermesh

`hypermesh` provides exact 3D triangle-mesh Boolean operations for the Hyper
geometry stack. It validates finite closed piecewise-winding-number (PWN)
meshes, constructs one exact global surface arrangement, classifies its cells,
and selects closure-certified triangle boundaries for one or many expressions.

Coordinates remain canonical `hyperreal::Real` values throughout topology.
File formats and solid-modeling grammar belong in adapters such as CSGRS.

This README describes crate version `0.1.0`.

## Exactness and policy

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
          │
          ▼
 BooleanMeshBatch
   ├─ one shared Point3<Real> vertex arena
   └─ compact u32 triangles/provenance per requested result
```

There is one public Boolean entry point: `boolean`. Additional expression
roots reuse the same intersections, corefinement, cells, windings, and exact
vertex arena.

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

| Purpose | API |
| --- | --- |
| Exact Boolean evaluation | `boolean` |
| Built-in or arbitrary truth request | `BooleanProgram`, `BooleanExpression`, `BooleanOp` |
| Shared exact output | `BooleanMeshBatch`, `BooleanMeshResult`, `TriangleSource` |
| Reusable indexed input | `TriangleMesh`, `TriangleMeshRef`, `Triangle` |
| Policy and aggregate certainty | `MeshContext`, `PredicatePolicy`, `MeshOutcome`, `MeshCertainty` |
| Explicit native conversion | `BooleanMeshBatch::into_triangle_meshes` |
| Input validation | `polygon_soup` |
| Exact pairwise geometry | `intersect_polygons`, `PairwiseIntersection` |
| Acceleration | `ExactBvh`, `ExactPointBvh` |
| Convex hull | `convex_hull`, `convex_hull_with_coplanar_groups` |
| Mesh queries and editing | methods on `TriangleMesh` |
| Errors | `HypermeshError`, `HypermeshResult<T>` |

Primitive-float GPU buffers are explicit presentation boundaries. Do not feed
their approximated coordinates back into topology decisions.

## Features

| Feature | Default | Effect |
| --- | --- | --- |
| `fuzz-bounded-campaign` | no | Enables explicitly bounded fuzz campaign behavior. |
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

- [Hyperreal](https://github.com/timschmidt/hyperreal) supplies canonical exact
  scalars.
- [Hyperlimit](https://github.com/timschmidt/hyperlimit) supplies policy-aware
  certified predicates.
- [Hyperlattice](https://github.com/timschmidt/hyperlattice) supplies exact
  point, vector, matrix, and projective carriers.
- [Hypertri](https://github.com/timschmidt/hypertri) supplies policy-aware exact
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
