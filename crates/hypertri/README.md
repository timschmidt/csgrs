# hypertri

`hypertri` is a triangulation crate for the hyperreal geometry stack. It owns
earcut-style polygon triangulation and incremental Delaunay / constrained
Delaunay topology, with irreversible topology decisions routed through exact
predicate helpers over `hyperreal::Real`.

In the Hyper ecosystem, `hypertri` is the straight-edge topology layer.
`hypercurve` owns curved contours and regions, `hyperlimit` supplies exact
orientation/in-circle predicates, and `hypertri` turns line-only polygon and
PSLG inputs into validated triangles without treating primitive floats as
topology values.

The crate treats `f64` as an interop boundary. Finite `f64` coordinates are
validated and lifted into exact hyperreal-backed coordinates before topology is
decided. The default feature set is intentionally core-only: enable each
triangulation algorithm explicitly so downstream crates only compile the
algorithm code they use. Enable `f64-interop` only at IO, rendering, or
compatibility boundaries, and pair it with the algorithm features whose `f64`
entry points you need.

## WASM Demo

The deployed WASM app is available at
<https://timschmidt.github.io/hypertri/>.

## Hyper Stack Links

- [hyperreal](../hyperreal/README.md): exact rational, symbolic, and computable
  real arithmetic.
- [hyperlimit](../hyperlimit/README.md): exact predicate policy and certified
  geometric decisions.
- [hyperlattice](../hyperlattice/README.md): small exact vector, matrix, and
  transform algebra.
- [hypercurve](../hypercurve/README.md): planar curve, contour, region, and
  boolean geometry.
- [hypertri](../hypertri/README.md): exact polygon triangulation and constrained
  Delaunay topology.
- [hypermesh](../hypermesh/README.md): 3D mesh boolean experiments and the
  future exact-aware mesh-topology layer.
- [hypersolve](../hypersolve/README.md): experimental exact-aware solver layer.
- [hyperdrc](../hyperdrc/README.md): PCB design-readiness checks over exact-aware
  geometry adapters.
- [hyperphysics](../hyperphysics/README.md): placeholder physics-domain crate
  for the exact geometry stack.
- [csgrs](../csgrs/readme.md): constructive solid geometry and polygon boolean
  engine used by HyperDRC and available as an interop target.

## Current Status

Implemented and tested:

- Exact point and polygon input types backed by `hyperreal::Real`.
- Ring normalization from flat vertices plus `hole_indices`.
- Polygon structural facts for vertex/ring counts, known degenerate and
  axis-aligned edges, exact-rational coordinate summaries, symbolic dependency
  summaries, signed ring area, and local turn consistency.
- Earcut-style triangulation for simple and holed polygons.
- Earcut hot-loop diagnostics for candidate ear tests, prepared local
  reflex/convex fact reuse, exact triangle-AABB containment rejects, and
  remaining exact containment predicates.
- Local-intersection curing and split fallback for difficult earcut inputs.
- Exact incremental Delaunay triangulation for point sets.
- Exact D-dimensional Delaunay complex construction over `Real` coordinates
  for small validation/oracle workloads.
- Closed-ring constrained triangulation for one exterior ring plus holes.
- Constraint recovery by exact edge flips.
- Exact vertex insertion at proper constraint intersections.
- Constraint splitting at inserted intersection vertices and existing collinear
  vertices.
- Re-legalization of unconstrained edges with exact in-circle predicates.
- General planar straight-line graph recovery over the convex hull for
  non-polygon constraint sets, including separated closed cycles.
- Runtime polygon algorithm selection when `runtime-select` is enabled.
- Optional `f64` entry points that reject non-finite input and exact-lift finite
  coordinates before topology is decided.
- Optional `serde` support for exact points, polygon inputs, diagnostics, and
  triangulation result records.
- WASM UI example built with Trunk and deployed through GitHub Pages.

For constrained output, `constraints()` reports the caller's original
constraints and `constraint_edges()` reports the planarized protected
subsegments that are actually present as triangulation edges. Exact results
provide `validate()` for topology checks and
`validate_unconstrained_edges_are_delaunay()` for the local constrained
Delaunay legality check on unprotected interior edges.

Known scope boundaries:

- Common-scale / homogeneous rational vector facts are retained only through
  `hyperreal` summaries today; a dedicated prepared-polygon input type can make
  those schedules first-class without changing topology semantics.
- The CDT implementation is intentionally exact and validation-heavy. Richer
  prepared object facts and DCEL storage are performance/locality improvements;
  they are not required for the accepted topology contract.

## Traditional Triangulation Problems

Triangulation is full of irreversible local choices. Ear clipping depends on
convex/reflex signs and point-in-triangle tests; Delaunay insertion depends on
orientation and in-circle signs; constrained recovery depends on segment
incidence and edge-flip legality. With ordinary floating point, a single
misclassified near-collinear or near-cocircular case can create inverted
triangles, missing constraints, or non-terminating repair loops.

`hypertri` addresses that by separating topology from numeric approximation.
Finite `f64` input can be exact-lifted at the boundary, but algorithm branches
consume `Real` coordinates and `hyperlimit` predicates. Performance work
focuses on reducing exact predicate calls before they happen: retained polygon
facts, local convex/reflex caches, triangle-AABB rejects, source-ring metadata,
runtime algorithm selection, diagnostics counters, and validation APIs. This
keeps exactness concentrated at branch points instead of expanding every
coordinate expression eagerly.

## API Shape

The exact API uses `Real` directly. Enable the algorithm feature that provides
the entry point before using it:

```toml
[dependencies]
hypertri = { version = "0.1", default-features = false, features = ["earcut"] }
```

```rust
use hypertri::{Point2, Real};

fn main() -> hypertri::Result<()> {
    let points = vec![
        Point2::new(Real::from(0), Real::from(0)),
        Point2::new(Real::from(1), Real::from(0)),
        Point2::new(Real::from(0), Real::from(1)),
    ];

    let triangles = hypertri::earcut(&points, &[])?;
    assert_eq!(triangles.len(), 3);
    Ok(())
}
```

The optional `f64` module is for IO, rendering, tests, and compatibility:

```toml
[dependencies]
hypertri = { version = "0.1", default-features = false, features = ["earcut", "f64-interop"] }
```

```rust
fn main() -> hypertri::Result<()> {
    let triangles = hypertri::f64::earcut(
        &[[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]],
        &[],
    )?;

    assert_eq!(triangles.len(), 3);
    Ok(())
}
```

Those APIs lift finite primitive floats into exact `Real` values before
topology branches run.

## Feature Flags

- `default`: no triangulation algorithms; core exact types, polygon facts,
  predicates, and errors only.
- `all-algorithms`: convenience feature enabling `earcut`, `cdt`, and `nd`.
- `earcut`: polygon triangulation by ear clipping.
- `cdt`: Delaunay and constrained Delaunay APIs.
- `nd`: exact D-dimensional Delaunay complex API.
- `runtime-select`: runtime polygon algorithm selection when multiple
  algorithms are compiled.
- `f64-interop`: opt-in plain `f64` entry points that exact-lift finite input.
- `serde`: derive/implement serialization for public exact topology records.

`spade`, `earcutr`, and the `delaunay` crate are source/API references for the
port. `earcutr` is a dev-only differential oracle for ordinary polygon tests;
production/runtime builds do not build or link it. The D-dimensional API keeps
the same exact-arithmetic boundary as the rest of `hypertri`: it is a local
`Real`-backed Delaunay complex oracle rather than a dependency on the external
float-oriented triangulation crate.

## Semantic Boundary

`hypertri` owns triangulation topology: polygon normalization, ring and hole
inputs, linked earcut nodes, CDT records, constraint graphs, protected edge
metadata, runtime algorithm selection, output validation, and triangulation
result records. Exact orientation, ring area, local turn, and in-circle signs
are delegated to `hyperlimit`, while polygon object facts consume scalar
summaries from `hyperreal::Real`.

Structural metadata should be retained when it is cheap to discover: source
vertex ids, duplicate classes, collinear chains, exact ring area signs, local
turn consistency, integer-grid, dyadic-scale, shared-denominator, symbolic
dependency facts, bounding boxes, convex/reflex bits, constraint-subsegment
provenance, protected-edge flags, and cavity boundary facts. These facts select
faster exact algorithms and reduce candidate sets; they do not permit lossy
topology decisions.

`earcut_report` returns the same triangle indices as `earcut` plus hot-loop
diagnostics such as ear-candidate tests, containment candidates, prepared
reflex/convex lookups and updates, exact reflex/convex containment rejects,
triangle-AABB rejects, and remaining triangle-containment predicate calls.
These counters are for benchmarking and algorithm selection work only; exact
predicates remain the source of topology decisions.

Exact triangulation in this crate follows Yap's exact geometric computation
contract: preserve enough scalar and object structure to make irreversible
topology choices through exact predicates, and surface uncertainty explicitly.
It does not mean canonicalizing every coordinate or polygon expression before
triangulation begins.

## Testing

The test suite combines fixed adversarial cases with `proptest` fuzz-style
generators over exact integer and rational inputs. The fuzz properties check
topology invariants such as valid triangle indices, non-degenerate triangle
index triples, constrained edges preserved by CDT recovery, and
exact local Delaunay legality on unconstrained interior edges.
Dev-only differential tests compare ordinary polygon cases against `earcutr`
by triangle-count topology and area preservation; exact hyperreal predicates
remain the correctness oracle for degeneracies and near-degeneracies.
The opt-in `cargo-fuzz` target in `fuzz/` generates exact rational polygon and
PSLG/ND cases and checks public API invariants without linking `earcutr`.

Useful local checks:

```text
cargo test
cargo test --features all-algorithms
cargo test --features earcut,f64-interop
cargo check --manifest-path examples/hypertri_ui/Cargo.toml --target wasm32-unknown-unknown
cargo bench --bench earcut --features earcut,f64-interop
cargo bench --bench delaunay --features cdt,f64-interop
cargo bench --bench exact --features all-algorithms
cargo +nightly fuzz run topology_invariants
```

The GitHub Pages workflow checks out `hypertri`, `hyperreal`, and `hyperlimit`
as sibling repositories because the manifests intentionally use local path
dependencies while the crates are being developed together.

## References

Bareiss, Erwin H. "Sylvester's Identity and Multistep Integer-Preserving
Gaussian Elimination." *Mathematics of Computation*, vol. 22, no. 103, 1968,
pp. 565-578.

Boehm, Hans-J., Robert Cartwright, Mark Riggle, and Michael J. O'Donnell.
"Exact Real Arithmetic: A Case Study in Higher Order Programming." *Proceedings
of the 1986 ACM Conference on LISP and Functional Programming*, 1986, pp.
162-173.

de Berg, Mark, Otfried Cheong, Marc van Kreveld, and Mark Overmars.
*Computational Geometry: Algorithms and Applications*. 3rd ed., Springer, 2008.

Delaunay, Boris. "Sur la sphère vide." *Bulletin de l'Académie des Sciences de
l'URSS. Classe des sciences mathématiques et naturelles*, no. 6, 1934, pp.
793-800.

Ericson, Christer. *Real-Time Collision Detection*. Morgan Kaufmann, 2005.

Lee, Der-Tsai, and Arthur K. Lin. "Generalized Delaunay Triangulation for
Planar Graphs." *Discrete & Computational Geometry*, vol. 1, 1986, pp.
201-217.

Meisters, Gary H. "Polygons Have Ears." *The American Mathematical Monthly*,
vol. 82, no. 6, 1975, pp. 648-651.

Shewchuk, Jonathan Richard. "Adaptive Precision Floating-Point Arithmetic and
Fast Robust Geometric Predicates." *Discrete & Computational Geometry*, vol.
18, no. 3, 1997, pp. 305-363.

Shewchuk, Jonathan Richard, and Brielin C. Brown. "Fast Segment Insertion and
Incremental Construction of Constrained Delaunay Triangulations."
*Computational Geometry*, vol. 48, no. 8, 2015, pp. 554-574,
doi:10.1016/j.comgeo.2015.04.006.

Yap, Chee K. "Towards Exact Geometric Computation." *Computational Geometry*,
vol. 7, nos. 1-2, 1997, pp. 3-23.
