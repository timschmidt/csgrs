# hypertri

`hypertri` owns exact-aware triangulation for the Hyper geometry stack. It provides
earcut-style polygon triangulation, incremental Delaunay and constrained Delaunay
topology, a D-dimensional TDS model plus a small exact Delaunay oracle, and optional
`f64` interop that lifts finite inputs into `hyperreal::Real` before topology is
decided.

The crate is intentionally algorithm-feature-gated. Downstream crates can compile only
the triangulation surfaces they use while keeping exact predicate semantics consistent.

## WASM Demo

The deployed WASM app is available at <https://timschmidt.github.io/hypertri/>.

## Hyper Ecosystem

`hypertri` is the straight-edge topology layer.

- [hyperreal](https://github.com/timschmidt/hyperreal): exact coordinate and scalar
  values.
- [hyperlimit](https://github.com/timschmidt/hyperlimit): exact orientation,
  in-circle, in-sphere, D-dimensional determinant, segment, ring, and
  classification predicates.
- [hypercurve](https://github.com/timschmidt/hypercurve): curved-region owner that can
  hand off line-only inputs after certified projection or flattening.
- [hypermesh](https://github.com/timschmidt/hypermesh): 3D mesh layer that can reuse
  planar triangulation for face-region assembly.
- [hyperpath](https://github.com/timschmidt/hyperpath),
  [hyperdrc](https://github.com/timschmidt/hyperdrc), and
  [hyperphysics](https://github.com/timschmidt/hyperphysics): downstream consumers of
  exact triangle topology.

## Typical Triangulation Problems

Triangulation is dominated by irreversible local choices: convex/reflex tests,
point-in-triangle checks, orientation signs, in-circle signs, segment incidence, and
edge-flip legality. A single near-collinear or near-cocircular float misclassification
can produce inverted triangles, missing constraints, or repair loops that do not
terminate.

`hypertri` handles those branch points with exact `Real` coordinates and `hyperlimit`
predicates. Performance work focuses on avoiding unnecessary exact calls before they
happen: retained polygon facts, local convex/reflex caches, triangle-AABB rejects,
source-ring metadata, runtime algorithm selection, diagnostics counters, and validation
APIs.

## Main Types

- `Point2`, `PolygonInput`, `PolygonInputFacts`, `PolygonRings`, `RingRange`, and
  `Constraint` describe exact polygon and PSLG inputs.
- `EarcutReport` and `EarcutDiagnostics` expose polygon triangulation diagnostics.
- `DelaunayTriangulation` and `ConstrainedDelaunayTriangulation` describe 2D Delaunay
  outputs and protected constraint edges.
- `PointD`, `VertexHandle`, `CellHandle`, `FacetKey`, `Facet`, `Face`, `Cell`,
  `TriangulationDataStructureD`, `TriangulationD`, `DelaunayTriangulationD`,
  `TdsCombinatorialValidationReportD`, `TdsManifoldValidationReportD`,
  `TdsGeometricValidationReportD`, `Simplex`, `DelaunayComplex`,
  `DelaunayInsertionReportD`, `BistellarFlipD`, `BistellarFlipReportD`, and
  `BistellarFlipApplyReportD` provide the D-dimensional model, TDS,
  validation, flip precondition/rewrite, and small exact oracle surfaces.
- `TriangulationOptions`, `PolygonTriangulationAlgorithm`, `QualityPolicy`, and
  `PolygonTriangulationPlan` describe runtime selection when enabled.
- Optional `f64` entry points are boundary adapters that reject non-finite coordinates
  and exact-lift finite values.

## Precision Model

Native inputs use `Real`. Optional `f64` APIs are for IO, rendering, tests, and
compatibility; they exact-lift finite floats before topology branches execute. Exact
orientation, ring-area, segment, in-circle, in-sphere, and D-dimensional determinant
signs flow through `hyperlimit`.

Topology validation is part of the precision story. Results expose validation helpers,
and constrained output distinguishes caller constraints from planarized protected
subsegments that are actually present as triangulation edges.

## Performance Model

`hypertri` avoids paying for exact predicates by preserving cheap structure: ring
ranges, signed area, local turn consistency, coordinate summaries, duplicate and
collinear facts, bounding boxes, convex/reflex caches, triangle-AABB rejects,
constraint-subsegment provenance, and diagnostics counters. These facts reduce
candidate sets and guide runtime algorithm selection without permitting float topology
decisions.

## Current Status

Implemented today:

- exact point, polygon, ring, constraint, and polygon-fact types;
- earcut-style triangulation for simple and holed polygons, with diagnostics,
  local-intersection curing, and split fallback;
- incremental Delaunay and constrained Delaunay triangulation, including constraint
  recovery, splitting, and exact in-circle re-legalization;
- a dynamic D-dimensional TDS model with stable handles, explicit infinite
  vertex/cell semantics, canonical facet keys, report-bearing reciprocal
  neighbor, finite-facet manifold, and finite-cell geometric validation, and
  small exact D-dimensional Delaunay complex construction backed by
  `hyperlimit` determinant predicates for validation/oracle workloads;
- oracle-backed D-dimensional insertion reports that identify exact
  empty-sphere conflict cells, canonical conflict-boundary facets, and the
  rebuilt exact complex while the production TDS cavity-stitcher remains future
  work;
- non-mutating D-dimensional bistellar flip reports that validate local
  Lawson/Pachner circuit arity, removed-cell presence, inserted-cell affine
  independence, and exact Delaunay legality before any future TDS mutation
  scheduler exists;
- functional D-dimensional flip rewrites on the exact complex oracle, replacing
  removed cells with inserted cells and validating the resulting complex before
  returning it;
- runtime polygon algorithm selection when enabled;
- optional finite-`f64` entry points and optional `serde` support;
- topology validation, local constrained-Delaunay validation, property tests, fuzz
  targets including exact D-dimensional flip round trips, and benchmarks.

Known limits: prepared polygon schedules and DCEL storage are still future performance
work. The accepted topology contract is exact and validation-heavy by design.

## Installation

Enable only the algorithms you use:

```toml
[dependencies]
hypertri = { version = "0.2.0", default-features = false, features = ["earcut"] }
```

Feature summary:

- `earcut`, `cdt`, and `nd` enable the three algorithm families.
- `all-algorithms` enables all algorithm families.
- `runtime-select` enables runtime polygon algorithm selection.
- `f64-interop` adds finite-`f64` boundary entry points.
- `serde` serializes public exact topology records.

## Usage

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

## Development

Useful local checks:

```text
cargo test
cargo test --features all-algorithms
cargo test --features earcut,f64-interop
cargo bench --bench earcut --features earcut,f64-interop
cargo bench --bench delaunay --features cdt,f64-interop
cargo bench --bench exact --features all-algorithms
```

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
