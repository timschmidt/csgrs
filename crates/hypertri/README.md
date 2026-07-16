<h1>
  hypertri
  <img src="./doc/hypertri.png" alt="hypertri logo" width="144" align="right">
</h1>

`hypertri` owns exact-aware triangulation for the Hyper geometry stack. It provides
earcut-style polygon triangulation, incremental Delaunay and constrained Delaunay
topology, a D-dimensional TDS model plus a small exact Delaunay oracle, and optional
`f64` interop that lifts finite inputs into `hyperreal::Real` before topology is
decided.

The crate is intentionally algorithm-feature-gated. Downstream crates can compile only
the triangulation surfaces they use while keeping exact predicate semantics consistent.

## WASM Demo

The deployed WASM app is available at <https://timschmidt.github.io/hypertri/>.

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

## API Overview

- `Point2`, `PolygonInput`, `PolygonInputFacts`, `PolygonRings`, `RingRange`, and
  `Constraint` describe exact polygon and PSLG inputs.
- `EarcutReport` and `EarcutDiagnostics` expose polygon triangulation diagnostics.
- `DelaunayTriangulation` and `ConstrainedDelaunayTriangulation` describe 2D Delaunay
  outputs and protected constraint edges.
- `cdt::delaunay_spatial` and `f64::delaunay_spatial` provide an opt-in,
  deterministic BRIO-style insertion schedule for large unordered batches while
  retaining caller point indices.
- `PointD`, `Simplex`, and `DelaunayComplex` provide the small exhaustive
  D-dimensional semantic oracle; insertion and bistellar-flip report types retain the
  exact facts behind each proposed rewrite.
- `TriangulationDataStructureD`, its stable vertex/cell/facet handles, and the
  combinatorial, manifold, and geometric validation reports provide the dynamic
  D-dimensional storage layer.
- `TriangulationOptions`, `PolygonTriangulationAlgorithm`, `QualityPolicy`, and
  `PolygonTriangulationPlan` describe runtime selection when enabled.
- Optional `f64` entry points are boundary adapters that reject non-finite coordinates
  and exact-lift finite values.

## Precision and Performance Model

Native inputs use `Real`. Optional `f64` APIs are for IO, rendering, tests, and
compatibility; they exact-lift finite floats before topology branches execute. Exact
orientation, ring-area, segment, in-circle, in-sphere, and D-dimensional determinant
signs flow through `hyperlimit`.

Topology validation is part of the precision contract. Results expose validation
helpers, and constrained output distinguishes caller constraints from the planarized
protected subsegments actually present as triangulation edges.

To contain expression growth, the algorithms retain ring ranges, signed areas, local
turn consistency, exact-rational summaries, duplicate/collinear facts, bounding boxes,
convex/reflex caches, constraint-subsegment provenance, and diagnostics counters. These
facts reject candidates and guide runtime selection, but never substitute a floating
point branch for an exact topology decision.

Measured optimization results and the reference-by-reference implementation audit are
recorded in [`PERFORMANCE.md`](PERFORMANCE.md).

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
  circuit arity, removed-cell presence, inserted-cell affine
  independence, and exact Delaunay legality before any future TDS mutation
  scheduler exists;
- functional D-dimensional flip rewrites on the exact complex oracle, replacing
  removed cells with inserted cells and validating the resulting complex before
  returning it;
- runtime polygon algorithm selection when enabled;
- optional finite-`f64` entry points and optional `serde` support;
- no local `earcutr` dependency; `earcutr` is only a crates.io dev-dependency for
  comparison/regression fixtures, while runtime triangulation code is owned here;
- topology validation, local constrained-Delaunay validation, property tests, fuzz
  targets including exact D-dimensional flip round trips, and benchmarks.

Known limits: prepared polygon schedules and DCEL storage are still future performance
work. The accepted topology contract is exact and validation-heavy by design.

## Installation

Enable only the algorithms you use:

```toml
[dependencies]
hypertri = { version = "0.4.0", default-features = false, features = ["earcut"] }
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

This program is available as the compiling [`examples/basic.rs`](examples/basic.rs)
example and can be run with `cargo run --example basic --features earcut`.

With `cdt`, use exact Delaunay or constrained Delaunay topology:

```rust
use hypertri::{cdt, Constraint, Point2, Real};

fn main() -> hypertri::Result<()> {
    let points = vec![
        Point2::new(Real::from(0), Real::from(0)),
        Point2::new(Real::from(2), Real::from(0)),
        Point2::new(Real::from(2), Real::from(2)),
        Point2::new(Real::from(0), Real::from(2)),
    ];

    let delaunay = cdt::delaunay(&points)?;
    let spatial = cdt::delaunay_spatial(&points)?;
    let constrained = cdt::constrained_delaunay(
        &points,
        &[Constraint::new(0, 1), Constraint::new(1, 2)],
    )?;
    delaunay.validate()?;
    spatial.validate()?;
    constrained.validate()?;
    Ok(())
}
```

See [`examples/cdt.rs`](examples/cdt.rs) for the compiling example.

`delaunay` preserves the historical caller-order insertion schedule.
`delaunay_spatial` uses biased randomized rounds with exact median spatial
ordering; it can select a different valid diagonal on cocircular inputs because
the Delaunay triangulation is then non-unique.

With `nd`, build and validate a small D-dimensional exact oracle complex:

```rust
use hypertri::{nd, PointD, Real};

fn main() -> hypertri::Result<()> {
    let points = vec![
        PointD::new(vec![Real::from(0), Real::from(0)]),
        PointD::new(vec![Real::from(1), Real::from(0)]),
        PointD::new(vec![Real::from(0), Real::from(1)]),
    ];

    let complex = nd::delaunay_complex(&points)?;
    complex.validate()?;
    assert_eq!(complex.cells().len(), 1);
    Ok(())
}
```

See [`examples/nd.rs`](examples/nd.rs) for the compiling example.

For mutable combinatorial storage, construct a `TriangulationDataStructureD`, add
finite or infinite vertices and full cells, inspect its validation reports, and wrap a
valid structure in `TriangulationD`.

## Development

Useful local checks:

```text
cargo fmt --all -- --check
cargo test --all-features
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --all-features --no-deps
cargo check --examples --benches --all-features
cargo test --features earcut,f64-interop
cargo run --example basic --features earcut
cargo run --example cdt --features cdt
cargo run --example nd --features nd
cargo bench --bench earcut --features earcut,f64-interop
cargo bench --bench delaunay --features cdt,f64-interop
cargo bench --bench exact --features all-algorithms
```

## References

- Bareiss, Erwin H. "Sylvester's Identity and Multistep Integer-Preserving
  Gaussian Elimination." *Mathematics of Computation* 22.103 (1968): 565-578.
  [doi:10.1090/S0025-5718-1968-0226829-0](https://doi.org/10.1090/S0025-5718-1968-0226829-0)
- Amenta, Nina, Sunghee Choi, and Günter Rote. "Incremental Constructions con
  BRIO." *Proceedings of the Nineteenth Annual Symposium on Computational
  Geometry* (2003): 211-219.
  [doi:10.1145/777792.777824](https://doi.org/10.1145/777792.777824)
- Boehm, Hans-J., Robert Cartwright, Mark Riggle, and Michael J. O'Donnell.
  "Exact Real Arithmetic: A Case Study in Higher Order Programming." *LFP '86*
  (1986): 162-173.
  [doi:10.1145/319838.319860](https://doi.org/10.1145/319838.319860)
- Boissonnat, Jean-Daniel, Olivier Devillers, Sylvain Pion, Monique Teillaud,
  and Mariette Yvinec. "Triangulations in CGAL." *Computational Geometry*
  22.1-3 (2002): 5-19.
  [doi:10.1016/S0925-7721(01)00054-2](https://doi.org/10.1016/S0925-7721%2801%2900054-2)
- Bowyer, Adrian. "Computing Dirichlet Tessellations." *The Computer Journal*
  24.2 (1981): 162-166.
  [doi:10.1093/comjnl/24.2.162](https://doi.org/10.1093/comjnl/24.2.162)
- de Berg, Mark, Otfried Cheong, Marc van Kreveld, and Mark Overmars.
  *Computational Geometry: Algorithms and Applications*. 3rd ed. Springer,
  2008. [doi:10.1007/978-3-540-77974-2](https://doi.org/10.1007/978-3-540-77974-2)
- Delaunay, Boris. "Sur la sphère vide." *Bulletin de l'Académie des Sciences
  de l'URSS*, no. 6 (1934): 793-800.
- Ericson, Christer. *Real-Time Collision Detection*. Morgan Kaufmann, 2005.
- Lawson, Charles L. "Software for C1 Surface Interpolation." *Mathematical
  Software III* (1977): 161-194.
  [doi:10.1016/B978-0-12-587260-7.50011-X](https://doi.org/10.1016/B978-0-12-587260-7.50011-X)
- Lee, Der-Tsai, and Arthur K. Lin. "Generalized Delaunay Triangulation for
  Planar Graphs." *Discrete & Computational Geometry* 1 (1986): 201-217.
  [doi:10.1007/BF02187695](https://doi.org/10.1007/BF02187695)
- Meisters, Gary H. "Polygons Have Ears." *The American Mathematical Monthly*
  82.6 (1975): 648-651.
  [doi:10.2307/2319703](https://doi.org/10.2307/2319703)
- Pachner, Udo. "P.L. Homeomorphic Manifolds Are Equivalent by Elementary
  Shellings." *European Journal of Combinatorics* 12.2 (1991): 129-145.
  [doi:10.1016/S0195-6698(13)80080-7](https://doi.org/10.1016/S0195-6698%2813%2980080-7)
- Shewchuk, Jonathan Richard. "Adaptive Precision Floating-Point Arithmetic and
  Fast Robust Geometric Predicates." *Discrete & Computational Geometry* 18.3
  (1997): 305-363.
  [doi:10.1007/PL00009321](https://doi.org/10.1007/PL00009321)
- Shewchuk, Jonathan Richard, and Brielin C. Brown. "Fast Segment Insertion and
  Incremental Construction of Constrained Delaunay Triangulations."
  *Computational Geometry* 48.8 (2015): 554-574.
  [doi:10.1016/j.comgeo.2015.04.006](https://doi.org/10.1016/j.comgeo.2015.04.006)
- Watson, David F. "Computing the n-Dimensional Delaunay Tessellation with
  Application to Voronoi Polytopes." *The Computer Journal* 24.2 (1981):
  167-172. [doi:10.1093/comjnl/24.2.167](https://doi.org/10.1093/comjnl/24.2.167)
- Yap, Chee K. "Towards Exact Geometric Computation." *Computational Geometry*
  7.1-2 (1997): 3-23.
  [doi:10.1016/0925-7721(95)00040-2](https://doi.org/10.1016/0925-7721%2895%2900040-2)
- Implementation lineage and comparison projects:
  [Mapbox Earcut](https://github.com/mapbox/earcut),
  [earcutr](https://github.com/frewsxcv/earcutr),
  [Spade](https://github.com/Stoeoef/spade), and the
  [CGAL triangulation packages](https://doc.cgal.org/latest/Manual/packages.html#PkgTriangulations).

## Hyper Ecosystem

`hypertri` builds on [hyperreal](https://github.com/timschmidt/hyperreal) and
[hyperlimit](https://github.com/timschmidt/hyperlimit), and supplies exact
straight-edge topology to [hypermesh](https://github.com/timschmidt/hypermesh),
[hyperbrep](https://github.com/timschmidt/hyperbrep),
[hyperpath](https://github.com/timschmidt/hyperpath), and the other
[Hyper geometry crates](https://github.com/timschmidt?tab=repositories&q=hyper&type=source).
