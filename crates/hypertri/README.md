<h1>
  hypertri
  <img src="./doc/hypertri.png" alt="hypertri logo" width="144" align="right">
</h1>

`hypertri` owns exact-aware triangulation for the Hyper geometry stack. It
provides polygon triangulation, incremental Delaunay and constrained Delaunay
topology, and a small D-dimensional triangulation data structure and exact
Delaunay oracle.

The crate owns straight-edge connectivity decisions. Curved boundaries belong
to Hypercurve, general mesh topology and 3D Booleans belong to Hypermesh, and
solid-model grammar and conversions belong to CSGRS.

## Why exact triangulation?

Triangulators make irreversible local choices: whether a vertex is convex,
whether a point lies inside an ear, whether an edge is legal, or whether a
constraint intersects another segment. A wrong near-collinear or
near-cocircular floating-point classification can invert a triangle, lose a
constraint, or send a repair loop down the wrong branch.

Hypertri keeps those choices on the exact side of the API:

```text
Real points + polygon/constraint facts
                    │
                    ▼
      Hyperlimit orientation, incidence,
       in-circle, and in-sphere predicates
                    │
         ┌──────────┼───────────┐
         ▼          ▼           ▼
      earcut    Delaunay/CDT   D-dimensional TDS
         │          │           │
         └──────────┴───────────┘
                    ▼
       indexed topology + validation
```

Optional `f64` adapters reject non-finite inputs and lift each finite binary
float to its exact represented value before topology is decided.

## Primary types

| Type | Purpose |
| --- | --- |
| `Point2`, `ExactPoint` | Exact 2D point used by native triangulators. |
| `PolygonInput` | Owned flat polygon buffer with earcut-compatible hole starts and retained facts. |
| `PolygonInputFacts`, `RingInputFacts` | Conservative exact-set, winding, convexity, and degeneracy scheduling facts. |
| `Constraint` | One caller-indexed segment in a planar straight-line graph. |
| `Triangle`, `TriangleIndices` | Indexed triangle and flat earcut-compatible index output. |
| `earcut::EarcutReport` | Polygon result plus non-certifying workload diagnostics. |
| `cdt::DelaunayTriangulation` | Exact 2D points and Delaunay triangles. |
| `cdt::ConstrainedDelaunayTriangulation` | Caller constraints, planarized protected edges, exact Steiner points, and triangles. |
| `PointD`, `TriangulationDataStructureD`, `DelaunayComplex` | D-dimensional points, dynamic combinatorial storage, and small exact oracle complex. |
| `Error`, `Result<T>` | Invalid input, unsupported capability, predicate, topology, and validation failures. |

## Quick start

Enable the polygon triangulator:

```sh
cargo new exact-triangulation
cd exact-triangulation
cargo add hypertri --features earcut
```

Equivalent manifest entry:

```toml
[dependencies]
hypertri = { version = "0.4.1", default-features = false, features = ["earcut"] }
```

Replace `src/main.rs` with:

<!-- quickstart:start -->
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
<!-- quickstart:end -->

Run it with `cargo run`. The same source is
[`examples/basic.rs`](examples/basic.rs); the test suite compiles it and checks
that it remains identical to this README block.

## API guide

### Describe polygon input

| Task | API |
| --- | --- |
| Construct exact points | `Point2::new` |
| Build a checked owned polygon | `PolygonInput::new`, `PolygonInput::from_parts` |
| Borrow or consume its buffers | `vertices`, `hole_indices`, `into_parts` |
| Read retained structure | `facts` and `PolygonInputFacts` query methods |
| Interpret hole starts | `polygon::rings_from_hole_indices`, `polygon::open_ring_indices` |

The exterior ring begins at vertex zero. Each `hole_indices` entry is the
start of a hole in the same flat point buffer.

### Triangulate polygons (`earcut`)

| Task | API |
| --- | --- |
| Return flat indices | `earcut`, `earcut::triangulate` |
| Include diagnostics | `earcut_report`, `earcut::triangulate_report` |
| Supply an advanced kernel | `earcut::triangulate_with_kernel`, `triangulate_report_with_kernel` |

The result contains three indices per triangle. Diagnostics measure candidate,
containment, bounding-box, cure, and split-fallback work; they are not topology
certificates.

### Build 2D Delaunay topology (`cdt`)

| Task | API |
| --- | --- |
| Preserve caller-order insertion | `cdt::delaunay` |
| Use deterministic spatial insertion | `cdt::delaunay_spatial` |
| Recover constraints | `cdt::constrained_delaunay` |
| Inspect a Delaunay result | `points`, `triangles`, `into_parts` |
| Inspect constrained output | `points`, `constraints`, `constraint_edges`, `triangles`, `into_parts_with_constraint_edges` |
| Audit topology | `validate`, `validate_unconstrained_edges_are_delaunay` |

Constraint planarization may append exact Steiner vertices. `constraints()`
retains caller input; `constraint_edges()` returns the protected subsegments
actually present in the triangulation.

`delaunay_spatial` retains caller indices but can select another valid diagonal
on cocircular input because the Delaunay triangulation is not unique.

### Select a polygon algorithm at runtime (`runtime-select`)

| Task | API |
| --- | --- |
| Configure | `TriangulationOptions`, `PolygonTriangulationAlgorithm`, `QualityPolicy` |
| Triangulate an owned input | `triangulate_polygon` |
| Retain selection evidence | `triangulate_polygon_with_report` |
| Use borrowed buffers | `runtime::triangulate_polygon_points` |

Runtime selection can choose only algorithms compiled into the crate. Its
input facts guide scheduling; exact predicates still certify the topology.

### Work in D dimensions (`nd`)

| Task | API |
| --- | --- |
| Construct points and simplexes | `PointD::new`, `Simplex::new` |
| Build the small exact oracle | `nd::delaunay_complex`, `DelaunayComplex::from_parts` |
| Inspect and validate | `points`, `cells`, `validate` |
| Create mutable storage | `TriangulationDataStructureD::new`, `add_finite_vertex`, `add_infinite_vertex`, `add_cell` |
| Inspect stable handles | `vertex`, `cell`, `facet`, `facet_key`, `vertices`, `cells` |
| Validate the TDS | combinatorial, manifold, and geometric `validate_*` methods and reports |
| Wrap valid storage | `TriangulationD::new`, `DelaunayTriangulationD::new` |
| Analyze insertion | `DelaunayInsertionReportD::from_parts`, `insert_point_oracle` |
| Analyze/apply flips | `validate_bistellar_flip`, `flip_oracle`, `BistellarFlipD` and report types |

The D-dimensional complex is intended as a small semantic oracle and validation
surface. It is not presented as a production large-data tessellator.

### Use finite-float adapters (`f64-interop`)

`f64::earcut`, `f64::delaunay`, `f64::delaunay_spatial`, and
`f64::constrained_delaunay` accept finite coordinate pairs. Non-finite values
return `Error`; finite values are exact-lifted before any topology branch.

### Use predicates and kernels

The `predicates` module exposes triangulation-facing orientation, containment,
segment, in-circle, in-sphere, and D-dimensional predicate adapters. The
`kernel` module contains the `Kernel` abstraction and `ExactKernel` used by
algorithm implementations. Most applications should call a triangulator
rather than assemble topology from these lower-level pieces.

## Features

| Feature | Default | Effect |
| --- | --- | --- |
| `earcut` | no | Enables exact polygon triangulation. |
| `cdt` | no | Enables 2D Delaunay and constrained Delaunay triangulation. |
| `nd` | no | Enables D-dimensional TDS, validation, insertion, flip, and exact oracle APIs. |
| `all-algorithms` | no | Enables `earcut`, `cdt`, and `nd`. |
| `runtime-select` | no | Enables runtime polygon algorithm selection. |
| `f64-interop` | no | Enables finite-`f64` boundary adapters. |
| `serde` | no | Serializes public exact topology records; retained facts are rebuilt when required. |

Hypertri has no default features. Enable only the algorithm families your
application uses.

## Guarantees and boundaries

- Native topology decisions use `Real` coordinates and Hyperlimit predicates.
- Primitive floats are accepted only by the explicit `f64` adapter module.
- Validation methods check index, orientation, constraint, manifold, and local
  Delaunay invariants appropriate to each result type.
- Retained input facts and diagnostics guide work but do not replace exact
  predicates.
- Duplicate points, invalid ring/constraint indices, and unsupported
  configurations return typed errors.
- Curves must be segmented or otherwise converted by their owning crate before
  entering straight-edge triangulation.
- Hypertri returns connectivity; mesh ownership, Boolean classification, and
  solid grammar belong to higher layers.

## Common workflows

The checked-in examples cover each algorithm family:

```sh
cargo run --example basic --features earcut
cargo run --example cdt --features cdt
cargo run --example nd --features nd
```

The browser demonstration is deployed at
<https://timschmidt.github.io/hypertri/> and its source lives in
[`examples/hypertri_ui`](examples/hypertri_ui).

## Further documentation

[`PERFORMANCE.md`](PERFORMANCE.md) records benchmark methodology and retained
optimization evidence. Generate the complete API reference with:

```sh
cargo doc --open --all-features
```

## References

- Amenta, Nina, Sunghee Choi, and Günter Rote. “Incremental Constructions con
  BRIO.” *SoCG 2003*, pp. 211–219.
  [doi:10.1145/777792.777824](https://doi.org/10.1145/777792.777824).
- Boissonnat, Jean-Daniel, et al. “Triangulations in CGAL.”
  *Computational Geometry*, vol. 22, 2002, pp. 5–19.
  [doi:10.1016/S0925-7721(01)00054-2](https://doi.org/10.1016/S0925-7721%2801%2900054-2).
- Bowyer, Adrian. “Computing Dirichlet Tessellations.” *The Computer
  Journal*, vol. 24, no. 2, 1981, pp. 162–166.
  [doi:10.1093/comjnl/24.2.162](https://doi.org/10.1093/comjnl/24.2.162).
- Lee, Der-Tsai, and Arthur K. Lin. “Generalized Delaunay Triangulation for
  Planar Graphs.” *Discrete & Computational Geometry*, vol. 1, 1986,
  pp. 201–217. [doi:10.1007/BF02187695](https://doi.org/10.1007/BF02187695).
- Meisters, Gary H. “Polygons Have Ears.” *The American Mathematical Monthly*,
  vol. 82, no. 6, 1975, pp. 648–651.
  [doi:10.2307/2319703](https://doi.org/10.2307/2319703).
- Pachner, Udo. “P.L. Homeomorphic Manifolds Are Equivalent by Elementary
  Shellings.” *European Journal of Combinatorics*, vol. 12, no. 2, 1991,
  pp. 129–145.
  [doi:10.1016/S0195-6698(13)80080-7](https://doi.org/10.1016/S0195-6698%2813%2980080-7).
- Shewchuk, Jonathan Richard, and Brielin C. Brown. “Fast Segment Insertion and
  Incremental Construction of Constrained Delaunay Triangulations.”
  *Computational Geometry*, vol. 48, no. 8, 2015, pp. 554–574.
  [doi:10.1016/j.comgeo.2015.04.006](https://doi.org/10.1016/j.comgeo.2015.04.006).
- Shewchuk, Jonathan Richard. “Adaptive Precision Floating-Point Arithmetic
  and Fast Robust Geometric Predicates.” *Discrete & Computational Geometry*,
  vol. 18, 1997, pp. 305–363.
  [doi:10.1007/PL00009321](https://doi.org/10.1007/PL00009321).
- Watson, David F. “Computing the n-Dimensional Delaunay Tessellation with
  Application to Voronoi Polytopes.” *The Computer Journal*, vol. 24, no. 2,
  1981, pp. 167–172.
  [doi:10.1093/comjnl/24.2.167](https://doi.org/10.1093/comjnl/24.2.167).
- Yap, Chee K. “Towards Exact Geometric Computation.” *Computational
  Geometry*, vol. 7, 1997, pp. 3–23.
  [doi:10.1016/0925-7721(95)00040-2](https://doi.org/10.1016/0925-7721%2895%2900040-2).

Meisters motivates ear clipping; Bowyer, Watson, Lee, Lin, Amenta, Choi, and
Rote cover the Delaunay/CDT construction families; Pachner motivates bistellar
rewrites; Shewchuk, Brown, and Yap establish robust and exact decision
boundaries.

## Acknowledgements

Hypertri is developed by Timothy Schmidt. Its exact earcut-style algorithm
follows the topology of
[Mapbox Earcut](https://github.com/mapbox/earcut), with numeric decisions
replaced by Hyperlimit predicates. [`earcutr`](https://github.com/frewsxcv/earcutr)
is used only as a development-time differential oracle. The project also
compares behavior and design with [Spade](https://github.com/Stoeoef/spade)
and CGAL's triangulation packages; those projects are not production
dependencies.

## License and contributing

Hypertri is available under either the MIT License or the Apache License 2.0,
as declared in [`Cargo.toml`](Cargo.toml). The repository's [`LICENSE`](LICENSE)
contains the MIT terms.

Changes should preserve exact predicate decisions and validation coverage.
Before submitting a change, run:

```sh
cargo fmt --all -- --check
cargo test --all-features
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --all-features
```
