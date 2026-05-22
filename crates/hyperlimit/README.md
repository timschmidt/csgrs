<h1>
  hyperlimit
  <img src="./doc/hyperlimit.png" alt="Hyper, a clever mathematician" width="144" align="right">
</h1>

`hyperlimit` provides exact geometry predicates over `hyperreal::Real` values. Predicate
calls return both the classified result and provenance for how the result was decided.

The crate is not a polygon, mesh, BSP, CSG, or intersection engine. It owns reusable
predicate semantics and escalation policy; object topology belongs in the higher crate
that owns the geometry.

## Hyper Ecosystem

`hyperlimit` is the shared exact decision layer. It owns predicate semantics, policy,
classification enums, prepared predicate views, and construction-session provenance;
object storage stays in the crate that owns the geometry.

- [hyperreal](https://github.com/timschmidt/hyperreal): scalar values, structural facts,
  and bounded refinement.
- [hyperlattice](https://github.com/timschmidt/hyperlattice): vector/matrix facts that
  own vector, point, matrix, shared-scale, and homogeneous projective carriers.
- [hypercurve](https://github.com/timschmidt/hypercurve),
  [hypertri](https://github.com/timschmidt/hypertri), and
  [hypermesh](https://github.com/timschmidt/hypermesh): geometry/topology crates that
  should use shared predicate policy rather than local epsilon rules.
- [hypersolve](https://github.com/timschmidt/hypersolve),
  [hyperpath](https://github.com/timschmidt/hyperpath),
  [hyperdrc](https://github.com/timschmidt/hyperdrc), and
  [hyperphysics](https://github.com/timschmidt/hyperphysics): domain crates that need
  reusable exact decisions and auditable unknowns.
- [hyperbrep](https://github.com/timschmidt/hyperbrep), [hypersdf](https://github.com/timschmidt/hypersdf),
  [hypervoxel](https://github.com/timschmidt/hypervoxel), [hypercircuit](https://github.com/timschmidt/hypercircuit),
  [hyperparts](https://github.com/timschmidt/hyperparts), [hyperpack](https://github.com/timschmidt/hyperpack),
  and [hyperevolution](https://github.com/timschmidt/hyperevolution): sibling crates
  that should keep exact decisions report-bearing instead of local tolerance-based.

## Typical Predicate Problems

Geometry algorithms usually fail at branch points: a determinant near zero, a point
exactly on a segment, a cocircular/cospherical test, or a broad-phase shortcut that
disagrees with topology. Pure `f64` code often patches those cases with tolerances, but
one wrong sign can change triangulation, booleans, mesh topology, clearance reports, or
solver active sets.

`hyperlimit` makes the escalation ladder part of the API. It uses structural facts,
exact reducers, certified interval/ball filters, and bounded `Real` refinement. If the
configured policy cannot certify a result, it returns `Unknown` with provenance rather
than inventing a float decision.

## Main Types

- `Point2`, `Point3`, point facts, shared-scale point views, `HomogeneousPoint3`, and
  `HomogeneousLine3` are predicate-facing re-exports of lattice-owned object carriers.
- `Plane3`, `Plane3Facts`, `PreparedPlane3`, and homogeneous plane-incidence helpers
  keep 3D sidedness and projective incidence under predicate policy.
- `PredicateOutcome<T>`, `PredicateReport<T>`, `PredicateCertificate`, `Certainty`,
  `Escalation`, `PredicatePrecisionStage`, and `PredicateApiSemantics` describe what was
  decided and how.
- `PredicatePolicy` controls refinement and approximate-edge behavior.
- `Sign`, `LineSide`, `PlaneSide`, `TriangleLocation`, `SegmentIntersection`,
  `TriangleTriangleIntersection`, `RingPointLocation`, interval, and AABB
  classifications are the common result enums.
- `orient_d`, `insphere_d`, and `affine_independent_d` provide the first exact
  D-dimensional determinant predicate boundary for triangulation and mesh crates.
- Prepared segment, triangle, AABB, line, circle/sphere, plane, and halfspace-system
  helpers retain facts for repeated decisions.
- `SupportDop3` and `SupportSlab3` retain exact support-axis bounds and source
  witnesses for reusable k-DOP/bounding-volume slab predicates.
- `HalfspaceFeasibilityReport` records exact 3D halfspace feasibility witnesses,
  active plane sets, and Farkas-style infeasibility certificates for replayable
  convex-kernel prechecks; `PreparedHalfspaceSystem3` adds the borrowed
  session-prepared form for repeated feasibility queries.
- Session types such as `ExactGeometrySession`, `ConstructionCertificate`,
  `VersionedFacts`, and `VersionedPrepared` track cache freshness and construction
  provenance.

## Precision Model

Predicate coordinates are `Real` values. The resolver tries exact structural facts,
determinant term facts, exact reducers, certified interval/ball filters, and bounded
`Real` refinement. Approximate edge policy is explicit and labeled; it is not proof
producing. If policy cannot prove a result, the public result is `Unknown`.

Higher crates should carry object facts such as sparse coordinates, ring structure,
plane facts, or prepared bounds, but the final topology-changing decision should remain
exact or explicitly unknown.

## Numerical Explosion

`hyperlimit` combats numerical explosion by staging predicate work. Structural facts,
prepared bounds, determinant schedules, certified filters, and bounded `Real`
refinement are tried before generic exact expansion. When those stages cannot certify a
sign or relation, the result stays `Unknown` rather than forcing unbounded arithmetic.

## Performance Model

`hyperlimit` is designed to avoid expensive exact work in common cases. It uses
structural zero/sign facts, prepared point/segment/triangle/AABB facts, determinant
schedule hints, certified filters, and versioned prepared objects before generic
refinement. Optional batch APIs and the `parallel` feature let callers evaluate many
independent predicates under the same policy.

Dispatch tracing exists to show whether predicates are using structural facts, exact
reducers, filters, bounded refinement, or fallback paths.

## Current Status

Version `0.4.0` is an early but usable predicate crate. It currently includes:

- predicate-facing re-exports of lattice-owned `Point2`, `Point3`, shared-scale point
  facts, homogeneous points, and Pluecker lines;
- `Plane3`, prepared plane facts, homogeneous point/plane incidence classification,
  and exact two-plane/three-plane intersection wrappers;
- exact real and point ordering, squared-distance comparison, interval, AABB, segment,
  ring, triangle, line, plane, orientation, in-circle, in-sphere, and
  D-dimensional orientation/in-sphere/affine-independence predicates;
- exact report-bearing 3D triangle/triangle classification that composes
  plane-side rejection, segment/triangle edge replay, and coplanar projected
  overlap predicates;
- exact segment/ray triangle reports that retain support-plane crossing
  parameters, candidate points, and point/triangle replay evidence;
- exact support k-DOP slab carriers with witness-preserving point and AABB projection
  classifiers;
- exact 3D halfspace feasibility reports over `Plane3` systems, using active-set
  candidates, point-plane replay, Farkas-style negative certificates, and a
  prepared/session form instead of primitive LP tolerances;
- prepared segment, triangle, AABB, line, circle/sphere, plane, and halfspace-system
  helpers for repeated decisions;
- `PredicateOutcome`, `PredicateReport`, `PredicateCertificate`, certainty,
  precision-stage, API-semantics, and policy types;
- versioned sessions, construction certificates, cached approximate-view labels, and
  optional parallel batch APIs.

Known limits: `hyperlimit` intentionally stops at reusable predicates and small
classifiers. It does not store curves, triangulations, meshes, solver active sets, or
domain-specific geometry.

## Installation

```toml
[dependencies]
hyperlimit = "0.3.0"
```

Current local development version:

```toml
hyperlimit = "0.4.0"
```

Feature summary:

- `std`: default support feature.
- `parallel`: enables batch predicate variants backed by Rayon.
- `dispatch-trace`: records predicate dispatch provenance during benchmarks.

## Usage

```rust
use hyperlimit::{
    Plane3, Point2, Point3, Sign, classify_homogeneous_point_plane,
    intersect_three_planes, orient2d,
};
use hyperreal::Real;

let a = Point2::new(Real::from(0), Real::from(0));
let b = Point2::new(Real::from(1), Real::from(0));
let c = Point2::new(Real::from(0), Real::from(1));

assert_eq!(orient2d(&a, &b, &c).value(), Some(Sign::Positive));

let px = Plane3::new(Point3::new(Real::from(1), Real::from(0), Real::from(0)), Real::from(-1));
let py = Plane3::new(Point3::new(Real::from(0), Real::from(1), Real::from(0)), Real::from(-2));
let pz = Plane3::new(Point3::new(Real::from(0), Real::from(0), Real::from(1)), Real::from(-3));
let point = intersect_three_planes(&px, &py, &pz);
assert_eq!(classify_homogeneous_point_plane(&point, &px).value(), Some(true));
```

## Development

Useful local checks:

```sh
cargo test
cargo test --no-default-features
cargo test --all-features
cargo test --features parallel
cargo bench --bench predicates
```

## References

Bentley, Jon Louis, and Thomas A. Ottmann. "Algorithms for Reporting and
Counting Geometric Intersections." *IEEE Transactions on Computers*, vol. C-28,
no. 9, 1979, pp. 643-647.

de Berg, Mark, Otfried Cheong, Marc van Kreveld, and Mark Overmars.
*Computational Geometry: Algorithms and Applications*. 3rd ed., Springer, 2008.

Hormann, Kai, and Alexander Agathos. "The Point in Polygon Problem for
Arbitrary Polygons." *Computational Geometry*, vol. 20, no. 3, 2001, pp.
131-144.

Moore, Ramon E. *Interval Analysis*. Prentice-Hall, 1966.

Klosowski, James T., Martin Held, Joseph S. B. Mitchell, Henry Sowizral, and
Karel Zikan. "Efficient Collision Detection Using Bounding Volume Hierarchies
of k-DOPs." *IEEE Transactions on Visualization and Computer Graphics*, vol. 4,
no. 1, 1998, pp. 21-36.

Seidel, Raimund. "Small-Dimensional Linear Programming and Convex Hulls Made
Easy." *Discrete & Computational Geometry*, vol. 6, 1991, pp. 423-434.

Schrijver, Alexander. *Theory of Linear and Integer Programming*. Wiley, 1986.

Shewchuk, Jonathan Richard. "Adaptive Precision Floating-Point Arithmetic and
Fast Robust Geometric Predicates." *Discrete & Computational Geometry*, vol.
18, no. 3, 1997, pp. 305-363.

Yap, Chee K. "Towards Exact Geometric Computation." *Computational Geometry*,
vol. 7, nos. 1-2, 1997, pp. 3-23.

## Benchmarks and Development

Run checks:

```sh
cargo test
cargo test --no-default-features
cargo test --all-features
```

Run the broad feature set:

```sh
cargo test --features parallel
```

Run benchmarks:

```sh
cargo bench --bench predicates
```

The generated benchmark summary is in [`benchmarks.md`](benchmarks.md).

Run dispatch tracing separately:

```sh
cargo bench --bench predicates --features dispatch-trace -- --write-dispatch-trace-md
```

The generated trace summary is in [`dispatch_trace.md`](dispatch_trace.md).

## License

MIT OR Apache-2.0.
