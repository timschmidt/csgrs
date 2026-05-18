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

`hyperlimit` is the shared exact decision layer.

- [hyperreal](https://github.com/timschmidt/hyperreal): scalar values, structural facts,
  and bounded refinement.
- [hyperlattice](https://github.com/timschmidt/hyperlattice): vector/matrix facts that
  can help prepare predicates.
- [hypercurve](https://github.com/timschmidt/hypercurve),
  [hypertri](https://github.com/timschmidt/hypertri), and
  [hypermesh](https://github.com/timschmidt/hypermesh): geometry/topology crates that
  should use shared predicate policy rather than local epsilon rules.
- [hypersolve](https://github.com/timschmidt/hypersolve),
  [hyperpath](https://github.com/timschmidt/hyperpath),
  [hyperdrc](https://github.com/timschmidt/hyperdrc), and
  [hyperphysics](https://github.com/timschmidt/hyperphysics): domain crates that need
  reusable exact decisions and auditable unknowns.

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

- `Point2`, `Point3`, `Plane3`, and geometry fact types provide small predicate-facing
  objects.
- `PredicateOutcome<T>`, `PredicateReport<T>`, `PredicateCertificate`, `Certainty`,
  `Escalation`, `PredicatePrecisionStage`, and `PredicateApiSemantics` describe what was
  decided and how.
- `PredicatePolicy` controls refinement and approximate-edge behavior.
- `Sign`, `LineSide`, `PlaneSide`, `TriangleLocation`, `SegmentIntersection`,
  `RingPointLocation`, interval, and AABB classifications are the common result enums.
- `orient_d`, `insphere_d`, and `affine_independent_d` provide the first exact
  D-dimensional determinant predicate boundary for triangulation and mesh crates.
- Prepared segment, triangle, AABB, line, circle/sphere, and plane helpers retain facts
  for repeated decisions.
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

## Performance Model

`hyperlimit` is designed to avoid expensive exact work in common cases. It uses
structural zero/sign facts, prepared point/segment/triangle/AABB facts, determinant
schedule hints, certified filters, and versioned prepared objects before generic
refinement. Optional batch APIs and the `parallel` feature let callers evaluate many
independent predicates under the same policy.

Dispatch tracing exists to show whether predicates are using structural facts, exact
reducers, filters, bounded refinement, or fallback paths.

## Current Status

Version `0.2.0` is an early but usable predicate crate. It currently includes:

- `Point2`, `Point3`, `Plane3`, and predicate-facing structural fact carriers;
- exact real and point ordering, squared-distance comparison, interval, AABB, segment,
  ring, triangle, line, plane, orientation, in-circle, in-sphere, and
  D-dimensional orientation/in-sphere/affine-independence predicates;
- prepared segment, triangle, AABB, line, circle/sphere, and plane helpers for repeated
  decisions;
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

Feature summary:

- `std`: default support feature.
- `parallel`: enables batch predicate variants backed by Rayon.
- `dispatch-trace`: records predicate dispatch provenance during benchmarks.

## Usage

```rust
use hyperlimit::{Point2, Sign, orient2d};
use hyperreal::Real;

let a = Point2::new(Real::from(0), Real::from(0));
let b = Point2::new(Real::from(1), Real::from(0));
let c = Point2::new(Real::from(0), Real::from(1));

assert_eq!(orient2d(&a, &b, &c).value(), Some(Sign::Positive));
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
