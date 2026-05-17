<h1>
  hyperlimit
  <img src="./doc/hyperlimit.png" alt="Hyper, a clever mathematician" width="144" align="right">
</h1>

`hyperlimit` provides hyperreal-first exact geometry predicates that report both
the classified result and how the result was decided.

It implements 2D/3D orientation, in-circle, in-sphere, line classification, and
plane classification over `hyperreal::Real`. The resolver uses Real structural
facts, exact structural term-sign filters, exact arithmetic, and bounded
hyperreal sign refinement. Primitive `f32` and `f64` are not predicate Real
values; they should only appear at rendering, IO, and interop boundaries.

## Hyper Ecosystem

`hyperlimit` is the exact decision layer: predicate policy, escalation order,
and result provenance for higher crates that must not invent local epsilons.

- [hyperreal](https://github.com/timschmidt/hyperreal): exact rational, symbolic, and computable
  real arithmetic.
- [hyperlimit](https://github.com/timschmidt/hyperlimit): exact predicate policy and certified
  geometric decisions.
- [hyperlattice](https://github.com/timschmidt/hyperlattice): small exact vector, matrix, and
  transform algebra.
- [hypercurve](https://github.com/timschmidt/hypercurve): planar curve, contour, region, and
  boolean geometry.
- [hypertri](https://github.com/timschmidt/hypertri): exact polygon triangulation and constrained
  Delaunay topology.
- [hypermesh](https://github.com/timschmidt/boolmesh): 3D mesh boolean experiments and the
  future exact-aware mesh-topology layer.
- [hypersolve](https://github.com/timschmidt/hypersolve): experimental exact-aware solver layer.
- [hyperdrc](https://github.com/timschmidt/hyperdrc): PCB design-readiness checks over exact-aware
  geometry adapters.
- [hyperphysics](https://github.com/timschmidt/hyperphysics): placeholder physics-domain crate
  for the exact geometry stack.
- [csgrs](https://github.com/timschmidt/csgrs): constructive solid geometry and polygon boolean
  engine used by HyperDRC and available as an interop target.

## Traditional Predicate Problems

Geometry algorithms usually fail at branch points: a determinant whose sign is
nearly zero, a point exactly on a segment, a circle test on cospherical points,
or a broad-phase filter that silently disagrees with the exact topology. Pure
`f64` code often papers over those cases with tolerances, but one wrong sign can
change a triangulation, boolean result, or clearance report.

`hyperlimit` makes the escalation ladder part of the API. It tries structural
facts first, then exact reducers and certified interval/ball filters, then
bounded `Real` refinement. If those stages cannot certify a result under the
caller policy, it returns `Unknown` with provenance instead of manufacturing a
float decision. The performance goal is not to run the most expensive exact
path every time; it is to make common exact cases cheap while preserving a
clear proof boundary for hard cases.

## Semantic Boundary and Structural Dispatch

`hyperlimit` is the exact decision layer. It owns reusable geometric predicate
traits, outcomes, policies, escalation stages, and small classifiers whose
semantics are shared by higher crates. It does not own Real representation,
vector/matrix storage, curve booleans, triangulation topology, solver models, or
domain-specific geometry.

Predicate dispatch may use facts carried by lower layers, but decisions must
remain exact. Current shortcuts use exact structural zero/sign facts, exact
arithmetic, and bounded hyperreal refinement. Future fast paths should accept
metadata such as integer-grid scale, dyadic denominator class, symbolic Real
class, sparse-zero masks, affine-transform kind, coordinate bounds expressed as
exact structural certificates, and source normalization facts. Those facts
should select faster determinant expansions or skip known-zero terms before
generic expression construction; they must not reintroduce primitive-float
predicate filters.

In Yap's exact geometric computation sense, a predicate result is exact because
its decision path is exact, filtered by a proof-producing certificate, or
reported as unknown. `hyperlimit` therefore preserves and consumes structural
facts instead of eagerly canonicalizing every scalar expression up front.

`hyperlimit` should keep reusable predicates here and leave object ownership
above it: segment/ring storage in `hypercurve` or `hypertri`, DCEL state in
`hypertri`, and active-set policy in `hypersolve`.

## Current State

Version `0.2.0` is an early, documented predicate crate with a small public API.
It is not a mesh, polygon, BSP, CSG, or intersection library.

Implemented:

- `Point2` and `Point3` with `Real` coordinates
- `RealZeroKnowledge`, `Point2DisplacementFacts`, `Segment2Facts`,
  `Triangle2Facts`, `TriangleEdge2`, and `Aabb2Facts` for predicate-facing
  structural dispatch metadata
- `compare_reals`, `compare_point2_lexicographic`, `point2_equal`, and
  report-bearing variants that expose predicate provenance
- `compare_point2_distance_squared`
- `classify_real_closed_interval`, `real_in_closed_interval`,
  `classify_closed_interval_intersection`, `closed_intervals_intersect`
- `certified_interval_sign` and `certified_ball_sign` as proof-producing
  pre-refinement sign filters over exact enclosures
- `classify_point_aabb2`, `point_in_aabb2`,
  `classify_aabb2_intersection`, `aabb2s_intersect`
- `orient2d`, `orient3d`
- `incircle2d`, `insphere3d`
- `classify_point_line`
- `classify_point_segment`, `point_on_segment`,
  `classify_segment_intersection`, `PreparedSegment2`, and
  cached-`Segment2Facts` variants
- `classify_point_triangle`, `PreparedTriangle2`, and cached-`Triangle2Facts`
  variants
- `ring_area_sign`, `classify_point_ring_even_odd`, `point_in_ring_even_odd`
- `Plane3`, `classify_point_plane`, `classify_point_oriented_plane`
- batch APIs, with optional Rayon parallel variants
- `PredicateOutcome<T>` with certainty and deciding stage
- `PredicatePolicy` for exact/refined predicate behavior
- Real structural-fact helpers and bounded sign refinement

## Installation

```toml
[dependencies]
hyperlimit = "0.2.0"
```

From sibling checkouts:

```toml
[dependencies]
hyperlimit = { path = "../hyperlimit" }
```

## Quick Start

```rust
use hyperlimit::{Point2, Sign, orient2d};
use hyperreal::Real;

let a = Point2::new(Real::from(0), Real::from(0));
let b = Point2::new(Real::from(1), Real::from(0));
let c = Point2::new(Real::from(0), Real::from(1));

let outcome = orient2d(&a, &b, &c);
assert_eq!(outcome.value(), Some(Sign::Positive));
```

Line-side classification maps orientation signs into geometry names:

```rust
use hyperlimit::{LineSide, Point2, classify_point_line};
use hyperreal::Real;

let from = Point2::new(Real::from(0), Real::from(0));
let to = Point2::new(Real::from(1), Real::from(0));
let point = Point2::new(Real::from(0), Real::from(1));

assert_eq!(
    classify_point_line(&from, &to, &point).value(),
    Some(LineSide::Left)
);
```

Plane classification works with explicit plane equations:

```rust
use hyperlimit::{Plane3, PlaneSide, Point3, classify_point_plane};
use hyperreal::Real;

let plane = Plane3::new(
    Point3::new(Real::from(0), Real::from(0), Real::from(1)),
    Real::from(-2),
);
let point = Point3::new(Real::from(0), Real::from(0), Real::from(3));

assert_eq!(
    classify_point_plane(&point, &plane).value(),
    Some(PlaneSide::Above)
);
```

## Outcomes and Policy

Predicates return `PredicateOutcome<T>`:

```rust
use hyperlimit::{Point2, PredicateOutcome, orient2d};
use hyperreal::Real;

let a = Point2::new(Real::from(0), Real::from(0));
let b = Point2::new(Real::from(1), Real::from(0));
let c = Point2::new(Real::from(0), Real::from(1));

match orient2d(&a, &b, &c) {
    PredicateOutcome::Decided { value, certainty, stage } => {
        println!("{value:?} from {certainty:?} at {stage:?}");
    }
    PredicateOutcome::Unknown { needed, stage } => {
        println!("unknown at {stage:?}; needs {needed:?}");
    }
}
```

Certainty values:

- `Exact`
- `Filtered`

Stages:

- `Structural`
- `Filter`
- `Exact`
- `Refined`
- `Undecided`

The shared sign resolver attempts:

1. Real structural sign
2. determinant term filters from exact structural zero/sign facts
3. exact Real or exact predicate hooks
4. bounded Real sign refinement
5. `Unknown`

Use policy-specific functions when the default strict behavior is not desired:

```rust
use hyperlimit::{Point2, PredicatePolicy, orient::orient2d_with_policy};
use hyperreal::Real;

let a = Point2::new(Real::from(0), Real::from(0));
let b = Point2::new(Real::from(1), Real::from(1));
let c = Point2::new(Real::from(2), Real::from(2));

let policy = PredicatePolicy {
    allow_refinement: false,
    ..PredicatePolicy::STRICT
};
let outcome = orient2d_with_policy(&a, &b, &c, policy);
println!("{outcome:?}");
```

Report-bearing APIs also expose `PredicateCertificate`. Use
`PredicateCertificate::precision_stage()` when callers need a stable public
classification of the exact-computation ladder without coupling to determinant
internals:

| Precision stage | Meaning |
| --- | --- |
| `StructuralFact` | Cached Real or object facts proved the predicate. |
| `ExactReducer` | Exact arithmetic or a fixed exact kernel proved it. |
| `CertifiedFilter` | A conservative exact enclosure/filter proved it. |
| `BoundedRefinement` | Bounded Real refinement proved it. |
| `ExplicitApproximatePolicy` | An opt-in approximate edge policy decided it. |
| `Unknown` | No proof-producing stage is known. |

`PredicateCertificate::is_proof_producing()` is `false` for explicit
approximate policy fallbacks and unknown certificates. This keeps rendering and
interop convenience separate from exact topology decisions.

`PredicateApiSemantics` is the coarser API-boundary label:

| API semantic | Use |
| --- | --- |
| `ExactPreserving` | Decided predicate reports backed by proof-producing certificates. |
| `ApproximationDeferring` | APIs that return explicit uncertainty instead of approximating. |
| `ApproximationForcing` | Explicit approximate edge policies or views. |
| `CachePopulating` | Prepared facts and versioned caches that aid scheduling but do not prove topology. |
| `PolicyDependent` | APIs whose escalation behavior is chosen by `PredicatePolicy`. |

Reports expose this with `PredicateReport::api_semantics()`, certificates with
`PredicateCertificate::api_semantics()`, and policies with
`PredicatePolicy::api_semantics()`. Session-level carriers mirror the same
labels: `CachedApproximateView` is `ApproximationForcing`, `VersionedFacts` is
`CachePopulating`, `VersionedPrepared` binds borrowed prepared predicates to a
construction version as `CachePopulating`, `ConstructionCertificate` follows
its predicate route, and `ExactGeometrySession` is `PolicyDependent`. Stale
versioned prepared objects are diagnostics for recomputation or slower
scheduling; they are not topology certificates.

## Real Model

Predicate coordinates are `hyperreal::Real`. The predicate layer reads:

- known sign
- exact zero/nonzero status
- exact rational state
- bounded sign refinement

Higher crates should preserve these facts alongside geometric objects when they
can reuse them across many predicates.

## Optional Features

| Feature | Purpose |
| --- | --- |
| `std` | Default feature. No special public API is attached to it currently. |
| `parallel` | Enables batch predicate variants under the same Real API. |
| `dispatch-trace` | Records predicate dispatch provenance during benchmarks. |

## Module Map

- `real`: Real fact mapping and borrowed Real arithmetic helpers
- `geometry::facts`: point-displacement, segment, and AABB extent structural
  facts for exact fast-path selection
- `predicate`: outcome, certainty, policy, and sign types
- `resolve`: shared internal sign-resolution pipeline
- `predicates::aabb`: exact stateless 2D axis-aligned box classifiers
- `predicates::distance`: exact squared-distance comparison predicates
- `predicates::interval`: exact Real interval containment and intersection
  classifiers
- `predicates::filters`: certified interval and ball sign filters that return
  explicit uncertainty instead of approximate tolerance decisions
- `predicates::order`: exact Real and point ordering predicates
- `orient`: points and orientation/circle/sphere predicates
- `predicates::segment`: exact point-on-segment and segment-intersection
  classifiers
- `predicates::ring`: exact signed-area/winding sign and even-odd point-ring
  helpers
- `plane`: explicit and oriented plane classification
- `classify`: line and plane side enums
- `batch`: sequential and optional parallel batch APIs
- `session`: predicate policy, versioned reports, versioned facts, versioned
  prepared predicates, approximate-view metadata, and construction
  certificates
- `error`: crate-local error/result types

Common public types and functions are re-exported from the crate root.

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
