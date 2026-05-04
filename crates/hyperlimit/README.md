# predicated

`predicated` is a Rust crate for geometry-oriented predicates that keep track of *how* a geometric decision was made. It provides 2D/3D orientation predicates, in-circle and in-sphere predicates, and basic line/plane classification helpers over scalar types that can expose structural facts.

The crate is intended to sit between scalar libraries that know more than “this is just a number” and higher-level geometry kernels. Before using an expensive robust fallback, `predicated` asks scalar values and intermediate determinant terms for information such as known sign, exact zero, provable nonzero status, exact/rational state, magnitude bounds, and sign-refinement capability.

## Status

This is an early `0.1.0` crate. The public surface is small, and some features are present as integration points rather than complete geometry subsystems.

Currently implemented:

* Generic `Point2<S>` and `Point3<S>` coordinate containers.
* `orient2d` and `orient3d`.
* `incircle2d` and `insphere3d`.
* `classify_point_line` for oriented 2D line-side classification.
* `Plane3`, `classify_point_plane`, and `classify_point_oriented_plane`.
* Predicate outcomes that record result certainty and the stage that decided the result.
* Scalar traits for structural facts and `f64` filter conversion.
* Optional adapters for `robust`, `geogram_predicates`, `hyperreal`, and `realistic_blas`.

Not currently a full geometry kernel:

* No mesh, polygon, BSP, CSG, or intersection kernel is implemented here.
* No interval-arithmetic backend is implemented yet, although an `interval` feature flag exists.
* Robust fallback paths require finite coordinates that can be converted to `f64`.

## Installation

From a local checkout:

```toml
[dependencies]
predicated = { path = "../predicated" }
```

From Git:

```toml
[dependencies]
predicated = { git = "https://github.com/timschmidt/predicated" }
```

Optional backend features can be enabled as needed:

```toml
[dependencies]
predicated = {
    git = "https://github.com/timschmidt/predicated",
    features = ["robust"]
}
```

The crate currently uses Rust edition 2024.

## Quick start

```rust
use predicated::{orient2d, Point2, Sign};

let a = Point2::new(0.0_f64, 0.0);
let b = Point2::new(1.0_f64, 0.0);
let c = Point2::new(0.0_f64, 1.0);

let outcome = orient2d(&a, &b, &c);
assert_eq!(outcome.value(), Some(Sign::Positive));
```

Line-side classification maps the same orientation sign into geometry terminology:

```rust
use predicated::{classify_point_line, LineSide, Point2};

let from = Point2::new(0.0_f64, 0.0);
let to = Point2::new(1.0_f64, 0.0);
let point = Point2::new(0.0_f64, 1.0);

assert_eq!(classify_point_line(&from, &to, &point).value(), Some(LineSide::Left));
```

Plane classification works with either an explicit plane equation or an oriented plane through three points:

```rust
use predicated::{classify_point_plane, Plane3, PlaneSide, Point3};

let plane = Plane3::new(Point3::new(0.0_f64, 0.0, 1.0), -2.0);
let point = Point3::new(0.0_f64, 0.0, 3.0);

assert_eq!(classify_point_plane(&point, &plane).value(), Some(PlaneSide::Above));
```

## Predicate outcomes

Predicates return `PredicateOutcome<T>` rather than just `T`:

```rust
use predicated::{orient2d, Point2, PredicateOutcome};

let a = Point2::new(0.0_f64, 0.0);
let b = Point2::new(1.0_f64, 0.0);
let c = Point2::new(0.0_f64, 1.0);

match orient2d(&a, &b, &c) {
    PredicateOutcome::Decided { value, certainty, stage } => {
        println!("{value:?} decided with {certainty:?} certainty at {stage:?}");
    }
    PredicateOutcome::Unknown { needed, stage } => {
        println!("undecided at {stage:?}; needs {needed:?}");
    }
}
```

`certainty` is one of:

* `Exact`: known by structural/exact information or exact robust fallback.
* `Filtered`: known by a conservative filter.
* `Approximate`: known only by an approximate sign. Approximate results are opt-in.

`stage` records where the decision came from:

* `Structural`
* `Filter`
* `Exact`
* `Refined`
* `RobustFallback`
* `Undecided`

## Escalation pipeline

The shared sign-resolution path currently attempts stages in this order:

1. Ask the scalar/intermediate value for a structural sign.
2. Run the predicate-specific filter, when one exists.
3. Use exact/rational scalar facts when allowed by policy.
4. Ask the scalar to refine its sign when allowed by policy.
5. Use robust backend fallback when available and allowed by policy.
6. Return an approximate sign only if the policy explicitly allows it.
7. Otherwise return `PredicateOutcome::Unknown`.

The default policy is `PredicatePolicy::STRICT`, which does not return approximate topology. `PredicatePolicy::APPROXIMATE` is useful for previews and debugging.

Policy-specific variants live in their modules:

```rust
use predicated::{orient::orient2d_with_policy, Point2, PredicatePolicy};

let a = Point2::new(0.0_f64, 0.0);
let b = Point2::new(1.0_f64, 1.0);
let c = Point2::new(2.0_f64, 2.0);

let outcome = orient2d_with_policy(&a, &b, &c, PredicatePolicy::APPROXIMATE);
println!("{outcome:?}");
```

## Scalar model

The starter scalar abstraction is intentionally small.

`StructuralScalar` can expose optional facts:

* known sign
* exact zero
* provable nonzero status
* exact/rational status
* magnitude bounds
* sign refinement down to a requested precision

`PredicateScalar` adds the arithmetic and conversion needed by the current predicates:

* clone/debug
* `Add`, `Sub`, and `Mul`
* `to_f64()` for filters and fallback adapters

Primitive `f32` and `f64` implement these traits. Rich scalar backends can implement them to make the predicate layer exploit more information before falling back.

## Predicates and filters

Current predicate behavior:

* `orient2d` computes the 2D determinant, checks structural information, uses a signed-term/magnitude filter, then uses an `f64` determinant filter when possible.
* `orient3d` computes the 3D orientation determinant, checks structural information, uses signed-term/magnitude filtering, then uses an `f64` determinant filter when possible.
* `classify_point_plane` evaluates `normal.x * point.x + normal.y * point.y + normal.z * point.z + offset` and includes an `f64` filter for the explicit plane equation.
* `classify_point_oriented_plane` delegates to `orient3d`.
* `classify_point_line` delegates to `orient2d`.
* `incircle2d` and `insphere3d` compute their determinant expressions generically, then rely on scalar sign resolution and optional robust fallback. `insphere3d` also uses signed-term filtering for its final left/right expression.

## Optional features

| Feature          | What it currently enables                                                                                                                                                                |
| ---------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `std`            | Default feature flag. No special public API is attached to it currently.                                                                                                                 |
| `robust`         | Enables fallback through the `robust` crate for finite `f64`-convertible orientation, in-circle, and in-sphere inputs.                                                                   |
| `geogram`        | Enables fallback through the Rust-port branch of `geogram_predicates`. When both `geogram` and `robust` are enabled, the orientation and circle/sphere fallback dispatch uses `geogram`. |
| `hyperreal`      | Implements `StructuralScalar` and `PredicateScalar` for `hyperreal::Real`, forwarding structural sign, zero, exact/rational, magnitude, approximate `f64`, and refinement facts.         |
| `realistic-blas` | Implements `StructuralScalar` and `PredicateScalar` for `realistic_blas::Scalar<B>`, forwarding the scalar facts exposed by `realistic_blas`.                                            |
| `interval`       | Declared feature flag reserved for future interval support. No interval backend is implemented yet.                                                                                      |

The `geogram` feature depends on `geogram_predicates` from the `dev-rust-port` Git branch, so expect that dependency to behave more like an integration target than a stable crates.io dependency.

## Module map

* `scalar`: `StructuralScalar`, `PredicateScalar`, `ScalarFacts`, and `MagnitudeBounds`.
* `predicate`: `Sign`, `Certainty`, `SignKnowledge`, `Escalation`, `PredicateOutcome`, `RefinementNeed`, and `PredicatePolicy`.
* `filter`: conservative determinant sign filters.
* `resolve`: shared internal sign-resolution pipeline.
* `orient`: point types and orientation/circle/sphere predicates.
* `plane`: explicit and oriented plane classification.
* `classify`: `LineSide` and `PlaneSide` enums.
* `backend`: optional backend adapters and backend capability descriptors.
* `error`: crate-local error/result types.

Common types and functions are re-exported at the crate root, including `Point2`, `Point3`, `Plane3`, `orient2d`, `orient3d`, `incircle2d`, `insphere3d`, `classify_point_line`, `classify_point_plane`, `classify_point_oriented_plane`, `LineSide`, `PlaneSide`, `PredicateOutcome`, `PredicatePolicy`, and scalar traits.

## Development

Run the default test suite:

```sh
cargo test
```

Run tests with a fallback backend:

```sh
cargo test --features robust
cargo test --features geogram
```

Run tests with structural scalar integrations:

```sh
cargo test --features hyperreal
cargo test --features realistic-blas
```

Because some features pull optional external crates, it can be useful to test the minimal default build and each integration feature separately.

## License

The repository includes an MIT license file. The package metadata currently declares `MIT OR Apache-2.0`, so check both `LICENSE` and `Cargo.toml` before publishing or redistributing modified packages.
