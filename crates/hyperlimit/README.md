# predicated

`predicated` is a Rust crate for geometry-oriented predicates that record *how*
a geometric decision was made. It provides 2D/3D orientation predicates,
in-circle and in-sphere predicates, and basic line/plane classification helpers
over scalar types that can expose structural facts.

The crate sits between scalar libraries that know more than "this is just a
number" and higher-level geometry kernels. Before using an expensive robust
fallback, `predicated` asks scalar values and intermediate determinant terms for
facts such as known sign, exact zero, provable nonzero status, exact/rational
state, magnitude bounds, interval bounds, and sign-refinement capability.

## Status

This is an early `0.1.0` crate. The public surface is intentionally small, and
this is not a full geometry kernel. It does not provide mesh, polygon, BSP, CSG,
or intersection algorithms.

Currently implemented:

- Generic `Point2<S>` and `Point3<S>` coordinate containers.
- `orient2d` and `orient3d`.
- `incircle2d` and `insphere3d`.
- `classify_point_line` for oriented 2D line-side classification.
- `Plane3`, `classify_point_plane`, and `classify_point_oriented_plane`.
- Predicate outcomes that record result certainty and deciding stage.
- Scalar traits for structural facts, magnitude facts, refinement, and `f64`
  filter conversion.
- Optional adapters for `robust`, `geogram_predicates`, `hyperreal`,
  `realistic_blas`, and `inari`.

Robust fallback paths require finite coordinates that can be converted to `f64`.
For interval inputs, only singleton finite intervals are exposed to `f64`
fallback to avoid classifying interval midpoints as if they were exact points.

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
    features = ["robust", "hyperreal"]
}
```

The crate uses Rust edition 2024.

## Quick Start

```rust
use predicated::{orient2d, Point2, Sign};

let a = Point2::new(0.0_f64, 0.0);
let b = Point2::new(1.0_f64, 0.0);
let c = Point2::new(0.0_f64, 1.0);

let outcome = orient2d(&a, &b, &c);
assert_eq!(outcome.value(), Some(Sign::Positive));
```

Line-side classification maps orientation signs into geometry terminology:

```rust
use predicated::{classify_point_line, LineSide, Point2};

let from = Point2::new(0.0_f64, 0.0);
let to = Point2::new(1.0_f64, 0.0);
let point = Point2::new(0.0_f64, 1.0);

assert_eq!(classify_point_line(&from, &to, &point).value(), Some(LineSide::Left));
```

Plane classification works with either an explicit plane equation or an oriented
plane through three points:

```rust
use predicated::{classify_point_plane, Plane3, PlaneSide, Point3};

let plane = Plane3::new(Point3::new(0.0_f64, 0.0, 1.0), -2.0);
let point = Point3::new(0.0_f64, 0.0, 3.0);

assert_eq!(classify_point_plane(&point, &plane).value(), Some(PlaneSide::Above));
```

## Predicate Outcomes

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

- `Exact`: known by structural/exact information or exact robust fallback.
- `Filtered`: known by a conservative numeric, interval, or magnitude filter.
- `Approximate`: known only by an approximate sign. Approximate results are
  opt-in.

`stage` records where the decision came from:

- `Structural`
- `Filter`
- `Exact`
- `Refined`
- `RobustFallback`
- `Undecided`

## Escalation Pipeline

The shared sign-resolution path attempts stages in this order:

1. Ask the scalar or final intermediate value for a structural sign.
2. Run predicate-specific structural term and conservative numeric filters.
3. Use exact/rational scalar facts when allowed by policy.
4. Ask the scalar to refine its sign when allowed by policy.
5. Use robust backend fallback when available and allowed by policy.
6. Return an approximate sign only if the policy explicitly allows it.
7. Otherwise return `PredicateOutcome::Unknown`.

The default policy is `PredicatePolicy::STRICT`, which does not return
approximate topology. `PredicatePolicy::APPROXIMATE` is useful for previews and
debugging.

Policy-specific variants live in their modules:

```rust
use predicated::{orient::orient2d_with_policy, Point2, PredicatePolicy};

let a = Point2::new(0.0_f64, 0.0);
let b = Point2::new(1.0_f64, 1.0);
let c = Point2::new(2.0_f64, 2.0);

let outcome = orient2d_with_policy(&a, &b, &c, PredicatePolicy::APPROXIMATE);
println!("{outcome:?}");
```

## Scalar Model

`StructuralScalar` can expose optional facts:

- known sign
- exact zero
- provable nonzero status
- exact/rational status
- magnitude bounds
- sign refinement down to a requested precision

`PredicateScalar` adds the arithmetic and conversion needed by the current
predicates:

- clone/debug
- `Add`, `Sub`, and `Mul`
- `to_f64()` for filters and fallback adapters

Primitive `f32` and `f64` implement these traits. Rich scalar backends can
implement them to let the predicate layer exploit more information before
falling back.

## Predicate Behavior

- `orient2d` computes the 2D determinant, checks structural information, uses a
  signed-term/magnitude filter, then uses an `f64` determinant filter when
  possible.
- `orient3d` computes the 3D orientation determinant, checks structural
  information, uses signed-term/magnitude filtering, then uses an `f64`
  determinant filter when possible.
- `classify_point_plane` evaluates
  `normal.x * point.x + normal.y * point.y + normal.z * point.z + offset` and
  includes an `f64` filter for the explicit plane equation.
- `classify_point_oriented_plane` delegates to `orient3d`.
- `classify_point_line` delegates to `orient2d`.
- `incircle2d` and `insphere3d` compute their determinant expressions
  generically, then rely on scalar sign resolution and optional robust fallback.
  `insphere3d` also uses signed-term filtering for its final left/right
  expression.

## Optional Features

| Feature | What it enables |
| --- | --- |
| `std` | Default feature flag. No special public API is attached to it currently. |
| `robust` | Fallback through the `robust` crate for finite `f64`-convertible orientation, in-circle, and in-sphere inputs. |
| `geogram` | Fallback through the Rust-port `dev-rust-port` branch of `geogram_predicates`. When both `geogram` and `robust` are enabled, fallback dispatch prefers Geogram. |
| `hyperreal` | Implements `StructuralScalar` and `PredicateScalar` for `hyperreal::Real`, forwarding structural sign, zero, exact/rational, magnitude, approximate `f64`, and refinement facts. |
| `realistic-blas` | Implements `StructuralScalar` and `PredicateScalar` for `realistic_blas::Scalar<B>`, forwarding scalar facts exposed by `realistic_blas`. |
| `interval` | Implements `StructuralScalar` and `PredicateScalar` for `inari::Interval`. |

The `geogram` feature depends on `geogram_predicates` from the `dev-rust-port`
Git branch. Its in-circle and in-sphere APIs expose symbolic perturbation, so
`predicated` calls both perturbation polarities and returns `Sign::Zero` when
they disagree. This preserves unperturbed boundary semantics.

## Scalar Backends

### Primitive Floats

`f32` and `f64` are supported without optional features. They provide finite
approximations and basic magnitude facts. Degenerate strict predicates return
`Unknown` unless a robust fallback feature is enabled.

### hyperreal

With `hyperreal`, predicates consume the current `hyperreal` structural API:
sign, zero knowledge, exact rational state, magnitude bits, finite `f64`
approximations, and bounded sign refinement.

### realistic_blas

With `realistic-blas`, predicates consume facts forwarded by
`realistic_blas::Scalar<B>`. Geometry policy remains in this crate; matrix and
vector operations remain in `realistic_blas`.

### inari Intervals

With `interval`, `inari::Interval` values become predicate scalars. Intervals
that exclude zero provide filtered sign knowledge, singleton zero provides exact
zero, and finite interval magnitude bounds feed determinant term filters.

`to_f64` is intentionally exposed only for singleton finite intervals. This
prevents robust/geogram fallback from accidentally classifying an interval
midpoint when the interval still represents multiple possible values.

`inari` 2.0.0 requires Haswell-class SIMD on x86-64. Build interval-enabled
targets with a suitable CPU flag, for example:

```sh
RUSTFLAGS='-Ctarget-cpu=haswell' cargo test --features interval
```

## Module Map

- `scalar`: `StructuralScalar`, `PredicateScalar`, `ScalarFacts`, and
  `MagnitudeBounds`.
- `predicate`: `Sign`, `Certainty`, `SignKnowledge`, `Escalation`,
  `PredicateOutcome`, `RefinementNeed`, and `PredicatePolicy`.
- `filter`: conservative determinant sign filters.
- `resolve`: shared internal sign-resolution pipeline.
- `orient`: point types and orientation/circle/sphere predicates.
- `plane`: explicit and oriented plane classification.
- `classify`: `LineSide` and `PlaneSide` enums.
- `backend`: optional backend adapters and backend capability descriptors.
- `error`: crate-local error/result types.

Common types and functions are re-exported at the crate root, including
`Point2`, `Point3`, `Plane3`, `orient2d`, `orient3d`, `incircle2d`,
`insphere3d`, `classify_point_line`, `classify_point_plane`,
`classify_point_oriented_plane`, `LineSide`, `PlaneSide`, `PredicateOutcome`,
`PredicatePolicy`, and scalar traits.

## Development

Run the default and minimal test suites:

```sh
cargo test
cargo test --no-default-features
```

Run tests with fallback backends:

```sh
cargo test --features robust
cargo test --features geogram
```

Run tests with structural scalar integrations:

```sh
cargo test --features hyperreal
cargo test --features realistic-blas
RUSTFLAGS='-Ctarget-cpu=haswell' cargo test --features interval
```

Run the broad feature matrix:

```sh
cargo test --features geogram,robust,hyperreal,realistic-blas
RUSTFLAGS='-Ctarget-cpu=haswell' cargo test --features geogram,robust,hyperreal,realistic-blas,interval
cargo clippy --all-targets
RUSTFLAGS='-Ctarget-cpu=haswell' cargo clippy --all-targets --features geogram,robust,hyperreal,realistic-blas,interval
```

## License

The package metadata declares `MIT OR Apache-2.0`. Check the repository license
files and `Cargo.toml` before publishing or redistributing modified packages.
