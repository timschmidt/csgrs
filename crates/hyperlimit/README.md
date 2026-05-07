# predicated

`predicated` provides geometry predicates that report both the result and how
the result was decided.

It implements 2D/3D orientation, in-circle, in-sphere, line classification, and
plane classification over scalar types that can expose structural facts. The
resolver tries cheap scalar and determinant facts before exact arithmetic,
bounded refinement, or robust fallback.

## Relationship to Other Crates

- `hyperreal` supplies exact/symbolic scalar facts and bounded sign refinement
  through `hyperreal::Real`.
- `realistic_blas` supplies `Scalar<B>` values for vector and matrix code and
  forwards backend-neutral scalar facts to `predicated`.
- `predicated` owns predicate policy, escalation order, fallback selection, and
  result provenance.

`predicated` does not own scalar expression internals or linear algebra types.

## Current State

Version `0.1.2` is an early, documented predicate crate with a small public API.
It is not a mesh, polygon, BSP, CSG, or intersection library.

Implemented:

- `Point2<S>` and `Point3<S>`
- `orient2d`, `orient3d`
- `incircle2d`, `insphere3d`
- `classify_point_line`
- `Plane3`, `classify_point_plane`, `classify_point_oriented_plane`
- batch APIs, with optional Rayon parallel variants
- `PredicateOutcome<T>` with certainty and deciding stage
- `PredicatePolicy` for strict, filtered, refined, robust, and approximate
  behavior
- scalar traits for structural facts, finite `f64` filters, borrowed arithmetic,
  and bounded sign refinement
- optional adapters for `hyperreal`, `realistic_blas`, `inari`, `robust`, and
  `geogram_predicates`

Strict policies do not return approximate topology. Approximate signs are
available only when the policy explicitly allows them.

## Installation

```toml
[dependencies]
predicated = "0.1.2"
```

From sibling checkouts:

```toml
[dependencies]
predicated = { path = "../predicated" }
```

Enable only the backends you need:

```toml
[dependencies]
predicated = {
    version = "0.1.2",
    features = ["hyperreal", "robust"]
}
```

## Quick Start

```rust
use predicated::{Point2, Sign, orient2d};

let a = Point2::new(0.0_f64, 0.0);
let b = Point2::new(1.0_f64, 0.0);
let c = Point2::new(0.0_f64, 1.0);

let outcome = orient2d(&a, &b, &c);
assert_eq!(outcome.value(), Some(Sign::Positive));
```

Line-side classification maps orientation signs into geometry names:

```rust
use predicated::{LineSide, Point2, classify_point_line};

let from = Point2::new(0.0_f64, 0.0);
let to = Point2::new(1.0_f64, 0.0);
let point = Point2::new(0.0_f64, 1.0);

assert_eq!(
    classify_point_line(&from, &to, &point).value(),
    Some(LineSide::Left)
);
```

Plane classification works with explicit plane equations:

```rust
use predicated::{Plane3, PlaneSide, Point3, classify_point_plane};

let plane = Plane3::new(Point3::new(0.0_f64, 0.0, 1.0), -2.0);
let point = Point3::new(0.0_f64, 0.0, 3.0);

assert_eq!(
    classify_point_plane(&point, &plane).value(),
    Some(PlaneSide::Above)
);
```

## Outcomes and Policy

Predicates return `PredicateOutcome<T>`:

```rust
use predicated::{Point2, PredicateOutcome, orient2d};

let a = Point2::new(0.0_f64, 0.0);
let b = Point2::new(1.0_f64, 0.0);
let c = Point2::new(0.0_f64, 1.0);

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
- `RobustFloat`
- `Approximate`

Stages:

- `Structural`
- `Filter`
- `Exact`
- `Refined`
- `RobustFallback`
- `Undecided`

The shared sign resolver attempts:

1. scalar structural sign
2. determinant term filters and conservative numeric filters
3. exact scalar or exact predicate hooks
4. bounded scalar sign refinement
5. robust backend fallback
6. approximate sign, only when policy allows it
7. `Unknown`

Use policy-specific functions when the default strict behavior is not desired:

```rust
use predicated::{Point2, PredicatePolicy, orient::orient2d_with_policy};

let a = Point2::new(0.0_f64, 0.0);
let b = Point2::new(1.0_f64, 1.0);
let c = Point2::new(2.0_f64, 2.0);

let outcome = orient2d_with_policy(&a, &b, &c, PredicatePolicy::APPROXIMATE);
println!("{outcome:?}");
```

## Scalar Model

`StructuralScalar` can expose:

- known sign
- exact zero status
- provable nonzero status
- exact/rational state
- magnitude bounds
- bounded sign refinement

`PredicateScalar` adds arithmetic and finite `f64` conversion for filters and
fallbacks. `BorrowedPredicateScalar` adds borrowed `add_ref`, `sub_ref`, and
`mul_ref` operations so richer scalar backends avoid cloning every determinant
term.

Primitive `f32` and `f64` implement the scalar traits without optional features.

## Optional Features

| Feature | Purpose |
| --- | --- |
| `std` | Default feature. No special public API is attached to it currently. |
| `parallel` | Rayon-backed batch predicate variants. |
| `hyperreal` | Implements predicate scalar traits for `hyperreal::Real`. |
| `realistic-blas` | Implements predicate scalar traits for `realistic_blas::Scalar<B>`. |
| `interval` | Implements predicate scalar traits for `inari::Interval`. |
| `robust` | Robust fallback through the `robust` crate for finite `f64`-convertible inputs. |
| `geogram` | Robust fallback through the Rust-port `geogram_predicates` branch. |

Interval values expose finite `f64` fallback only for singleton finite
intervals. This avoids treating interval midpoints as exact geometry.

`inari` 2 requires Haswell-class SIMD on x86-64:

```sh
RUSTFLAGS='-Ctarget-cpu=haswell' cargo test --features interval
```

When both `geogram` and `robust` are enabled, fallback dispatch prefers Geogram.
For Geogram in-circle and in-sphere fallback, both symbolic-perturbation
polarities are checked and disagreement is reported as zero.

## Module Map

- `scalar`: scalar traits and fact types
- `predicate`: outcome, certainty, policy, and sign types
- `filter`: conservative determinant filters
- `resolve`: shared internal sign-resolution pipeline
- `orient`: points and orientation/circle/sphere predicates
- `plane`: explicit and oriented plane classification
- `classify`: line and plane side enums
- `batch`: sequential and optional parallel batch APIs
- `backend`: optional backend adapters
- `error`: crate-local error/result types

Common public types and functions are re-exported from the crate root.

## Benchmarks and Development

Run checks:

```sh
cargo test
cargo test --no-default-features
cargo test --features robust
cargo test --features hyperreal
cargo test --features realistic-blas
```

Run the broad feature set:

```sh
cargo test --features geogram,robust,hyperreal,realistic-blas,parallel
RUSTFLAGS='-Ctarget-cpu=haswell' cargo test --features geogram,robust,hyperreal,realistic-blas,interval,parallel
```

Run benchmarks:

```sh
cargo bench --bench predicates
```

The generated benchmark summary is in [`benchmarks.md`](benchmarks.md).

Render predicate demo plots:

```sh
cargo run --example predicate_plots -- --out doc/predicate-plots --size 512
```

See [`plots.md`](plots.md) for the plot gallery.

## License

MIT OR Apache-2.0.
