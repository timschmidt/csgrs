# hyperreal

`hyperreal` provides exact rational arithmetic, symbolic real values, and lazy
computable real approximation.

It is useful when code needs more information than an `f64` can provide:
structural sign facts, exact zero/nonzero knowledge, exact rational access,
bounded sign refinement, and recognizable forms such as `pi`, `e`, square
roots, logarithms, and rational trig constants.

## Main Types

- `Rational`: arbitrary-precision exact rationals, including exact IEEE-754
  `f32`/`f64` import.
- `Computable`: lazy real expressions approximated at a requested binary
  precision.
- `Real`: a rational scale plus a symbolic/computable class. It preserves exact
  structure when doing so helps arithmetic, predicates, or approximation.
- `RealStructuralFacts`: conservative public facts about sign, zero status,
  magnitude, and exact-rational state.
- `Simple`: a small Lisp-like expression parser, enabled by the default
  `simple` feature.

## Numeric Model

`hyperreal` is built around three layers that deliberately keep exact and
symbolic information available before approximation:

- `Rational` is the exact arithmetic base. It stores arbitrary-precision
  numerator/denominator values and performs exact reduction, dyadic detection,
  square extraction, shared-denominator dot products, and exact IEEE-754 import.
- `Computable` is the lazy approximation layer. It represents exact-real
  expression graphs such as sums, products, inverses, roots, logs, trig kernels,
  and shared constants. It approximates only when a caller asks for a binary
  precision, then caches the result and conservative sign/magnitude facts.
- `Real` is the public symbolic scalar. It stores an exact rational scale plus a
  compact symbolic class and, when needed, a `Computable` certificate. Common
  classes include exact one, powers/products of `pi` and `e`, selected square
  roots, logarithms, trig forms, and factored constant products.

The performance policy follows from that composition: reduce exact rational and
symbolic structure first, retain reusable forms like `pi`, `e`, `sqrt(2)`, and
small-log constants, defer numeric approximation until the final requested
precision, and reuse cached approximations when repeated matrix, vector, or
predicate workloads ask for digits.

## Relationship to Other Crates

- `realistic_blas` uses `hyperreal::Real` as its default exact/symbolic scalar
  backend and forwards `hyperreal` structural facts through its `Scalar` type.
- `liminal` can consume `hyperreal::Real` directly, using structural facts,
  finite `f64` approximations, and bounded sign refinement before robust
  fallback.

`hyperreal` owns scalar representation and approximation. It does not own vector
or matrix algebra, and it does not decide geometry topology.

## Current State

The crate is benchmark-driven and no longer just a direct port of computable
real ideas. Current implementation work includes:

- exact rational and dyadic fast paths
- dedicated identity constructors for common exact ones and zeros
- cached internal constants for `pi`, `tau`, `e`, common square roots, and
  common logarithms
- symbolic classes for selected `pi`, `e`, `sqrt`, `ln`, `sin(pi*q)`, and
  `tan(pi*q)` forms
- exact trig, inverse-trig, logarithm, exponential, and inverse-hyperbolic
  shortcuts where the input structure is recognizable
- argument reduction and prescaled kernels for transcendental approximation
- structural sign, zero, nonzero, magnitude, and exact-rational queries
- bounded sign refinement through `sign_until` and `refine_sign_until`
- borrowed arithmetic paths for `Rational` and `Real`
- `serde` support for expression structure, excluding transient caches and abort
  signals

This is a scalar library for exact/symbolic experimentation, predicate filters,
and small algebraic workloads. It is active and benchmark-driven, but it is not
a dense numeric BLAS replacement.

## Installation

```toml
[dependencies]
hyperreal = "0.10.6"
```

Without the `Simple` parser and calculator binary:

```toml
[dependencies]
hyperreal = { version = "0.10.6", default-features = false }
```

Feature flags:

| Feature | Default | Purpose |
| --- | --- | --- |
| `simple` | yes | Enables `Simple` and the package calculator binary. |

## Examples

### Exact Rationals

```rust
use hyperreal::Rational;

let a = Rational::fraction(7, 8).unwrap();
let b = Rational::fraction(9, 10).unwrap();

assert_eq!(a + b, Rational::fraction(79, 40).unwrap());
```

### Symbolic Reals

```rust
use hyperreal::{Rational, Real};

let x = Real::new(Rational::new(2)).sqrt().unwrap();
let y = Real::new(Rational::new(3)).sqrt().unwrap();
let z = x * y;

let approx: f64 = z.into();
assert!(approx > 2.44 && approx < 2.45);
```

### Computable Approximation

```rust
use hyperreal::{Computable, Rational};

let x = Computable::rational(Rational::fraction(7, 5).unwrap()).sin();
let scaled = x.approx(-40);

assert_ne!(scaled, 0.into());
```

### Structural Facts

```rust
use hyperreal::{Rational, Real, RealSign, ZeroKnowledge};

let value = Real::new(Rational::new(2)).sqrt().unwrap();
let facts = value.structural_facts();

assert_eq!(facts.sign, Some(RealSign::Positive));
assert_eq!(facts.zero, ZeroKnowledge::NonZero);
assert!(!facts.exact_rational);
assert_eq!(value.refine_sign_until(-64), Some(RealSign::Positive));
```

Facts are conservative. Missing sign or magnitude information means the fact
was not proven cheaply.

### Simple Expressions

```rust
use hyperreal::Simple;

let expr: Simple = "(* (+ pi pi) (sin (/ 1 5)))".parse().unwrap();
let value = expr.evaluate(&Default::default()).unwrap();

let _: f64 = value.into();
```

`Simple` supports arithmetic, roots, powers, logs, exponentials, trig, inverse
trig, inverse hyperbolic functions, integers, decimals, fractions, `pi`, and
`e`.

## Conversions

Supported conversions include:

- integer types to `Rational` and `Real`
- finite `f32`/`f64` to `Rational` and `Real` by exact IEEE-754 decoding
- `Real` to `f32`/`f64` by approximation
- `Real::to_f64_approx()` for borrowed finite approximation used by filters

Float import rejects `NaN` and infinities. `to_f64_approx()` returns `None`
when no finite `f64` approximation can be produced.

## Performance Notes

Performance shortcuts are intentionally documented next to the code that uses
them. The main techniques are:

- keep exact rational and dyadic values outside generic computable graphs
- build identity values through dedicated constructors and clone cached named
  constants instead of rebuilding them
- preserve lightweight symbolic classes only where benchmarks show value
- reduce trig and exponential arguments before entering series kernels
- use endpoint and tiny-argument transforms for inverse trig and inverse
  hyperbolic functions
- answer structural queries from certificates before refining approximations
- use borrowed arithmetic to reduce expression-graph cloning in callers such as
  `realistic_blas` and `liminal`

Benchmark suites:

```sh
cargo bench --bench library_perf
cargo bench --bench numerical_micro
cargo bench --bench borrowed_ops
cargo bench --bench float_convert
cargo bench --bench scalar_micro
cargo bench --bench adversarial_transcendentals
```

The generated benchmark summary is in [`benchmarks.md`](./benchmarks.md).
Hand-maintained profiling anchors and regression goals for the `Rational`,
`Real`, and `Computable` paths are in [`PERFORMANCE.md`](./PERFORMANCE.md).

Run dispatch tracing separately:

```sh
cargo bench --bench dispatch_trace --features dispatch-trace
```

The generated trace summary is in [`dispatch_trace.md`](./dispatch_trace.md).

## Development

Common checks:

```sh
cargo fmt --check
cargo test
cargo bench --bench numerical_micro
```

When adding a shortcut, add a focused correctness test and a benchmark row for
the smallest affected surface. Keep the shortcut only if it improves the target
without regressing broader `realistic_blas` or `liminal` benchmarks.

## License

(C) https://github.com/timschmidt Apache-2.0 / MIT

(C) https://github.com/tialaramex/realistic/ Apache-2.0

(C) https://github.com/siefkenj Apache-2.0
