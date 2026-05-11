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
- `Simple`: a small Lisp-like expression parser, enabled by the optional
  `simple` feature.

## Source Documentation

The crate-level README is the public orientation. More detailed implementation
notes live next to the source:

- [`src/README.md`](./src/README.md): project layout, numerical expectations,
  error model, tracing/benchmark expectations, and development constraints.
- [`src/rational/README.md`](./src/rational/README.md): `Rational` storage,
  reduction rules, conversion behavior, parser expectations, and exact
  arithmetic fast paths.
- [`src/real/README.md`](./src/real/README.md): `Real` representation,
  symbolic classes, structural facts, API expectations, and an ASCII diagram of
  the pieces stored in a `Real`.
- [`src/computable/README.md`](./src/computable/README.md): `Computable`
  expression graphs, lazy approximation, caches, precision expectations, and
  kernel organization.

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

With the `Simple` parser and calculator binary:

```toml
[dependencies]
hyperreal = { version = "0.10.6", features = ["simple"] }
```

Feature flags:

| Feature | Default | Purpose |
| --- | --- | --- |
| `simple` | no | Enables `Simple` and the package calculator binary. |

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

Requires the `simple` feature.

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

Finite decimal and fraction strings parse losslessly through `Rational`; the
parser also accepts leading `+` signs and digit separators where the rational
parser supports them. Scientific notation is not a supported exact text format.
`-0.0` imports as exact rational zero, so IEEE signed-zero information is not
preserved.

`PartialEq` on `Real` is structural, not a full computer-algebra equality
proof. Two expressions may print/debug similarly and approximate identically
while still comparing unequal if they were built through different computable
expression histories. Use structural facts, exact-rational extraction, or
explicit approximation/refinement when semantic equivalence rather than
representation identity is the question.

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

## Provenance and Acknowledgements

`hyperreal` descends from the
[`realistic`](https://github.com/tialaramex/realistic/) project and continues
that project's interest in practical computable real arithmetic while extending
the scalar model with exact rational structure, symbolic reductions, structural
facts, and benchmark-driven integration with `realistic_blas` and `liminal`.

Special thanks to [siefkenj](https://github.com/siefkenj), whose contributions
are part of the project's provenance and license history.

## References

These are the papers and books cited by source comments in this crate. They are
listed here in MLA style so implementation notes can point to one citeable
reference list.

- Bareiss, Erwin H. "[Sylvester's Identity and Multistep Integer-Preserving
  Gaussian Elimination](https://www.ams.org/mcom/1968-22-103/S0025-5718-1968-0226829-0/)."
  *Mathematics of Computation*, vol. 22, no. 103, 1968, pp. 565-578.
  American Mathematical Society, https://doi.org/10.1090/S0025-5718-1968-0226829-0.
- Boehm, Hans-Juergen, Robert Cartwright, Mark Riggle, and Michael J.
  O'Donnell. "[Exact Real Arithmetic: A Case Study in Higher Order
  Programming](https://doi.org/10.1145/319838.319860)." *Proceedings of the
  1986 ACM Conference on LISP and Functional Programming*, ACM, 1986,
  pp. 162-173.
- Boehm, Hans-J. "[Towards an API for the Real
  Numbers](https://doi.org/10.1145/3385412.3386037)." *Proceedings of the
  41st ACM SIGPLAN International Conference on Programming Language Design and
  Implementation*, ACM, 2020, pp. 562-576.
- Brent, Richard P. "[Fast Multiple-Precision Evaluation of Elementary
  Functions](https://doi.org/10.1145/321941.321944)." *Journal of the ACM*,
  vol. 23, no. 2, 1976, pp. 242-251.
- Brent, Richard P., and Paul Zimmermann.
  "[Modern Computer Arithmetic](https://doi.org/10.1017/CBO9780511921698)."
  Cambridge University Press, 2010.
- Middeke, Johannes, David J. Jeffrey, and Christoph Koutschan.
  "[Common Factors in Fraction-Free Matrix
  Decompositions](https://doi.org/10.1007/s11786-020-00495-9)."
  *Mathematics in Computer Science*, vol. 15, 2021, pp. 589-608.
- Payne, Mary H., and Robert N. Hanek.
  "[Radian Reduction for Trigonometric
  Functions](https://doi.org/10.1145/1057600.1057602)." *ACM SIGNUM
  Newsletter*, vol. 18, no. 1, 1983, pp. 19-24.
- Shewchuk, Jonathan Richard. "[Adaptive Precision Floating-Point Arithmetic
  and Fast Robust Geometric
  Predicates](https://doi.org/10.1007/PL00009321)." *Discrete &
  Computational Geometry*, vol. 18, no. 3, 1997, pp. 305-363.
- Smith, Luke, and Joan Powell. "[An Alternative Method to Gauss-Jordan
  Elimination: Minimizing Fraction
  Arithmetic](https://doi.org/10.63301/tme.v20i2.1957)." *The Mathematics
  Educator*, vol. 20, no. 2, 2011, pp. 44-50.
- Yap, Chee-Keng. "[Towards Exact Geometric
  Computation](https://doi.org/10.1016/0925-7721(95)00040-2)." *Computational
  Geometry*, vol. 7, nos. 1-2, 1997, pp. 3-23.

## License

(C) https://github.com/timschmidt Apache-2.0 / MIT

(C) https://github.com/tialaramex/realistic/ Apache-2.0

(C) https://github.com/siefkenj Apache-2.0
