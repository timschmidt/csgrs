# Hyperreal

Hyperreal is a Rust library for exact rational arithmetic and computable real
arithmetic. It started from Hans Boehm's "Towards an API for the Real Numbers"
model and has since grown into a more performance-focused Rust implementation
with symbolic tracking, exact shortcuts, structural inspection APIs, borrowed
arithmetic support, and a benchmark suite for the hot numerical paths.

`Hyperreal` is the independently useful scalar foundation of a linear algebra and geometry stack:

- `realistic_blas` uses `hyperreal::Real` as its default exact/symbolic scalar
  backend.
- `predicated` can consume `hyperreal::Real` directly as a predicate scalar,
  or indirectly through `realistic_blas::Scalar`.
- The public handoff surface is intentionally conservative: downstream crates
  use `structural_facts`, `exact_rational`, `to_f64_approx`,
  `refine_sign_until`, and `sign_until` instead of depending on private
  evaluator internals.

## What it provides

- `Rational`
  - arbitrary-precision rational values
  - exact arithmetic
  - conversions to and from integers and IEEE-754 floats
- `Computable`
  - lazy real-number evaluation to requested precision
  - transcendental functions such as `exp`, `ln`, `sqrt`, `sin`, `cos`, `tan`, and inverse trig/hyperbolic kernels
  - caching, structural simplification, and targeted argument reduction
  - conservative structural facts through `structural_facts`
  - bounded sign refinement through `sign_until`
- `Real`
  - a higher-level real type that combines exact rational structure with computable irrational parts
  - symbolic handling for common classes such as square roots, logarithms, exponentials, and rational multiples of `pi`
  - exact inverse trig shortcuts and inverse hyperbolic construction with domain checks
  - public exact-rational access through `exact_rational`
  - conservative sign, zero, exactness, and magnitude facts through `structural_facts`
  - borrowed finite `f64` approximation through `to_f64_approx`
- Structural fact types
  - `RealSign`
  - `ZeroKnowledge`
  - `MagnitudeBits`
  - `RealStructuralFacts`
- `Simple`
  - a small Lisp-like expression parser and evaluator for interactive use
  - enabled by the default `simple` Cargo feature

## Current state

The project is no longer just a straight Java port. The current codebase includes:

- direct and benchmarked transcendental fast paths
- exact and symbolic trig, inverse trig, log, exp, and inverse hyperbolic shortcuts
- exact special-form recognition for rational multiples of `pi`, including
  `sin(pi*r)`, `cos(pi*r)`, `tan(pi*r)`, and principal-branch inverse
  compositions where the structure is known
- borrowed `Rational` and `Real` arithmetic APIs
- public structural inspection for robust downstream filtering and predicates
- bounded sign refinement that stops at the requested precision floor
- owned exact-rational access that does not expose internal representation
- benchmark documentation generated into [`benchmarks.md`](./benchmarks.md), including current Criterion means and confidence intervals
- Criterion benchmark suites for:
  - library-level behavior
  - numerical kernels
  - borrowed-vs-owned arithmetic
  - float conversion
  - scalar structural and arithmetic microbenchmarks
- internal separation between public exact facts and planner-only evaluator facts

The current implementation is suitable as an exact/symbolic scalar backend for
experimentation, predicate filtering, and benchmark-driven numerical work. It
is not intended to compete with dense numeric BLAS libraries for large matrix
kernels; higher-level crates should treat it as a rich scalar type and choose
their own algebra and geometry policies.

## Installation

```toml
[dependencies]
hyperreal = "0.10.4"
```

To build only the numeric library without the `Simple` expression parser:

```toml
[dependencies]
hyperreal = { version = "0.10.4", default-features = false }
```

Cargo features:

- `simple` (default): builds and exports `Simple` and the package calculator binary.

## Examples

### Exact rationals

```rust
use hyperreal::Rational;

let a = Rational::fraction(7, 8).unwrap();
let b = Rational::fraction(9, 10).unwrap();
let c = a + b;

assert_eq!(c, Rational::fraction(79, 40).unwrap());
```

### Real arithmetic

```rust
use hyperreal::{Rational, Real};

let x = Real::new(Rational::new(2)).sqrt().unwrap();
let y = Real::new(Rational::new(3)).sqrt().unwrap();
let z = x * y;

let approx: f64 = z.into();
assert!(approx > 2.44 && approx < 2.45);
```

### Computable values

```rust
use hyperreal::{Computable, Rational};

let x = Computable::rational(Rational::fraction(7, 5).unwrap()).sin();
let approx = x.approx(-40);

assert_ne!(approx, 0.into());
```

### Structural inspection

```rust
use hyperreal::{Rational, Real, RealSign, ZeroKnowledge};

let value = Real::new(Rational::new(2)).sqrt().unwrap();
let facts = value.structural_facts();

assert_eq!(facts.sign, Some(RealSign::Positive));
assert_eq!(facts.zero, ZeroKnowledge::NonZero);
assert!(!facts.exact_rational);

let sign = value.refine_sign_until(-64);
assert_eq!(sign, Some(RealSign::Positive));
```

Structural facts are conservative. A missing sign or magnitude means the value was not proven by cheap structural inspection; it does not imply the fact is false. Bounded sign refinement may populate approximation caches, but it terminates at or before the requested precision floor.

### Simple expressions

```rust
use hyperreal::Simple;

let expr: Simple = "(* (+ pi pi) (sin (/ 1 5)))".parse().unwrap();
let value = expr.evaluate(&Default::default()).unwrap();

let _: f64 = value.into();
```

## Simple expression language

`Simple` is enabled by the default `simple` feature and uses a Lisp-like syntax:

- arithmetic: `+`, `-`, `*`, `/`
- roots and powers: `sqrt`, `pow`, `^`
- logs and exponentials: `ln`, `log10`, `exp`, `e`
- trig: `sin`, `cos`, `tan`, `asin`, `acos`, `atan`
- inverse hyperbolic: `asinh`, `acosh`, `atanh`

Examples:

```text
(+ 1 2 3 4)
(* (+ pi pi) (sin (/ 1 5)))
(pow (+ 3/2 4/7) 9/2)
(sqrt 9)
```

Numeric literals may be:

- integers: `42`
- decimals: `2.75`
- fractions: `11/7`

Built-in names include `pi` and `e`.

## Conversions

Hyperreal supports Rust conversion traits where they make sense:

- integer types -> `Rational` / `Real`
- `f32` / `f64` -> `Rational` / `Real` via exact IEEE-754 decoding
- `Real` -> `f32` / `f64` via nearest representable floating-point value
- `Real::to_f64_approx()` for a borrowed, finite-only approximation useful for filtering

Float conversions from IEEE-754 values are fallible on `NaN` and infinities. `Real::to_f64_approx()` returns `None` when no finite `f64` approximation can be produced, including overflow to infinity; values too small for `f64` may underflow to `Some(0.0)`.

## Serialization

The crate includes `serde` support. `Computable` serializes its expression structure, but not transient runtime state such as approximation caches or abort signals.

## Performance

Performance is now an explicit project goal.

Current work in the tree includes:

- specialized transcendental kernels
- faster large-argument reduction for trig, inverse trig, and `exp`
- exact rational, symbolic, and domain-error shortcuts
- stable inverse hyperbolic construction paths
- structural fact and bounded sign-refinement shortcuts, including very fast public zero-status checks for exact values
- borrowed arithmetic improvements, with separate benchmarks for exact, symbolic, scaled, and unscaled public paths
- allocation-free detection of power-of-two rational scales in scalar folding paths
- benchmark-guided evaluator and public-wrapper refactoring
- stack-level benchmark coverage through `realistic_blas` and `predicated`,
  which helps separate scalar construction costs from vector/matrix and
  predicate policy costs

Benchmark targets:

- Benchmark output reference with current Criterion values: [`benchmarks.md`](./benchmarks.md)
- `cargo bench --bench library_perf`
- `cargo bench --bench numerical_micro`
- `cargo bench --bench borrowed_ops`
- `cargo bench --bench float_convert`
- `cargo bench --bench scalar_micro`

The generated benchmark reference is useful for spotting which layer a slowdown belongs to. For example, `borrowed_ops` covers direct owned-vs-borrowed arithmetic, while `scalar_micro` separates exact structural queries, unscaled public `Real` addition, and scaled public `Real` addition.

### Performance Techniques

The implementation deliberately keeps a set of small symbolic and structural
shortcuts. These are not algebra-system ambitions; they are the cases that show
up in `hyperreal`, `realistic_blas`, and `predicated` benchmarks.

- Constants such as `pi`, `tau`, `e`, common square-root scales, and common
  logarithms are cached internally and cloned from thread-local storage.
- `Rational` has dyadic fast paths for IEEE-754 imports and power-of-two scale
  reduction, avoiding general gcd work in common `f64`-derived values.
- `Real` stores a rational scale plus a symbolic/computable class. Same-class
  addition and subtraction adjust only the rational scale.
- Exact symbolic classes are kept for `pi`, positive powers of `pi`, `1/pi`,
  `e^q/pi`, `pi*e^q`, signed `pi^n*e^q` products, square roots, `pi*sqrt(q)`,
  selected logs, log products, rational `sin(pi*q)`, and rational
  `tan(pi*q)`.
- Constant-product multiplication, division, and reciprocal paths stay inside
  the symbolic `pi^n*e^q` family when this avoids generic computable nodes.
  The lightweight `1/pi` and `e^q/pi` forms are kept separate from the boxed
  general product because scalar and matrix division by `pi` are hot enough to
  measure.
- `ln(e^x)` and `ln(a*e^x)` collapse to `x` or `ln(a)+x`; small integer-power
  logarithms reuse cached scaled-log constants.
- Exact trig and inverse-trig tables recognize small rational multiples of
  `pi`, `sqrt(2)/2`, `sqrt(3)/2`, `sqrt(3)`, and `sqrt(3)/3`.
- Rational and square-root domain checks for inverse trig and inverse
  hyperbolic functions run before generic computable construction.
- Tiny and endpoint inverse trig/hyperbolic cases use dedicated computable
  kernels or `ln1p`-style transforms to avoid slow generic formulas and
  cancellation.
- `sqrt`, integer powers, multiplication, and computable squaring peel exact
  rational scale factors and preserve square-root structure where benchmarks
  show that it helps.
- `Computable` carries cheap structural bounds, exact-sign caches, precision
  caches, and dominant-term sign shortcuts so sign, zero, and magnitude queries
  can often return without refinement.
- Trig and exponential computable kernels use argument-size knowledge,
  prescaled forms, and identity-based reductions for large and tiny arguments.
- Borrowed `Real` arithmetic exists so downstream vector, matrix, and predicate
  kernels can avoid cloning expression graphs.
- Abort-aware APIs attach the signal only to cloned computable values that may
  refine; cheap structural queries still try to decide before evaluating.

When adding another shortcut, add a focused test plus a Criterion row in the
smallest relevant bench. Keep it only when the targeted row improves without
regressing the broader `realistic_blas` or `predicated` paths.

## Notes

- Some computations are intentionally lazy and may run for a long time if you request difficult values at high precision.
- `Real::abort` can be used to attach an external stop signal to long-running evaluation.
- Public structural APIs expose conservative numeric facts without exposing private evaluator internals.
- Planner-only evaluator facts remain private; downstream crates should use `structural_facts`, `exact_rational`, `refine_sign_until`, and `sign_until`.

## Related crates

- `realistic_blas`: vector, matrix, and complex arithmetic over a crate-owned
  `Scalar` wrapper. Its default backend is `hyperreal`, and it forwards
  structural facts for predicate users.
- `predicated`: geometry-oriented predicates and classification helpers. It
  owns predicate escalation policy and can use `hyperreal` either directly or
  through `realistic_blas`.
