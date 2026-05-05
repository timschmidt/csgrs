# Hyperreal

Hyperreal is a Rust library for exact rational arithmetic and computable real arithmetic. It started from Hans Boehm's "Towards an API for the Real Numbers" model and has since grown into a more performance-focused Rust implementation with symbolic tracking, exact shortcuts, structural inspection APIs, borrowed arithmetic support, and a benchmark suite for the hot numerical paths.

## What it provides

- `Rational`
  - arbitrary-precision rational values
  - exact arithmetic
  - conversions to and from integers and IEEE-754 floats
- `Computable`
  - lazy real-number evaluation to requested precision
  - transcendental functions such as `exp`, `ln`, `sqrt`, `sin`, `cos`, and `tan`
  - caching, structural simplification, and targeted argument reduction
  - conservative structural facts through `structural_facts`
  - bounded sign refinement through `sign_until`
- `Real`
  - a higher-level real type that combines exact rational structure with computable irrational parts
  - symbolic handling for common classes such as square roots, logarithms, exponentials, and rational multiples of `pi`
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
- exact and symbolic trig/log shortcuts
- borrowed `Rational` and `Real` arithmetic APIs
- public structural inspection for robust downstream filtering and predicates
- bounded sign refinement that stops at the requested precision floor
- owned exact-rational access that does not expose internal representation
- Criterion benchmark suites for:
  - library-level behavior
  - numerical kernels
  - borrowed-vs-owned arithmetic
  - float conversion
- internal separation between public exact facts and planner-only evaluator facts

The evaluator refactor plan lives in [`evaluator-refactor.md`](./evaluator-refactor.md).

## Installation

```toml
[dependencies]
hyperreal = "0.10.1"
```

To build only the numeric library without the `Simple` expression parser:

```toml
[dependencies]
hyperreal = { version = "0.10.1", default-features = false }
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
- faster large-argument reduction for trig and `exp`
- exact rational and symbolic shortcuts
- structural fact and bounded sign-refinement shortcuts
- borrowed arithmetic improvements
- benchmark-guided evaluator refactoring

Benchmark targets:

- `cargo bench --bench library_perf`
- `cargo bench --bench numerical_micro`
- `cargo bench --bench borrowed_ops`
- `cargo bench --bench float_convert`

## Notes

- Some computations are intentionally lazy and may run for a long time if you request difficult values at high precision.
- `Real::abort` can be used to attach an external stop signal to long-running evaluation.
- Public structural APIs expose conservative numeric facts without exposing private evaluator internals.
- Planner-only evaluator facts remain private; downstream crates should use `structural_facts`, `exact_rational`, `refine_sign_until`, and `sign_until`.
