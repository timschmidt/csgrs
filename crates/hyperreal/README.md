# Hyperreal

Hyperreal is a Rust library for exact rational arithmetic and computable real arithmetic. It started from Hans Boehm's "Towards an API for the Real Numbers" model and has since grown into a more performance-focused Rust implementation with symbolic tracking, exact shortcuts, borrowed arithmetic support, and a benchmark suite for the hot numerical paths.

## What it provides

- `Rational`
  - arbitrary-precision rational values
  - exact arithmetic
  - conversions to and from integers and IEEE-754 floats
- `Computable`
  - lazy real-number evaluation to requested precision
  - transcendental functions such as `exp`, `ln`, `sqrt`, `sin`, `cos`, and `tan`
  - caching, structural simplification, and targeted argument reduction
- `Real`
  - a higher-level real type that combines exact rational structure with computable irrational parts
  - symbolic handling for common classes such as square roots, logarithms, exponentials, and rational multiples of `pi`
- `Simple`
  - a small Lisp-like expression parser and evaluator for interactive use

## Current state

The project is no longer just a straight Java port. The current codebase includes:

- direct and benchmarked transcendental fast paths
- exact and symbolic trig/log shortcuts
- borrowed `Rational` and `Real` arithmetic APIs
- Criterion benchmark suites for:
  - library-level behavior
  - numerical kernels
  - borrowed-vs-owned arithmetic
  - float conversion
- ongoing evaluator work to separate:
  - exact public semantics
  - planning-only sign / MSD facts used for internal scheduling

The evaluator refactor plan lives in [`evaluator-refactor.md`](./evaluator-refactor.md).

## Installation

```toml
[dependencies]
hyperreal = "0.9.1"
```

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

### Simple expressions

```rust
use hyperreal::Simple;

let expr: Simple = "(* (+ pi pi) (sin (/ 1 5)))".parse().unwrap();
let value = expr.evaluate(&Default::default()).unwrap();

let _: f64 = value.into();
```

## Simple expression language

`Simple` uses a Lisp-like syntax:

- arithmetic: `+`, `-`, `*`, `/`
- roots and powers: `sqrt`, `pow`, `^`
- logs and exponentials: `ln`, `log10`, `exp`, `e`
- trig: `sin`, `cos`, `tan`

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

Float conversions are fallible on `NaN` and infinities.

## Serialization

The crate includes `serde` support. `Computable` serializes its expression structure, but not transient runtime state such as approximation caches or abort signals.

## Performance

Performance is now an explicit project goal.

Current work in the tree includes:

- specialized transcendental kernels
- faster large-argument reduction for trig and `exp`
- exact rational and symbolic shortcuts
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
- Public APIs are being tightened so exact facts and planner-only facts stay separate; this matters for both correctness and WASM stack-safety work.
