<h1>
  hyperreal
  <img src="./doc/hyper.png" alt="Hyper, a clever mathematician" width="144" align="right">
</h1>

`hyperreal` provides exact rational arithmetic, symbolic real values, and lazy
computable real approximation.

It is useful when code needs more information than an `f64` can provide:
structural sign facts, exact zero/nonzero knowledge, exact rational access,
bounded sign refinement, and recognizable forms such as `pi`, `e`, square
roots, logarithms, and rational trig constants.

## Numeric Model

`hyperreal` is built around three layers that deliberately keep exact and
symbolic information available before approximation:

- [`Rational`](src/rational/README.md): is the exact arithmetic base. It stores arbitrary-precision
  numerator/denominator values and performs exact reduction, dyadic detection,
  square extraction, shared-denominator dot products, and exact IEEE-754 import.
- [`Computable`](src/computable/README.md): is the lazy approximation layer. It represents exact-real
  expression graphs such as sums, products, inverses, roots, logs, trig kernels,
  and shared constants. It approximates only when a caller asks for a binary
  precision, then caches the result and conservative sign/magnitude facts.
- [`Real`](src/real/README.md): is the public symbolic scalar. It stores an exact rational scale plus a
  compact symbolic class and, when needed, a `Computable` certificate. Common
  classes include exact one, powers/products of `pi` and `e`, selected square
  roots, logarithms, trig forms, and factored constant products.
- `RealStructuralFacts`: conservative public facts about sign, zero status,
  magnitude, and exact-rational state.
- `Simple`: a small Lisp-like expression parser, enabled by the optional
  `simple` feature.

## Relationship to Other Crates

- `hyperlattice` uses `hyperreal::Real` as its default exact/symbolic scalar
  backend. It forwards `hyperreal` structural facts through its `Scalar` type
  and adds vector, matrix, transform, and retained-geometry facts around them.
- `liminal` can consume `hyperreal::Real` directly, using structural facts,
  finite `f64` approximations, and bounded sign refinement before robust
  fallback.
- `hypersolve` is the experimental solver layer. Its current direction is to
  evaluate constraints through symbolic references to variables, reuse
  reductions across iterations, and route repeated residual and geometry
  kernels through `hyperreal` and `hyperlattice` instead of rebuilding scalar
  expressions from scratch.

`hyperreal` owns scalar representation and approximation. It does not own vector
or matrix algebra, and it does not decide geometry topology. The stack is
layered intentionally: scalar facts live here, object-level facts live in
`hyperlattice`/geometry layers, and decision procedures live above them.

## Why Exact Reals?

Most numerical programs live between two useful but incomplete models:
integers and floating-point numbers.

Integers are exact and composable. Addition, multiplication, equality, and
ordering have clear mathematical meaning, and arbitrary-precision integer
libraries can grow as needed. But integer arithmetic cannot directly represent
division, roots, `pi`, logarithms, rotations, or most geometric coordinates
without adding another representation around it.

Rationals extend integers with exact division. They can represent values such
as `1/10`, imported finite floats, and many determinant or dot-product results
without rounding. Their cost is canonicalization: numerators and denominators
grow, greatest-common-divisor reduction is not free, and naive repeated
arithmetic can spend most of its time reducing intermediate fractions that
later cancel.

Floats solve a different problem. They are fixed-size, fast, cache-friendly,
and supported directly by hardware. They are excellent for simulation,
graphics, statistics, and many approximation tasks. Their limitation is that
they approximate almost every real value, and that approximation is part of the
value. `0.1` is rounded, algebraic identities can fail after cancellation,
near-zero signs can be artifacts of previous operations, and equality answers a
machine-representation question rather than a mathematical one. In geometric
or constraint code, one wrong sign can change topology: a point can move to the
wrong side of a line, an intersection can appear or disappear, or a solver can
choose the wrong branch.

`hyperreal` takes a third route. It keeps exact and symbolic structure alive
for as long as it is useful, then approximates only when a caller asks for a
precision or a decision cannot be answered structurally. Exact rationals remain
rationals. Dyadic values retain cheap denominator structure. Common constants
and forms such as `pi`, `e`, selected roots, logarithms, and trig constants
carry symbolic classes. Computable expression graphs provide lazy
approximation when symbolic structure is no longer enough.

This does not make real arithmetic free, and it is not a full computer algebra
system. Some equality questions remain undecidable without more context, and
some expressions eventually require refinement. The difference is that callers
can ask better questions before rounding: known zero or nonzero, structural
sign, exact-rational access, conservative magnitude, or bounded sign
refinement. That is the niche this stack targets.

## Current State

The crate is active, benchmark-driven, and no longer just a direct port of
computable real ideas. Current implementation work includes:

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
- cached approximation and structural-fact propagation through computable nodes
- borrowed arithmetic paths for `Rational` and `Real`
- shared-denominator and signed-product-sum hooks used by matrix/vector callers
  to delay rational canonicalization
- `serde` support for expression structure, excluding transient caches and abort
  signals
- dispatch tracing and targeted benchmark suites for scalar, approximation,
  symbolic, adversarial, and stack-facing regressions
- source-level READMEs for `Rational`, `Real`, and `Computable`, plus
  `structural_facts.txt` for planned and implemented fact propagation

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

`Rational` is the exact base layer. It is useful for imported measurements,
test fixtures, small coefficients, and any value that should not become binary
floating-point noise before the rest of the stack sees it.

```rust
use hyperreal::Rational;
use std::convert::TryFrom;

let a = Rational::fraction(7, 8).unwrap();
let b = Rational::fraction(9, 10).unwrap();

assert_eq!(a + b, Rational::fraction(79, 40).unwrap());

// Finite floats import by exact IEEE-754 decoding, not by decimal rounding.
let half = Rational::try_from(0.5_f64).unwrap();
assert_eq!(half, Rational::fraction(1, 2).unwrap());

// Decimal and fraction strings parse into exact rationals.
let decimal: Rational = "12.125".parse().unwrap();
let fraction: Rational = "97/8".parse().unwrap();
assert_eq!(decimal, fraction);
```

### Symbolic Reals

`Real` keeps a rational scale plus a symbolic/computable class. That lets common
forms simplify or expose facts before approximation is needed.

```rust
use hyperreal::{Rational, Real, RealSign, ZeroKnowledge};

let x = Real::new(Rational::new(2)).sqrt().unwrap();
let y = Real::new(Rational::new(3)).sqrt().unwrap();
let z = x * y;

let approx: f64 = z.into();
assert!(approx > 2.44 && approx < 2.45);

let half = Real::new(Rational::fraction(1, 2).unwrap());
let angle = half * Real::pi();
let cosine = angle.cos().unwrap();

// Recognizable symbolic/trig forms can answer facts without a full equality
// proof or high-precision decimal expansion.
let facts = cosine.structural_facts();
assert_eq!(facts.zero, ZeroKnowledge::Zero);
assert_eq!(cosine.refine_sign_until(-32), Some(RealSign::Zero));
```

### Computable Approximation

`Computable` is the lazy approximation layer. It stores an expression graph and
only computes a scaled integer approximation when a precision is requested.

```rust
use hyperreal::{Computable, Rational, RealSign};

let x = Computable::rational(Rational::fraction(7, 5).unwrap()).sin();
let scaled = x.approx(-40);

assert_ne!(scaled, 0.into());

// Sign refinement asks for only enough precision to decide the sign down to a
// requested floor. The result may be `None` for unresolved or truly difficult
// cases, so callers can decide whether to refine further or use a fallback.
let near_pi = Computable::pi().add(Computable::rational(
    Rational::fraction(-22, 7).unwrap(),
));
assert_eq!(near_pi.sign_until(-8), Some(RealSign::Positive));
```

### Structural Facts

Structural facts are conservative certificates. They are designed for filters,
predicates, and higher-level kernels that want to avoid approximation unless a
decision actually requires it.

```rust
use hyperreal::{Rational, Real, RealSign, ZeroKnowledge};

let value = Real::new(Rational::new(2)).sqrt().unwrap();
let facts = value.structural_facts();

assert_eq!(facts.sign, Some(RealSign::Positive));
assert_eq!(facts.zero, ZeroKnowledge::NonZero);
assert!(!facts.exact_rational);
assert_eq!(value.refine_sign_until(-64), Some(RealSign::Positive));

let exact = Real::new(Rational::fraction(9, 18).unwrap());
let exact_facts = exact.structural_facts();

assert_eq!(exact.exact_rational(), Some(Rational::fraction(1, 2).unwrap()));
assert_eq!(exact_facts.sign, Some(RealSign::Positive));
assert_eq!(exact_facts.zero, ZeroKnowledge::NonZero);
assert!(exact_facts.exact_rational);
```

Facts are conservative. Missing sign or magnitude information means the fact
was not proven cheaply.

### Stack-Facing Filters

The surrounding geometry stack uses `hyperreal` values as scalar certificates:
try cheap structural facts first, use finite approximation when that is enough,
and only then request bounded refinement.

```rust
use hyperreal::{Rational, Real, RealSign};

fn classify_positive(value: &Real) -> Option<bool> {
    if let Some(sign) = value.structural_facts().sign {
        return Some(sign == RealSign::Positive);
    }

    if let Some(approx) = value.to_f64_approx() {
        if approx > 1e-12 {
            return Some(true);
        }
        if approx < -1e-12 {
            return Some(false);
        }
    }

    value
        .refine_sign_until(-80)
        .map(|sign| sign == RealSign::Positive)
}

let offset = Real::pi() - Real::new(Rational::fraction(22, 7).unwrap());
assert_eq!(classify_positive(&offset), Some(true));
```

This pattern is the intended handoff to `hyperlattice` and `liminal`: cheap
facts route the common case, approximation is delayed until useful, and bounded
refinement remains available for hard predicate boundaries.

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

It can also be useful for small configuration- or test-facing formulas:

```rust
use hyperreal::{Rational, Simple};

let expr: Simple = "(sqrt (/ 49 64))".parse().unwrap();
let value = expr.evaluate(&Default::default()).unwrap();

assert_eq!(value.exact_rational(), Some(Rational::fraction(7, 8).unwrap()));
```

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

The implementation strategy is intentionally thin: preserve facts that are
already known, reduce before approximating, and keep expensive scalar queries
out of hot algebra loops. Performance shortcuts are documented next to the code
that uses them, but most of them follow the same themes:

- Preserve facts at the layer that discovered them. `hyperreal` keeps exact
  rational, dyadic, symbolic, sign, zero, magnitude, and approximation-cache
  facts; `hyperlattice` and higher layers keep object facts such as affine,
  diagonal, triangular, point/direction, retained transform, known coordinate
  zero, and shared determinant/cofactor structure.
- Use exact and symbolic reductions before generic approximation. Exact
  rationals, dyadics, named constants, roots, logarithms, selected trig forms,
  identity values, endpoint cases, tiny-argument transforms, and reduced
  trig/exponential arguments should simplify or classify before entering
  broader computable kernels.
- Approximate only when a decision or output precision requires it. Cache the
  resulting approximation plus conservative sign or magnitude facts, and answer
  later structural queries from certificates before refining again.
- Keep hot kernels predictable. Prefer deterministic fast paths guarded by
  cheap retained facts, borrowed arithmetic, shared-denominator/product-sum
  reducers, and cached constants over speculative scalar probing or expression
  graph cloning inside dense loops.
- Split backend-sensitive loop shapes narrowly. Compact approximate backends
  should keep flat interval-friendly routes when possible, while exact
  hyperreal/hyperreal-rational paths may use capability-gated reducers or
  symbolic shortcuts when benchmarks show a real win.
- Benchmark families, not isolated rows. A shortcut should improve the target
  surface without making nearby functions or stack-facing `hyperlattice` and
  `liminal` paths erratic.

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
without regressing broader `hyperlattice` or `liminal` benchmarks.

## Provenance and Acknowledgements

`hyperreal` descends from the
[`realistic`](https://github.com/tialaramex/realistic/) project and continues
that project's interest in practical computable real arithmetic.

Special thanks to [siefkenj](https://github.com/siefkenj), whose contributions
improved realistic and hyperreal.

## References

These are the papers and books which contribute ideas or methods to this crate.
They are listed here in MLA style for easy citation.

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
