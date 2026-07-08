<h1>
  hyperreal
  <img src="./doc/hyper.png" alt="Hyper, a clever mathematician" width="144" align="right">
</h1>

`hyperreal` provides exact rational arithmetic, symbolic real values, lazy computable
real approximation, and conservative structural facts for the Hyper ecosystem. It is
the scalar layer used by the surrounding exact-aware geometry, solver, physics, circuit,
routing, packing, voxel, and design-readiness crates.

The crate is not a full computer algebra system, and it does not try to canonicalize
every real expression eagerly. Its job is narrower: preserve enough exact, symbolic,
and refinement-ready structure that higher layers can make certified decisions or
report explicit uncertainty without quietly falling back to primitive floating point.

## Hyper Ecosystem

`hyperreal` is the scalar substrate. The rest of the Hyper stack should keep its own
object-level facts, but use `hyperreal` values and certificates when scalar exactness
matters.

Core layers:

- [hyperreal](https://github.com/timschmidt/hyperreal): exact rational, symbolic, and
  computable real arithmetic.
- [hyperlattice](https://github.com/timschmidt/hyperlattice): small exact vector,
  matrix, transform, and shared-scale algebra.
- [hyperlimit](https://github.com/timschmidt/hyperlimit): exact predicate policy,
  escalation, and result provenance.

Geometry and solver layers:

- [hypercurve](https://github.com/timschmidt/hypercurve): planar curves, contours,
  regions, offsets, and boolean-boundary work.
- [hypertri](https://github.com/timschmidt/hypertri): exact polygon triangulation,
  Delaunay, and constrained Delaunay topology.
- [hypermesh](https://github.com/timschmidt/hypermesh): 3D mesh validation, topology,
  and exact-aware boolean preflight.
- [hyperbrep](https://github.com/timschmidt/hyperbrep): retained BREP topology,
  planar surfaces, trim evidence, tessellation manifests, and mesh handoff reports.
- [hypersdf](https://github.com/timschmidt/hypersdf): signed-distance and implicit-field
  carriers with exact-aware sampling, classification, solver, mesh, and voxel handoffs.
- [hypersolve](https://github.com/timschmidt/hypersolve): symbolic residuals,
  solver preparation, and candidate certification.
- [csgrs](https://github.com/timschmidt/csgrs/tree/hyperreal): Multi-modal CAD kernel, owns CSG

Domain and proposal layers:

- [hyperpath](https://github.com/timschmidt/hyperpath): routing, toolpath, tangent,
  clearance, and path-provenance carriers.
- [hyperdrc](https://github.com/timschmidt/hyperdrc): PCB design-readiness checks and
  manufacturing package evidence.
- [hyperphysics](https://github.com/timschmidt/hyperphysics): exact-aware materials,
  mass properties, contact, field, and simulation handoff reports.
- [hypercircuit](https://github.com/timschmidt/hypercircuit): circuit MNA carriers,
  residual replay, and coupled electrothermal reports.
- [hyperparts](https://github.com/timschmidt/hyperparts): source-attributed part,
  interface, process, and compatibility facts.
- [hyperpack](https://github.com/timschmidt/hyperpack): exact-aware packing models and
  feasibility replay.
- [hypervoxel](https://github.com/timschmidt/hypervoxel): exact-aware voxel grid
  frames, sparse-grid facts, and adapter manifests.
- [hyperevolution](https://github.com/timschmidt/hyperevolution): exact-aware search,
  fitness, archive, and replay-policy carriers.

## Typical Real-Number Problems

Most numerical programs choose between fixed-size floats, full rational arithmetic,
symbolic algebra, or local epsilon rules.

Floats are fast and hardware-friendly, but they round nearly every nontrivial value.
That approximation becomes part of the value: `0.1` is not exactly one tenth, cancellation
can manufacture near-zero signs, and equality answers a machine-representation question
rather than a mathematical one. In geometry, routing, and constraint code, one wrong sign
can move a point to the wrong side of a line, change a triangulation, or accept the wrong
solver branch.

Rationals give exact division, exact finite-float import, and exact determinant or
dot-product results, but numerator and denominator growth can dominate runtime if every
intermediate is fully canonicalized. Symbolic representations preserve more meaning, but
they are incomplete unless the program becomes a full algebra system.

`hyperreal` takes the middle path used by the Hyper stack: keep exact and symbolic
structure alive, expose cheap conservative facts, refine only when a caller asks for a
decision, and make lossy primitive-float export a named edge API rather than a hidden
fallback.

## Main Types

- [`Rational`](src/rational/README.md) is the exact arithmetic base. It stores
  arbitrary-precision numerator/denominator values and supports exact reduction,
  dyadic detection, square extraction, shared-denominator product sums, decimal/fraction
  parsing, and exact finite IEEE-754 import.
- [`Real`](src/real/README.md) is the public scalar for the stack. It combines an exact
  rational scale with a compact symbolic/computable class so common values such as
  exact ones, powers/products of `pi` and `e`, selected roots, logarithms, trig forms,
  and removable small-angle limits can expose facts before approximation.
- [`Computable`](src/computable/README.md) is the lazy approximation layer. It stores an
  exact-real expression graph and computes scaled integer approximations only at a
  requested binary precision.
- `RealStructuralFacts`, `RealDetailedFacts`, `CertifiedRealSign`,
  `CertifiedRealOrdering`, and `CertifiedRealEquality` are conservative certificates
  for sign, zero status, magnitude, rational state, primitive-float status, domain
  status, and comparison outcomes.
- `RealExactSetFacts` records useful exact-set summaries such as denominator class,
  dyadic exponent class, and sign pattern for higher-level reducers.
- `Simple`, enabled by the `simple` feature, is a small Lisp-like expression parser and
  calculator surface for tests, examples, and configuration-facing formulas.
- `Problem` is the crate-local error type used by fallible scalar operations.

## Precision Model

`hyperreal` treats precision as an API contract, not an implementation accident.

- Finite `f32` and `f64` imports decode the exact IEEE-754 value. They do not reinterpret
  a binary float as a decimal measurement.
- Decimal and fraction strings parse through `Rational`, so text inputs such as
  `12.125` and `97/8` remain exact.
- `Real::structural_facts()` exposes conservative sign, zero, nonzero, magnitude,
  exact-rational, symbolic, primitive-float, and domain facts without requiring a full
  equality proof.
- `refine_sign_until`, `sign_until`, and related certification APIs request bounded
  refinement only when a caller needs more precision.
- `PartialEq` on `Real` is structural. It is not a complete algebraic equality proof.
  Use exact-rational extraction, structural certificates, or explicit refinement when
  semantic equality is the question.
- `Real::to_f32_lossy()` and `Real::to_f64_lossy()` are named edge exports for rendering,
  IO, diagnostics, and third-party interop. They are not predicate, ordering, equality,
  or topology decisions.

In Yap's exact geometric computation sense, exactness here means preserving enough
certified structure to decide later predicates exactly or report uncertainty. It does
not mean expanding every scalar expression to its largest possible canonical form.

## Numerical Explosion

`hyperreal` combats numerical explosion by preserving factored structure and asking for
bounded refinement only at decision boundaries. Dyadic schedules, shared-denominator
reducers, cached constants, computable approximation caches, exact-set facts, and
lossy export labels keep large rationals and symbolic graphs from becoming the default
representation for every operation.

## Performance Model

Exact arithmetic is only useful in a systems stack if it is measured and kept under
control. `hyperreal` uses several small performance strategies together:

- Preserve factored structure. Rationals, dyadics, constants, roots, logarithms, and
  selected trig forms stay classified so later reducers can avoid expanding large
  numerators, denominators, or expression graphs.
- Normalize at useful boundaries. Dyadic denominators reduce by shifts, product sums can
  share denominators, and matrix/vector callers can delay full rational canonicalization
  until accumulated terms have had a chance to combine.
- Remove trivial work early. Canonical zeros, ones, identity constructors, all-zero sums,
  exact endpoints, and known domain facts avoid constructing larger expressions.
- Prefer structural facts before scalar probing. Many hot branches need sign, zero,
  nonzero, magnitude, dyadic, or exact-rational facts rather than a fresh approximation.
- Approximate at requested precision. `Computable` nodes use argument reduction,
  prescaled kernels, cancellation-aware transforms, shared constants, and precision-aware
  caches so refinement grows with caller demand.
- Keep hot kernels predictable. Borrowed arithmetic, shared-denominator/product-sum
  reducers, cached constants, and capability-gated symbolic shortcuts are preferred over
  speculative approximation in dense loops.
- Measure regressions directly. Dispatch tracing and benchmark families track GCDs,
  rational temporaries, peak operand sizes, repeated approximation, exact reducer use,
  cache pressure, and stack-facing behavior for `hyperlattice` and `hyperlimit`.

## Current Status

Version `0.13.1` is active and benchmark-driven. Current implementation work includes:

- exact rational and dyadic fast paths;
- dedicated constructors and cached constants for common zeros, ones, `pi`, `tau`, `e`,
  common square roots, and common logarithms;
- symbolic classes for selected `pi`, `e`, `sqrt`, `ln`, `sin(pi*q)`, and `tan(pi*q)`
  forms;
- exact trig, inverse-trig, logarithm, exponential, and inverse-hyperbolic shortcuts
  where the input structure is recognizable;
- argument reduction and prescaled kernels for transcendental approximation;
- structural sign, zero, nonzero, magnitude, exact-rational, exact-set, and domain
  queries;
- bounded sign refinement and certified equality/ordering/sign reports;
- cached approximation and structural-fact propagation through computable nodes;
- borrowed arithmetic paths for `Rational` and `Real`;
- shared-denominator and signed-product-sum hooks used by matrix/vector callers;
- `serde` support for expression structure, excluding transient caches and abort
  signals;
- dispatch tracing and targeted benchmark suites for scalar, approximation, symbolic,
  adversarial, and stack-facing regressions;
- source-level READMEs for `Rational`, `Real`, and `Computable`, plus
  `structural_facts.txt` for planned and implemented fact propagation.

Known limits: some equality questions remain undecidable without more context, and some
symbolic expressions eventually require refinement. Higher crates should preserve
object-level facts rather than asking the scalar layer to infer geometry or topology.

## Installation

```toml
[dependencies]
hyperreal = "0.13.1"
```

With the `Simple` parser and calculator binary:

```toml
[dependencies]
hyperreal = { version = "0.13.1", features = ["simple"] }
```

Feature flags:

| Feature | Default | Purpose |
| --- | --- | --- |
| `simple` | no | Enables `Simple` and the package calculator binary. |
| `cached-f32-approx` | no | Caches selected `f32` approximation paths. |
| `cached-f64-approx` | no | Caches selected `f64` approximation paths. |
| `dispatch-trace` | no | Records scalar dispatch and rational-growth counters. |
| `serde` | no | Enables JSON/CBOR conversion APIs and serializable expression types. |

## Examples

### Exact Rationals

`Rational` is useful for measurements, fixtures, coefficients, and imported finite
values that must not become binary floating-point noise before the rest of the stack
sees them.

```rust
use hyperreal::Rational;
use std::convert::TryFrom;

let a = Rational::fraction(7, 8).unwrap();
let b = Rational::fraction(9, 10).unwrap();

assert_eq!(a + b, Rational::fraction(71, 40).unwrap());

let half = Rational::try_from(0.5_f64).unwrap();
assert_eq!(half, Rational::fraction(1, 2).unwrap());

let decimal: Rational = "12.125".parse().unwrap();
let fraction: Rational = "97/8".parse().unwrap();
assert_eq!(decimal, fraction);
```

### Symbolic Reals And Facts

`Real` keeps a rational scale plus symbolic or computable structure. Recognizable forms
can expose facts before a caller asks for a high-precision approximation.

```rust
use hyperreal::{Rational, Real, RealSign, ZeroKnowledge};

let x = Real::new(Rational::new(2)).sqrt().unwrap();
let y = Real::new(Rational::new(3)).sqrt().unwrap();
let z = x * y;

let approx = z.to_f64_lossy().unwrap();
assert!(approx > 2.44 && approx < 2.45);

let half = Real::new(Rational::fraction(1, 2).unwrap());
let cosine = (half * Real::pi()).cos();
let facts = cosine.structural_facts();

assert_eq!(facts.zero, ZeroKnowledge::Zero);
assert_eq!(cosine.refine_sign_until(-32), Some(RealSign::Zero));
```

### Computable Approximation

`Computable` stores an expression graph and only computes a scaled integer
approximation when a precision is requested.

```rust
use hyperreal::{Computable, Rational, RealSign};

let x = Computable::rational(Rational::fraction(7, 5).unwrap()).sin();
let scaled = x.approx(-40);
assert_ne!(scaled, 0.into());

let near_pi = Computable::pi().add(Computable::rational(
    Rational::fraction(-22, 7).unwrap(),
));
assert_eq!(near_pi.sign_until(-8), Some(RealSign::Positive));
```

### Stack-Facing Decisions

Higher crates should ask for cheap facts first, then request bounded refinement only
when a decision needs a sign.

```rust
use hyperreal::{Rational, Real, RealSign};

fn classify_positive(value: &Real) -> Option<bool> {
    if let Some(sign) = value.structural_facts().sign {
        return Some(sign == RealSign::Positive);
    }

    value
        .refine_sign_until(-80)
        .map(|sign| sign == RealSign::Positive)
}

let offset = Real::pi() - Real::new(Rational::fraction(22, 7).unwrap());
assert_eq!(classify_positive(&offset), Some(true));
```

### Simple Expressions

Requires the `simple` feature.

```rust
use hyperreal::{Rational, Simple};

let expr: Simple = "(sqrt (/ 49 64))".parse().unwrap();
let value = expr.evaluate(&Default::default()).unwrap();

assert_eq!(value.exact_rational(), Some(Rational::fraction(7, 8).unwrap()));
```

`Simple` supports arithmetic, roots, powers, logs, exponentials, stable scalar
helpers (`ln_1p`/`log1p`, `ln_1m`/`log1m`, `expm1`, `softplus`,
`logaddexp`, `logsubexp`, `logit`, `sigmoid`), trig and pi-scaled trig
(`sin_pi`, `cos_pi`, `tan_pi`), small-angle helpers (`sinc`, `sinc_pi`,
`cosc`), cancellation helpers (`sqrt1pm1`, `sqrt1m1`, `hypot_minus`),
product-sum helpers (`mul_add`, `sum_products`, `diff_of_products`),
polynomial helpers (`eval_poly`, `eval_rational_poly`), vector length helpers
(`hypot2`, `hypot3`),
exact-root/rational-power helpers (`cbrt`, `root_n`, `pow_rational`),
certified integer helpers (`floor_certified`,
`ceil_certified`, `round_certified`, `trunc_certified`, `fract_certified`,
`rem_euclid_certified`), inverse trig, inverse hyperbolic functions, normal
distribution helpers (`erf`, `erfc`, `erfcx`, `dnorm`,
`pnorm`, `normal_sf`, `pnorm_upper`, `normal_interval`, `pnorm_diff`,
`log_pnorm`, `log_normal_sf`, `log_dnorm`, `erfinv`, `erfcinv`, `qnorm`,
`qnorm_upper`, `normal_pdf`, `normal_cdf`, `normal_survival`,
`normal_quantile`, `normal_mills`, `normal_hazard`, `normal_log_hazard`,
`normal_inverse_mills`, `hermite_probabilists`, `dnorm_derivative`,
`gaussian_derivative`, `standard_normal_moment`, `normal_interval_moment`,
`truncated_normal_mean`, `truncated_normal_variance`, `gamma`, `lgamma`,
`beta`, `ln_beta`/`lbeta`, `regularized_beta`, `regularized_beta_q`,
`regularized_gamma_p`, `regularized_gamma_q`, `chi_square_cdf`,
`chi_square_sf`), integers, decimals, fractions, `pi`, and `e`.

Stability-oriented scalar forms keep common statistical expressions from being
assembled out of cancellation-prone generic arithmetic:

| Form | Meaning | Domain |
| --- | --- | --- |
| `(ln_1p x)` / `(log1p x)` | natural log of `1 + x` | `x > -1` |
| `(ln_1m x)` / `(log1m x)` | natural log of `1 - x` | `x < 1` |
| `(expm1 x)` | `exp(x) - 1` with a small-argument kernel | all real inputs |
| `(softplus x)` | `ln(1 + exp(x))` | all real inputs |
| `(logaddexp a b)` | `ln(exp(a) + exp(b))` | all real inputs |
| `(logsubexp a b)` | `ln(exp(a) - exp(b))` | certifiable `a > b` |
| `(logit p)` | `ln(p / (1 - p))` | `0 < p < 1` |
| `(sigmoid x)` | `1 / (1 + exp(-x))` | all real inputs |

Pi-scaled trig forms preserve rational-turn intent before falling back to
generic radian multiplication:

| Form | Meaning | Domain |
| --- | --- | --- |
| `(sin_pi x)` | `sin(pi * x)` with exact rational-turn cases | all real inputs |
| `(cos_pi x)` | `cos(pi * x)` with exact rational-turn cases | all real inputs |
| `(tan_pi x)` | `tan(pi * x)` with exact rational-turn cases | all non-pole inputs |

Small-angle forms preserve removable limits instead of exposing a false
division-by-zero at the origin:

| Form | Meaning | Domain |
| --- | --- | --- |
| `(sinc x)` | `sin(x) / x`, with `sinc(0) = 1` | all real inputs |
| `(sinc_pi x)` | `sin(pi * x) / (pi * x)`, with `sinc_pi(0) = 1` | all real inputs |
| `(cosc x)` | `(1 - cos(x)) / x^2`, with `cosc(0) = 1/2` | all real inputs |

Cancellation helper forms preserve common square-root differences instead of
forcing users to spell them as near-equal subtraction:

| Form | Meaning | Domain |
| --- | --- | --- |
| `(sqrt1pm1 x)` | `sqrt(1 + x) - 1` | `x >= -1` |
| `(sqrt1m1 x)` | `sqrt(1 - x) - 1` | `x <= 1` |
| `(hypot_minus x y)` | `sqrt(x^2 + y^2) - x` | all real inputs |

Product-sum forms preserve common fused arithmetic shapes before expanding into
generic arithmetic. `sum_products` takes flat product pairs:

| Form | Meaning | Domain |
| --- | --- | --- |
| `(mul_add a b c)` | `a*b + c` | all real inputs |
| `(sum_products x0 y0 x1 y1 ...)` | `x0*y0 + x1*y1 + ...` | even number of operands |
| `(diff_of_products a b c d)` | `a*b - c*d` | all real inputs |

Polynomial forms preserve Horner and rational-polynomial evaluation structure.
Simple has flat operands, so rational polynomials use a numerator coefficient
count after `x`; polynomial coefficients are constant-first. Bernstein and
de Casteljau operations carry curve-basis semantics and belong in curve-level
crates such as `hypercurve`.

| Form | Meaning | Domain |
| --- | --- | --- |
| `(eval_poly x c0 c1 c2 ...)` | `c0 + c1*x + c2*x^2 + ...` in Horner form | at least one coefficient |
| `(eval_rational_poly x n c0 ... d0 ...)` | numerator polynomial divided by denominator polynomial; `n` is numerator coefficient count | exact non-negative integer `n`, at least one denominator coefficient, non-zero denominator value |

Scalar vector-length forms route through the exact dot-product reducers before
taking square roots, so rational Pythagorean cases can stay exact:

| Form | Meaning | Domain |
| --- | --- | --- |
| `(hypot2 x y)` | `sqrt(x^2 + y^2)` | all real inputs |
| `(hypot3 x y z)` | `sqrt(x^2 + y^2 + z^2)` | all real inputs |

Root and exact rational-power forms preserve perfect rational roots before
falling back to exact-real rational exponents:

| Form | Meaning | Domain |
| --- | --- | --- |
| `(cbrt x)` | cube root of `x` | all real inputs |
| `(root_n x n)` | nth root of `x` | exact positive integer `n`; negative `x` requires odd `n` |
| `(pow_rational x q)` | `x^q` with exact rational exponent `q` | all positive `x`; negative `x` requires an odd denominator |

Certified integer forms make discontinuous decisions through exact rational
shortcuts or bounded exact-real comparison. If a boundary cannot be certified,
they return `Problem::Exhausted` rather than rounding through a primitive float:

| Form | Meaning | Domain |
| --- | --- | --- |
| `(floor_certified x)` | greatest integer `<= x` | certifiable integer boundary |
| `(ceil_certified x)` | least integer `>= x` | certifiable integer boundary |
| `(round_certified x)` | nearest integer, ties away from zero | certifiable half-integer boundary |
| `(trunc_certified x)` | integer part toward zero | certifiable sign and integer boundary |
| `(fract_certified x)` | `x - trunc_certified(x)` | certifiable truncation |
| `(rem_euclid_certified x m)` | Euclidean remainder for positive modulus | certifiable `m > 0` and quotient floor |

Normal-distribution forms use the same `Real` methods as Rust callers:

| Form | Meaning | Domain |
| --- | --- | --- |
| `(erf x)` | error function | all real inputs |
| `(erfc x)` | complementary error function | all real inputs |
| `(erfcx x)` | scaled complementary error function, `exp(x^2) * erfc(x)` | all real inputs |
| `(dnorm x)` | standard normal density | finite values with `abs(x) <= 10` |
| `(pnorm x)` | standard normal cumulative distribution | finite values with `abs(x) <= 10` |
| `(normal_sf x)` | standard normal upper-tail probability, `1 - pnorm(x)` | finite values with `abs(x) <= 10` |
| `(pnorm_upper x)` | alias for `normal_sf` | finite values with `abs(x) <= 10` |
| `(normal_interval lo hi)` | standard normal probability mass over `[lo, hi]` | finite bounds with `abs(bound) <= 10` and `lo <= hi` |
| `(pnorm_diff lo hi)` | alias for `normal_interval` | finite bounds with `abs(bound) <= 10` and `lo <= hi` |
| `(log_pnorm x)` | natural log of the standard normal CDF | finite values with `abs(x) <= 10` |
| `(log_normal_sf x)` | natural log of the standard normal upper tail | finite values with `abs(x) <= 10` |
| `(log_dnorm x)` | natural log of the standard normal density | all real inputs |
| `(erfinv y)` | inverse error function | `-1 < y < 1` |
| `(erfcinv y)` | inverse complementary error function | `0 < y < 2` |
| `(qnorm p)` | inverse standard normal CDF | `pnorm(-10) < p < pnorm(10)` |
| `(qnorm_upper p)` | inverse standard normal upper-tail probability | `pnorm(-10) < 1 - p < pnorm(10)` |
| `(normal_pdf x mean sigma)` | normal density with mean and standard deviation | `sigma > 0`, standardized `x` within the density window |
| `(normal_cdf x mean sigma)` | normal CDF with mean and standard deviation | `sigma > 0`, standardized `x` within the CDF window |
| `(normal_survival x mean sigma)` | normal upper-tail probability with mean and standard deviation | `sigma > 0`, standardized `x` within the survival window |
| `(normal_quantile p mean sigma)` | normal quantile with mean and standard deviation | `sigma > 0`, and `p` in the supported quantile window |
| `(normal_mills x)` | upper-tail Mills ratio, `normal_sf(x) / dnorm(x)` | all real inputs |
| `(normal_hazard x)` | standard normal hazard rate, `dnorm(x) / normal_sf(x)` | all real inputs |
| `(normal_log_hazard x)` | natural log of the standard normal hazard rate | finite values with `abs(x) <= 10` |
| `(normal_inverse_mills x)` | lower-tail inverse Mills ratio, `dnorm(x) / pnorm(x)` | finite values with `abs(x) <= 10` |
| `(hermite_probabilists n x)` | probabilists' Hermite polynomial `He_n(x)` | exact non-negative integer `n` |
| `(dnorm_derivative n x)` | nth derivative of the standard normal density | exact non-negative integer `n`, finite `x` with `abs(x) <= 10` |
| `(gaussian_derivative n x)` | alias for `dnorm_derivative` | exact non-negative integer `n`, finite `x` with `abs(x) <= 10` |
| `(standard_normal_moment n)` | raw standard normal moment `E[X^n]` | exact non-negative integer `n` |
| `(normal_interval_moment lo hi n)` | unnormalized raw moment over `[lo, hi]` | exact non-negative integer `n`, finite bounds with `abs(bound) <= 10` and `lo <= hi` |
| `(truncated_normal_mean lo hi)` | mean of a standard normal truncated to `[lo, hi]` | finite bounds with `abs(bound) <= 10` and `lo < hi` |
| `(truncated_normal_variance lo hi)` | variance of a standard normal truncated to `[lo, hi]` | finite bounds with `abs(bound) <= 10` and `lo < hi` |
| `(gamma x)` | gamma function `Gamma(x)` | exact integer or half-integer `x`, excluding non-positive integer poles |
| `(lgamma x)` | natural log of `abs(Gamma(x))` | exact integer or half-integer `x`, excluding non-positive integer poles |
| `(beta a b)` | beta function `B(a, b)` through gamma closed forms | exact integer or half-integer arguments whose gamma ratio is defined |
| `(ln_beta a b)` / `(lbeta a b)` | natural log of `abs(B(a, b))` | exact integer or half-integer arguments whose gamma ratio is defined |
| `(regularized_beta a b x)` | regularized incomplete beta `I_x(a, b)` | exact positive integer `a` and `b`, `0 <= x <= 1` |
| `(regularized_beta_q a b x)` | complement `1 - I_x(a, b)` | exact positive integer `a` and `b`, `0 <= x <= 1` |
| `(regularized_gamma_p a x)` | regularized lower incomplete gamma `P(a, x)` | exact positive integer or half-integer `a`, `x >= 0` |
| `(regularized_gamma_q a x)` | regularized upper incomplete gamma `Q(a, x)` | exact positive integer or half-integer `a`, `x >= 0` |
| `(chi_square_cdf x k)` | chi-square CDF with `k` degrees of freedom | `x >= 0`, exact positive integer `k` |
| `(chi_square_sf x k)` | chi-square upper-tail probability | `x >= 0`, exact positive integer `k` |

Inputs outside those supported numeric ranges return `Problem` rather than silently
falling back to primitive floating point.
`ln_1p`/`log1p`, `ln_1m`/`log1m`, `logsubexp`, `logit`, and `tan_pi` return
`Problem::NotANumber` outside their open domains.
`sqrt1pm1` and `sqrt1m1` return `Problem::SqrtNegative` when their radicand is
known negative.
`root_n` rejects degree zero and even roots of negative values; `pow_rational`
inherits the existing negative-base rational exponent policy.
Reversed normal-interval bounds return `Problem::NotANumber`; equal bounds return exact
zero.
Parametric normal forms standardize exactly as `(x - mean) / sigma`, or
`mean + sigma * qnorm(p)` for quantiles.
`normal_mills` uses the stable `sqrt(pi/2) * erfcx(x / sqrt(2))` form.
Gaussian derivatives keep the Hermite polynomial part exact and multiply by
the shared standard normal density.
Standard moments use exact double-factorial closed forms; interval and truncated
moments use boundary-density recurrences over `normal_interval`.
Regularized gamma supports the integer and half-integer cases that reduce to
finite recurrences over `erf`/`erfc`, `exp(-x)`, `sqrt(x)`, and exact factorial
coefficients; chi-square helpers are thin wrappers through `P(k/2, x/2)` and
`Q(k/2, x/2)`.
`softplus`, `logaddexp`, `logsubexp`, `logit`, and `sigmoid` use sign-stable
forms so callers do not need to spell them through `ln(1 + exp(x))`,
`ln(exp(a) +/- exp(b))`, `ln(p) - ln(1 - p)`, or `1 / (1 + exp(-x))`.

## Conversions

Supported conversions include:

- integer types to `Rational` and `Real`;
- finite `f32`/`f64` to `Rational` and `Real` by exact IEEE-754 decoding;
- `Real` to `f32`/`f64` by approximation;
- `Real::to_f32_lossy()` and `Real::to_f64_lossy()` for borrowed primitive exports;

Float import rejects `NaN` and infinities. Borrowed lossy export returns `None` when no
finite primitive-float approximation can be produced. Scientific notation is not a
supported exact text format. `-0.0` imports as exact rational zero, so IEEE signed-zero
information is not preserved.

## Documentation And Benchmarks

Useful local checks:

```sh
cargo fmt --check
cargo test
RUSTDOCFLAGS=-Dwarnings cargo doc --no-deps
cargo bench --bench numerical_micro
cargo bench --bench borrowed_ops
cargo bench --bench float_convert
cargo bench --bench scalar_micro
cargo bench --bench library_perf --features simple
cargo bench --bench adversarial_transcendentals
cargo bench --bench adversarial_library
```

Run dispatch tracing separately:

```sh
cargo bench --bench dispatch_trace --features dispatch-trace
```

The generated benchmark summary is in [`benchmarks.md`](./benchmarks.md). Profiling
anchors and regression goals for `Rational`, `Real`, and `Computable` are in
[`PERFORMANCE.md`](./PERFORMANCE.md). Dispatch summaries are written to
[`dispatch_trace.md`](./dispatch_trace.md) when tracing is enabled.

For a full post-change performance pass, run:

```sh
cargo bench --all-features
```

The `library_perf` benchmark now has focused groups for the newer stable scalar
substrate, geometry/polynomial helpers, Gaussian/scientific helpers, and `Simple`
parser exposure. The adversarial library run includes tiny residuals, near-domain
boundaries, root and rational-power cases, normal tails, finite gamma/beta
recurrences, and polynomial/vector helper shapes. Dispatch tracing has matching
rows (`real/stable_scalar_substrate`, `real/geometry_polynomial_substrate`, and
`real/normal_scientific_substrate`) so benchmark findings can be connected back
to the representation branch that was taken.

When adding a shortcut, add a focused correctness test and a benchmark row for the
smallest affected surface. Keep the shortcut only if it improves the target without
regressing broader stack-facing paths.

## Provenance and Acknowledgements

`hyperreal` descends from the [`realistic`](https://github.com/tialaramex/realistic/)
project and continues that project's interest in practical computable real arithmetic.
Special thanks to [siefkenj](https://github.com/siefkenj), whose contributions improved
realistic and hyperreal.

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
- Odrzywołek, Andrzej. "[All Elementary Functions from a Single Binary
  Operator](https://arxiv.org/abs/2603.21852)." *arXiv*, 2026,
  arXiv:2603.21852. Related implementation: Schmidt, Tim.
  "[emlmath](https://github.com/timschmidt/emlmath)." GitHub, 2026. Relevant
  to `Computable` graph evaluation and expression-tree lowering.
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
