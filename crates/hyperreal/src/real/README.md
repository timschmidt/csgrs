# Real

`Real` is the public symbolic scalar. It combines an exact rational scale, a
compact symbolic class, an optional lazy computable certificate, and an optional
abort signal.

## Stored parts

```text
+-----------------------------------------------------------------------+
| Real                                                                  |
|                                                                       |
|  rational: Rational                                                   |
|  + exact signed scale                                                 |
|  + zero is represented here                                           |
|  + multiplies every nonzero symbolic/computable class                 |
|                                                                       |
|  class: Class                                                         |
|  + One                         exact rational only                    |
|  + Pi, PiPow, PiInv            pi-family certificates                 |
|  + Exp, PiExp, ConstProduct    e/pi product certificates              |
|  + Sqrt, PiSqrt, ...Sqrt       factored square-root certificates      |
|  + Ln, LnAffine, LnProduct     logarithm certificates                 |
|  + Log10                       base-10 logarithm certificate          |
|  + SinPi, TanPi                rational trig certificates             |
|  + Irrational                  opaque computable value                |
|                                                                       |
|  computable: Option<Computable>                                       |
|  + lazy approximation graph                                           |
|  + shared constants and cached approximations live inside Computable  |
|  + absent only when the rational/class certificate is sufficient      |
|                                                                       |
|  signal: Option<Signal>                                               |
|  + optional abort hook for bounded/refinement callers                 |
+-----------------------------------------------------------------------+
```

The `Class` value is a certificate, not the entire number. The mathematical
value is `rational * class_value`, with `Computable` available when numeric
approximation is required.

## Module map

- `mod.rs`: public export and semantic module split.
- `arithmetic.rs`: representation, symbolic classes, constructors,
  simplification, arithmetic, elementary functions, display, and most tests.
- `constructors.rs`: public constructor grouping.
- `facts.rs`: structural fact API grouping.
- `approximation.rs`: approximation-facing API grouping.
- `linear_combination.rs`: exact linear combination and product-sum helpers.
- `convert.rs`: primitive and rational conversions.
- `tests.rs`: semantic and regression tests.

Most implementation still lives in `arithmetic.rs` because private fields and
hot simplification paths are tightly coupled. Avoid moving code just for file
shape unless benchmarks show no cost.

## API expectations

- `Real::new(Rational)` creates exact rational values.
- named constants such as `pi`, `e`, and `tau` use cached/shared construction.
- arithmetic preserves recognizable symbolic structure where benchmarks justify
  it.
- fallible methods return `Problem` for known domain errors.
- `ln_1p`/`log1p`, `ln_1m`/`log1m`, and `expm1` preserve small-residual intent
  instead of forcing users to spell `ln(1+x)`, `ln(1-x)`, or `exp(x)-1` as
  cancellation-prone generic arithmetic; `softplus`, `logaddexp`,
  `logsubexp`, `logit`, and `sigmoid` are stable public compositions built on
  those primitives.
- `sin_pi`, `cos_pi`, and `tan_pi` expose rational-turn trig directly; exact
  rational inputs reuse the same `SinPi`/`TanPi` certificates and exact
  small-denominator tables that ordinary trig reaches through `pi` products.
- `sinc`, `sinc_pi`, and `cosc` preserve removable small-angle limits at zero
  instead of requiring users to spell them as division-heavy generic
  arithmetic.
- `sqrt1pm1`, `sqrt1m1`, and `hypot_minus` preserve common square-root
  cancellation patterns used by offsets, normalized vectors, and curvature
  calculations.
- `mul_add`, `sum_products`, and `diff_of_products` expose common product-sum
  forms directly, reusing exact-rational product reducers and omitting
  known-zero product lanes.
- `eval_poly` and `eval_rational_poly` preserve Horner and
  rational-polynomial evaluation structure instead of requiring callers to
  expand polynomial arithmetic by hand. Bernstein and de Casteljau operations
  carry curve-basis semantics and live in higher geometry crates.
- `hypot2` and `hypot3` reuse the exact dot-product reducers before square
  roots, so rational lengths such as 3-4-5 stay exact and symbolic zero axes
  reduce through `abs`.
- `cbrt`, `root_n`, and `pow_rational` preserve exact rational perfect roots;
  non-perfect positive roots fall back to rational-exponent computable forms,
  and negative odd roots are handled by symmetry.
- `floor_certified`, `ceil_certified`, `round_certified`, `trunc_certified`,
  `fract_certified`, and `rem_euclid_certified` expose discontinuous integer
  decisions only when exact rational shortcuts or bounded exact-real comparison
  can certify the relevant boundary.
- `erf`, `erfc`, `erfcx`, `dnorm`, `pnorm`, `normal_sf`, `pnorm_upper`,
  `normal_interval`, `pnorm_diff`, `log_pnorm`, `log_normal_sf`, `log_dnorm`,
  `erfinv`, `erfcinv`, `qnorm`, and `qnorm_upper` expose computable Gaussian
  helpers; `dnorm`/`pnorm`/`normal_sf`, log tails, and interval bounds are
  bounded to finite inputs with `abs(x) <= 10`, normal intervals require
  `lo <= hi`, `log_dnorm` uses the analytic `-x^2/2 - ln(2*pi)/2` form,
  `erfinv`/`erfcinv` use open domains, and quantiles are bounded to
  probabilities strictly between `pnorm(-10)` and `pnorm(10)`.
- parametric normal helpers (`normal_pdf`, `normal_cdf`, `normal_survival`,
  `normal_quantile`) require `sigma > 0`, standardize exactly before entering
  the standard-normal helper, and inherit the same standard-normal resource
  windows.
- Mills and hazard helpers use the standard upper-tail convention for
  `normal_mills = normal_sf / dnorm` and `normal_hazard = dnorm / normal_sf`;
  `normal_mills` is built through the stable `sqrt(pi/2) * erfcx(x/sqrt(2))`
  identity, while `normal_inverse_mills` is the lower-tail `dnorm / pnorm`
  convention and inherits the normal CDF window.
- `hermite_probabilists`, `dnorm_derivative`, and `gaussian_derivative` keep the
  probabilists' Hermite polynomial recurrence exact; only the shared normal
  density factor enters the computable approximation layer.
- `standard_normal_moment` uses exact double-factorial closed forms, while
  `normal_interval_moment`, `truncated_normal_mean`, and
  `truncated_normal_variance` reuse `normal_interval` plus boundary density
  recurrence terms.
- `gamma`, `lgamma`, `beta`, `ln_beta`/`lbeta`, `regularized_beta`, and
  `regularized_beta_q` provide exact integer/half-integer gamma and finite
  positive-integer beta forms for scientific scalar workloads without moving
  root-solving policy into the scalar layer.
- `regularized_gamma_p` and `regularized_gamma_q` support exact positive
  integer and half-integer shape parameters with `x >= 0`, using finite
  recurrences over existing `erf`/`erfc`, `exp`, `sqrt`, and exact factorial
  coefficients; `chi_square_cdf` and `chi_square_sf` wrap those forms as
  `P(k/2, x/2)` and `Q(k/2, x/2)` for positive integer degrees of freedom.
- benchmark coverage for these public helpers lives in `library_perf` under
  stable scalar, geometry/polynomial, normal/scientific, and `Simple` surface
  groups; adversarial and dispatch-trace rows cover tiny residuals, domain
  boundaries, normal tails, finite gamma/beta recurrences, and exact
  product/polynomial shapes.
- structural queries return conservative facts and should not force expensive
  approximation when representation facts are enough.
- borrowed arithmetic should avoid unnecessary expression cloning.
- conversion to primitive floats approximates; conversion from finite primitive
  floats is exact.

## Numerical expectations

`Real` should prefer this order:

1. answer from exact rational structure
2. answer from symbolic class facts
3. simplify symbolically into a smaller exact/certified form
4. construct or reuse a `Computable`
5. approximate only at requested precision

This is why many methods contain special cases for exact rationals, dyadics,
pi/e products, square roots, logarithms, and rational trig endpoints.

## Numerical explosion controls

`Real` delays expansion by keeping scale, symbolic class, and computable
fallback separate:

- exact rational and dyadic values stay accessible for reducers and primitive
  conversions
- symbolic constants and factored forms are preserved so later operations can
  cancel, classify, or answer sign/domain questions structurally
- endpoint, identity, inverse, and small-argument rewrites shrink expressions
  before a generic computable graph is built
- borrowed arithmetic should reuse existing structure rather than cloning graph
  nodes solely to ask scalar questions
- bounded sign and magnitude refinement is used only when retained facts cannot
  decide a branch

## Error expectations

Errors are semantic domain failures, not "could not prove cheaply" failures.
For example, a known-negative square root fails. A value whose sign is not
cheaply known may move to a computable path or bounded refinement path instead
of immediately failing, depending on the method.

## Performance expectations

Performance-sensitive code should document why a non-obvious representation is
kept. Typical reasons:

- preserving exact rational access for matrix/vector kernels
- keeping `pi`, `e`, and `sqrt` factors separate so later operations cancel
  them before approximation
- using cached computable constants rather than rebuilding kernels
- avoiding generic computable graphs for exact endpoints
- keeping direct expression shapes when Criterion shows they inline better
