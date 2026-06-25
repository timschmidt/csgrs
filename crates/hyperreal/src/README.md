# hyperreal source map

This directory contains the implementation behind the public scalar API. The
main design rule is to preserve exact or symbolic structure for as long as it is
cheap and useful, approximate only when a caller asks for numeric precision, and
cache approximations/facts that are expensive to rediscover.

## Project layout

- `lib.rs`: public exports and crate-level API surface.
- `problem.rs`: domain and arithmetic errors shared by public types.
- `structural.rs`: conservative public sign, zero, magnitude, and exactness
  facts.
- `trace.rs` and `dispatch_trace`: optional instrumentation used by targeted
  dispatch tracing.
- `serde.rs`: serialization for durable expression structure. Transient caches
  and abort signals are intentionally not serialized.
- `simple.rs`: optional expression parser and evaluator behind the `simple`
  feature.
- `rational/`: exact arbitrary-precision rational arithmetic. See
  [`rational/README.md`](./rational/README.md).
- `real/`: public symbolic scalar type layered over rational and computable
  values. See [`real/README.md`](./real/README.md).
- `computable/`: lazy exact-real expression graph and approximation kernels. See
  [`computable/README.md`](./computable/README.md).

## Numerical expectations

- Exact rationals stay exact until an API explicitly asks for approximation or
  conversion to a primitive float.
- Symbolic constants and recognizable forms stay symbolic when doing so avoids
  precision loss or repeated approximation.
- Approximation is requested by binary precision. More negative precision means
  more bits after the binary point.
- Approximation may cache intermediate values. Cache hits must not change
  semantics, structural facts, or later higher-precision approximations.
- Structural facts are conservative. Unknown means "not cheaply proven", not
  false.
- `Real` equality is structural, not a complete computer-algebra theorem prover.

## Numerical explosion controls

The implementation should prevent growth at the earliest layer that has enough
information:

- `Rational` keeps signs separate, recognizes dyadic denominators, canonicalizes
  zeros, and gives reducers shared-denominator/product-sum paths so repeated
  GCD work is not forced into every term.
- `Real` keeps exact rational parts and symbolic classes separate, preserving
  `pi`, `e`, roots, logarithms, recognized trig endpoints, and removable
  small-angle limits until they can simplify, cancel, or provide structural
  facts.
- Product-sum helpers preserve determinant and predicate shapes long enough to
  reuse exact rational reducers and avoid building avoidable product/add trees.
- Fixed-size vector helpers reuse exact dot-product reducers before square
  roots so lengths and distances avoid avoidable rational bloat.
- nth-root helpers collapse exact rational perfect roots before falling back to
  rational-exponent computable forms.
- `Computable` is the fallback for values that need refinement, not the default
  container for every expression. Kernels reduce arguments, use stable forms
  near cancellation points, and cache precision-indexed approximations without
  weakening later higher-precision requests.
- Higher layers should pass retained facts into reducers instead of issuing
  scalar sign or approximation probes inside dense matrix/vector lanes.

## Error model

Public fallible operations return `Problem` rather than panicking for ordinary
numeric domain failures:

- divide by zero
- square root of known-negative values
- zero-degree roots, even roots of known-negative values, and unsupported
  negative-base rational powers
- logarithm of non-positive values
- `ln_1p`/`log1p` inputs with `x <= -1`
- `ln_1m`/`log1m` inputs with `x >= 1`
- `logsubexp` inputs without a certifiable `a > b`
- `sqrt1pm1` inputs with `x < -1`, and `sqrt1m1` inputs with `x > 1`
- `logit` inputs outside `0 < p < 1`
- `tan_pi` inputs at tangent poles
- certified integer rounding or Euclidean-remainder boundaries that cannot be
  proved within the bounded exact-real refinement policy
- `rem_euclid_certified` with a non-positive modulus
- inverse trig / inverse hyperbolic domain failures
- normal density/CDF inputs outside the supported finite approximation window
- normal upper-tail inputs outside the supported finite approximation window
- normal interval bounds outside the supported finite approximation window, or
  reversed interval bounds
- normal log-CDF and log-upper-tail inputs outside the supported finite
  approximation window
- inverse error-function inputs outside their open probability domains
- normal quantile inputs outside the supported probability window
- parametric normal forms with non-positive standard deviation, or whose
  standardized value/probability is outside the supported standard-normal window
- normal log-hazard and lower-tail inverse Mills inputs outside the supported
  finite approximation window
- Gaussian derivative forms with a non-integer or negative derivative order, or
  whose density input is outside the supported finite approximation window
- normal moment forms with a non-integer or negative order, reversed interval
  bounds, degenerate truncated intervals, or interval bounds outside the
  supported finite approximation window
- gamma and beta forms with unsupported non-integer/half-integer arguments, or
  gamma poles at non-positive integers
- regularized beta forms with non-positive or non-integer shape parameters, or
  `x` outside `[0, 1]`
- regularized gamma forms with non-positive or non-integer/half-integer shape
  parameters, or negative `x`
- chi-square forms with non-positive degrees of freedom or negative `x`
- invalid primitive float import such as `NaN` or infinity

Internal `expect` calls are reserved for representation invariants that should
already have been proven by construction.

## Performance and tracing expectations

Performance work should follow the same priorities as the implementation:

- defer approximation as long as exact or symbolic structure can answer the
  question
- reduce symbolic or rational structure before constructing generic computable
  graphs
- reuse costly approximations and named constants
- avoid unnecessary allocation and cloning on borrowed arithmetic paths
- use inexpensive structural facts before falling back to approximation
- specialize only paths that have stable targeted benchmark evidence

Performance-driven choices should have adjacent comments, especially when the
code shape is less direct than the mathematical formula. Existing comments cite
papers where an algorithmic principle matters, for example fraction-free
elimination and exact-real arithmetic.

Use dispatch tracing to explain a path and Criterion benchmarks to decide
whether a change is worth keeping. Trace reductions without Criterion runtime
improvements are not enough.

The current benchmark surface mirrors the newer scalar API in four places:
`library_perf` has focused Criterion groups for stable scalar helpers,
geometry/polynomial helpers, Gaussian/scientific helpers, and `Simple` parser
exposure; `adversarial_library` samples tiny residuals, domain edges, tails,
roots, recurrences, and product/polynomial shapes; `dispatch_trace` has matching
rows for the same groups; and `benchmarks.md` is regenerated by the benchmark
binaries with the current means and confidence intervals.

## Type-level documentation

- [`Rational`](./rational/README.md): exact storage and arithmetic layer.
- [`Real`](./real/README.md): public symbolic scalar and structural fact layer.
- [`Computable`](./computable/README.md): lazy expression graph and numeric
  approximation layer.
