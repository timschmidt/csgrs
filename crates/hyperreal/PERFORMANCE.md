# Hyperreal Performance Profile

These notes are hand-maintained profiling anchors. `benchmarks.md` and
`dispatch_trace.md` are generated; this file records the current best timing
targets, the important dispatch paths behind them, and the goals to preserve or
improve during later optimization work.

Timings below are Criterion medians from the stored benchmark data on
2026-05-08. Treat them as local guardrails, not portable absolute limits.

## Benchmark Commands

Core hyperreal checks:

```sh
cargo test
cargo bench --bench scalar_micro
cargo bench --bench numerical_micro
cargo bench --bench adversarial_transcendentals
cargo bench --bench borrowed_ops
cargo bench --bench float_convert
cargo bench --bench library_perf
cargo bench --bench dispatch_trace --features dispatch-trace
```

Cross-crate regression checks:

```sh
cargo bench --bench mathbench -- 'scalar_trig/hyperreal.*/(0.1|1.23456789|1e6|1e30|1000pi_eps)/(sin|cos)'
cargo bench --bench mathbench -- 'matrix[34]/hyperreal'
cargo bench --bench mathbench --features hyperreal-dispatch-trace -- --write-dispatch-trace-md
cargo bench --manifest-path ../hyperlimit/Cargo.toml --bench predicates
cargo bench --manifest-path ../hyperlimit/Cargo.toml --bench predicates --features dispatch-trace -- --write-dispatch-trace-md
```

## Fuzz coverage

The standalone `fuzz` workspace covers four runtime-bearing public families:

| Target | Exactness and API boundary |
| --- | --- |
| `rational_arithmetic` | Rational construction, every core arithmetic ownership path, inverse/powers, truncation/fraction decomposition, and exact dyadic conversion |
| `real_exact` | Exact Real arithmetic, fused dot/product-sum kernels, prepared determinant filters, certified facts/comparisons, exact conversion, and serde round trips |
| `real_elementary` | Domain-bearing roots, logarithms, powers, trigonometric, inverse/hyperbolic, normal, error, and gamma-family construction with forced lazy evaluation |
| `computable_approximation` | Direct Computable graph construction, transcendental dispatch, repeatable multi-precision approximation, structural facts, and bounded sign refinement |

Inputs remain bounded exact rationals. Primitive-float values are requested only
through the explicitly lossy output API and are checked for finiteness, never
used as proof. The live campaign found and fixed a public-contract defect where
`Rational::dyadic_to_f64_exact` debug-asserted on non-dyadic input despite its
`Option` return type; arbitrary non-dyadics now return `None`.

```sh
cargo check --manifest-path fuzz/Cargo.toml --bins
CCACHE_DISABLE=1 cargo +nightly fuzz run -s none rational_arithmetic --fuzz-dir fuzz -- -runs=1000 -timeout=10 -max_len=64
CCACHE_DISABLE=1 cargo +nightly fuzz run -s none real_exact --fuzz-dir fuzz -- -runs=1000 -timeout=10 -max_len=64
CCACHE_DISABLE=1 cargo +nightly fuzz run -s none real_elementary --fuzz-dir fuzz -- -runs=1000 -timeout=10 -max_len=64
CCACHE_DISABLE=1 cargo +nightly fuzz run -s none computable_approximation --fuzz-dir fuzz -- -runs=1000 -timeout=10 -max_len=64
```

The `-s none` smoke setting is needed only in ptrace-managed environments where
LeakSanitizer cannot attach. Normal local campaigns should retain the default
AddressSanitizer configuration.

## Rational Path

Current timing anchors:

| Row | Median |
| --- | ---: |
| `construction_speed/rational_one` | 15.5 ns |
| `construction_speed/rational_new_one` | 16.9 ns |
| `borrowed_op_overhead/rational_clone_pair` | 44.3 ns |
| `pure_scalar_algorithm_speed/rational_mul` | 117.5 ns |
| `borrowed_op_overhead/rational_add_refs` | 385.0 ns |
| `pure_scalar_algorithm_speed/rational_add` | 399.2 ns |
| `pure_scalar_algorithm_speed/rational_div` | 595.3 ns |
| `borrowed_op_overhead/rational_add_owned` | 973.5 ns |
| `dense_algebra/rational_dot_64` | 36.8 us |
| `dense_algebra/rational_matmul_8` | 229.8 us |

Relevant path notes:

- Integer identity constructors avoid BigInt conversion and reduction.
- Dyadic denominators use shift-only reduction instead of full gcd.
- Dispatch tracing records rational temporary construction, reductions, gcds,
  power-of-two common factors, common-factor distributions, and peak operand
  sizes. Matrix regressions should be investigated with those counters before
  changing algebra code.
- Exact f64 imports are intentionally kept rational/dyadic when possible so
  `hyperlattice` and `hyperlimit` can stay on structural paths.

Goals:

- Keep `rational_one` and `rational_new_one` under 20 ns.
- Keep rational clone pairs under 50 ns.
- Avoid adding gcds to dyadic import and matrix hot paths.
- If rational add/div rows move, inspect dispatch trace counters before
  assuming the operation itself changed.

## Real Path

Current timing anchors:

| Row | Median |
| --- | ---: |
| `construction_speed/real_from_i32_one` | 74.4 ns |
| `construction_speed/real_new_rational_one` | 74.6 ns |
| `construction_speed/real_one` | 75.5 ns |
| `pure_scalar_algorithm_speed/real_exact_mul` | 186.8 ns |
| `pure_scalar_algorithm_speed/real_exact_add` | 454.5 ns |
| `pure_scalar_algorithm_speed/real_exact_div` | 664.9 ns |
| `structural_query_speed/pi_minus_three_sign_query` | 34.9 ns |
| `symbolic_reductions/pi_minus_three_facts` | 38.2 ns |
| `dense_algebra/real_dot_36` | 28.3 us |
| `dense_algebra/real_matmul_6` | 153.0 us |

Scalar trig anchors from `hyperlattice`:

| Row | Median |
| --- | ---: |
| `hyperreal/1e30_cos` | 89.3 ns |
| `hyperreal/1e6_sin` | 90.3 ns |
| `hyperreal/1000pi_eps_sin` | 90.9 ns |
| `hyperreal/1000pi_eps_cos` | 91.0 ns |
| `hyperreal/0.1_cos` | 152.7 ns |
| `hyperreal/0.1_sin` | 153.9 ns |
| `hyperreal/1.23456789_cos` | 204.6 ns |
| `hyperreal/1.23456789_sin` | 209.0 ns |
| `hyperreal-rational/1000pi_eps_sin` | 855.4 ns |
| `hyperreal-rational/1000pi_eps_cos` | 861.9 ns |

Relevant path notes:

- `Real::sin` and `Real::cos` keep large exact rationals at the Real layer and
  construct large-rational deferred Computable nodes directly. This is what
  keeps the `1e6`, `1e30`, and f64 `1000pi_eps` rows in the 90 ns range.
- `ConstOffset` values of the form `k*pi + eps` reduce to the rational residual
  before trig. This is the important path for rational `1000pi_eps`.
- `Real::clone` normally rebuilds symbolic computable certificates rather than
  cloning cold payloads, but `ConstOffset` is intentionally cloned because
  rebuilding its cached-pi plus offset tree dominated the rational
  `1000pi_eps` benchmark.
- Exact pi multiples use `SinPi`/`TanPi` certificates where useful. Plain
  rational trig stays in Computable, but now enters owned rational constructors
  to avoid redundant Ratio construction.
- `pi - 3` and similar almost-simple constants are expected to answer sign and
  full structural facts around 35-40 ns.

Goals:

- Keep large scalar trig and f64 `1000pi_eps` rows under 100 ns.
- Keep small scalar trig rows such as `0.1` under 160 ns.
- Bring medium scalar rows such as `1.23456789` below 200 ns without regressing
  large rows.
- Keep rational `1000pi_eps` sin/cos under 1 us.
- Investigate remaining rational exact-special hot spots, especially
  `hyperreal-rational/pi_7_cos`, rational endpoint inverse trig, and
  `hyperreal-rational/e_acosh`.
- Any new symbolic class must show wins in `scalar_micro`, `hyperlattice`, and
  `hyperlimit`; otherwise keep the representation simpler.

## Computable Path

Current timing anchors:

| Row | Median |
| --- | ---: |
| cached trig/inverse/hyperbolic rows | 37-40 ns |
| `computable_transcendentals/sin_zero_cold_p96` | 34.2 ns |
| `computable_transcendentals/tan_zero_cold_p96` | 34.0 ns |
| `computable_transcendentals/cos_zero_cold_p96` | 75.5 ns |
| `computable_transcendentals/cos_cold_p96` | 1.49 us |
| `computable_transcendentals/sin_cold_p96` | 1.59 us |
| `computable_transcendentals/cos_f64_cold_p96` | 1.70 us |
| `computable_transcendentals/sin_f64_cold_p96` | 1.73 us |
| `computable_transcendentals/sin_1e30_cold_p96` | 2.07 us |
| `computable_transcendentals/cos_1e30_cold_p96` | 2.20 us |
| `computable_transcendentals/sin_1e6_cold_p96` | 2.29 us |
| `computable_transcendentals/cos_1e6_cold_p96` | 2.30 us |
| `computable_transcendentals/tan_cold_p96` | 3.38 us |
| `computable_transcendentals/asin_cold_p96` | 6.55 us |
| `computable_transcendentals/acos_cold_p96` | 8.92 us |
| `computable_transcendentals/acosh_cold_p128` | 9.47 us |

Relevant path notes:

- Large exact-rational sin/cos/tan use deferred nodes with direct half-pi
  residual arithmetic rather than constructing a generic reduced expression
  graph.
- Medium exact rationals use direct `pi/2 - r` residual nodes for sin/cos and
  cotangent complement nodes for tan.
- Small exact rationals now use rational-backed prescaled trig nodes so
  construction avoids a child Ratio node. The approximation dispatcher
  materializes the same rational input only when digits are requested.
- Scaled inverse-trig compositions use a conservative exact upper bound in pi
  units through rational products, sums, binary shifts, and admitted asin/acos
  ranges. Arguments certified within `[-pi/2, pi/2]` enter the prescaled kernel
  without calling `approx(-1)` merely to choose a reduction path.
- Cached approximation rows are intentionally very sensitive to code layout.
  During optimization, keep helper functions away from the middle of hot
  `sin`/`cos`/`tan` kernels unless the low-level numerical benches prove there
  is no regression.
- Dispatch trace path names to watch: `large-rational-deferred`,
  `medium-rational-half-pi-rewrite`, `structural-small-prescaled`,
  `integer-pi-plus-rational`, and `generic-half-pi-reduction`.
- The Payne--Hanek principle that only the low quotient bits and reduced
  residual matter is also applied to the narrow exact-rational interval
  `7/2 <= |x| <= 39/10`.  That interval certifies a nearest half-pi multiple
  of `+/-2` without approximating pi.  The former slow offender
  `tan(3 + 190/219)` fell from 11.34 us to 3.29 us at p=-96 (about 71%).

Goals:

- Keep cached rows below 45 ns and zero rows below 80 ns.
- Keep cold sin/cos baseline around 1.5-1.6 us and avoid widening the
  sin/cos gap.
- Bring large exact-rational cold sin/cos closer to 2 us or below.
- Reduce tan cold paths toward 3 us without changing pole behavior.
- The biggest remaining low-level targets are inverse trig and hyperbolic
  cold paths: `acos`, `asin`, `atan`, `acosh`, `asinh`, and large `exp`.

### Retained asinh series crossover

The exact-rational asinh dispatcher formerly sent every value with binary MSD
`<= -1` through the direct Taylor recurrence. That includes the whole
`[1/2, 1)` binade, where convergence becomes progressively slower. The retained
threshold limits the series to MSD `<= -2`; larger subunit rationals use the
existing cancellation-safe
`ln1p(x + x^2 / (sqrt(1 + x^2) + 1))` transform. Both paths remain exact
Computable graphs and round only at the requested approximation precision.

Paired 100-sample Criterion runs at 128 bits measured:

| Input | Series control | Retained `ln1p` path | Change |
| --- | ---: | ---: | ---: |
| `asinh(1/2)` | 6.866 us | 6.355 us | 8.36% faster |
| `asinh(3/4)` | 16.344 us | 4.695 us | 71.18% faster |

The new three-quarters sentinel guards the crossover and its exact
`asinh(3/4) = ln(2)` value. Construction tracing now records
`near-zero-ln1p-transform` for the mid case while the tiny case retains
`exact-small-rational-series`. The complete all-target/all-feature gate, strict
Clippy, and warning-denied documentation passed. The Computable approximation
and public Real elementary fuzz targets each completed 1,000 sanitizer-backed
runs without a failure, reaching 4,254 and 6,355 coverage edges respectively.

Two inverse-trig follow-ups were measured and fully removed. Routing the
`acos(7/10)` square-root residual through the generic Computable atan graph
raised the paired asin/acos rows from 5.954/5.689 us to 7.787/7.466 us
(30--31% slower). Explicitly reducing that graph around the cached
`atan(1/2)` anchor still measured 7.632/7.537 us. The direct
`atan_sqrt_rational_small` kernel therefore remains the correct schedule for
this exact-rational range.

Negative rational acos values now use the retained `pi - acos(|x|)` node over
their complete domain instead of expanding mid-range values through nested
half-pi/asin identities. A stack regression composes both positive and negative
rational acos phases with exact gear-like carrier and rolling-angle scales,
then constructs and evaluates all corresponding sine/cosine coordinates. This
keeps the representation exact and bounded without a binary floating-point
probe or recursive constructor expansion.

## Reference Audit (2026-07-15)

This audit read every work in the README reference list, mapped each proposed
mechanism to the implementation, and retained code only when a focused trace,
benchmark, and correctness test supported it.

| Reference | Transferable mechanism | Result in hyperreal |
| --- | --- | --- |
| Bareiss (1968) | Exact division and fraction-free elimination keep intermediate coefficients integral. | Already reflected in delayed rational reduction and product-sum aggregation.  General elimination is outside this scalar crate. |
| Boehm et al. (1986) | Precision-driven functional exact reals, cached best approximations, variable-precision Newton steps, and balanced expression trees. | The representation, approximation cache, and Newton kernels already follow the paper.  Balanced arbitrary-length sums were measured and rejected; details below. |
| Boehm (2020) | Separate terminating approximate comparison from potentially divergent exact comparison; preserve symbolic facts and cached recursive approximations. | Existing structural facts, bounded refinement, exact float import, explicit lossy export, and cached `Computable` graphs cover the applicable API.  A fixed rational-size cap would change exact-rational extraction semantics and was not adopted. |
| Brent (1976) | Variable-precision Newton iteration and high-precision AGM/Landen elementary functions. | Newton reciprocal and square root are already variable precision.  AGM was not introduced because the paper itself notes that conventional kernels win at modest precision, which is the measured regime here. |
| Brent--Zimmermann (2010) | Staged argument reduction, `ln1p` symmetry, binary splitting, and asymptotically fast pi/functions. | Existing trig reduction, the `x/(2+x)` logarithm transform, Newton kernels, and binary-split exponential cover the useful mechanisms.  AGM or Chudnovsky pi is reserved for evidence of an extreme-precision bottleneck. |
| Johansson (2015) | Table-based argument reduction plus rectangular splitting shortens medium-precision elementary-function series while retaining rigorous error bounds. | Retained a minimal exact-rational `atan(2/3)` table point assembled from the existing pi and `atan(1/5)` caches.  The representative interval sweep improved by 24% at 32 bits, 30% at 96 bits, and 41% at 256 bits; a larger table and rectangular splitting remain unjustified at the current operand sizes. |
| Middeke--Jeffrey--Koutschan (2021) | Predict systematic common row/column factors in fraction-free matrix decompositions. | No LU/QR decomposition exists here on which to attach the three-entry factor predictor.  Rational aggregation already shares denominators and strips dyadic/common factors. |
| Odrzywolek (2026) | Lower elementary expressions to the binary `exp(x)-ln(y)` operator. | Rejected for this runtime: lowering expands the graph and imports complex principal-branch and infinity semantics absent from this real-only API. |
| Payne--Hanek (1983) | Reduce huge trig arguments using only the quotient bits and residual bits that affect the result. | Retained as an exact narrow-sector certificate for the promoted tangent tail; measured result below. |
| Shewchuk (1997) | Floating filters followed by adaptive nonoverlapping expansions and exact fallback. | Conservative f64 filters plus prepared exact-word and arbitrary-precision fallbacks already provide the profitable first and final stages.  Expansion stages remain a cross-stack candidate only if near-degenerate traces show exact fallback dominates. |
| Smith--Powell (2011) | Avoid pivot normalization until the end of Gauss--Jordan elimination. | Consistent with delayed division, but the crate has no row-reduction API.  Adding one would be a new subsystem, not a local optimization. |
| Yap (1997) | Exact decisions may use approximations; compile recurring expressions, carry error bounds, and drive precision from the root. | This is the architecture of `Real`/`Computable` structural graphs, certified approximations, and predicate filters.  Algebraic root isolation and geometric-object packages belong above this scalar substrate. |

### Retained experiment: certified tangent sector

The promoted slow-offender trace identified `tan(3 + 190/219)` as repeatedly
entering generic half-pi reduction.  The interval
`7/2 <= |x| <= 39/10` lies strictly between `3*pi/4` and `5*pi/4`, so the
nearest half-pi multiple is exactly `+/-2`.  A rational comparison now proves
that sector before approximation and reuses the already-computed exact
magnitude classification.

| Case, p=-96 | Before | After | Result |
| --- | ---: | ---: | ---: |
| `tan(3 + 190/219)` | 11.34 us | 3.29 us | -71% |
| `tan(-(3 + 177/200))` | about 11 us | 3.22 us | same certified path |
| `tan(-(5 + 15/187))` | about 7.6-7.8 us | 7.84 us | unchanged sentinel |
| `tan(-(7 + 5/6))` | about 5.4 us | 5.41 us | unchanged sentinel |

The dispatch trace must contain `near-large-rational-deferred`,
`large-rational-direct-quotient`, `fixed-half-pi-multiple-2`, and
`quarter-pi-large-rational` for the positive target, with no generic fallback.
The numerical cross-reference test covers both signs and the inclusive upper
boundary.

### Rejected experiment: balanced arbitrary-length Real sums

Boehm et al. suggest balancing long addition trees.  A pairwise balanced
`Real::sum_refs`/`sum_owned` reducer was benchmarked on 64 symbolic square
roots.  It increased construction from 5.87 us to 14.17 us and
construction-plus-`to_f64_lossy` from 32.74 us to 118.87 us.  Vec allocation,
extra cloning, and loss of the cheap left-fold shape outweighed the shallower
tree.  The implementation was removed; the two `real_sum_refs_64_symbolic`
benchmark rows remain as regression guards.

### Retained experiment: two-thirds arctangent table reduction

Johansson's medium-precision elementary-function work suggests reducing an
argument against a small table before entering a power series.  The exact
identity `atan(2/3) = pi/4 - atan(1/5)` provides an unusually cheap table point:
both source constants already have shared caches, and
`atan(r) - atan(2/3) = atan((3r-2)/(3+2r))` keeps the residual rational.
For `1/2 < r <= 4/5`, its magnitude is at most `1/8`, compared with as much as
`1/3` under the previous unit anchor.

| Case | Before | After | Result |
| --- | ---: | ---: | ---: |
| four-point interval sweep, p=-32 | 6.56 us | 5.02 us | -24% |
| four-point interval sweep, p=-96 | 13.15 us | 9.22 us | -30% |
| four-point interval sweep, p=-256 | 34.81 us | 20.31 us | -41% |
| upper edge `atan(4/5)`, p=-96 | 2.82 us | 2.68 us | -5% |
| representative `atan(7/10)`, p=-96 | 3.29 us | 2.11 us | -36% |

The sweep covers `11/20`, `3/5`, `7/10`, and `4/5`; the upper-edge row guards
the point with the smallest expected gain.  The full rational inverse-trig
cross-reference grid passes, and dispatch tracing records
`two-thirds-anchor-shared` with the existing pi and `atan(1/5)` caches.

### Architecture and measurement triggers

- Shewchuk expansion stages become applicable only if predicate traces in `hyperlimit` or
  `hypermesh` prove that near-degenerate floating inputs frequently reach the
  arbitrary-precision fallback.  The paper reports nontrivial ordinary-input
  overhead, so a scalar-only microbenchmark is insufficient justification.
- Chudnovsky/binary-split pi or AGM elementary functions become applicable only if an
  extreme-precision benchmark shows the current Machin/Taylor kernels dominate
  end-to-end work.  The current measured workload is below that crossover.
- Fraction-free LU/QR common-factor prediction belongs in the crate that
  owns a general matrix decomposition, not in the exact scalar substrate.
- Additional arctangent table points or rectangular splitting require a measured
  precision/input band that remains series-dominated.  The retained two-thirds point
  captures the largest residual in the current exact-rational interval without
  growing the shared-constant representation.

## Regression Triage

When a scalar row regresses:

1. Regenerate traces first, separately from Criterion.
2. Check whether the row moved from a specialized path to a generic path.
3. If the trace path is unchanged, suspect code layout, extra clone/certificate
   rebuilds, or rational reduction counters before changing algorithms.
4. Re-run the smallest affected Criterion filter, then one cross-crate guard
   from `hyperlattice` and one from `hyperlimit`.

For this snapshot, the most important regression sentinels are:

- `scalar_trig/hyperreal/(1e6|1e30|1000pi_eps)/(sin|cos)`: under 100 ns
- `scalar_trig/hyperreal-rational/1000pi_eps/(sin|cos)`: under 1 us
- `structural_query_speed/pi_minus_three_*`: sign/facts around 35-40 ns
- `computable_transcendentals/*_cached_*`: under 45 ns
- `matrix3|matrix4/hyperreal*`: no broad regression after clone or symbolic
  representation changes
