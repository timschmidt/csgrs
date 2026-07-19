# Hyperreal Performance Profile

These notes are hand-maintained profiling anchors. `benchmarks.md` and
`dispatch_trace.md` are generated; this file records the current best timing
targets, the important dispatch paths behind them, and the goals to preserve or
improve during later optimization work.

Timings below are Criterion medians from the stored benchmark data through
2026-07-18. Treat them as local guardrails, not portable absolute limits.

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
| `pure_scalar_algorithm_speed/rational_mul_retained_general` | 10.38 ns |
| `pure_scalar_algorithm_speed/rational_mul_wide_dyadic_cold` | 166.78 ns |
| `pure_scalar_algorithm_speed/rational_add` (retained) | 8.47 ns |
| `pure_scalar_algorithm_speed/rational_sub` (retained) | 9.05 ns |
| `pure_scalar_algorithm_speed/rational_add_wide_dyadic_cold` | 87.78 ns |
| `pure_scalar_algorithm_speed/rational_sub_wide_dyadic_cold` | 87.78 ns |
| `pure_scalar_algorithm_speed/rational_inverse_owned_cold` | 21.13 ns |
| `pure_scalar_algorithm_speed/rational_inverse_retained` | 7.45 ns |
| `pure_scalar_algorithm_speed/rational_neg_owned_cold` | 7.12 ns |
| `pure_scalar_algorithm_speed/rational_neg_retained` | 6.14 ns |
| `pure_scalar_algorithm_speed/real_exact_add` (retained) | 22.78 ns |
| `pure_scalar_algorithm_speed/real_exact_sub` (retained) | 22.58 ns |
| `pure_scalar_algorithm_speed/rational_div` | 595.3 ns |
| `borrowed_op_overhead/rational_add_owned` | 973.5 ns |
| `dense_algebra/rational_dot_64` | 36.8 us |
| `dense_algebra/rational_matmul_8` | 229.8 us |

Relevant path notes:

- Integer identity constructors avoid BigInt conversion and reduction.
- Dyadic denominators use shift-only reduction instead of full gcd.
- Reduced dyadics with odd magnitude at most 63 and denominator through `2^63`
  share canonical immutable storage.
- Each immutable rational retains one exact multiplication result under weak operand
  keys in both commutative directions. The cache is bounded, cycle-free, and ignored
  by serialization; misses continue through the same exact word/BigUint kernels.
- Linear-result admission is adaptive. Shared storage on either operand retains
  immediately; otherwise a one-byte relaxed reuse hint records the first borrowed
  observation, the second result is admitted, and later calls reuse it. This keeps
  one-shot operands allocation-light while making retained outer carriers visible
  without cloning their scalar fields. The hint fits existing `RationalData` padding,
  keeping that allocation at 96 bytes. The lazily allocated arithmetic cache holds
  two weak-keyed linear results and, for shared values, one reciprocal and one
  opposite-sign result. Unary owners retain their result strongly while reverse
  edges are weak, so repeated division and negation reuse stable identities without
  ownership cycles. Five polymorphic entries leave room for both unary pairs and two
  linear results regardless of which operation initializes the box. A dedicated lazy
  slot retains an exact square factor and residual only after repeated
  square extraction is observed, without displacing those arithmetic entries. Sum and
  directed-difference entries can also
  occupy opposite operand caches and remain ignored by serialization. Occupied
  entries are checked before constructing a candidate. Cold wide-dyadic add/sub
  sentinels measured 87.78 ns; cold owned inversion measured 21.13 ns and retained
  inversion 7.45 ns; unique owned negation measured 7.12 ns and retained negation
  6.14 ns.
- Exact-rational `Real += &Real`, `Real -= &Real`, and `Real *= &Real` replace
  only the rational scale and invalidate the lossy approximation accelerator,
  preserving the existing exact class payload in place. Every build caches a
  borrowed `f64` view in the already-present atomic slot; default-feature exact
  rational clones leave it empty, while `cached-f64-approx` builds copy a
  populated view across those clones.
- When a dyadic denominator product overflows `u128` but both numerators and their
  product fit, multiplication cancels and multiplies those numerators in registers
  before allocating only the final exact result.
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
| `pure_scalar_algorithm_speed/real_exact_mul_retained` | 23.03 ns |
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
- Keep exact-rational inverse-sine construction below 200 ns across signs and
  endpoint/mid-domain schedules.
- Any new symbolic class must show wins in `scalar_micro`, `hyperlattice`, and
  `hyperlimit`; otherwise keep the representation simpler.

### Prepared rational predicate queries

Repeated geometric predicates can now prepare the floating interval for an
exact-rational homogeneous point once and reuse its values and conservative
conversion-error radii across several fixed linear forms. The affine 3D helper
sets the homogeneous weight to exact `1.0` with zero error instead of
reconverting the same rational one for every plane test. A filter that cannot
certify separation still returns `None` and reaches the unchanged arbitrary-
precision product-sum fallback.

The motivating `hypermesh` paths improved by 2.46--3.03% end to end in matched
on/off release runs. Direct tests cover positive, negative, and boundary-
inconclusive prepared queries, as well as the affine exact-one specialization.
A 15-second `real_exact` sanitizer campaign completed 63,207 executions without
a target failure.

### Exact-MSD domain certificates

Exact symbolic values with a unit-magnitude outer rational scale now promote
their certified sign and exact binary MSD into comparisons with one. A positive
value with MSD above zero is provably greater than one; one with MSD below zero
is provably less. The same certificate supplies absolute comparisons for
inverse-trigonometric and inverse-hyperbolic domain facts. Non-unit outer scales
remain unknown because multiplying two exact MSDs can carry into the next
binade; a `3e/8 > 1` regression protects that boundary.

`Real::acosh` consumes the certificate before constructing `x - 1`. In the
cross-crate exact-symbolic `acosh(e)` row, this reduced construction from
997.60 ns to 116.50 ns (88.32%) while the hyperlattice facade still performed
its own preflight. The exact subtraction/refinement path remains active for
uncertified values.

### One-pass rational-turn cosine reduction

Non-tabulated `cos(q*pi)` formerly asked the cosine table to reduce `q`, then
constructed `q + 1/2` and sent that new rational through the complete sine
curve reduction. The cosine reducer now returns either an exact table value or
the canonical signed `SinPi` complement in one visit. The resulting `Real` has
the same outer sign, reduced rational, class, and computable certificate as the
former half-turn identity, so inverse identities and exact equality are
unchanged.

Fresh 100-sample cross-crate Criterion runs measured the exact-symbolic
`pi/7` cosine construction at 486.27 ns before and 201.99 ns after, a 58.46%
improvement. The retained path is 63.44% faster than Numerica 128 at 552.42 ns
and 88.47% faster than Symbolica at 1.7514 us. The direct `Real::cos_pi(1/7)`
sentinel measured 213.00 ns. The exact tabulated control `cos(pi/3)` remained
on its table path at 46.271 ns.

Cross-stack dispatch evidence fell from 14 events to 12. Rational comparisons
fell from three to one, the half-turn addition disappeared, and the trace now
records `pi-rational-direct-sinpi-certificate`. Signed multi-period regressions
compare the complete exact result with `sin_pi(q + 1/2)` and also retain a
finite approximation oracle.

### Signed deferred exact-rational inverse sine

Exact-rational `asin` formerly expanded positive mid-domain and endpoint
inputs into `pi/2 - acos(x)` during public construction. Negative values first
negated the rational, recursively repeated that dispatch, built the same
complement graph, and then added an outer negation. A single signed
`AsinRational` node now retains the input instead. Tiny values still enter the
direct odd series. Mid-high and endpoint magnitudes form the cancellation-safe
acos complement inside the cold approximation kernel and combine its terms
once with two guard bits; smaller non-tiny rationals retain the former
adaptive complement graph on the first cold approximation.

Fresh 100-sample cross-stack construction runs measured exact rational
`asin(0.999999)` at 239.49 ns before and 156.22 ns after (34.8% faster), and
`asin(-0.999999)` at 358.40 ns before and 152.54 ns after (57.4% faster). The
retained rows are 93.9--94.1% faster than Numerica 128 and 98.8% faster
than Symbolica on the same inputs. The direct public `asin(7/10)` sentinel
measured 96.02 ns, with the positive and negative endpoint sentinels at
111.58 ns and 106.43 ns.

Cold p=-96 approximation also improved: the final 100-sample positive endpoint
row fell from 2.0483 us to 1.8843 us (8.0%), while the signed adversarial row
fell from 2.4611 us to 1.9225 us (21.9%). Differential tests compare the new node
with the former explicit acos complement at p=-16, -40, -96, and -256 across
both signs, the 7/8 schedule boundary, direct mid-domain values, the adaptive
3/10 schedule, and endpoint values.
Cross-stack construction traces fell from 14 to 11 events for the positive
endpoint and from 15 to 9 for the negative endpoint; the latter no longer
constructs pi, an acos node, a second rational node, or either negation wrapper.
Sanitizer-backed live campaigns completed 24,241 public Real elementary cases
and 544 direct Computable approximation cases without a failure.

The retained rational residual can also serve directly as the squared argument
of the specialized atan series. Avoiding a wide re-square of the sampled root
reduced the fresh `asin(7/10)` p=-96 cold row from 6.4495 us to
6.0863 us (5.6%). The paired standalone `acos(7/10)` row remained effectively
unchanged at 5.6700 us because it retains the sampled-root schedule that is
faster for that independent entry point.

### Bounded exact-integer exponential powers

Positive exact-integer exponentials from 2 through 256 now reuse the shared
exact `e` constant and build `e^n` by binary exponentiation. The former path
constructed an `ln(2)` quotient, rounded the reduction index, and retained a
large prescaled exponential graph even though the input already identified the
integer power. Zero and one keep their canonical shortcuts; negative integers
and values above 256 retain the cancellation-safe range-reduction fallback.

At p=-128, the 100-sample `exp(128)` cold row fell from 7.0691 us to 4.7178 us
(33.3%). The limit sentinel `exp(256)` measured 6.8843 us, while the adjacent
fallback `exp(257)` measured 12.4353 us. Cross-library construction of the same
exact `exp(128)` expression fell from 3.0952 us on the old graph to 251.06 ns
(91.9%); the retained path is 4.16 times faster than Numerica 128 at 1.0444 us
and 7.60 times faster than Symbolica at 1.9075 us. The exact-rational facade
measured 252.53 ns.

The first binary-power prototype exposed an over-optimistic magnitude estimate
in chained squares and products. Structural MSD estimates are now propagated
through those nodes only when their child estimates are exact, and the square
kernel obtains a certified cached MSD before setting its working precision.
An exhaustive oracle compares every exponent from 2 through 256 with the former
`ln(2)` reduction at p=-40, with deeper p=-128 sentinels at 2, 13, 128, and 256.
Sanitizer-backed campaigns then completed 26,468 public elementary cases and
364 direct approximation cases without a failure.

The regenerated trace reduces `computable/exp_large_rational` from 29 events to
5, records `bounded-integer-e-power`, and removes the old `ln2-range-reduction`
and its rational add, multiply, comparison, and word-result traffic.

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
| `computable_transcendentals/exp_large_cold_p128` | 4.72 us |
| `computable_transcendentals/asin_cold_p96` | 6.09 us |
| `computable_transcendentals/acos_cold_p96` | 5.67 us |
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
  cold paths: `acos`, `asin`, `atan`, `acosh`, and `asinh`.

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

### Retained experiment: exact square-factor screens

Exact f64 vector coordinates become dyadic rationals, so their squared norm
often has a denominator that is a large power of two. The former rational
square extractor repeatedly divided that denominator by four and issued
separate arbitrary-precision remainder probes for every small square factor
and fixed residual divisor. Those probes dominated exact square-root
construction even though the input shape was simple.

The retained extractor splits a large power-of-two exponent in constant time.
For other large integers it first applies exact quadratic-residue screens
modulo 64 and 63, then shares one remainder across the small square factors
and one across the fixed divisor schedule. The screens only reject residue
classes that no integer square can occupy; factor extraction and canonical
residual reconstruction remain exact. Exhaustive roots through 4096, large
power-of-two exponents, and every scheduled factor have dedicated regression
coverage.

| Workload | Before | After | Result |
| --- | ---: | ---: | ---: |
| exact dyadic vector-norm radicand | 2.097 us | 432.04 ns | 79.4% faster |
| Hyperlattice `vec3 magnitude` | 3.067 us | 798.41 ns | 74.0% faster |
| Hyperlattice `vec3 normalize` ledger | 5.30 us | 3.30 us | 37.7% faster |
| Hyperlattice `vec4 magnitude` ledger | 2.64 us | 832.44 ns | 68.5% faster |

An eager full perfect-square test and a specialized three/four-term
sum-of-squares API did not improve the end-to-end rows, so both experiments
were removed. Sanitizer-backed nightly fuzzing completed 774,516 rational
arithmetic cases, 93,237 exact-real cases, and 35,767 elementary-real cases
without a failure. Dispatch tracing distinguishes residue rejection, the
large-power-of-two path, and both shared-remainder schedules.

### Retained experiment: exact square-root reductions

Repeated public square roots were still re-running exact square-factor
extraction even though the immutable rational radicand was unchanged. The
retained path now records a one-byte reuse observation, keeps the first call on
the original exact extractor, and only admits the exact square factor and
residual on the second observation. Later calls clone those two
canonical results. The lazy pair is bounded, ignored by serialization, and
cannot point back to its source, while reciprocal, negation, and both linear
cache identities remain independently available. The added observation byte
fits existing padding, so `RationalData` remains 96 bytes.

Fresh 50-sample direct medians measured 165.32 ns for a fresh uniquely owned
`sqrt(90)` and 78.79 ns for its retained shared-input route, a 52.3% reduction.
More expensive repeated reductions fell from 433.54 ns to 75.33 ns for the exact
dyadic vector-norm sentinel and from 2.03 us to 54.31 ns for the non-dyadic
sum-of-squares sentinel. The cold fixture is deliberately outside the global
small-integer pool, so it also proves one-shot inputs do not receive an eager
cache allocation.

On Hyperlattice's matched four-case scalar facade, exact f64 imports now measure
49.18 ns and explicit rationals 34.07 ns, versus 96.34 ns for Numerica 128 and
1.478 us for Symbolica. Both exact forms beat the fixed-precision control. The
four individual cases also beat Numerica, including the imported tiny dyadic
(83.26 ns versus 94.73 ns) and imported `e` (63.00 ns versus 100.52 ns).
Regression tests prove exact factor equality, stable retained identities,
cycle-free destruction, and coexistence with both unary and both linear pairs;
dispatch tracing records `reuse-observed` followed by `retained-reduction`.

### Retained experiment: exact dyadic/general product cancellation

Profiling one exact f64 coordinate multiplied by a reciprocal vector-norm
radical placed most samples in the word-sized rational multiplication and
result-reduction paths. The retained multiplier recognizes products with one
power-of-two denominator, removes internal dyadic factors by shifts, reduces
raw general parts when necessary, and cross-cancels both operands before
forming either product. Power-of-two numerators over odd denominators provide
a cheap proof that the general operand is already reduced; small opposing
numerators use one remainder before the binary GCD. The arbitrary-precision
counterpart applies the same cancellation schedule before wide products.

The generic word path remains defensive because internal decimal construction
may temporarily carry values such as `16/10`. It bypasses its final reduction
only when denominator-one, unit-numerator, or dyadic structural facts prove
both inputs reduced. Checked multiplication, rather than a shift-count check,
guards the reconstructed denominator against word overflow. The all-feature
adversarial benchmark caught both boundaries during development; final
regressions cover unreduced even and odd decimal factors, wide raw general
parts, and overflowing shifted denominators.

| Workload | Before | After | Result |
| --- | ---: | ---: | ---: |
| exact dyadic reciprocal-radical scale | 558.37 ns | 239.73 ns | 57.1% faster |
| wide dyadic/general cross-cancel sentinel | 1.263 us | 1.194 us | 5.5% faster |
| Hyperlattice `vec3 normalize` | 3.30 us | 2.57 us | 22.2% faster |
| Hyperlattice `vec4 normalize` | 3.62 us | 3.16 us | 12.6% faster |

A shared batch-scaling API was also measured across vector normalization. It
did not improve vec3 and changed the other rows by only a few percent, so the
API and its Hyperlattice caller were removed. The retained optimization stays
inside exact rational multiplication and does not introduce a floating-point
decision boundary.

### Retained experiment: native machine-sized integer powers

Profiling Hyperlattice's exact scalar `powi(..., 5)` facade placed the hot path
in three generic `Real` multiplications, repeated rational reductions, and
temporary arbitrary-precision storage. Hyperreal now raises reduced word-sized
rationals in checked `u128` storage when the powered numerator and denominator
fit, constructs a dyadic denominator from its exact shift when they do not, and
uses the former arbitrary-precision schedule as the exact fallback. The public
`Real::powi_i64` entry point also avoids allocating a `BigInt` exponent and
retains the existing rational and symbolic `Real::powi` semantics.

Fresh cross-library medians for the four-case Hyperlattice facade moved from
376.76 ns to 161.11 ns for exact f64 inputs and from 2.813 us to 210.93 ns for
explicit rational inputs. The Numerica 128 control was 84.53 ns and Symbolica
was 1.545 us, reducing the exact-f64/Numerica gap from 4.41x to 1.91x while
remaining 9.6x faster than Symbolica. Hyperreal's direct exact-17 benchmark
moved from 290.51 ns to 115.72 ns, and the Rational row from 185.66 ns to
80.40 ns.

The cross-stack trace records `native-real-i64-kernel`,
`real/powi-i64/rational-exact`, and either `rational/powi/word-sized` or
`dyadic-denominator-shift`. Exact equivalence tests cover rational, radical,
symbolic, unknown-sign, negative-exponent, zero-domain, and `i64::MIN` cases;
none of the new dispatch decisions use a primitive approximation.

A follow-up retained path now records one byte of reuse evidence for exponents
two through five. The first call stays on the direct checked-integer kernel;
later calls use an explicitly ordered repeated-squaring chain whose edges are
already covered by bounded exact-product retention. Commutative multiplication
also checks the right operand's retained edge when the left slot is occupied.
No power-result cache is added, and the extra atomic fits existing padding so
`RationalData` remains 96 bytes.

The cold unique fifth-power sentinel is 234.46 ns, while its retained shared-base
counterpart is 59.16 ns. On the matched four-case Hyperlattice facade, exact-f64
inputs measure 43.44 ns and explicit rational inputs 75.84 ns, versus 83.31 ns
for Numerica 128 and 1.507 us for Symbolica. Both exact input forms now beat the
fixed-precision control, while the unrelated direct exponent-17 sentinel remains
at 83.67 ns.

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

### Retained forward-hyperbolic crossover and primitive views

Forward `sinh`, `cosh`, and `tanh` now keep the two-exponential structural
identity for ordinary exact rationals and symbolic values, where it remains the
cheapest exact graph. Exact rationals with magnitude at least eight instead use
one stable `expm1` identity; negative large inputs first enter odd/even symmetry
so the residual never approaches minus one. Integer multiples of an exact
logarithm still collapse to exact rationals before either generic route.

The public lossy `f64` edge now uses the lock-free cache slot already present in
every `Real`. Forward-hyperbolic results seed that view only when the input is an
exact rational with a finite primitive view. This accelerator is never consulted
by arithmetic, equality, sign, domain, or topology decisions, and every later
exact mutation invalidates it normally.

| Direct construction case | Before | After | Result |
| --- | ---: | ---: | ---: |
| `sinh(ln(2))` exact collapse | 258.31 ns | 140.12 ns | 45.75% faster |
| `cosh(ln(2))` exact collapse | 275.88 ns | 141.73 ns | 48.63% faster |
| `tanh(ln(2))` exact collapse | 546.35 ns | 281.85 ns | 48.41% faster |
| `sinh(1)` generic | 648.26 ns | 367.72 ns | 43.28% faster |
| `cosh(1)` generic | 589.89 ns | 337.13 ns | 42.85% faster |
| `tanh(1)` generic | 873.54 ns | 502.86 ns | 42.43% faster |

The retained trace records `generic-exp-identity` for `1/2`,
`generic-expm1-identity` for `20`, and one `negative-symmetry` dispatch per
operation for `-20`. A focused `perf` profile of the large-tanh output row found
the remaining cost in exact node construction, rational conversion, and
allocation; the cached primitive read itself no longer appears as a hot path.

### Thread-local tracing and paired word reduction

Dispatch recording enablement and counters now share thread-local ownership.
Concurrent recording scopes can reset and drain only their own events, removing
the global mutex from every diagnostic dispatch and eliminating cross-test
trace races. Hypercurve's two parallel dispatch tests passed 100 consecutive
default-harness runs after this change.

The exact rational aggregate layer now initializes a word LCM from its first
live denominator, uses native `u64` binary GCD when possible, and recognizes
2/5-smooth decimal denominators through a precomputed power table. Complex
products can request a paired word reducer that converts four components once
and returns `(ac - bd, ad + bc)` as two independently canonical rationals.
Overflow and non-word inputs fall back to the existing arbitrary-precision
signed-product reducers.

These scalar changes support both cold and retained object schedules. A new
observational reuse fact returns false on an isolated rational's first query and
true on subsequent borrowed queries, without consulting approximations or
altering exact arithmetic. Hyperlattice uses it to distinguish a 222.77 ns cold
dyadic complex product from a 138.77 ns retained borrowed product.

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
