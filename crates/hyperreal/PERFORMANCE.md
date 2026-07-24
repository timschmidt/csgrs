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

## Binary32 export fast path

Exact-rational `Real::to_f32_lossy` now narrows the allocation-free binary64
view at this explicitly approximate IO boundary instead of materializing a
Computable graph and refining it at binary32 precision. When a binary64 view is
already retained, narrowing is reused only if its adjacent binary64 values map
to the same binary32 result; midpoint-adjacent values retain the full fallback.
The binary64-cache build publishes the rational proposal for later rows without
adding storage. Overflow and signed-zero handling remain explicit, and property
tests compare cached and uncached results over 512 generated rationals plus a
retained midpoint regression.

In the 48,384-row CSG adapter corpus this reduced `f32` export from roughly
59.4 ms to 1.04 ms (98.2%) while preserving the finite interleaved row/index
contract used by the HyperMesh boundary. The HyperMesh large-buffer sentinel
records the current direct binary32 conversion cost independently.

## Fuzz coverage

The standalone `fuzz` workspace covers four runtime-bearing public families:

| Target | Exactness and API boundary |
| --- | --- |
| `rational_arithmetic` | Rational construction, every core arithmetic ownership path, inverse/powers, truncation/fraction decomposition, and exact dyadic conversion |
| `real_exact` | Exact Real arithmetic, fused dot/product-sum and dyadic line-intersection kernels, lazy-coordinate canonical boundaries, prepared determinant filters, certified facts/comparisons, exact conversion, and serde round trips |
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
- The fused exact-dyadic line-intersection kernel still canonicalizes both
  segment parameters for ordering, but its two point coordinates retain the
  exact affine numerator and shared determinant internally. A public
  numerator/denominator view, exact `Real` extraction, formatting, hashing,
  serialization, integer/dyadic query, root, aggregate dyadic specialization,
  or lossy IO view computes one canonical value in the node's existing primary
  cache slot. Sign and exact cross-product comparison need no reduction. The
  `RationalData` allocation remains 88 bytes.
- On Hypercurve's matched 21-sample, 50-iteration star1024 exact-contour row,
  this boundary measured 12.761 ms/operation versus the 13.517 ms clean
  checkpoint (5.6% faster). Three standalone one-operation runs used
  31,288--31,504 KiB peak RSS (31,416 KiB median), effectively flat against
  the 31,336 KiB clean anchor. The star64 dispatch trace recorded 40 fused
  intersections and no coordinate canonicalization during Boolean assembly.
  A 30-second AddressSanitizer `real_exact` campaign completed 85,423 inputs
  without failure after adding fused-intersection equivalence, canonical
  numerator/denominator, and serialization checks.
- Dense exact-dyadic dots align lane products in checked `u128` totals before
  allocating arbitrary-precision magnitudes. Wide aligned sums and shifts fall
  through to the former `BigUint` reducer. Hyperlattice's public vec3 dot fell
  from 453.02 ns to 227.11 ns and vec4 dot from 408.77 ns to 223.75 ns; both
  now beat Numerica 128 at 257.12 ns and 326.88 ns. The sparse vec4 control
  remains 51.45 ns.
- General rational reduction, add/subtract, and product-sum LCM construction
  keep pairs through `u128` in the native binary GCD and dispatch exactly
  mixed-width pairs (one operand through `u128`, one wider) to one full-width
  remainder followed by that native reducer. Balanced wide inputs stay on
  `BigUint`'s binary GCD. Routing balanced wide reductions through the custom
  cross-cancellation algorithm regressed a 500-operation cold-union profile
  from 1.28 s to 1.85--1.90 s. The mixed-width dispatch instead reduced the
  same alternating-input profile to 1.22--1.26 s (roughly 3--4%).
- Reduced dyadics with odd magnitude at most 63 and denominator through `2^63`
  share canonical immutable storage.
- Each product pair retains one exact multiplication result under a weak operand
  key that is queried in both commutative directions. The cache is bounded,
  cycle-free, and ignored by serialization; misses continue through the same exact
  word/BigUint kernels.
- Linear-result admission is adaptive. Storage sharing alone is not arithmetic
  reuse: a one-byte relaxed hint records the first borrowed observation, the
  second result is admitted, and later calls reuse it. An existing product or
  linear cache remains immediate reuse evidence. This keeps cloned one-shot
  operands allocation-light while preserving repeated-pair reuse. The hint fits
  existing `RationalData` padding,
  keeping that allocation at 88 bytes. The lazily allocated arithmetic cache holds
  up to three weak-keyed binary results (sums, directed differences, or secondary
  products) and, for shared values, one reciprocal and one
  opposite-sign result. Unary owners retain their result strongly while reverse
  edges are weak, so repeated division and negation reuse stable identities without
  ownership cycles. Five polymorphic entries leave room for both unary pairs and at
  least two binary results regardless of which operation initializes the box. A dedicated lazy
  slot retains an exact square factor and residual only after repeated
  square extraction is observed, without displacing those arithmetic entries. Sum and
  directed-difference entries can also
  occupy opposite operand caches and remain ignored by serialization. Occupied
  entries are checked before constructing a candidate. Cold wide-dyadic add/sub
  sentinels measured 87.78 ns; cold owned inversion measured 21.13 ns and retained
  inversion 7.45 ns; unique owned negation measured 7.12 ns and retained negation
  6.14 ns.
- Dense exact self-dots reuse those existing bounded product and linear entries
  only after the leading coordinate has already shown borrowed-arithmetic reuse.
  The first evaluation, vectors containing zero coordinates, and equal-but-distinct
  operands retain the aggregate zero-pruned reducer. On Hyperlattice's three-term
  regression sentinel this adaptive route reduced a retained self-dot from
  140.66 ns to 59.50 ns (57.7%) while the cold row remained effectively unchanged
  at 211.16 ns versus 210.75 ns.
- Repeated normalization can give one coordinate two stable product partners: itself
  in the norm and the shared inverse norm in the output. If both operands' primary
  product slots are occupied, one existing polymorphic binary entry now retains the
  secondary product. A single bit in the existing retained-facts byte limits a
  conflicted self-dot to one admission attempt; full boxes then fall back instead of
  repeatedly rebuilding an uncacheable schedule. `RationalData` remains 88 bytes,
  one-shot normalization does not allocate this secondary box, and serialization
  still ignores every accelerator. Integer-radical inversion also reuses the exact
  `Sqrt` class radicand instead of reconstructing an equal rational node, while the
  four-term self-dot uses a balanced sum and canonicalizes the `1 + 1` identity.
  Hyperlattice's retained wide-dyadic normalization sentinel fell from 1.5685 us to
  roughly 0.30 us; public exact-dyadic vec3/vec4 normalization now measures
  516.77 ns/585.67 ns versus Numerica 128 at 590.64 ns/676.71 ns.
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
- Finite binary64 inputs are imported as exact dyadic rationals so
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

Exactly imported binary64 vector coordinates are dyadic rationals, so their squared norm
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
fits existing padding, so `RationalData` remains 88 bytes.

Fresh 50-sample direct medians measured 165.32 ns for a fresh uniquely owned
`sqrt(90)` and 78.79 ns for its retained shared-input route, a 52.3% reduction.
More expensive repeated reductions fell from 433.54 ns to 75.33 ns for the exact
dyadic vector-norm sentinel and from 2.03 us to 54.31 ns for the non-dyadic
sum-of-squares sentinel. The cold fixture is deliberately outside the global
small-integer pool, so it also proves one-shot inputs do not receive an eager
cache allocation.

On Hyperlattice's matched four-case scalar facade, exact binary64-derived dyadics now measure
49.18 ns and explicit rationals 34.07 ns, versus 96.34 ns for Numerica 128 and
1.478 us for Symbolica. Both exact forms beat the fixed-precision control. The
four individual cases also beat Numerica, including the imported tiny dyadic
(83.26 ns versus 94.73 ns) and imported `e` (63.00 ns versus 100.52 ns).
Regression tests prove exact factor equality, stable retained identities,
cycle-free destruction, and coexistence with both unary and both linear pairs;
dispatch tracing records `reuse-observed` followed by `retained-reduction`.

### Retained experiment: exact dyadic/general product cancellation

Profiling one exact binary64-derived dyadic coordinate multiplied by a reciprocal vector-norm
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
376.76 ns to 161.11 ns for exact dyadic inputs and from 2.813 us to 210.93 ns for
explicit rational inputs. The Numerica 128 control was 84.53 ns and Symbolica
was 1.545 us, reducing the exact-dyadic/Numerica gap from 4.41x to 1.91x while
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
`RationalData` remains 88 bytes.

The cold unique fifth-power sentinel is 234.46 ns, while its retained shared-base
counterpart is 59.16 ns. On the matched four-case Hyperlattice facade, exact-dyadic
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

### Common-scale exact complex quotients

Exact complex division now has a scalar-owned quotient kernel. Four rational
components are converted once, each complex pair is lifted to one common
integer scale, and the conjugate product and norm are formed before either
output is canonicalized. Dyadic inputs use exponent alignment and shift
cancellation; arbitrary word denominators use two LCMs and cross-cancel the
left/right scales before multiplication. Equal denominators and equal scales
bypass GCD entirely. Wider values fall back to the existing arbitrary-precision
conjugate-product, norm, and exact-division operations.

This changes neither division-by-zero semantics nor representation exactness.
In Hyperlattice's 50-sample comparison, exact-dyadic-input complex division measured
373.81 ns and decimal-rational division 474.95 ns, versus 615.42 ns for
Numerica128 and 22.03 us for Symbolica. Borrowed division measured 349.79 ns
and 457.13 ns for the two Hyperreal inputs versus 503.22 ns for Numerica128.

### Direct dyadic approximate filter views

The borrowed rational-to-`f64` view now recognizes power-of-two denominators
within binary64's normal exponent range before computing an exact shifted
most-significant bit. The denominator's certified shift directly constructs
the exact binary64 power of two, avoiding both the shifted BigUint comparison
and a second BigUint conversion. Non-dyadic rationals, extreme exponents, and
all general Computable values retain the previous path. This remains only an
approximate filter view: predicate error bounds and exact fallbacks are
unchanged.

A preserved release binary and the candidate each prepared 500 sphere/box
arrangements with fresh thread-local Boolean state. Retired instructions were
stable to 0.01% or better across seven runs:

| exact Boolean | previous view | direct dyadic view | result |
| --- | ---: | ---: | ---: |
| union | 12,591,702,744 | 12,151,185,037 | 3.50% fewer instructions |
| difference | 10,131,421,412 | 9,870,401,358 | 2.58% fewer instructions |

One-operation fresh-process measurements, after subtracting identical input
construction and process overhead, confirmed 3.21% fewer union instructions
and 2.38% fewer difference instructions. The matched five-sample kernel run
still identifies cold CSGRS-to-CGAL gaps for both operations, while retained
CSGRS extraction is 17.75x faster for union and 20.46x faster for difference;
CSGRS also exceeds the tight OpenCascade rows at both temperatures.

Validation passed all 524 all-feature library tests and every integration,
oracle, benchmark-smoke, strict Clippy, warning-denied rustdoc, benchmark-build,
and fuzz-build gate. AddressSanitizer campaigns completed 1,000 rational,
1,292 Real-exact, 2,439 Real-elementary, and 1,000 Computable executions without
failure. All-feature Hyperlattice, Hyperlimit, Hypersolve, Hypercurve, and
Hypermesh suites passed, as did all 304 downstream CSGRS library tests.

### Combined dyadic exponents and one-reduction rational means

Normal dyadic filter views now borrow the numerator's leading limbs, retain a
sticky bit through a round-to-odd reduction, and combine numerator and
denominator exponents before constructing binary64 scale factors. This handles
ratios whose numerator and denominator are individually outside binary64 while
avoiding BigUint conversion, `powi`, and division on the normal-result path.
Subnormal and non-dyadic inputs retain the general exact-magnitude fallback. A
5,000-case generated oracle compares the resulting bits with a 53-bit GMP/MPFR
rounding for 65- to 2,048-bit dyadic numerators.

Against the preceding direct-denominator view, seven runs of 500 fresh
sphere/box arrangements measured:

| exact Boolean | direct denominator | combined exponent | result |
| --- | ---: | ---: | ---: |
| union | 12,151,588,013 | 11,171,830,223 | 8.06% fewer instructions |
| difference | 9,866,988,732 | 8,914,639,640 | 9.65% fewer instructions |

`Rational::mean_refs` adds a scalar-owned exact aggregate for borrowed values.
It scans once to select a dyadic, equal-denominator, or general LCM schedule,
incorporates the element count into the final denominator, and canonicalizes
only the result. Dyadic, equal-denominator, mixed-LCM, zero, wide, and empty
schedules are checked against expanded exact arithmetic. On a four-value exact
rational mean, Hyperreal measured 222.36--224.87 ns versus 231.29--232.21 ns
for GMP.

Validation passed all 526 all-feature library tests and every integration,
oracle, benchmark-smoke, strict Clippy, warning-denied rustdoc, benchmark-build,
and fuzz-build gate. AddressSanitizer campaigns completed 1,000 rational,
1,300 Real-exact, 2,437 Real-elementary, and 1,000 Computable executions
without failure. All-feature Hyperlattice, Hyperlimit, Hypersolve, Hypercurve,
and Hypermesh suites passed, as did all 304 downstream CSGRS library tests.

### Reused dyadic product-sum plans

Fixed signed product sums now retain the denominator shifts, maximum shift, and
wide-reducer decision produced while classifying their factors. The exact
dyadic reducer and ordering comparison consume that plan directly instead of
rescanning every denominator after word arithmetic is rejected. Non-dyadic
inputs also avoid repeating a failed dyadic scan before entering the general
LCM reducer. The selected reducer and its exact result are unchanged.

A preserved release binary and the candidate each prepared 500 fresh
sphere/box arrangements. Across seven runs, the combined plan reduced both
instructions and cycles:

| exact Boolean | previous instructions | planned instructions | instruction result | cycle result |
| --- | ---: | ---: | ---: | ---: |
| union | 9,105,085,369 | 9,000,801,433 | 1.15% fewer | 1.91% fewer |
| difference | 7,526,352,842 | 7,431,776,799 | 1.26% fewer | 1.80% fewer |

In a matched 15-sample cross-kernel run, cold CSGRS difference measured
1.882 ms versus 1.887 ms for CGAL EPECK, while union measured 2.677 ms versus
2.409 ms. Retained CSGRS difference and union were respectively 19.93x and
13.68x faster than CGAL; the cold union remains the next measured gap.

Validation passed all 526 all-feature library tests and the complete
all-target integration, oracle, and benchmark-smoke gate, plus strict Clippy,
warning-denied rustdoc, and every fuzz-target build. AddressSanitizer campaigns
completed 1,000 Rational and 1,293 Real-exact executions without failure. All
1,067 executed Hypermesh tests and 369 downstream CSGRS all-feature library
tests passed.

### Native operation GCD for word pairs

The rational-operation reducer now keeps pairs whose magnitudes both fit
`u128` in the existing native binary GCD instead of converting the same values
back through `BigUint`'s binary GCD. Mixed-width pairs retain their single wide
remainder, and balanced arbitrary-precision pairs retain the backend reducer.
Direct 500,000-operation profiles across generated 32-, 64-, 96-, and 128-bit
pairs used 7.7--14.7% of the backend instructions and 10.8--22.4% of its
cycles, including the identical pair-to-`u128` classification cost.

In the alternating-input CSGRS guard, 500 exact sphere/box operations showed
the downstream effect without an arrangement-cache hit. Across 15 runs, union
instructions fell from 10,175,942,673 to 10,034,912,735 (1.39%) and cycles
fell 0.23%. Difference instructions fell from 8,022,265,477 to 7,935,606,012
(1.08%), with cycles neutral (+0.01%).

Validation passed all 526 all-feature library tests and the complete all-target
gate, strict Clippy, warning-denied rustdoc, and every fuzz-target build.
AddressSanitizer campaigns completed 1,000 Rational and 1,124 Real-exact
executions without failure. Hypermesh's full all-target/all-feature suite and
all 370 downstream CSGRS library tests plus integrations passed.

When exactly one native operand fits `u64`, the word GCD now takes one exact
`u128` remainder and finishes in the existing `u64` binary reducer. A
power-of-two small operand needs only trailing-zero counts. Balanced two-limb
operands retain the subtraction/shift reducer, avoiding repeated compiler-rt
division. Direct 500,000-operation profiles of 96- and 128-bit magnitudes
against 16-, 32-, 48-, and 64-bit divisors used 20.7--39.1% of the former
instructions and 14.9--33.4% of its cycles. In the post-shared-output CSGRS
guard, union instructions fell another 0.14% and cycles 0.78%; difference
instructions fell 0.14% with cycles neutral.

### Structural operation GCD certificates

The rational-operation GCD now resolves mixed-width identity and power-of-two
operands without a full-width remainder or binary reduction, and returns equal
wide operands directly. Word pairs retain the identical native dispatch;
general mixed pairs retain one exact remainder, and general wide pairs retain
the backend binary reducer. Dispatch tracing now reports the selected algorithm
instead of labeling every operation GCD as backend binary.

One exact rational-offset sphere/box union issued 759 operation GCDs: 129 wide
identity, 189 wide power-of-two, seven equal-wide, 85 native-word, 24 mixed
wide/word, and 325 general backend-binary calls. Thus the structural proofs
removed 325 backend entries without changing any exact result. A broader
one-remainder experiment for balanced wide pairs was rejected after regressing
union instructions 2.19% and difference instructions 1.37%.

Eight alternating counter runs each performed 500 fresh, globally shifted 8x4
sphere/box operations:

| operation | backend-only instructions | structural instructions | instruction result | cycle result |
| --- | ---: | ---: | ---: | ---: |
| union | 8,037,894,768 | 7,975,123,366 | 0.78% fewer | 0.45% fewer |
| difference | 6,788,466,262 | 6,745,313,386 | 0.64% fewer | 0.68% fewer |

Heap profiles over 100 unions fell from 1,909,758 to 1,884,759 allocations,
removing 24,999 allocations, or 249.99 per operation.

Validation passed the 527-test all-feature library gate and its complete
all-target integration, oracle, and benchmark-smoke matrix; the 460-test default
library gate plus integrations and doctests; strict Clippy; warning-denied
rustdoc; every fuzz-target build; and 20-second AddressSanitizer fuzz campaigns
covering 488,852 rational-arithmetic and 92,656 exact-real cases. Downstream
validation passed Hypermesh's 962-test all-feature and benchmark-smoke gate,
no-default build, strict Clippy, warning-denied rustdoc, benchmark and fuzz-target
builds, locked release WebAssembly build, and 371-case AddressSanitizer Boolean
pipeline campaign, followed by CSGRS's 370-test all-feature library gate and all
integration suites.

### Borrowed dyadic comparison digits

Exact rational comparison no longer materializes a shifted `BigUint` when two
dyadic cross-products have the same bit width. A most-significant-first iterator
combines adjacent borrowed `u64` digits with the residual bit shift, while common
whole-value shifts cancel before iteration. Unequal bit widths and equal
denominators keep their existing constant-time exits. Dispatch tracing records
the selected path as `dyadic-borrowed-digits`.

A 5,000-case generated oracle plus shifts bracketing consecutive 64-bit
boundaries compares the borrowed walk with materialized arbitrary-precision
shifts. The new public GMP comparison row measured a 261-bit dyadic ordering at
17.334 ns for Hyperreal versus 37.180 ns for GMP, making Hyperreal 53.4% faster.

Eight order-alternating counter pairs each performed 500 fresh, globally shifted
8x4 sphere/box operations:

| operation | shifted allocation instructions | borrowed-digit instructions | instruction result | cycle result |
| --- | ---: | ---: | ---: | ---: |
| union | 4,817,143,439 | 4,730,655,727 | 1.80% fewer | 1.31% fewer |
| difference | 4,165,783,610 | 4,077,290,648 | 2.12% fewer | 0.83% fewer |

Heap profiles over 100 unions fell from 1,235,801 to 1,158,120 allocations,
removing 77,681 allocations, or 776.81 per operation (6.29%).

Validation passed the 529-test all-feature library gate and complete all-target
integration, oracle, and benchmark-smoke matrix; the 461-test default library
gate plus integrations and doctests; strict Clippy; warning-denied rustdoc; all
benchmark and fuzz-target builds; and 20-second AddressSanitizer campaigns over
491,333 rational-arithmetic and 90,527 exact-real cases. Downstream validation
passed Hypermesh's 962-test all-feature/all-target gate, no-default build, strict
Clippy, warning-denied rustdoc, benchmark and fuzz-target builds, locked release
WebAssembly build, and 369-case AddressSanitizer Boolean pipeline, followed by
CSGRS's 370-test all-feature library gate and every integration suite.

### Prepared projected rational point queries

Certified 2D line filters can now consume a `PreparedRationalPoint3Query` and
select two coordinate axes without reconverting the same arbitrary-precision
rationals. Fixed line endpoints can be projected from the same retained
value/error intervals. Invalid projections and intervals that cannot certify a
sign still return `None`; the caller's exact predicate remains authoritative.
The existing affine four-term query constructor retains its direct conversion
path so unrelated point-plane predicates do not pay for the new abstraction.

Eight alternating counter runs each performed 500 fresh, globally shifted
sphere/box operations through downstream CSGRS:

| exact Boolean | repeated-conversion instructions | prepared-point instructions | instruction result | cycle result |
| --- | ---: | ---: | ---: | ---: |
| union | 9,955,432,140 | 9,516,772,993 | 4.41% fewer | 4.04% fewer |
| difference | 8,488,857,196 | 8,487,528,295 | 0.02% fewer | neutral |

In the union profile, `Rational::to_f64_lossy` fell from 4.91% to 2.09% self
time. Heap profiles added only 45 allocations over 50 unions (0.9 per
operation) for the prepared-query vector. A focused regression compares direct
and prepared positive, negative, and uncertain line signs and rejects invalid
axis projections.

Validation passed the complete default and all-feature test suites, all targets,
the explicit GMP API-coverage audit, Clippy with warnings denied, warning-clean
documentation, benchmark compilation, and every fuzz-target build. Twenty-second
ASAN campaigns completed 505,059 `rational_arithmetic` executions and 92,851
`real_exact` executions without failure. Downstream Hypermesh passed its full
test/build/lint/documentation/benchmark/WASM matrix plus 365 ASAN Boolean-pipeline
executions, and downstream CSGRS passed all 370 library tests and every integration
test.

### Canonical primitive small integers

Every signed and unsigned primitive `Rational::from` conversion now classifies
its magnitude before materializing a `BigUint`. Zero and one retain their
identity constructors, magnitudes 2 through 64 reuse the existing canonical
small-integer storage, and larger primitive values materialize exactly once.
`Rational::new(i64)` enters through the same constructor, so primitive widths
and signs no longer implement different storage policies. Exact value and
storage-identity tests cover positive and negative conversions through `u8`,
`u128`, `i8`, `i128`, and `Rational::new`.

Matched 30-sample Criterion measurements show the allocation-free retained
path:

| constructor | previous | canonical primitive | result |
| --- | ---: | ---: | ---: |
| `Rational::from(4_u8)` | 38.53 ns | 4.51 ns | 88.3% faster |
| `Rational::from(-4_i8)` | 36.64 ns | 5.15 ns | 85.9% faster |
| `Real::from(4_u8)` | 50.17 ns | 16.65 ns | 66.8% faster |
| `Real::from(-4_i8)` | 49.92 ns | 17.76 ns | 64.4% faster |

### Aggregate dense 4x4 rational inverse

`Real::exact_rational_matrix4_inverse_known_exact` now gives fixed-matrix
callers one scalar-owned cofactor operation, analogous to the existing 3x3
entry point. Twelve minors, the determinant, and sixteen cofactors stay in the
`Rational` layer and are wrapped only when the final matrix is returned. The
operation is exact and reports `DivideByZero` for a singular matrix; it adds no
cache or object storage.

In Hyperlattice's matched comparison this reduced exact-dyadic dense 4x4
inverse from 23.016 us to 21.288 us and the explicit-rational row from about
7.85 us to 6.877 us. Checked and abort-aware exact-dyadic inversion now measure
20.987 us and 21.108 us. A common-integer-scale prototype was rejected after
heterogeneous binary64 exponents widened its intermediates and regressed the
public row to 83.3 us.

### Certified dyadic two-factor product sums

Fixed determinant and cofactor rows use signed sums whose terms each contain
two rational factors. Prepared matrix callers now classify the complete input
once and route certified binary64-derived rationals directly to the dot
reducer's exact shift-aligned accumulator. The generic product-sum entry point
does not probe this specialization, so authored non-dyadic rationals retain the
word/LCM schedule without repeated failed denominator scans.

Downstream matched Hyperlattice medians moved exact-dyadic mat3 reciprocal from
5.537 us to 4.146 us and checked inverse from 4.794 us to 4.226 us. Mat4
reciprocal moved from 21.676 us to 18.257 us and checked inverse from 21.229 us
to 18.756 us. Explicit-rational controls remained 2.860/3.248 us for mat3 and
7.432/7.456 us for mat4. The mixed-denominator six-term scalar control improved
from 476.94 ns to 459.67 ns after removing the exploratory generic probe.

### GCD operand trace and ordered two-limb tail

`RationalTraceStats` now groups exact GCD operands into both-`u64`, mixed
`u64`/`u128`, both-`u128`, and arbitrary-precision buckets. This preserves the
existing aggregate and peak-bit counters while making cross-crate traces useful
for choosing a native or wide algorithm from measured operand distributions.
Hypercurve's exact star64 Boolean reports 465 GCDs: 80 mixed native-width, 80
balanced two-limb, and 305 arbitrary-precision calls, with a 359-bit peak.

The balanced `u128` Euclidean tail now orders its odd inputs before taking the
first remainder. Previously an ascending pair paid for a compiler-runtime
`u128` remainder whose result was necessarily the smaller input, only to swap on
the next iteration. The change removes that call and leaves the tuned remainder-
to-`u64` plus binary-GCD schedule and canonical result unchanged. In the same
downstream twenty-operation Callgrind workload, the complete exact Boolean path
fell from 102,991,860 to 99,745,892 instructions after this scalar fix, certified
dyadic determinant dispatch, and retained broad-phase boxes. A randomized 20,000-
pair word-GCD oracle, the complete 542-test all-feature suite, the 471-test
no-default suite, GMP API coverage, and all downstream crate suites passed. A
1,000-run nightly AddressSanitizer rational-arithmetic campaign reached 1,735
coverage points and 4,317 feature edges without failure.

The retained 128-bit crossover sentinel measures the selected native path at
458.54 ns versus 5.333 us for the otherwise identical allocation-heavy BigUint
Euclidean reference. The wider 192-, 512-, 1,024-, and 4,096-bit sentinels remain
separate, so a future word-tail change cannot hide behind a Lehmer improvement.

### Certified dyadic quotients and parameterized points

Callers that have already classified both operands as exact dyadics can now divide
without first expanding their power-of-two denominators. The scalar kernel
cross-cancels the two stored magnitudes, applies only the net binary shift, and
constructs the canonical rational directly. The related 2D aggregate returns a
parameter and `origin + parameter * delta` by forming two dyadic affine numerators
before quotient construction. This removes the two canonical general-rational
products and additions that a line intersection otherwise creates for its point.

On small cached operands, the standalone exact quotient measured 82.02 ns versus
58.74 ns for the benchmark's 128-bit MPFR approximation. The parameter-plus-point
aggregate measured 366.75 ns versus 150.93 ns for MPFR. Those rows compare dispatch
overhead, not identical semantics: Hyperreal returns canonical unbounded exact
rationals, while the MPFR row rounds at 128 bits.

GMP/MPFR remains a competitive benchmark and test-oracle dependency through the
development-only `rug` entry. It is absent from Hyperreal's normal release graph;
the retained quotient and aggregate are implemented entirely by Hyperreal's native
rational and magnitude kernels.

In downstream Hypercurve star64 intersection, the combined dyadic determinant,
quotient, and point schedule reduced a twenty-operation Callgrind run from
99,745,892 to 81,698,829 instructions (18.09%). Prepared-region DHAT allocation
fell from 11,640,961 bytes in 100,732 blocks to 10,608,905 bytes in 83,760 blocks
(8.87% fewer bytes and 16.85% fewer blocks); peak live heap fell 9.87%. The trace
contains no expanded division-cross numerator or denominator events, and final
rational reductions fell to 284. Exhaustive signed dyadic scales are compared with
general division, and the `real_exact` fuzzer differentially checks both new APIs.

### Native-word dyadic quotient results

When both stored dyadic magnitudes fit `u128`, the certified quotient kernel now
keeps cross-cancellation and the net power-of-two scale in native words. It enters
the existing reduced-word result constructor only after the final numerator and
denominator are known. A checked shift retains the arbitrary-precision path for a
word-sized input whose scaled result no longer fits `u128`; an already-wide input
takes the same fallback. Tests compare both routes with general exact division, and
the dispatch trace locks one native and one wide-result selection.

In the downstream twenty-operation Hypercurve star64 workload, stripped Callgrind
fell from 75,700,486 to 74,297,181 instructions (1.85%). Prepared-region DHAT
allocation fell from 83,735 to 81,575 blocks (2.58%); reads fell 1.39% and writes
0.52%, while allocated bytes were effectively flat. A matched 31-sample,
500-iteration run measured ordinary exact region output at 223.696 us/iter versus
29.410 us for the fastest approximate competitor. Hyperreal implements the branch
with its own `u128` and `BigUint` kernels; GMP/MPFR remains development-only
benchmark/oracle tooling and is absent from the normal release graph.

### Small-quotient two-limb GCD steps

The balanced `u128` Euclidean tail resolves quotients one through four with native
subtraction. For larger quotients it divides the high dividend limb by a strict
upper bound on the divisor, producing a quotient that cannot overshoot, then
corrects the residual once or retains the exact full-width remainder fallback. On
supported 64-bit targets, a `u128` remainder enters a compiler-runtime helper; the
star64 trace showed thousands of those calls even though the high-limb estimate is
usually exact or one low. Deterministic random pairs and adversarial quotient,
low-high-limb, and near-overflow cases are checked against the Euclidean reference.

The selected 128-bit GCD sentinel first fell from 458.54 ns to 169.64 ns with the
small-quotient steps, then to 138.75 ns with the bounded high-limb estimate (another
18.2%, or 69.7% overall). In the downstream exact star64 region workload, the latter
change reduces the twenty-operation Callgrind total from 77,709,243 to 76,985,290
instructions (0.93%). Prepared-region allocation remains exactly 9,608,934 bytes in
83,760 blocks. The implementation remains wholly native; GMP/MPFR is still confined
to competitive benchmarks and test oracles and is absent from the release graph.

### Paired affine determinant filter

The paired 2D affine filter converts four segment endpoints to certified exact-dyadic
`f64` views once, then exposes the two orientation directions independently so callers
retain a same-side early exit. Every returned sign uses the existing conservative
roundoff bound; an inconclusive determinant remains `None` for the homogeneous word
filter or arbitrary-precision exact fallback. Hypercurve's twenty-operation star64
trace reduced calls to the cached scalar conversion from 62,990 to 47,222 (25.0%) and
reduced the complete stripped Callgrind lane from 76,985,290 to 76,081,746 instructions
(1.17%). Prepared-region allocation blocks were unchanged, while DHAT reads fell 1.00%.
This filter and every fallback are implemented by Hyperreal; the development-only
GMP/MPFR oracle remains absent from the release dependency graph.

### Retained exact-dyadic affine determinant inputs

Geometry caches that have already proved a lossless binary64 view can now prepare
the paired affine determinant filter directly from those retained coordinates.
The filter applies the same conservative roundoff bound, and non-finite or
inconclusive arithmetic still returns no sign for the caller's exact word or
arbitrary-precision fallback. This avoids repeating eight scalar-cache loads for
each candidate without treating a binary64 result as topology evidence by itself.

In the downstream Hypercurve star64 intersection, reusing contour-level endpoint
certificates reduced a 5,000-operation hardware-counter run from 8,274,445,706 to
8,038,736,058 retired instructions (2.85%). The post-change symbol trace no longer
contains `Real::exact_dyadic_f64_cached` among functions above 0.2% self-time. A
15-sample run measured exact region output at 143.792 us/iter; the fastest
approximate competitor measured 27.348 us/iter, so the exact path remains 5.26
times slower.

### Repeated-denominator crossing reduction boundary

The downstream star1024 exact-contour profile now attributes 15.24% self time to
the tuned `u128` GCD, 5.46% to fixed-stack dyadic products, 2.72% to the fused
line-intersection wrapper, and 1.21% to the remaining compiler-runtime `u128`
division. The workload constructs 5,320 proper crossings and records 21,280
rational cancellations: two parameters and two coordinates per crossing. The
two point coordinates reuse the parameter divisor algebraically, but canonical
standalone rationals still require the residual direction/determinant
cancellations.

Matched end-to-end trials rejected several locally plausible replacements.
Stripping powers of two during the wide loop and shortening its small-quotient
decision tree each regressed about 0.7%. A direct x86-64 two-limb-by-one-limb
remainder regressed about 0.6%. A scalar Lehmer batch increased the selected
128-bit sentinel from roughly 134 ns to 368 ns. Moving the product cache out of
every rational node increased star1024 contour time from roughly 13.44 to 14.11
ms because retained arithmetic then needed extra allocations. Selecting the
line with the larger parameter cancellation and deriving coordinate
cancellations from a separately certified primitive direction were also slower
or flat after their proof costs were included.

These results set the next crossover: another standalone GCD algorithm is not
justified by this geometry trace. A material improvement must amortize
canonicalization across the shared determinant or retain a compact crossing
form until the event consumer actually needs independent canonical rationals.
Any such representation must preserve exact equality, ordering, and public
standalone `Rational` output; GMP remains benchmark/oracle-only.

### Direct fixed-stack product accumulation

Dyadic determinant, dot-product, parameter-ordering, and affine-point kernels
previously shifted every native or wide product into a zeroed six-limb temporary
and then added all six limbs into the destination. The shared accumulator now
checks the occupied shifted range first, synthesizes each aligned limb directly
at its destination, and propagates an arithmetic carry only while it remains
live. This removes one full stack temporary and one unconditional six-limb pass
without changing the fixed-width admission boundary or arbitrary-precision
fallback.

Matched downstream Hypercurve contour trials measured star64 at 53.849 versus
55.065 us/iteration (2.2% faster), star256 at 448.589 versus 467.143
us/iteration (4.0%), and star1024 at 7.203 versus 7.550 ms/iteration (4.6%).
In the 500-operation star1024 profile, accumulator self-time fell from 17.46% to
12.64%. The star64 heaptrack workload remained exactly 59,096 allocations and
697.47 KiB peak heap; eleven standalone star1024 runs moved median process RSS
from 24,172 to 23,928 KiB, within the expected process-layout variance and with
no memory regression.

Boundary tests compare narrow and wide-narrow aligned products, carry
propagation, and overflow fallback with `BigUint`. The 561-test all-feature
Hyperreal suite, the complete downstream Hypercurve all-feature suite, strict
Clippy, warning-denied rustdoc, and a 10,000-run AddressSanitizer region-Boolean
campaign (5,902 coverage points and 18,712 feature edges) passed.

### Normalized fixed-width dyadic quotient ordering

Retained line intersections store each parameter as a signed `u128` dyadic
numerator divided by a signed `u128` dyadic denominator. Comparing two such
parameters previously sent both cross-products through the general six-limb
signed-product accumulator even though each magnitude product is bounded to
256 bits. The compact comparator now forms those products directly in four
`u64` limbs. Different effective bit lengths decide immediately; equal lengths
are shifted conceptually to a common most-significant bit and compared one limb
at a time. Sign reversal is applied only after the magnitude order is known.
No approximation or arbitrary-precision materialization is involved.

The ordinary accumulator route remains available because its smaller inlined
body has better instruction locality for short sorts. Downstream Hypercurve
selects the normalized route only for certified crossing sets of at least
1,024 events. Two alternating paired star1024 contour trials measured
6.765--6.948 ms/iteration with normalized products versus
7.219--7.316 ms/iteration through the preceding accumulator route, a 5--6%
end-to-end gain. Star64 and star256 remain on the previous path and were neutral
across repeated paired runs.

The 500-operation star1024 profile attributes 6.63% self-time to the normalized
comparator while accumulator self-time falls from 12.60% to 6.07%, leaving only
the final exact point-construction uses. A 20,000-case deterministic oracle
compares both compact implementations against `BigUint` cross-products across
both signs and denominator shifts through 2,047. The complete 562-test
all-feature Hyperreal suite, downstream Hypercurve suite, strict Clippy,
warning-denied rustdoc, and the 10,000-run AddressSanitizer region-Boolean
campaign (5,906 coverage points and 18,772 feature edges) pass.

### Reused affine determinant certificates

The certified binary64 affine determinant filter previously classified each
rounded product and the final difference independently even after establishing
a normal aggregate error bound. The product-magnitude sum and its scaled error
bound are sufficient: when the latter is normal it also dominates the absolute
rounding error of a subnormal product or difference, while an overflow makes
the magnitude non-normal. Removing the redundant intermediate classifications
preserves the same conservative topology boundary and admits only cases whose
absolute underflow error is already covered.

The reusable two-dimensional filter now also retains its checked line direction
instead of recomputing it for every query, and accepts two retained exact-dyadic
binary64 points in one call. Downstream Hypercurve prepares the fixed source
line once per broad-phase suffix and lazily prepares the other direction only
after the first same-side rejection fails. No approximate result is cached:
every returned sign remains independently bounded, and inconclusive directions
or points still enter the exact word and arbitrary-precision fallbacks.

Two alternating 21-sample star1024 contour comparisons measured
5.996--6.121 ms/iteration with reused certificates versus 6.115--6.251 ms at
the preceding exact prefix-sweep checkpoint, about 2% faster end to end. The
complete comparison matrix measured ordinary/prepared exact contours at
5.878/5.856 ms, versus 19.807 ms for Cavalier, 10.233 ms for `i_overlay`, and
10.604 ms for `geo`. Star64 remained neutral within run noise and star256
improved about 2%. The ordinary four-segment rectangle-union contour path also
improved from 5.471 to 5.367 us/iteration.

Tests retain the 20,000-case exact determinant oracle and add direct coverage
for retained binary64 pairs, direction overflow, safely dominated subnormal
products, and aggregate-underflow fallback. Ten downstream star1024 contour
operations remain exactly 1,104,312 allocations, 2,192 temporaries, and
16.58 MiB peak heap. The complete 563-test all-feature Hyperreal suite,
downstream Hypercurve suite, strict Clippy, and warning-denied rustdoc pass.
The 10,000-run AddressSanitizer differential Boolean campaign completed
without failure at 5,892 coverage points and 18,786 feature edges.

### Prepared exact-dyadic line point plans

Compact and wide fused line-intersection kernels previously converted both
source lines to native dyadic words and rebuilt both exact deltas for every
crossing. Hypercurve's retained candidate stream is already grouped by its
first segment, so the first endpoint words and delta now live in a
`PreparedExactDyadicLine2` for that segment's entire group. The second line,
three determinants, and both final rational coordinates remain independently
computed per crossing. Inputs outside the word-sized endpoint/delta envelope
still return `None` for the unchanged general exact path.

Two alternating 21-sample star1024 contour trials measured
5.733--5.737 ms/iteration with the prepared exact line versus
5.865--5.880 ms at the preceding predicate-reuse checkpoint, a further
2.2--2.5% improvement. Dedicated smaller trials measured 53.155 us at star64
versus 53.560 us, and 0.423 ms at star256 versus 0.434 ms. The ordinary
four-segment rectangle path remained healthy at 5.339 us versus 5.487 us.

The complete comparison matrix measured ordinary exact star1024 contours at
5.816 ms, and a dedicated prepared-contour rerun measured 5.863 ms. Finite
competitor rows were 19.590 ms for Cavalier, 10.316 ms for `i_overlay`, and
10.275 ms for `geo`. The preparation is stack-only: ten star1024 contour
operations remain exactly 1,104,312 allocations, 2,192 temporaries, and
16.58 MiB peak heap.

The existing 512-case compact and 256-case wide exact-arithmetic oracles now
run every admitted crossing through both one-shot and prepared plans, compare
both retained parameters and coordinates, and cover endpoint-word overflow
fallback. Both complete all-feature suites, strict Clippy, and warning-denied
rustdoc pass. The 10,000-run AddressSanitizer differential Boolean campaign
completed without failure at 5,892 coverage points and 18,825 feature edges.

### Direct binary64 dyadic word plans

The exact line AABB cache already retains each endpoint as an exact finite
binary64 value, but crossing construction still traversed and canonicalized
four retained `Rational` values to recover the same dyadic words. The direct
path now decodes sign, significand, and exponent from the IEEE-754 bits,
normalizes the power-of-two denominator, and enters the unchanged compact or
wide determinant plan. Non-finite values and endpoints outside the native-word
envelope return `None` for the existing exact fallback.

Two alternating 21-sample star1024 contour comparisons measured
5.634 and 5.542 ms/iteration on the direct path versus 5.769 and 5.686 ms at
the prepared-rational checkpoint, a 2.3--2.5% improvement. Seven-run hardware
counters for 320 fixed iterations fell from 8.247 to 8.080 billion cycles,
26.274 to 25.969 billion instructions, and 4.443 to 4.345 billion branches.
Matched star64 and star256 trials measured 51.242 us and 0.416 ms versus
52.424 us and 0.435 ms.

The complete star1024 matrix measured ordinary/prepared exact contours at
5.656/5.445 ms, versus 19.809 ms for Cavalier, 10.139 ms for `i_overlay`, and
10.074 ms for `geo`. Per-operation heap behavior and the 16.58 MiB peak are
unchanged. The profiled process has one additional 240-byte startup temporary,
so its ten-operation totals are 1,104,313 allocations and 2,193 temporaries
versus 1,104,312 and 2,192 at the preceding binary.

A 20,000-pattern randomized test proves that direct finite-binary64 decoding
matches the canonical rational word and rejection envelope. The compact
512-case crossing oracle exercises the direct plan too; a separate
approximately 200-bit determinant case covers the wide direct plan, along with
non-finite and oversized-endpoint fallback. Both complete all-feature suites,
strict Clippy, and warning-denied rustdoc pass. The 10,000-run AddressSanitizer
differential Boolean campaign completed without failure at 5,891 coverage
points and 18,881 feature edges.

### Native-word two-product sums

Compact line determinants and affine point numerators previously entered the
384-bit stack accumulator even when both aligned products and their signed sum
fit `u128`. They now try checked native multiplication, alignment, signed
addition/subtraction, and dyadic normalization first. Any product, shift, sum,
or cancellation result outside that envelope immediately reruns the unchanged
stack path; wide and arbitrary-precision behavior is unaffected.

In two same-layout alternating 21-sample star1024 contour comparisons, the
native path measured 5.575 and 5.653 ms/iteration versus 5.732 and 5.736 ms,
an improvement of 2.7% and 1.4%. Seven-run counters over 320 iterations
reduced instructions from 25.969 to 25.392 billion, branches from 4.345 to
4.152 billion, and branch misses from 23.52 to 21.04 million; cycle totals
were equal within run noise. Reversed star64 trials improved 2.7--3.0%, while
ordinary rectangle contours fell from 5.768 to 5.223 us.

Making the fixed two-term shape explicit instead of routing it through a
const-generic loop then reduced a 31-sample star1024 contour trial from
5.550 to 5.403 ms. Seven-run counters fell from 8.119 to 7.792 billion cycles
and 25.392 to 25.190 billion instructions. Final star64 and star256 trials
measured 49.806 us and 0.392 ms; rectangle contours measured 5.306 us.

The complete final star1024 matrix measured ordinary/prepared exact contours
at 5.661/5.376 ms, with the ordinary row noisier than its dedicated trial.
Competitors measured 19.857 ms for Cavalier, 10.119 ms for `i_overlay`, and
10.117 ms for `geo`. Heaptrack is unchanged from the direct-binary64
checkpoint: ten operations use 1,104,313 allocations, 2,193 temporaries, and
16.58 MiB peak heap.

A 20,000-case randomized arithmetic oracle compares every admitted native sum
with the 384-bit result and forces checked overflow deferrals. The compact and
wide line-intersection oracles cover the integrated fallback. Both complete
all-feature suites, strict Clippy, and warning-denied rustdoc pass. The
10,000-run AddressSanitizer differential Boolean campaign completed without
failure at 5,891 coverage points and 18,933 feature edges.

The next exact-line checkpoint adds compact retained point carriers alongside
the existing eager APIs. A carrier stores the two fixed-stack affine
numerators and their shared determinant denominator without allocating or
canonicalizing two `Rational` values. Its explicit `materialize` boundary
replays the same exact quotient construction. Compact and wide prepared-first
entry points let downstream topology retain this representation when point
coordinates are not immediately observed, while the original eager functions
keep their established code-generated hot path.

Randomized compact and wide intersection oracles compare carrier
materialization with the eager exact result, including sign, normalization,
and fallback cases. Layout guards cap the compact carrier at 160 bytes and the
wide carrier at 176 bytes. In Hypercurve's fixed star1024 exact-contour
workload, consuming these carriers lazily reduced ten-operation heaptrack
allocations from 1,104,313 to 464,773 and lowered the contour median from the
5.403 ms scalar checkpoint to about 3.95 ms. Exact parameters and arbitrary
precision fallback behavior are unchanged.

This API is deliberately a substrate for geometry carriers rather than a
general replacement for `Real`: callers that immediately need independent
canonical rationals should continue to use the eager point functions. Further
work is driven by whole-Hypercurve profiles, especially large complex
polynomial/rational Bézier, arc, spline/NURBS, offset, and region workloads.

### Primitive integer ratios and exact integer quotients

Elimination polynomials and projective coordinate tuples are invariant under
one shared nonzero scale. `Rational::clear_common_denominator_slice` now exposes
the existing exact common-denominator machinery for dynamic coefficient sets,
while `primitive_integer_ratio` additionally removes their integer content.
Both retain every component and apply one positive common scale; empty and
all-zero inputs remain shape-preserving.

Fraction-free Bareiss elimination also guarantees that its integer numerator is
exactly divisible by the previous integer pivot. The new
`checked_exact_integer_quotient` primitive verifies that contract with one
division/remainder operation and constructs the integer result directly. A
noninteger operand, zero divisor, or nonzero remainder returns `None`, allowing
callers to retain their general exact-division fallback.

In Hypercurve's one-cell all-family exact `CurveRegion2` workload, normalizing
rational-image resultants to primitive integer matrices and consuming the
exact quotient reduced ten-run instructions from 320,660,631 to 189,533,986
(40.9%). Median ordinary end-to-end time fell from 26.628 ms to 18.154 ms and
pair preparation from 16.830 ms to 9.016 ms. Exhaustive small signed quotient
tests, disparate-denominator image replay, and the full downstream topology
suite retain the same exact results.

`checked_exact_integer_cross_difference_quotient` extends that boundary to the
whole fraction-free recurrence. It computes `(a*b - c*d)/e` directly from
integer magnitudes, verifies the final divisibility, and returns `None` for
fractional operands, a zero divisor, or a remainder. Hypersolve retains its
general exact-arithmetic fallback for every rejected input. Exhaustive small
signed cross differences cover all sign, zero, divisible, and remainder cases.

Using the fused primitive in every dense, multi-right-hand-side, and sparse
Bareiss update reduced the same downstream Hypercurve sentinel from
151,620,313 to 131,393,603 instructions (13.3%). Its eleven-run ordinary
complete median fell from 14.994 ms to 12.985 ms and pair preparation from
8.682 ms to 7.844 ms, with identical exact topology.
The complete all-feature test and API-coverage suites, strict all-target
Clippy, and warning-denied rustdoc passed.

### Two-observation linear cache admission

The former linear-result policy treated `Arc::strong_count() > 1` as evidence
that an exact sum or directed difference would recur. Ownership clones are
common in immutable geometry carriers, however, and do not imply that the same
operand pair will be evaluated again. Those one-shot pairs allocated a bounded
arithmetic cache and retained a result that was never queried.

Binary linear admission now depends on an arithmetic observation, an existing
product cache, or an existing linear cache. A first isolated operation records
the existing one-byte fact, a second operation admits its result, and later
calls reuse that exact identity. Product, inverse, negation, and square-reduction
caches retain their established policies. The dense self-dot helper already
knows that its schedule is recurring, so it explicitly primes the intermediate
products before its sum tree; its second and later results retain the same
identity behavior.

Matched 100-sample Criterion runs add explicit fresh-but-cloned sentinels. The
one-shot add fell from 111.87 ns to 93.85 ns (16.1%), and subtraction from
116.40 ns to 97.04 ns (17.1%). Existing retained add/sub rows measured
8.37/8.94 ns, while the fresh unshared wide-dyadic controls remained within
their noise threshold.

On Hypercurve's one-cell all-family exact Boolean sentinel, the ten-run
instruction median fell from 77,532,932 to 76,301,712 (1.6%), 76.2% below the
original 320,660,631 baseline. Heaptrack allocations fell from 115,778 to
114,193, allocations beneath `retain_linear` from 7,971 to 3,718, and peak
heap from 1.96 to 1.53 MiB; temporary allocations measured 6,833. Eleven
ordinary runs had an 8.840 ms complete median, a 6.630 ms preparation median,
and a 0.376 ms exact-polyline projection median. Exact topology and checksum
were unchanged.

The complete Hyperreal, Hypersolve, and Hypercurve all-feature and
no-default-feature suites, formatting, warning-denied all-target Clippy and
rustdoc, and release WASM library builds passed. The downstream
AddressSanitizer region-Boolean replay completed all 2,509 executions at 5,903
coverage points and 19,183 feature edges; LeakSanitizer alone remained
disabled under ptrace.

### Fused exact integer scaled differences

`checked_exact_integer_scaled_difference` computes `a - b*k` directly from
the two integer magnitudes and a signed `i64` scale. It avoids constructing,
reducing, caching, and then discarding the intermediate rational product.
Fractional operands return `None`, allowing callers to preserve their general
exact-arithmetic fallback. Exhaustive small signed cases cover every sign and
zero arrangement; fractional rejection and a 192-bit case cover the remaining
representation boundaries.

Matched fresh 192-bit Criterion sentinels measured the composed multiply then
subtract at 310.13 ns and the fused operation at 102.50 ns, a 67.0% reduction.
Hypersolve now uses the fused primitive for every flat quotient-ring sample
entry `N - y*D`; failure preserves its established Sylvester-resultant
fallback. A zero scale or zero subtractand returns the already-immutable left
operand directly.

On Hypercurve's one-cell all-family exact Boolean sentinel, the ten-run
instruction median fell from 74,732,427 to 72,782,675 (2.6%), 77.3% below the
original 320,660,631 baseline. Heaptrack allocations fell from 112,178 to
108,842; temporary allocations fell from 6,833 to 6,236, peak heap remained
1.53 MiB, and peak RSS was 12.55 MiB. Eleven ordinary runs had a 7.672 ms
complete median, a 5.501 ms preparation median, and a 0.339 ms exact-polyline
projection median. Exact topology remained 9 candidate pairs, 48 fragments,
2 classifications, 4 decided operations, no blockers, and checksum 6.

The complete Hyperreal, Hypersolve, and Hypercurve all-feature and
no-default-feature suites, formatting, warning-denied all-target Clippy and
rustdoc, and release WASM library builds passed. The downstream
AddressSanitizer region-Boolean replay completed all 2,509 executions at 5,895
coverage points and 19,144 feature edges; LeakSanitizer alone remained
disabled under ptrace.

The checked integer cross-difference quotient now returns directly after sign
application when the divisor magnitude is one. This preserves every integer,
nonzero-divisor, and sign guard while avoiding a big-integer `div_rem` during
the first Bareiss stage. Positive and negative unit divisors are covered
alongside the existing exhaustive signed/divisibility cases.

A matched fresh 192-bit Criterion sentinel measured the composed
multiply/subtract/divide at 625.27 ns and the fused unit-divisor path at
190.15 ns, a 69.6% reduction. On the downstream Hypercurve sentinel, the
ten-run instruction median fell from 72,782,675 to 72,479,577 (0.4%), 77.4%
below the original baseline. Heaptrack allocations fell from 108,842 to
107,461; temporary allocations remained 6,236, peak heap remained 1.53 MiB,
and peak RSS was 12.65 MiB. Eleven ordinary runs had a 7.552 ms complete
median, a 5.446 ms preparation median, and a 0.353 ms exact-polyline projection
median, with unchanged topology and checksum.

The complete Hyperreal, Hypersolve, and Hypercurve all-feature and
no-default-feature suites, formatting, warning-denied all-target Clippy and
rustdoc, and release WASM library builds passed. The downstream
AddressSanitizer region-Boolean replay completed the requested 2,509-run
budget after 2,513 executions at 5,900 coverage points and 19,165 feature
edges; LeakSanitizer alone remained disabled under ptrace.

### Direct primitive big-integer ratios

`primitive_bigint_ratio` applies the same positive common scale and content
removal as `primitive_integer_ratio`, but writes the result directly as
`BigInt` coefficients. Scale-invariant polynomial and elimination kernels no
longer construct a temporary vector of `Rational` wrappers only to unpack
those integers immediately. Fixed disparate-denominator, sign, zero, and
empty cases compare the direct output with the retained rational components;
the API audit classifies both projective normalization schedules as having no
one-call GMP analogue.

Downstream Hypercurve now uses this representation for a primitive
pseudo-remainder sequence in rational GCD and Sturm work. Its rational Horner
evaluation uses the retained fixed two-product accumulator. Ordinary
value-preserving polynomial reduction and every symbolic-coefficient case
retain field division. A direct regression compares positive-scaled
pseudo-remainders with ordinary field remainders, including negative leading
coefficients and exact division.

On Hypercurve's one-cell all-family exact Boolean sentinel, the ten-run
instruction median fell from 64,678,125 to 61,647,633 (4.7%), 80.8% below the
original 320,660,631 baseline. Heaptrack allocation events rose from 96,817
to 98,024 and temporary events from 6,165 to 6,461, while peak heap fell from
1.49 to 1.41 MiB; peak RSS measured 12.45 MiB. Eleven ordinary runs had a
6.229 ms complete median, a 4.193 ms preparation median, and a 0.332 ms
exact-polyline projection median. Every run retained 9 candidate pairs,
48 fragments, 2 classifications, 4 decided operations, no blockers, and
checksum 6.

The complete Hyperreal, Hypersolve, and Hypercurve all-feature and
no-default-feature suites, formatting, warning-denied all-target Clippy,
all-feature and no-default-feature rustdoc, and default and no-default release
WASM library builds passed. The AddressSanitizer region-Boolean replay
completed all 2,509 executions at 5,896 coverage points and 19,157 feature
edges with no finding; LeakSanitizer alone remained disabled under ptrace.

### Single-owner product retention

Commutative product lookup probes both operands, but fresh results were
installed redundantly in both primary product slots. One retained edge is
sufficient to serve either operand order. Product retention now returns after
the first available canonical operand accepts the edge, tries the other
operand only when that slot is unavailable, and retains the established
secondary linear entry only when both primary slots are occupied. This leaves
the unused primary slot available for a distinct product and preserves the
existing repeated-power product chain.

An identity regression verifies that one fresh pair occupies exactly one
primary slot and that reversed multiplication returns the same retained
result. The existing direct/reversed, occupied-primary fallback, and repeated
small-power identity suites continue to cover the other cache schedules.

On Hypercurve's one-cell all-family exact Boolean sentinel, the rounded
ten-run instruction median fell from 31,366,779 to 31,293,247 (0.23%), 90.24%
below the original 320,660,631 baseline. Heaptrack allocation events fell
from 43,984 to 43,975; recorder-level temporary events remained 2,685 and the
postprocessor count remained 2,933. Peak heap remained 1.13 MiB, peak RSS
measured 11.26 MiB, and retained/leaked memory fell from 101.92 to 96.57 KiB.
Every measured run retained 9 candidate pairs, 48 fragments, 2
classifications, 4 decided operations, no blockers, and checksum 6.

The complete Hyperreal, Hypersolve, and Hypercurve all-feature and
no-default-feature suites, formatting, warning-denied all-target Clippy,
all-feature and no-default-feature rustdoc, and default and no-default release
WASM library builds passed. The downstream AddressSanitizer region-Boolean
replay completed all 2,509 executions at 5,892 coverage points and 19,115
feature edges with no finding; LeakSanitizer alone remained disabled under
ptrace.

### One-limb reduced-word materialization

The checked native rational kernels produce reduced `u128` numerator and
denominator parts. Their final constructor always entered `BigUint` through
its `u128` conversion, whose general implementation grows a digit vector in a
loop even when the value fits one machine limb. Most geometric coefficients
in the downstream trace fit `u64`.

Reduced-word materialization now selects the direct `u64` `BigUint`
constructor independently for each fitting part and retains the original
`u128` conversion for wide values. This changes neither reduction nor cache
admission. A boundary regression compares a near-`u64::MAX` fraction and a
100-bit dyadic fraction with the arbitrary-precision rational constructor.

On Hypercurve's one-cell all-family exact Boolean sentinel, the rounded
ten-run instruction median fell from 31,293,247 to 31,154,077 (0.44%), 90.28%
below the original 320,660,631 baseline. Inclusive instruction cost beneath
`from_reduced_word_parts` fell from 4,754,793 to 4,617,528 (2.9%). Heaptrack
allocation events remained 43,975; recorder-level temporary events remained
2,685 and the postprocessor count remained 2,933. Peak heap remained 1.13 MiB,
peak RSS fell from 11.26 to 11.12 MiB, and retained/leaked memory remained
96.57 KiB. Every measured run retained 9 candidate pairs, 48 fragments, 2
classifications, 4 decided operations, no blockers, and checksum 6.

The complete Hyperreal, Hypersolve, and Hypercurve all-feature and
no-default-feature suites, formatting, warning-denied all-target Clippy,
all-feature and no-default-feature rustdoc, and default and no-default release
WASM library builds passed. The downstream AddressSanitizer region-Boolean
replay completed the requested 2,509-run budget after 2,519 executions at
5,894 coverage points and 19,153 feature edges with no finding; LeakSanitizer
alone remained disabled under ptrace.

### Word-reduction denominator-shape tracing

The optional dispatch trace now classifies native odd-denominator reduction
into powers of three, five, or seven; mixed 3/5/7-smooth values; other small
odd values; other `u64` values; and wider values. This does not execute in
ordinary builds. Hypercurve's pathological benchmark can place its complete
mixed-family retained Boolean workload in one shared recording window and
print these raw counts with the existing rational reducer summary.

The one-cell trace recorded 3,027 odd-denominator reductions: 1,030 powers of
five, 56 powers of three, 93 powers of seven, 1,574 mixed 3/5/7-smooth
denominators, 72 other small denominators, 144 other `u64` denominators, and 8
wider denominators. Replacing the tuned binary GCD for the smooth group with
repeated exact divisibility loops was rejected: the ten-run instruction median
rose from 31,154,077 to approximately 31.316 million (0.52%) with unchanged
topology. Expanding canonical dyadic storage from odd magnitudes 63 through
127 was also rejected after it removed no allocations, retained seven extra
static rationals, and moved the median slightly upward.

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
