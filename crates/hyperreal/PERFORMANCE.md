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
cargo bench --bench predicates --features hyperlattice -- '(hyperlattice|hyperreal)'
cargo bench --bench predicates --features dispatch-trace,hyperlattice -- --write-dispatch-trace-md
```

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
- Cached approximation rows are intentionally very sensitive to code layout.
  During optimization, keep helper functions away from the middle of hot
  `sin`/`cos`/`tan` kernels unless the low-level numerical benches prove there
  is no regression.
- Dispatch trace path names to watch: `large-rational-deferred`,
  `medium-rational-half-pi-rewrite`, `structural-small-prescaled`,
  `integer-pi-plus-rational`, and `generic-half-pi-reduction`.

Goals:

- Keep cached rows below 45 ns and zero rows below 80 ns.
- Keep cold sin/cos baseline around 1.5-1.6 us and avoid widening the
  sin/cos gap.
- Bring large exact-rational cold sin/cos closer to 2 us or below.
- Reduce tan cold paths toward 3 us without changing pole behavior.
- The biggest remaining low-level targets are inverse trig and hyperbolic
  cold paths: `acos`, `asin`, `atan`, `acosh`, `asinh`, and large `exp`.

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
