# exactCorelib / Hyper benchmark comparison

Run date: 2026-08-21 (America/Detroit)

> **Historical adapter-tier result.** This report intentionally measures the
> original one-shot Rust/C-ABI integration path. The requested retained native
> C++ fixtures have since been implemented and measured across the same 160
> cases. See the
> [retained-native benchmark report](RETAINED_BENCHMARK_REPORT.md) for the closer
> kernel comparison and its substantially different result.

This report records a complete Criterion run of the 160 paired cases in this
harness: 320 benchmark functions and 9,740 measured samples. It should be read
together with the [exhaustive capability and correctness report](COMPARISON_REPORT.md).

Publication note: the `triangulation.delaunay_complex` adapter was independently
rewritten after this latency campaign to remove the earlier demo-derived source.
That single historical row does not measure the current empty-circle adapter;
the remaining 159 rows are unaffected.

## Executive result

For the Rust-facing paths measured here, **Hyper was faster in 156 of 160 pairs
by non-overlapping 95% median confidence intervals**. exactCore was faster in
two, and two intervals overlapped. That all-case count needs a correctness
qualification: one exactCore win is the known-wrong root count for the cubic
with roots 1, 2, and 3.

Restricting the result to the 123 semantically matched cases gives the cleanest
answer:

- Hyper won 122 of 123 (99.2%) by the interval rule.
- exactCore won one: `real.exp2`, at a 1.80× lower median latency.
- The geometric mean of `exactCore / Hyper` median ratios was 15.98× and the
  median ratio was 13.05×. These are descriptive summaries across one example
  per case, not workload-weighted application speedups.

| Eligibility | Pairs | Hyper CI wins | exactCore CI wins | Overlap | Geometric mean E/H | Median E/H |
|---|---:|---:|---:|---:|---:|---:|
| Matched | 123 | 122 | 1 | 0 | 15.98× | 13.05× |
| Adapted contracts | 5 | 4 | 0 | 1 | 3.19× | 4.15× |
| Divergent family | 32 | 30 | 1 | 1 | 17.49× | 20.60× |
| **All measured pairs** | **160** | **156** | **2** | **2** | **15.47×** | **13.65×** |

Here and throughout, `E/H` is exactCore median time divided by Hyper median
time. A value above one favors Hyper.

## What this result does and does not establish

This is a valid comparison of the **current public Rust integration paths in
this harness**. It is not an isolated C++-kernel-versus-Rust-kernel comparison.
The distinction is material:

- exactCore is called through a C ABI. Many calls parse decimal rational
  strings or build `BigRat`, `Expr`, point, vector, matrix, and polynomial
  objects inside the timed function. The Hyper closures commonly capture
  already-built native Rust values.
- Exact rational, complex, and polynomial results often cross the ABI as text,
  including allocation, canonicalization, formatting, buffer handling, and
  Rust `String` construction. Most Hyper results stay as native values.
- exactCore real-function calls end with `Expr::doubleValue()` inside the timed
  C++ adapter. Most corresponding Hyper calls return a lazy `Real` expression
  graph without forcing a numeric approximation. They therefore do not measure
  the same materialization depth.
- Geometry calls commonly construct exactCore `Expr`-backed objects from
  integer arrays during each FFI call, while Hyper points and primitives are
  usually constructed outside the measured closure.
- Some Hyper policy APIs return richer evidence than the reduced exactCore
  relation code; some adapted/divergent rows intentionally compare only the
  nearest shared observation.

The defensible conclusion is therefore: **a Rust caller using this wrapper and
these operation shapes sees substantially lower latency from Hyper on nearly
all shared operations**. The run does not support the broader claim that
exactCore's underlying C++ algorithms are intrinsically 15× slower. Answering
that question would require native C++ fixtures, matched object lifetimes,
matched output materialization, and equivalent precision/certification work.

## Run configuration

The four Criterion targets were run sequentially and pinned to logical CPU 4:

```console
taskset -c 4 cargo bench --locked --bench comparable -- \
  --warm-up-time 0.5 --measurement-time 1 --sample-size 30 --noplot
taskset -c 4 cargo bench --locked --features curve --bench curves -- \
  --warm-up-time 0.5 --measurement-time 1 --sample-size 30 --noplot
taskset -c 4 cargo bench --locked --features mesh --bench meshes -- \
  --warm-up-time 0.5 --measurement-time 1 --sample-size 30 --noplot
taskset -c 4 cargo bench --locked --features path --bench paths -- \
  --warm-up-time 0.5 --measurement-time 1 --sample-size 30 --noplot
```

The only matched exactCore win, `real.exp2`, was then confirmed with a longer
100-sample measurement, which replaces its original row in the aggregate:

```console
taskset -c 4 cargo bench --locked --bench comparable -- real.exp2 \
  --warm-up-time 2 --measurement-time 5 --sample-size 100 --noplot
```

| Item | Run value |
|---|---|
| Host | Fedora 43, Linux 7.1.8, x86-64 |
| CPU | AMD Ryzen 7 5800X3D, 8 cores / 16 threads, 96 MiB L3 |
| Affinity | Logical CPU 4; affinity only, not exclusive CPU isolation |
| Frequency policy | `amd-pstate-epp`, `powersave`, boost enabled; frequency not locked |
| Rust | rustc/cargo 1.97.0; LLVM 22.1.6 |
| Native compiler | GCC 15.3.1; exactCore adapter compiled C++11, `-O3 -DNDEBUG`, `CORE_LEVEL=3` |
| Native libraries | GMP 6.3.0, MPFR 4.2.2 |
| Criterion | 0.5.1; release/bench Rust profile |
| Collection | 0.5 s warm-up, 1 s measurement, 30 samples per function |
| Confirmation | `real.exp2`: 2 s warm-up, 5 s measurement, 100 samples per function |

### Source identities

The Hyper path dependencies were measured at these local Git revisions:

| Crate | Revision | Worktree note at reporting time |
|---|---|---|
| `hyperreal` | `276f08c53b66` | clean |
| `hyperlattice` | `ac2aaebeaf67` | clean |
| `hyperlimit` | `b0418bddff50` | tracked files clean; untracked fuzz artifacts/binary present |
| `hypertri` | `1e21c5788b19` | clean |
| `hypersolve` | `5279e41e3394` | clean |
| `hypercurve` | `6a844f28c869` | **tracked working-tree changes present** in curve-related source |
| `hypermesh` | `397f6fe01052` | clean |
| `hyperpath` | `d792aa8dc843` | clean |

The result therefore reflects the actual local `hypercurve` working tree, not
that commit alone; `hypermesh` and `hyperpath` also consume it as a dependency.
The exactCore `trunk` snapshot and this standalone harness had no usable VCS
revision metadata in the supplied workspace. `Cargo.lock`, the source tree, and
this report identify the measured state as far as the workspace permits.

### Host-load warning

The machine was heavily oversubscribed during collection: load averages were
roughly 22–24 on 16 logical CPUs. CPU affinity reduced migration but did not
reserve the physical core, suppress its SMT sibling, lock frequency, or remove
cache/memory contention. The short measurement windows amplify that concern.

Consequently, treat absolute nanosecond/microsecond values as a record of this
run, not portable hardware constants. Large, repeatedly separated ratios give
useful directional evidence for this integration path. Close ratios should be
rerun on an idle machine with an isolated physical core, its SMT sibling idle,
and a fixed performance policy before making release or procurement decisions.

## Statistical treatment and run quality

The analyzer uses Criterion's bootstrap median point estimate and marginal 95%
confidence interval for each function. A pair is labeled `hyper` only when the
lowest exactCore median bound is greater than the highest Hyper bound; it is
labeled `exactCore` under the converse condition; otherwise it is `overlap`.
This non-overlap rule is a conservative descriptive screen, not a paired
hypothesis test or a substitute for independent repetitions.

| Quality/result measure | Value |
|---|---:|
| Paired cases / functions | 160 / 320 |
| Measured samples | 9,740 |
| Criterion Tukey outliers | 964 (9.90%) |
| Median relative CI width across 320 function estimates | 3.51% |
| Pairs with either relative CI width above 10% | 80 / 160 |
| Pairs with either relative CI width above 25% | 41 / 160 |
| E/H ratio p10 / p50 / p90 | 3.55× / 13.65× / 66.85× |

Criterion's Tukey classification flags unusual samples; the count above does
not imply those observations were manually removed. The high wide-interval
count is consistent with the observed host contention and is why the complete
confidence intervals are retained in the appendix.

The median-ratio distribution was:

| Bucket | Pairs |
|---|---:|
| exactCore at least 1.25× faster | 2 |
| Within 25% | 3 |
| Hyper 1.25–2× faster | 2 |
| Hyper 2–10× faster | 52 |
| Hyper 10–100× faster | 91 |
| Hyper above 100× faster | 10 |

## Results by mathematical family

| Family | Pairs | Hyper / exactCore / overlap | Geometric mean E/H | Median E/H |
|---|---:|---:|---:|---:|
| Rational arithmetic | 9 | 9 / 0 / 0 | 99.75× | 172.35× |
| Real expressions | 29 | 28 / 1 / 0 | 20.33× | 28.43× |
| Complex arithmetic | 5 | 5 / 0 / 0 | 24.06× | 28.40× |
| Vectors | 14 | 14 / 0 / 0 | 14.49× | 12.74× |
| Matrices | 12 | 12 / 0 / 0 | 9.21× | 8.56× |
| 2D geometry | 23 | 23 / 0 / 0 | 18.53× | 13.02× |
| 3D geometry | 30 | 29 / 0 / 1 | 13.33× | 14.31× |
| Polynomials | 16 | 14 / 1 / 1 | 5.51× | 6.23× |
| Triangulation | 1 | 1 / 0 / 0 | 4.41× | 4.41× |
| Curves | 9 | 9 / 0 / 0 | 8.79× | 9.43× |
| Meshes | 5 | 5 / 0 / 0 | 27.49× | 24.82× |
| Paths | 7 | 7 / 0 / 0 | 16.69× | 8.30× |

Rational arithmetic shows the largest aggregate gap, but it is also the clearest
adapter-cost case: exactCore reparses and serializes arbitrary-precision values
for tiny operations that Hyper performs on captured native objects. Polynomial
work has the smallest family geometric-mean gap because both sides do more
substantive algebra/solver work relative to boundary overhead. The single
triangulation case favors Hyper 4.41×, but it compares the adapted `dt4` cell
complex and Hyper's triangulation contract on one six-point fixture; it is not
an asymptotic crossover study.

## Crossovers and near parity

| Operation | Eligibility | exactCore median [95% CI] | Hyper median [95% CI] | Median result | Interpretation |
|---|---|---:|---:|---:|---|
| `real.exp2` | matched | 5.75 µs [5.72, 5.77] | 10.32 µs [10.29, 10.45] | exactCore 1.80× | Confirmed with the longer rerun; exactCore forces `doubleValue`, while Hyper builds `2^x` through its generic lazy power path |
| `polynomial.root_count` | divergent | 46.48 µs [42.38, 49.40] | 79.99 µs [65.54, 91.06] | exactCore 1.72× | **Not a valid win:** exactCore returns 2 for the benchmark cubic whose three roots are 1, 2, and 3 |
| `geometry3.triangle_relation_6` | divergent | 33.03 µs [32.05, 34.22] | 34.24 µs [34.11, 34.71] | exactCore 1.04× | Marginal intervals overlap; contracts belong to a divergent triangle family |
| `polynomial.resultant` | adapted | 8.15 µs [8.05, 8.21] | 7.99 µs [7.16, 12.60] | Hyper 1.02× | Indistinguishable by the interval rule; result orientation is adapted |
| `real.exp10` | matched | 9.11 µs [9.07, 9.13] | 8.37 µs [8.34, 8.62] | Hyper 1.09× | Close median but separated intervals in this run |

The next smallest matched Hyper gaps were polynomial GCD at 1.39× and interval
root count at 1.89×. These are more plausible candidates for a controlled,
matched-materialization kernel study than the adapter-dominated scalar cases.

## Largest observed Hyper advantages

| Operation | exactCore | Hyper | E/H |
|---|---:|---:|---:|
| `rational.absolute` | 2.23 µs | 4.20 ns | 529.90× |
| `path.line_endpoint_equality` | 21.69 µs | 54.21 ns | 400.14× |
| `geometry2.line_relation_5` | 2.39 µs | 7.60 ns | 313.77× |
| `geometry2.line_relation_6` | 2.52 µs | 8.26 ns | 305.51× |
| `rational.reciprocal` | 2.34 µs | 10.24 ns | 228.69× |
| `rational.negate` | 4.41 µs | 20.83 ns | 211.65× |
| `rational.add` | 2.93 µs | 16.24 ns | 180.44× |
| `rational.subtract` | 2.84 µs | 16.49 ns | 172.35× |
| `rational.compare` | 577.35 ns | 5.06 ns | 114.15× |
| `rational.multiply` | 2.99 µs | 28.56 ns | 104.61× |

These are real end-to-end timings, but they should not be presented as kernel
speedups. They primarily expose per-call parsing/construction/serialization and
FFI costs versus operations on preconstructed Rust objects. Some very small
Hyper closures may also benefit more from inlining and constant visibility than
opaque FFI calls.

## Correctness eligibility

Performance only matters after contract and result validity. The status labels
in the appendix have these meanings:

- `matched`: the harness found the same mathematical contract and passing
  differential coverage. These 123 rows support the strongest comparison.
- `adapted`: a documented conversion or reduced common observation was needed.
  Timing includes that chosen adapter shape and should not be generalized to the
  full APIs.
- `divergent-family`: the benchmark belongs to an API family with a preserved
  correctness/contract divergence. It is timed for inventory completeness, not
  treated as performance-selection evidence.

In particular, the fast exactCore `polynomial.root_count` result is a faster
wrong answer on this exact fixture. The 32 divergent-family rows and five
adapted rows remain in the all-case statistics so the run inventory is honest,
but the matched-only result is the decision-quality headline.

## Recommended follow-up benchmark designs

Recommendations 1 and 2 below are now implemented by the benchmark-only opaque
C++ fixture ABI and the `exactCorelib-retained` Criterion tier. Their completed
measurements are reported in
[RETAINED_BENCHMARK_REPORT.md](RETAINED_BENCHMARK_REPORT.md); the remaining
recommendations still apply.

1. For native kernel costs, add a C++ Criterion-compatible or Google Benchmark
   fixture that constructs exactCore values once and returns native values;
   mirror that lifecycle and output forcing in Rust.
2. For Rust integration costs, keep this suite but add persistent opaque
   exactCore handles so callers can choose between one-shot parse/serialize and
   retained-object APIs.
3. Force both real implementations to a shared numeric/certification target
   before comparing transcendental operations.
4. Run at least five independent process-level repetitions on an idle host with
   isolated physical cores and fixed frequency policy; compare distributions of
   run medians, not only within-run bootstrap intervals.
5. Add size sweeps for integer bit length, polynomial degree/root structure,
   point count, and degeneracy. One fixture per operation measures constant
   factors, not scaling or crossover behavior.
6. Exclude known-wrong outputs from winner counts in any scorecard, while still
   retaining their timings as diagnostic data.

## Reproduce and inspect

The complete rounded medians and confidence intervals below are the durable
record of this historical run. `target/criterion` is mutable build output and
has since been refreshed by the retained-native campaign, including new Hyper
measurements; it no longer reproduces this report as a single campaign. After a
fresh adapter-tier rerun, the checked-in analyzer can compute summaries or emit
machine-readable detail with the explicit `adapter` selector:

```console
node scripts/analyze-criterion.mjs target/criterion summary adapter
node scripts/analyze-criterion.mjs target/criterion json adapter
node scripts/analyze-criterion.mjs target/criterion tsv adapter
node scripts/analyze-criterion.mjs target/criterion markdown adapter
```

The benchmark definitions are in `benches/comparable.rs`, `benches/curves.rs`,
`benches/meshes.rs`, and `benches/paths.rs`. Semantic status and correctness
evidence are documented in `coverage/comparable-api.tsv` and
`COMPARISON_REPORT.md`.

## Complete per-operation results

Times are Criterion median point estimates with bootstrap 95% confidence
intervals. `CI result` uses the non-overlapping marginal-interval rule described
above. Rows are sorted by operation name.

| Operation | Semantic status | exactCore median [95% CI] | Hyper median [95% CI] | Median advantage | CI result |
|---|---|---:|---:|---:|---|
| `bivariate.evaluate` | matched | 3.18 µs [3.17 µs, 3.22 µs] | 235.56 ns [223.91 ns, 245.98 ns] | Hyper 13.51× | hyper |
| `bivariate.resultant` | matched | 47.20 µs [42.04 µs, 50.70 µs] | 5.72 µs [4.64 µs, 8.47 µs] | Hyper 8.25× | hyper |
| `complex.add` | matched | 5.41 µs [5.35 µs, 5.46 µs] | 104.25 ns [102.64 ns, 106.06 ns] | Hyper 51.91× | hyper |
| `complex.divide` | matched | 14.45 µs [9.27 µs, 15.28 µs] | 508.73 ns [503.59 ns, 511.87 ns] | Hyper 28.40× | hyper |
| `complex.multiply` | matched | 6.91 µs [6.71 µs, 7.20 µs] | 915.27 ns [912.12 ns, 927.52 ns] | Hyper 7.54× | hyper |
| `complex.norm_squared` | matched | 2.79 µs [2.79 µs, 2.81 µs] | 196.96 ns [196.45 ns, 197.63 ns] | Hyper 14.18× | hyper |
| `complex.subtract` | matched | 5.30 µs [5.29 µs, 5.82 µs] | 103.89 ns [102.86 ns, 106.19 ns] | Hyper 51.05× | hyper |
| `curve.circle_circle_relation` | adapted | 7.73 µs [7.71 µs, 7.75 µs] | 2.12 µs [1.77 µs, 2.28 µs] | Hyper 3.65× | hyper |
| `curve.circle_point_distance` | matched | 6.15 µs [6.12 µs, 6.18 µs] | 946.93 ns [940.50 ns, 961.94 ns] | Hyper 6.50× | hyper |
| `curve.line_contains_point` | matched | 2.50 µs [2.47 µs, 2.53 µs] | 229.09 ns [227.10 ns, 229.88 ns] | Hyper 10.92× | hyper |
| `curve.line_intersection_topology` | matched | 24.35 µs [22.66 µs, 25.49 µs] | 1.27 µs [1.26 µs, 1.28 µs] | Hyper 19.16× | hyper |
| `curve.line_intersection_witness` | matched | 11.59 µs [11.45 µs, 12.86 µs] | 1.23 µs [1.22 µs, 1.23 µs] | Hyper 9.43× | hyper |
| `curve.line_length` | matched | 3.64 µs [3.58 µs, 4.05 µs] | 497.20 ns [494.47 ns, 500.44 ns] | Hyper 7.32× | hyper |
| `curve.line_side` | matched | 2.47 µs [2.46 µs, 2.51 µs] | 223.82 ns [222.26 ns, 232.81 ns] | Hyper 11.03× | hyper |
| `curve.segment_dispatch` | matched | 23.14 µs [22.40 µs, 25.53 µs] | 1.35 µs [1.35 µs, 1.36 µs] | Hyper 17.09× | hyper |
| `curve.supporting_line_circle` | adapted | 5.67 µs [5.65 µs, 5.71 µs] | 1.17 µs [1.11 µs, 1.86 µs] | Hyper 4.86× | hyper |
| `geometry2.area` | matched | 9.05 µs [6.84 µs, 9.26 µs] | 695.22 ns [665.68 ns, 727.25 ns] | Hyper 13.02× | hyper |
| `geometry2.between` | matched | 4.35 µs [3.02 µs, 4.68 µs] | 120.44 ns [119.73 ns, 121.19 ns] | Hyper 36.12× | hyper |
| `geometry2.circle_circle_distance` | adapted | 13.85 µs [9.41 µs, 17.59 µs] | 3.34 µs [2.46 µs, 4.45 µs] | Hyper 4.15× | hyper |
| `geometry2.circle_line` | divergent-family | 6.06 µs [5.64 µs, 10.16 µs] | 889.27 ns [878.01 ns, 926.82 ns] | Hyper 6.81× | hyper |
| `geometry2.circle_point_distance` | matched | 6.50 µs [6.42 µs, 6.56 µs] | 1.83 µs [1.55 µs, 2.11 µs] | Hyper 3.56× | hyper |
| `geometry2.circle_segment` | divergent-family | 8.34 µs [8.30 µs, 9.56 µs] | 1.95 µs [1.93 µs, 2.26 µs] | Hyper 4.28× | hyper |
| `geometry2.incircle` | matched | 5.25 µs [5.08 µs, 5.37 µs] | 161.12 ns [156.09 ns, 195.06 ns] | Hyper 32.56× | hyper |
| `geometry2.line_intersection` | matched | 11.72 µs [11.57 µs, 12.40 µs] | 593.93 ns [592.73 ns, 605.51 ns] | Hyper 19.74× | hyper |
| `geometry2.line_point_distance` | matched | 8.52 µs [8.49 µs, 8.60 µs] | 1.43 µs [1.41 µs, 1.45 µs] | Hyper 5.97× | hyper |
| `geometry2.line_relation_0` | divergent-family | 2.66 µs [2.65 µs, 2.67 µs] | 56.50 ns [55.94 ns, 57.30 ns] | Hyper 47.15× | hyper |
| `geometry2.line_relation_1` | divergent-family | 2.68 µs [2.67 µs, 2.69 µs] | 60.32 ns [57.10 ns, 80.91 ns] | Hyper 44.37× | hyper |
| `geometry2.line_relation_2` | divergent-family | 7.54 µs [4.05 µs, 8.13 µs] | 319.23 ns [311.90 ns, 329.21 ns] | Hyper 23.61× | hyper |
| `geometry2.line_relation_3` | divergent-family | 3.94 µs [3.92 µs, 3.97 µs] | 309.78 ns [300.62 ns, 355.26 ns] | Hyper 12.71× | hyper |
| `geometry2.line_relation_4` | divergent-family | 3.67 µs [3.61 µs, 3.77 µs] | 311.29 ns [308.44 ns, 315.57 ns] | Hyper 11.79× | hyper |
| `geometry2.line_relation_5` | divergent-family | 2.39 µs [2.38 µs, 2.43 µs] | 7.60 ns [7.53 ns, 7.82 ns] | Hyper 313.77× | hyper |
| `geometry2.line_relation_6` | divergent-family | 2.52 µs [2.42 µs, 2.70 µs] | 8.26 ns [8.06 ns, 8.82 ns] | Hyper 305.51× | hyper |
| `geometry2.orientation` | matched | 2.56 µs [2.55 µs, 2.57 µs] | 57.98 ns [57.39 ns, 60.04 ns] | Hyper 44.15× | hyper |
| `geometry2.point_distance` | matched | 3.46 µs [3.43 µs, 3.55 µs] | 304.18 ns [301.92 ns, 306.36 ns] | Hyper 11.38× | hyper |
| `geometry2.segment_point_distance` | matched | 10.97 µs [10.25 µs, 16.36 µs] | 1.76 µs [1.75 µs, 1.78 µs] | Hyper 6.24× | hyper |
| `geometry2.segment_relation_0` | matched | 2.71 µs [2.69 µs, 2.77 µs] | 70.08 ns [63.47 ns, 76.69 ns] | Hyper 38.72× | hyper |
| `geometry2.segment_relation_1` | matched | 23.15 µs [23.05 µs, 23.24 µs] | 557.56 ns [556.60 ns, 563.39 ns] | Hyper 41.52× | hyper |
| `geometry2.segment_relation_2` | matched | 2.83 µs [2.83 µs, 2.83 µs] | 538.55 ns [535.53 ns, 546.41 ns] | Hyper 5.25× | hyper |
| `geometry2.segment_relation_3` | matched | 3.86 µs [3.85 µs, 3.95 µs] | 386.57 ns [259.24 ns, 471.74 ns] | Hyper 9.99× | hyper |
| `geometry3.line_point_distance` | matched | 15.73 µs [15.47 µs, 16.68 µs] | 3.06 µs [2.87 µs, 3.23 µs] | Hyper 5.14× | hyper |
| `geometry3.line_relation_0` | matched | 4.69 µs [4.65 µs, 4.71 µs] | 910.21 ns [904.10 ns, 918.09 ns] | Hyper 5.15× | hyper |
| `geometry3.line_relation_1` | matched | 6.12 µs [6.06 µs, 6.23 µs] | 1.11 µs [1.09 µs, 1.12 µs] | Hyper 5.52× | hyper |
| `geometry3.line_relation_2` | matched | 6.99 µs [6.97 µs, 7.23 µs] | 161.20 ns [160.86 ns, 161.60 ns] | Hyper 43.35× | hyper |
| `geometry3.line_relation_3` | matched | 13.81 µs [13.48 µs, 14.55 µs] | 162.84 ns [161.92 ns, 168.46 ns] | Hyper 84.82× | hyper |
| `geometry3.line_relation_4` | matched | 6.16 µs [6.13 µs, 6.25 µs] | 1.79 µs [1.30 µs, 1.94 µs] | Hyper 3.43× | hyper |
| `geometry3.orientation` | matched | 6.40 µs [6.33 µs, 6.61 µs] | 324.57 ns [280.34 ns, 335.65 ns] | Hyper 19.72× | hyper |
| `geometry3.plane_point_distance` | matched | 17.79 µs [17.57 µs, 17.89 µs] | 628.64 ns [620.48 ns, 637.07 ns] | Hyper 28.30× | hyper |
| `geometry3.plane_relation_0` | divergent-family | 6.58 µs [6.56 µs, 6.63 µs] | 74.17 ns [72.41 ns, 74.99 ns] | Hyper 88.72× | hyper |
| `geometry3.plane_relation_1` | divergent-family | 6.24 µs [6.19 µs, 6.46 µs] | 97.14 ns [77.55 ns, 137.93 ns] | Hyper 64.24× | hyper |
| `geometry3.plane_relation_2` | divergent-family | 8.51 µs [8.22 µs, 9.61 µs] | 569.55 ns [556.74 ns, 579.73 ns] | Hyper 14.94× | hyper |
| `geometry3.plane_relation_3` | divergent-family | 12.22 µs [9.24 µs, 16.48 µs] | 294.92 ns [276.10 ns, 317.63 ns] | Hyper 41.44× | hyper |
| `geometry3.plane_relation_4` | divergent-family | 7.72 µs [7.69 µs, 7.94 µs] | 284.19 ns [280.43 ns, 308.76 ns] | Hyper 27.16× | hyper |
| `geometry3.plane_relation_5` | divergent-family | 8.53 µs [8.25 µs, 8.87 µs] | 282.11 ns [278.75 ns, 293.51 ns] | Hyper 30.23× | hyper |
| `geometry3.plane_relation_6` | divergent-family | 10.50 µs [10.44 µs, 10.59 µs] | 229.86 ns [228.52 ns, 230.56 ns] | Hyper 45.67× | hyper |
| `geometry3.plane_relation_7` | divergent-family | 20.69 µs [20.54 µs, 20.77 µs] | 236.86 ns [232.05 ns, 247.75 ns] | Hyper 87.33× | hyper |
| `geometry3.point_distance` | matched | 5.68 µs [5.45 µs, 6.14 µs] | 495.13 ns [493.82 ns, 526.02 ns] | Hyper 11.47× | hyper |
| `geometry3.segment_point_distance` | matched | 19.22 µs [18.25 µs, 19.82 µs] | 3.43 µs [3.40 µs, 3.44 µs] | Hyper 5.60× | hyper |
| `geometry3.segment_relation_0` | matched | 5.04 µs [4.77 µs, 5.30 µs] | 1.78 µs [1.72 µs, 1.86 µs] | Hyper 2.82× | hyper |
| `geometry3.segment_relation_1` | matched | 6.96 µs [6.87 µs, 6.98 µs] | 1.43 µs [1.42 µs, 1.44 µs] | Hyper 4.86× | hyper |
| `geometry3.segment_relation_2` | matched | 4.72 µs [4.70 µs, 4.91 µs] | 1.44 µs [1.43 µs, 1.46 µs] | Hyper 3.28× | hyper |
| `geometry3.segment_relation_3` | matched | 6.34 µs [6.33 µs, 6.34 µs] | 160.77 ns [160.69 ns, 161.07 ns] | Hyper 39.42× | hyper |
| `geometry3.triangle_relation_0` | divergent-family | 6.16 µs [6.09 µs, 6.23 µs] | 675.13 ns [629.66 ns, 767.56 ns] | Hyper 9.13× | hyper |
| `geometry3.triangle_relation_1` | divergent-family | 16.67 µs [16.56 µs, 16.82 µs] | 619.36 ns [615.84 ns, 626.20 ns] | Hyper 26.91× | hyper |
| `geometry3.triangle_relation_2` | divergent-family | 9.45 µs [9.43 µs, 9.53 µs] | 603.09 ns [597.82 ns, 746.02 ns] | Hyper 15.67× | hyper |
| `geometry3.triangle_relation_3` | divergent-family | 15.74 µs [15.52 µs, 16.63 µs] | 1.15 µs [737.51 ns, 1.19 µs] | Hyper 13.68× | hyper |
| `geometry3.triangle_relation_4` | divergent-family | 14.48 µs [14.36 µs, 14.92 µs] | 5.73 µs [5.69 µs, 5.81 µs] | Hyper 2.53× | hyper |
| `geometry3.triangle_relation_5` | divergent-family | 13.23 µs [12.66 µs, 16.57 µs] | 5.65 µs [5.64 µs, 5.68 µs] | Hyper 2.34× | hyper |
| `geometry3.triangle_relation_6` | divergent-family | 33.03 µs [32.05 µs, 34.22 µs] | 34.24 µs [34.11 µs, 34.71 µs] | exactCore 1.04× | overlap |
| `geometry3.volume` | matched | 16.52 µs [15.59 µs, 18.39 µs] | 1.21 µs [1.19 µs, 1.25 µs] | Hyper 13.63× | hyper |
| `matrix3.add` | matched | 6.09 µs [5.81 µs, 6.48 µs] | 763.66 ns [757.37 ns, 785.11 ns] | Hyper 7.97× | hyper |
| `matrix3.determinant` | divergent-family | 6.23 µs [5.86 µs, 6.67 µs] | 384.26 ns [382.34 ns, 400.02 ns] | Hyper 16.21× | hyper |
| `matrix3.inverse` | matched | 30.65 µs [30.61 µs, 30.68 µs] | 1.78 µs [1.78 µs, 1.79 µs] | Hyper 17.21× | hyper |
| `matrix3.multiply` | matched | 11.00 µs [10.90 µs, 11.05 µs] | 1.32 µs [1.31 µs, 1.33 µs] | Hyper 8.34× | hyper |
| `matrix3.subtract` | matched | 5.80 µs [5.77 µs, 6.24 µs] | 836.44 ns [818.99 ns, 916.08 ns] | Hyper 6.93× | hyper |
| `matrix3.transpose` | matched | 2.37 µs [2.36 µs, 2.38 µs] | 428.34 ns [425.37 ns, 434.19 ns] | Hyper 5.53× | hyper |
| `matrix4.add` | matched | 10.51 µs [10.45 µs, 10.59 µs] | 1.20 µs [1.20 µs, 1.20 µs] | Hyper 8.78× | hyper |
| `matrix4.determinant` | divergent-family | 13.39 µs [13.28 µs, 13.60 µs] | 1.60 µs [1.58 µs, 1.63 µs] | Hyper 8.39× | hyper |
| `matrix4.inverse` | matched | 76.01 µs [70.01 µs, 81.56 µs] | 6.63 µs [6.59 µs, 6.66 µs] | Hyper 11.47× | hyper |
| `matrix4.multiply` | matched | 21.64 µs [21.52 µs, 22.08 µs] | 2.32 µs [2.27 µs, 2.54 µs] | Hyper 9.33× | hyper |
| `matrix4.subtract` | matched | 11.08 µs [10.73 µs, 20.36 µs] | 1.46 µs [1.36 µs, 1.60 µs] | Hyper 7.57× | hyper |
| `matrix4.transpose` | matched | 4.81 µs [4.30 µs, 5.52 µs] | 550.31 ns [545.67 ns, 574.42 ns] | Hyper 8.74× | hyper |
| `mesh.plane_point_classification` | matched | 9.43 µs [9.40 µs, 9.52 µs] | 169.21 ns [168.31 ns, 210.84 ns] | Hyper 55.75× | hyper |
| `mesh.triangle_boundary_point` | divergent-family | 26.74 µs [22.03 µs, 37.57 µs] | 1.32 µs [1.25 µs, 1.49 µs] | Hyper 20.29× | hyper |
| `mesh.triangle_contains_point` | divergent-family | 16.91 µs [16.57 µs, 22.07 µs] | 632.30 ns [630.88 ns, 634.11 ns] | Hyper 26.75× | hyper |
| `mesh.triangle_contains_point_strictly` | divergent-family | 15.89 µs [15.70 µs, 16.15 µs] | 640.18 ns [635.91 ns, 649.12 ns] | Hyper 24.82× | hyper |
| `mesh.triangle_triangle_intersection` | divergent-family | 95.19 µs [75.32 µs, 102.56 µs] | 4.55 µs [4.50 µs, 4.58 µs] | Hyper 20.92× | hyper |
| `path.circle_circle_relation` | matched | 9.33 µs [8.70 µs, 15.49 µs] | 2.43 µs [2.42 µs, 2.45 µs] | Hyper 3.84× | hyper |
| `path.circle_point_membership` | matched | 10.98 µs [10.86 µs, 11.67 µs] | 221.87 ns [218.65 ns, 224.65 ns] | Hyper 49.51× | hyper |
| `path.circle_segment_intersection` | matched | 9.96 µs [9.92 µs, 10.16 µs] | 2.96 µs [2.94 µs, 2.99 µs] | Hyper 3.37× | hyper |
| `path.line_axis_classification` | matched | 4.02 µs [4.00 µs, 4.10 µs] | 1.25 µs [1.24 µs, 1.25 µs] | Hyper 3.22× | hyper |
| `path.line_endpoint_equality` | matched | 21.69 µs [21.38 µs, 22.00 µs] | 54.21 ns [53.31 ns, 54.60 ns] | Hyper 400.14× | hyper |
| `path.line_length` | matched | 3.71 µs [3.68 µs, 3.78 µs] | 446.71 ns [445.72 ns, 448.81 ns] | Hyper 8.30× | hyper |
| `path.line_parameter_order` | matched | 4.17 µs [3.99 µs, 4.34 µs] | 79.01 ns [76.53 ns, 91.96 ns] | Hyper 52.77× | hyper |
| `polynomial.binary_0` | matched | 4.11 µs [4.07 µs, 4.18 µs] | 375.91 ns [368.58 ns, 383.70 ns] | Hyper 10.93× | hyper |
| `polynomial.binary_1` | matched | 4.08 µs [4.06 µs, 4.10 µs] | 375.52 ns [373.25 ns, 376.47 ns] | Hyper 10.86× | hyper |
| `polynomial.binary_2` | matched | 6.40 µs [5.70 µs, 10.31 µs] | 363.16 ns [361.12 ns, 366.22 ns] | Hyper 17.63× | hyper |
| `polynomial.binary_3` | matched | 7.96 µs [7.72 µs, 8.18 µs] | 2.51 µs [2.36 µs, 2.62 µs] | Hyper 3.17× | hyper |
| `polynomial.binary_4` | matched | 9.32 µs [8.92 µs, 10.20 µs] | 260.99 ns [258.96 ns, 273.72 ns] | Hyper 35.70× | hyper |
| `polynomial.derivative` | matched | 3.42 µs [3.39 µs, 3.44 µs] | 101.58 ns [101.26 ns, 103.27 ns] | Hyper 33.64× | hyper |
| `polynomial.discriminant` | divergent-family | 28.43 µs [20.31 µs, 28.86 µs] | 7.74 µs [7.68 µs, 7.83 µs] | Hyper 3.67× | hyper |
| `polynomial.evaluate` | matched | 2.69 µs [2.68 µs, 2.69 µs] | 399.32 ns [380.88 ns, 405.76 ns] | Hyper 6.73× | hyper |
| `polynomial.gcd` | matched | 6.54 µs [6.47 µs, 6.61 µs] | 4.71 µs [4.15 µs, 4.88 µs] | Hyper 1.39× | hyper |
| `polynomial.resultant` | adapted | 8.15 µs [8.05 µs, 8.21 µs] | 7.99 µs [7.16 µs, 12.60 µs] | Hyper 1.02× | overlap |
| `polynomial.root_count` | divergent-family | 46.48 µs [42.38 µs, 49.40 µs] | 79.99 µs [65.54 µs, 91.06 µs] | exactCore 1.72× | exactCore |
| `polynomial.root_count_interval` | matched | 65.97 µs [64.60 µs, 76.47 µs] | 34.93 µs [34.37 µs, 35.45 µs] | Hyper 1.89× | hyper |
| `polynomial.root_isolation` | matched | 130.16 µs [127.38 µs, 134.18 µs] | 35.46 µs [34.41 µs, 35.85 µs] | Hyper 3.67× | hyper |
| `polynomial.square_free` | matched | 17.07 µs [16.49 µs, 18.23 µs] | 2.98 µs [2.93 µs, 3.09 µs] | Hyper 5.73× | hyper |
| `rational.absolute` | matched | 2.23 µs [2.21 µs, 2.24 µs] | 4.20 ns [4.19 ns, 4.23 ns] | Hyper 529.90× | hyper |
| `rational.add` | matched | 2.93 µs [2.91 µs, 2.97 µs] | 16.24 ns [16.17 ns, 16.30 ns] | Hyper 180.44× | hyper |
| `rational.compare` | matched | 577.35 ns [574.53 ns, 585.14 ns] | 5.06 ns [5.05 ns, 5.08 ns] | Hyper 114.15× | hyper |
| `rational.divide` | matched | 2.89 µs [2.88 µs, 2.93 µs] | 226.52 ns [224.83 ns, 240.36 ns] | Hyper 12.75× | hyper |
| `rational.multiply` | matched | 2.99 µs [2.94 µs, 3.05 µs] | 28.56 ns [28.49 ns, 28.63 ns] | Hyper 104.61× | hyper |
| `rational.negate` | matched | 4.41 µs [2.49 µs, 4.96 µs] | 20.83 ns [20.69 ns, 21.66 ns] | Hyper 211.65× | hyper |
| `rational.parts` | matched | 3.39 µs [3.37 µs, 3.39 µs] | 420.48 ns [419.92 ns, 422.42 ns] | Hyper 8.05× | hyper |
| `rational.reciprocal` | matched | 2.34 µs [2.33 µs, 2.35 µs] | 10.24 ns [10.23 ns, 10.25 ns] | Hyper 228.69× | hyper |
| `rational.subtract` | matched | 2.84 µs [2.84 µs, 2.85 µs] | 16.49 ns [16.42 ns, 16.54 ns] | Hyper 172.35× | hyper |
| `real.absolute` | matched | 1.71 µs [1.69 µs, 1.73 µs] | 60.07 ns [59.11 ns, 70.09 ns] | Hyper 28.44× | hyper |
| `real.acos` | matched | 22.45 µs [22.15 µs, 22.60 µs] | 263.52 ns [260.82 ns, 268.36 ns] | Hyper 85.19× | hyper |
| `real.add` | matched | 3.88 µs [3.84 µs, 3.96 µs] | 78.38 ns [72.33 ns, 87.50 ns] | Hyper 49.49× | hyper |
| `real.asin` | matched | 11.99 µs [11.93 µs, 12.20 µs] | 284.32 ns [282.08 ns, 361.86 ns] | Hyper 42.16× | hyper |
| `real.atan` | matched | 19.87 µs [18.70 µs, 21.55 µs] | 254.14 ns [248.56 ns, 451.48 ns] | Hyper 78.20× | hyper |
| `real.cbrt` | matched | 3.78 µs [3.77 µs, 3.89 µs] | 1.43 µs [1.36 µs, 1.45 µs] | Hyper 2.64× | hyper |
| `real.ceil` | matched | 5.71 µs [5.31 µs, 6.29 µs] | 162.04 ns [146.54 ns, 166.87 ns] | Hyper 35.22× | hyper |
| `real.cos` | matched | 6.46 µs [6.32 µs, 6.57 µs] | 408.18 ns [406.42 ns, 423.19 ns] | Hyper 15.83× | hyper |
| `real.cot` | matched | 11.03 µs [10.75 µs, 11.16 µs] | 1.28 µs [1.26 µs, 1.33 µs] | Hyper 8.62× | hyper |
| `real.divide` | matched | 4.20 µs [4.16 µs, 4.35 µs] | 199.25 ns [198.07 ns, 199.85 ns] | Hyper 21.08× | hyper |
| `real.e` | matched | 4.94 µs [3.84 µs, 5.56 µs] | 74.74 ns [74.49 ns, 75.22 ns] | Hyper 66.11× | hyper |
| `real.exp` | matched | 5.14 µs [5.09 µs, 5.29 µs] | 210.57 ns [206.30 ns, 213.77 ns] | Hyper 24.41× | hyper |
| `real.exp10` | matched | 9.11 µs [9.07 µs, 9.13 µs] | 8.37 µs [8.34 µs, 8.62 µs] | Hyper 1.09× | hyper |
| `real.exp2` | matched | 5.75 µs [5.72 µs, 5.77 µs] | 10.32 µs [10.29 µs, 10.45 µs] | exactCore 1.80× | exactCore |
| `real.floor` | matched | 5.25 µs [4.65 µs, 7.73 µs] | 111.28 ns [97.16 ns, 127.56 ns] | Hyper 47.20× | hyper |
| `real.ln` | matched | 6.03 µs [5.37 µs, 7.21 µs] | 187.62 ns [187.30 ns, 187.96 ns] | Hyper 32.12× | hyper |
| `real.log10` | matched | 8.48 µs [8.40 µs, 8.58 µs] | 298.11 ns [294.93 ns, 347.12 ns] | Hyper 28.43× | hyper |
| `real.log2` | matched | 5.43 µs [5.39 µs, 5.51 µs] | 304.89 ns [293.78 ns, 322.04 ns] | Hyper 17.81× | hyper |
| `real.multiply` | matched | 3.67 µs [3.66 µs, 3.69 µs] | 90.16 ns [78.87 ns, 100.40 ns] | Hyper 40.73× | hyper |
| `real.negate` | matched | 3.42 µs [3.34 µs, 3.48 µs] | 46.60 ns [46.48 ns, 46.66 ns] | Hyper 73.48× | hyper |
| `real.pi` | matched | 542.51 ns [540.34 ns, 543.45 ns] | 72.27 ns [57.92 ns, 79.87 ns] | Hyper 7.51× | hyper |
| `real.powi` | matched | 5.20 µs [5.11 µs, 5.28 µs] | 118.65 ns [116.57 ns, 120.15 ns] | Hyper 43.79× | hyper |
| `real.root_n` | matched | 5.29 µs [5.20 µs, 5.35 µs] | 1.46 µs [1.45 µs, 1.51 µs] | Hyper 3.63× | hyper |
| `real.sign` | matched | 8.76 µs [8.56 µs, 8.86 µs] | 484.89 ns [481.09 ns, 489.45 ns] | Hyper 18.07× | hyper |
| `real.sin` | matched | 7.85 µs [7.81 µs, 7.89 µs] | 409.18 ns [408.10 ns, 410.66 ns] | Hyper 19.18× | hyper |
| `real.sqrt` | matched | 2.40 µs [2.39 µs, 2.41 µs] | 207.90 ns [207.30 ns, 209.55 ns] | Hyper 11.53× | hyper |
| `real.square` | matched | 3.19 µs [2.44 µs, 3.65 µs] | 113.68 ns [109.65 ns, 132.16 ns] | Hyper 28.05× | hyper |
| `real.subtract` | matched | 3.68 µs [3.60 µs, 3.74 µs] | 68.94 ns [66.76 ns, 70.72 ns] | Hyper 53.38× | hyper |
| `real.tan` | matched | 11.62 µs [11.42 µs, 17.33 µs] | 227.45 ns [222.33 ns, 232.28 ns] | Hyper 51.09× | hyper |
| `triangulation.delaunay_complex` | adapted | 672.68 µs [598.87 µs, 719.89 µs] | 152.71 µs [150.65 µs, 160.66 µs] | Hyper 4.41× | hyper |
| `vector2.add` | matched | 1.40 µs [1.30 µs, 2.41 µs] | 195.12 ns [192.05 ns, 197.45 ns] | Hyper 7.18× | hyper |
| `vector2.dot` | matched | 1.36 µs [1.32 µs, 1.42 µs] | 109.07 ns [108.23 ns, 110.64 ns] | Hyper 12.47× | hyper |
| `vector2.norm` | matched | 2.71 µs [2.70 µs, 2.76 µs] | 168.35 ns [167.57 ns, 170.21 ns] | Hyper 16.12× | hyper |
| `vector2.subtract` | matched | 2.46 µs [2.38 µs, 2.58 µs] | 101.44 ns [100.36 ns, 103.13 ns] | Hyper 24.27× | hyper |
| `vector2.wedge` | matched | 4.41 µs [4.39 µs, 4.44 µs] | 151.43 ns [140.11 ns, 179.65 ns] | Hyper 29.11× | hyper |
| `vector3.add` | matched | 1.91 µs [1.88 µs, 1.95 µs] | 151.02 ns [149.14 ns, 236.83 ns] | Hyper 12.63× | hyper |
| `vector3.cross` | matched | 3.02 µs [3.02 µs, 3.05 µs] | 359.86 ns [358.44 ns, 362.86 ns] | Hyper 8.40× | hyper |
| `vector3.dot` | matched | 2.08 µs [2.06 µs, 2.17 µs] | 172.44 ns [167.72 ns, 176.65 ns] | Hyper 12.05× | hyper |
| `vector3.norm` | matched | 4.30 µs [4.29 µs, 5.75 µs] | 335.17 ns [334.42 ns, 338.37 ns] | Hyper 12.84× | hyper |
| `vector3.subtract` | matched | 1.99 µs [1.95 µs, 2.00 µs] | 152.50 ns [151.51 ns, 153.83 ns] | Hyper 13.05× | hyper |
| `vector4.add` | matched | 4.96 µs [4.45 µs, 5.27 µs] | 197.69 ns [197.47 ns, 197.89 ns] | Hyper 25.10× | hyper |
| `vector4.dot` | matched | 2.49 µs [2.45 µs, 2.54 µs] | 218.50 ns [216.10 ns, 260.45 ns] | Hyper 11.38× | hyper |
| `vector4.norm` | matched | 5.31 µs [5.24 µs, 5.53 µs] | 442.85 ns [439.03 ns, 489.66 ns] | Hyper 11.98× | hyper |
| `vector4.subtract` | matched | 4.91 µs [4.80 µs, 4.96 µs] | 203.91 ns [203.24 ns, 207.14 ns] | Hyper 24.07× | hyper |
