# exactCorelib / Hyper retained-native benchmark comparison

Run date: 2026-08-21 (America/Detroit)

This report records the upgraded native-fixture tier for all 160 paired cases in
the harness. It analyzes 320 benchmark functions (one
`exactCorelib-retained` and one Hyper implementation per case), 9,600 measured
samples, and Criterion's bootstrap median confidence intervals. The original
one-shot Rust/C-ABI integration campaign remains in
[BENCHMARK_REPORT.md](BENCHMARK_REPORT.md).

Publication note: the `triangulation.delaunay_complex` adapter was independently
rewritten after this latency campaign to remove the earlier demo-derived source.
That single historical row does not measure the current empty-circle adapter;
the remaining 159 rows are unaffected. The memory campaign was rerun after the
rewrite and does describe the published implementation.

## Executive result

Removing fixed-input parsing, exactCore input-object construction, output
serialization, and per-iteration FFI from the exactCore hot path materially
changes the answer. Across all 160 cases, Hyper has the lower non-overlapping
95% median interval in 109, exactCore in 49, and two overlap. On the 123
semantically matched contracts—the decision-quality subset—Hyper wins 80 and
exactCore wins 43, with no overlaps.

| Tier / eligibility | Pairs | Hyper / exactCore / overlap | Geometric mean E/H | Median E/H |
|---|---:|---:|---:|---:|
| One-shot adapter, matched (historical run) | 123 | 122 / 1 / 0 | 15.98× | 13.05× |
| **Retained native, matched** | **123** | **80 / 43 / 0** | **1.43×** | **1.99×** |
| One-shot adapter, all (historical run) | 160 | 156 / 2 / 2 | 15.47× | 13.65× |
| **Retained native, all** | **160** | **109 / 49 / 2** | **1.69×** | **2.21×** |

`E/H` is exactCore median latency divided by Hyper median latency. Values above
one favor Hyper. The adapter and retained campaigns were collected at different
times and host loads, so their ratios show the change in benchmark question;
they are not a controlled estimate of wrapper overhead.

The cleanest conclusion is: **for these fixed, retained native inputs, Hyper is
still faster more often, but the previous near-sweep was largely an integration
artifact rather than evidence that exactCore's kernels were intrinsically an
order of magnitude slower.** The outcome is family-dependent. exactCore wins
21 of 29 lazy-real construction rows, while Hyper wins every vector and complex
row, 11 of 12 matrix rows, 24 of 30 3D-geometry rows, and every mesh row.

## What changed in the harness

The correctness oracle and its original `exactCorelib` benchmark tier are
unchanged. The added path is a benchmark-only retained native ABI:

- `ec_benchmark_fixture_new(operation_id)` parses the ID, constructs all fixed
  exactCore inputs, and retains them in a type-erased C++ fixture.
- `ec_benchmark_fixture_run(fixture, iterations)` executes a native C++ batch,
  so each Criterion sample crosses FFI once rather than once per operation.
- `ec_benchmark_fixture_free(fixture)` releases the fixture through RAII.
- Compiler memory barriers surround every operation, and each native result is
  made observable to prevent dead-code elimination.
- Native results do not cross the ABI and are never rendered to strings.
- Lazy real operations retain and return `CORE::Expr` graphs; they no longer end
  with `doubleValue()`, matching Hyper's lazy `Real` construction depth more
  closely.
- Fixed points, lines, segments, circles, planes, triangles, vectors, matrices,
  polynomials, bivariates, and Delaunay lift coordinates are built once.
- The Rust `PreparedBenchmark` owner is deliberately neither `Send` nor
  `Sync`, preserving the audit's native-concurrency restriction.

The timed loop still includes work intrinsic to the called API: constructing a
returned value, allocating an output container, copying a polynomial before a
mutating operation, or building a Sturm sequence. Excluding those would measure
a different operation. A shared-pointer state dereference, two compiler
barriers, the loop counter, and one amortized virtual/FFI call per Criterion
sample remain as harness overhead.

The Hyper closures already retained their fixed native inputs. For
`rational.parts`, the prior Hyper-side `to_string()` was also replaced with
native numerator/denominator clones so the retained comparison contains no
formatting on either side.

## Results by semantic eligibility

| Eligibility | Pairs | Hyper / exactCore / overlap | Geometric mean E/H | Median E/H |
|---|---:|---:|---:|---:|
| Matched | 123 | 80 / 43 / 0 | 1.43× | 1.99× |
| Adapted contract | 5 | 0 / 4 / 1 | 0.66× | 0.62× |
| Divergent family | 32 | 29 / 2 / 1 | 3.66× | 3.54× |
| **All** | **160** | **109 / 49 / 2** | **1.69×** | **2.21×** |

Known-wrong or differing contracts remain outside the matched headline. In
particular, `polynomial.root_count` exercises exactCore's known incorrect count
for the three-root cubic; its intervals overlap and it is not evidence for
choosing either implementation. The adapted Delaunay row also overlaps.

## Results by mathematical family

| Family | Pairs | Hyper / exactCore / overlap | Geometric mean E/H | Median E/H |
|---|---:|---:|---:|---:|
| Rational arithmetic | 9 | 7 / 2 / 0 | 4.10× | 7.56× |
| Lazy real expressions | 29 | 8 / 21 / 0 | 0.35× | 0.35× |
| Complex arithmetic | 5 | 5 / 0 / 0 | 2.24× | 2.90× |
| Vectors | 14 | 14 / 0 / 0 | 3.19× | 3.75× |
| Matrices | 12 | 11 / 1 / 0 | 3.90× | 4.30× |
| 2D geometry | 23 | 16 / 7 / 0 | 1.81× | 2.48× |
| 3D geometry | 30 | 24 / 6 / 0 | 2.59× | 2.95× |
| Polynomials/bivariates | 16 | 10 / 5 / 1 | 1.35× | 1.69× |
| Triangulation | 1 | 0 / 0 / 1 | 1.52× | 1.52× |
| Curves | 9 | 5 / 4 / 0 | 1.10× | 1.16× |
| Meshes | 5 | 5 / 0 / 0 | 10.88× | 10.87× |
| Paths | 7 | 4 / 3 / 0 | 1.82× | 1.25× |

The real-expression aggregate favors exactCore by about 2.83× on a geometric
mean basis (`1 / 0.3535`). This is expression-graph construction, not
transcendental numeric throughput. `real.exp2` and `real.exp10` are the
largest gaps—about 346× and 292× in exactCore's favor—because the two libraries
take very different lazy construction paths. Conversely, Hyper's largest
matched advantages include path endpoint equality (34.4×), path parameter order
(26.5×), polynomial composition (19.2×), matrix-3 inverse-family work (16.1×),
`real.tan` (15.9×), and incircle (11.4×).

The 102× exactCore advantage on the tiny `bivariate.resultant` fixture deserves
special caution. exactCore computes and evaluates a simple `resY` polynomial;
Hyper returns its general resultant-system report. The mathematical
specialization is checked as matched, but output richness and general machinery
differ. It is not a general elimination-throughput claim.

## Distribution and run quality

| Quality/result measure | Value |
|---|---:|
| Paired cases / analyzed functions | 160 / 320 |
| Measured samples | 9,600 |
| Criterion Tukey outliers | 872 (9.08%) |
| Median relative CI width across 320 estimates | 0.50% |
| Pairs with either relative CI width above 10% | 8 / 160 |
| Pairs with either relative CI width above 25% | 5 / 160 |
| E/H ratio p10 / p50 / p90 | 0.25× / 2.21× / 9.34× |

| Median-ratio bucket | Pairs |
|---|---:|
| exactCore at least 1.25× faster | 42 |
| Within 25% | 15 |
| Hyper 1.25–2× faster | 21 |
| Hyper 2–10× faster | 68 |
| Hyper 10–100× faster | 14 |
| Hyper above 100× faster | 0 |

The process was pinned to logical CPU 4. The host used an AMD Ryzen 7 5800X3D
with `amd-pstate-epp`, `powersave`, and boost enabled; frequency was not
locked and the SMT sibling was not isolated. Load varied during the campaign
and was 5.68 at the final check, materially below the roughly 22–24 load of the
historical adapter campaign. Absolute latencies are therefore run records, not
portable constants. The confidence intervals characterize sampling within one
process campaign, not process-to-process reproducibility.

| Item | Run value |
|---|---|
| Host | Fedora 43, Linux 7.1.8, x86-64 |
| CPU | AMD Ryzen 7 5800X3D, 8 cores / 16 threads, 96 MiB L3 |
| Affinity | Logical CPU 4; affinity only |
| Rust | rustc/cargo 1.97.0; LLVM 22.1.6 |
| Native compiler | GCC 15.3.1; C++11, `-O3 -DNDEBUG`, `CORE_LEVEL=3` |
| Native libraries | GMP 6.3.0, MPFR 4.2.2 |
| Criterion | 0.5.1; bench/release profile |
| Collection | 0.5 s warm-up, 1 s measurement, 30 samples per function |

### Source identities

| Crate | Revision | Worktree note after collection |
|---|---|---|
| `hyperreal` | `276f08c53b66` | clean |
| `hyperlattice` | `ac2aaebeaf67` | clean |
| `hyperlimit` | `b0418bddff50` | tracked files clean; pre-existing untracked fuzz artifacts/binary |
| `hypertri` | `1e21c5788b19` | clean |
| `hypersolve` | `5279e41e3394` | clean |
| `hypercurve` | `64bd27107fdb` | clean |
| `hypermesh` | `397f6fe01052` | clean |
| `hyperpath` | `d792aa8dc843` | clean |

The exactCore snapshot and standalone harness have no usable VCS revision
metadata in this workspace.

### Remote-main publication audit

On 2026-08-21, every remote `main` ref was freshly fetched and each collection
commit above was verified as an ancestor of that ref. `hypersolve` and
`hypercurve` required fast-forward pushes; the other collection commits were
already published. Later main-line commits do not relabel the benchmark inputs:
the collection hashes remain the exact identities used for these results.

| Crate | Full collection commit | Fetched `origin/main` after publication | Status |
|---|---|---|---|
| `hyperreal` | `276f08c53b66a2b43941320edc9a054082faaf57` | `276f08c53b66a2b43941320edc9a054082faaf57` | exact tip |
| `hyperlattice` | `ac2aaebeaf676f837b28ed103a4391f473bd7786` | `ac2aaebeaf676f837b28ed103a4391f473bd7786` | exact tip |
| `hyperlimit` | `b0418bddff50183fa782e5caa6da6974a2b969a1` | `b0418bddff50183fa782e5caa6da6974a2b969a1` | exact tip |
| `hypertri` | `1e21c5788b19ef43cbc34fd9010d6da6ea3815dc` | `79dc4a4661960c7ebb90cdb84991385fb5d06874` | contained in newer main |
| `hypersolve` | `5279e41e3394602aebc815bfa133007270bd2e8c` | `5279e41e3394602aebc815bfa133007270bd2e8c` | pushed; exact tip |
| `hypercurve` | `64bd27107fdbc87eea1f5151211588bcc9b0718d` | `e26ecef1a23b6a8ea325ec7e1a066f6d87603e10` | pushed; contained in newer main |
| `hypermesh` | `397f6fe01052d75f7944e34f1822557d086259a5` | `7851c61671b51418f4170778344a725bc95cb380` | contained in newer main |
| `hyperpath` | `d792aa8dc843218b26fc0d1730033e5cd06bdf2f` | `d792aa8dc843218b26fc0d1730033e5cd06bdf2f` | exact tip |

## Statistical rule

The analyzer uses Criterion's bootstrap median point estimate and marginal 95%
confidence interval. A pair is a Hyper win only when exactCore's lower median
bound exceeds Hyper's upper bound; exactCore wins under the converse condition;
otherwise it is an overlap. This conservative interval screen is descriptive,
not a paired hypothesis test.

## Important limits

- These are one fixed input and size per operation. They measure constant factors,
  not bit-length, degree, point-count, degeneracy, or asymptotic crossovers.
- Retained input objects may expose normal object-level caching or structural
  sharing. That is the intended retained-object contract, but it differs from a
  cold-object workload.
- Lazy-real rows compare graph construction. A numeric-throughput campaign must
  force both libraries to the same precision and certification target.
- `matrix3.inverse` and `matrix4.inverse` time exactCore's
  `bareissInverse` determinant-plus-adjugate path, while Hyper returns a
  normalized inverse. They share the inverse computational family but not
  identical result materialization.
- Some Hyper policy APIs return richer evidence than exactCore's reduced relation
  codes. The semantic-status column identifies adapted and divergent families,
  but even matched APIs can have different result carriers.
- The 32 divergent-family rows and five adapted rows are inventory measurements,
  not performance-selection evidence.
- All native calls ran sequentially. The harness deliberately does not test
  concurrent exactCore throughput because the audit reproduced native
  deallocation corruption under parallel geometry calls.
- Compiler barriers constrain dead-code elimination but are not hardware memory
  fences. Very small nanosecond values remain sensitive to compiler and ABI
  details.
- Output construction intrinsic to each public operation remains timed; the
  upgrade removes fixed *input* construction and adapter formatting, not the
  operation's own result work.

## Reproduce and inspect

A full run registers all three functions per case: the historical one-shot
`exactCorelib` adapter, the new `exactCorelib-retained` fixture, and Hyper.

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

To collect only the retained comparison, use Criterion filters
`exactCorelib-retained` and the relevant `/hyper*` suffix, as was done for
this campaign. Regenerate the retained summary or full table with:

```console
node scripts/analyze-criterion.mjs target/criterion summary retained
node scripts/analyze-criterion.mjs target/criterion json retained
node scripts/analyze-criterion.mjs target/criterion tsv retained
node scripts/analyze-criterion.mjs target/criterion markdown retained
```

The historical adapter tier remains the analyzer default; pass `adapter`
explicitly as the fourth argument when desired.

## Complete retained-native result table

| Operation | Semantic status | exactCore median [95% CI] | Hyper median [95% CI] | Median advantage | CI result |
|---|---|---:|---:|---:|---|
| `bivariate.evaluate` | matched | 85.09 ns [85.03 ns, 85.23 ns] | 115.45 ns [113.94 ns, 119.33 ns] | exactCore 1.36× | exactCore |
| `bivariate.resultant` | matched | 24.37 ns [24.35 ns, 24.39 ns] | 2.49 µs [2.47 µs, 2.50 µs] | exactCore 101.98× | exactCore |
| `complex.add` | matched | 146.08 ns [145.87 ns, 146.32 ns] | 50.30 ns [50.18 ns, 50.45 ns] | Hyper 2.90× | hyper |
| `complex.divide` | matched | 909.34 ns [907.94 ns, 911.13 ns] | 234.33 ns [234.08 ns, 234.92 ns] | Hyper 3.88× | hyper |
| `complex.multiply` | matched | 495.85 ns [494.43 ns, 497.57 ns] | 440.41 ns [439.83 ns, 445.51 ns] | Hyper 1.13× | hyper |
| `complex.norm_squared` | matched | 147.31 ns [147.15 ns, 147.60 ns] | 99.69 ns [99.47 ns, 99.89 ns] | Hyper 1.48× | hyper |
| `complex.subtract` | matched | 151.77 ns [151.61 ns, 151.89 ns] | 49.98 ns [49.69 ns, 53.21 ns] | Hyper 3.04× | hyper |
| `curve.circle_circle_relation` | adapted | 295.43 ns [294.73 ns, 296.29 ns] | 545.60 ns [542.46 ns, 551.30 ns] | exactCore 1.85× | exactCore |
| `curve.circle_point_distance` | matched | 124.73 ns [124.61 ns, 124.93 ns] | 503.30 ns [501.36 ns, 534.41 ns] | exactCore 4.04× | exactCore |
| `curve.line_contains_point` | matched | 215.23 ns [214.88 ns, 215.78 ns] | 115.04 ns [114.52 ns, 115.76 ns] | Hyper 1.87× | hyper |
| `curve.line_intersection_topology` | matched | 2.27 µs [2.26 µs, 2.27 µs] | 590.57 ns [589.08 ns, 591.97 ns] | Hyper 3.84× | hyper |
| `curve.line_intersection_witness` | matched | 866.03 ns [864.45 ns, 871.80 ns] | 580.95 ns [579.38 ns, 586.57 ns] | Hyper 1.49× | hyper |
| `curve.line_length` | matched | 110.84 ns [110.67 ns, 111.17 ns] | 275.50 ns [274.73 ns, 284.70 ns] | exactCore 2.49× | exactCore |
| `curve.line_side` | matched | 131.11 ns [130.95 ns, 131.36 ns] | 112.79 ns [111.85 ns, 114.62 ns] | Hyper 1.16× | hyper |
| `curve.segment_dispatch` | matched | 2.28 µs [2.28 µs, 2.29 µs] | 650.40 ns [647.92 ns, 654.47 ns] | Hyper 3.51× | hyper |
| `curve.supporting_line_circle` | adapted | 551.36 ns [550.07 ns, 552.52 ns] | 563.21 ns [560.16 ns, 566.93 ns] | exactCore 1.02× | exactCore |
| `geometry2.area` | matched | 125.10 ns [124.97 ns, 125.36 ns] | 160.11 ns [159.77 ns, 160.82 ns] | exactCore 1.28× | exactCore |
| `geometry2.between` | matched | 128.83 ns [124.34 ns, 131.75 ns] | 52.02 ns [51.93 ns, 52.17 ns] | Hyper 2.48× | hyper |
| `geometry2.circle_circle_distance` | adapted | 281.59 ns [280.98 ns, 282.82 ns] | 1.09 µs [1.07 µs, 1.14 µs] | exactCore 3.87× | exactCore |
| `geometry2.circle_line` | divergent-family | 540.10 ns [537.17 ns, 542.80 ns] | 383.43 ns [383.18 ns, 384.65 ns] | Hyper 1.41× | hyper |
| `geometry2.circle_point_distance` | matched | 126.72 ns [125.00 ns, 129.54 ns] | 515.00 ns [512.58 ns, 524.04 ns] | exactCore 4.06× | exactCore |
| `geometry2.circle_segment` | divergent-family | 1.31 µs [1.30 µs, 1.32 µs] | 845.77 ns [843.77 ns, 847.77 ns] | Hyper 1.54× | hyper |
| `geometry2.incircle` | matched | 866.84 ns [863.63 ns, 871.33 ns] | 76.18 ns [76.10 ns, 76.38 ns] | Hyper 11.38× | hyper |
| `geometry2.line_intersection` | matched | 881.00 ns [877.56 ns, 884.64 ns] | 319.38 ns [318.73 ns, 320.68 ns] | Hyper 2.76× | hyper |
| `geometry2.line_point_distance` | matched | 478.76 ns [452.98 ns, 597.45 ns] | 637.42 ns [635.87 ns, 641.27 ns] | exactCore 1.33× | exactCore |
| `geometry2.line_relation_0` | divergent-family | 129.88 ns [129.74 ns, 130.13 ns] | 26.08 ns [26.02 ns, 26.10 ns] | Hyper 4.98× | hyper |
| `geometry2.line_relation_1` | divergent-family | 128.39 ns [128.24 ns, 128.75 ns] | 25.97 ns [25.94 ns, 26.03 ns] | Hyper 4.94× | hyper |
| `geometry2.line_relation_2` | divergent-family | 225.26 ns [224.21 ns, 225.97 ns] | 141.75 ns [141.43 ns, 141.90 ns] | Hyper 1.59× | hyper |
| `geometry2.line_relation_3` | divergent-family | 367.49 ns [365.49 ns, 371.36 ns] | 140.93 ns [140.59 ns, 141.23 ns] | Hyper 2.61× | hyper |
| `geometry2.line_relation_4` | divergent-family | 128.76 ns [128.50 ns, 129.50 ns] | 142.01 ns [141.84 ns, 142.27 ns] | exactCore 1.10× | exactCore |
| `geometry2.line_relation_5` | divergent-family | 11.46 ns [11.45 ns, 11.49 ns] | 3.88 ns [3.88 ns, 3.90 ns] | Hyper 2.95× | hyper |
| `geometry2.line_relation_6` | divergent-family | 11.44 ns [11.42 ns, 11.48 ns] | 4.12 ns [4.11 ns, 4.12 ns] | Hyper 2.78× | hyper |
| `geometry2.orientation` | matched | 128.52 ns [128.36 ns, 128.72 ns] | 25.91 ns [25.89 ns, 25.98 ns] | Hyper 4.96× | hyper |
| `geometry2.point_distance` | matched | 105.84 ns [105.25 ns, 106.19 ns] | 148.56 ns [148.44 ns, 148.78 ns] | exactCore 1.40× | exactCore |
| `geometry2.segment_point_distance` | matched | 1.39 µs [1.38 µs, 1.40 µs] | 807.30 ns [804.76 ns, 808.17 ns] | Hyper 1.72× | hyper |
| `geometry2.segment_relation_0` | matched | 218.52 ns [218.01 ns, 218.77 ns] | 27.70 ns [27.68 ns, 27.77 ns] | Hyper 7.89× | hyper |
| `geometry2.segment_relation_1` | matched | 2.31 µs [2.31 µs, 2.32 µs] | 248.83 ns [248.52 ns, 249.32 ns] | Hyper 9.30× | hyper |
| `geometry2.segment_relation_2` | matched | 32.82 ns [32.08 ns, 33.57 ns] | 247.91 ns [247.52 ns, 248.20 ns] | exactCore 7.55× | exactCore |
| `geometry2.segment_relation_3` | matched | 370.09 ns [369.37 ns, 370.43 ns] | 119.40 ns [119.20 ns, 119.96 ns] | Hyper 3.10× | hyper |
| `geometry3.line_point_distance` | matched | 943.94 ns [883.73 ns, 1.22 µs] | 1.31 µs [1.30 µs, 1.31 µs] | exactCore 1.38× | exactCore |
| `geometry3.line_relation_0` | matched | 409.31 ns [408.54 ns, 410.89 ns] | 392.75 ns [392.02 ns, 393.40 ns] | Hyper 1.04× | hyper |
| `geometry3.line_relation_1` | matched | 337.29 ns [336.66 ns, 338.13 ns] | 456.30 ns [455.99 ns, 456.95 ns] | exactCore 1.35× | exactCore |
| `geometry3.line_relation_2` | matched | 680.15 ns [679.64 ns, 681.78 ns] | 74.37 ns [74.30 ns, 74.48 ns] | Hyper 9.15× | hyper |
| `geometry3.line_relation_3` | matched | 740.51 ns [739.64 ns, 743.65 ns] | 74.18 ns [74.11 ns, 74.24 ns] | Hyper 9.98× | hyper |
| `geometry3.line_relation_4` | matched | 410.28 ns [408.83 ns, 411.53 ns] | 456.45 ns [455.64 ns, 457.58 ns] | exactCore 1.11× | exactCore |
| `geometry3.orientation` | matched | 722.60 ns [720.65 ns, 726.64 ns] | 74.26 ns [74.19 ns, 74.37 ns] | Hyper 9.73× | hyper |
| `geometry3.plane_point_distance` | matched | 815.12 ns [814.47 ns, 817.20 ns] | 258.11 ns [257.75 ns, 258.43 ns] | Hyper 3.16× | hyper |
| `geometry3.plane_relation_0` | divergent-family | 221.61 ns [221.08 ns, 222.50 ns] | 33.43 ns [32.90 ns, 35.02 ns] | Hyper 6.63× | hyper |
| `geometry3.plane_relation_1` | divergent-family | 106.68 ns [105.83 ns, 106.94 ns] | 32.83 ns [32.80 ns, 32.87 ns] | Hyper 3.25× | hyper |
| `geometry3.plane_relation_2` | divergent-family | 242.94 ns [229.99 ns, 249.83 ns] | 141.58 ns [141.51 ns, 142.01 ns] | Hyper 1.72× | hyper |
| `geometry3.plane_relation_3` | divergent-family | 559.66 ns [558.82 ns, 560.37 ns] | 142.23 ns [141.94 ns, 143.87 ns] | Hyper 3.93× | hyper |
| `geometry3.plane_relation_4` | divergent-family | 226.94 ns [225.35 ns, 228.69 ns] | 142.46 ns [141.99 ns, 143.39 ns] | Hyper 1.59× | hyper |
| `geometry3.plane_relation_5` | divergent-family | 642.43 ns [639.65 ns, 648.82 ns] | 142.34 ns [142.14 ns, 144.76 ns] | Hyper 4.51× | hyper |
| `geometry3.plane_relation_6` | divergent-family | 329.28 ns [327.84 ns, 331.06 ns] | 110.25 ns [110.16 ns, 110.43 ns] | Hyper 2.99× | hyper |
| `geometry3.plane_relation_7` | divergent-family | 419.82 ns [416.85 ns, 424.23 ns] | 109.46 ns [109.36 ns, 109.58 ns] | Hyper 3.84× | hyper |
| `geometry3.point_distance` | matched | 202.58 ns [201.94 ns, 203.07 ns] | 234.42 ns [233.88 ns, 235.06 ns] | exactCore 1.16× | exactCore |
| `geometry3.segment_point_distance` | matched | 4.22 µs [4.20 µs, 4.24 µs] | 1.57 µs [1.57 µs, 1.57 µs] | Hyper 2.69× | hyper |
| `geometry3.segment_relation_0` | matched | 509.33 ns [508.09 ns, 509.84 ns] | 390.73 ns [390.20 ns, 391.42 ns] | Hyper 1.30× | hyper |
| `geometry3.segment_relation_1` | matched | 955.06 ns [954.41 ns, 956.88 ns] | 589.00 ns [587.99 ns, 590.78 ns] | Hyper 1.62× | hyper |
| `geometry3.segment_relation_2` | matched | 30.86 ns [30.83 ns, 30.92 ns] | 590.02 ns [589.18 ns, 590.90 ns] | exactCore 19.12× | exactCore |
| `geometry3.segment_relation_3` | matched | 740.43 ns [738.59 ns, 741.66 ns] | 74.00 ns [73.97 ns, 74.06 ns] | Hyper 10.01× | hyper |
| `geometry3.triangle_relation_0` | divergent-family | 1.47 µs [899.22 ns, 1.47 µs] | 285.07 ns [284.94 ns, 285.93 ns] | Hyper 5.14× | hyper |
| `geometry3.triangle_relation_1` | divergent-family | 5.92 µs [5.89 µs, 5.94 µs] | 286.83 ns [286.51 ns, 287.29 ns] | Hyper 20.64× | hyper |
| `geometry3.triangle_relation_2` | divergent-family | 2.32 µs [2.31 µs, 2.33 µs] | 285.59 ns [285.17 ns, 285.85 ns] | Hyper 8.11× | hyper |
| `geometry3.triangle_relation_3` | divergent-family | 5.91 µs [5.89 µs, 5.93 µs] | 285.39 ns [285.03 ns, 285.93 ns] | Hyper 20.71× | hyper |
| `geometry3.triangle_relation_4` | divergent-family | 4.56 µs [4.55 µs, 4.57 µs] | 2.79 µs [2.79 µs, 2.79 µs] | Hyper 1.63× | hyper |
| `geometry3.triangle_relation_5` | divergent-family | 2.96 µs [2.95 µs, 2.96 µs] | 2.79 µs [2.79 µs, 2.80 µs] | Hyper 1.06× | hyper |
| `geometry3.triangle_relation_6` | divergent-family | 11.10 µs [11.06 µs, 11.13 µs] | 16.70 µs [16.66 µs, 16.74 µs] | exactCore 1.50× | exactCore |
| `geometry3.volume` | matched | 1.60 µs [1.60 µs, 1.61 µs] | 551.42 ns [548.00 ns, 554.38 ns] | Hyper 2.90× | hyper |
| `matrix3.add` | matched | 1.01 µs [1.00 µs, 1.01 µs] | 441.18 ns [439.77 ns, 463.11 ns] | Hyper 2.28× | hyper |
| `matrix3.determinant` | divergent-family | 2.10 µs [2.10 µs, 2.10 µs] | 181.26 ns [180.96 ns, 181.76 ns] | Hyper 11.59× | hyper |
| `matrix3.inverse` | matched | 13.96 µs [13.95 µs, 13.99 µs] | 868.58 ns [866.55 ns, 869.93 ns] | Hyper 16.08× | hyper |
| `matrix3.multiply` | matched | 3.33 µs [3.33 µs, 3.34 µs] | 603.84 ns [600.89 ns, 614.66 ns] | Hyper 5.52× | hyper |
| `matrix3.subtract` | matched | 1.02 µs [1.01 µs, 1.02 µs] | 455.91 ns [453.95 ns, 461.54 ns] | Hyper 2.23× | hyper |
| `matrix3.transpose` | matched | 195.95 ns [195.75 ns, 196.47 ns] | 296.84 ns [296.42 ns, 297.48 ns] | exactCore 1.51× | exactCore |
| `matrix4.add` | matched | 1.90 µs [1.89 µs, 1.93 µs] | 616.68 ns [616.02 ns, 618.18 ns] | Hyper 3.08× | hyper |
| `matrix4.determinant` | divergent-family | 5.35 µs [5.25 µs, 5.38 µs] | 796.71 ns [795.93 ns, 799.26 ns] | Hyper 6.71× | hyper |
| `matrix4.inverse` | matched | 33.70 µs [33.62 µs, 34.57 µs] | 3.64 µs [3.62 µs, 3.65 µs] | Hyper 9.27× | hyper |
| `matrix4.multiply` | matched | 6.89 µs [6.88 µs, 6.91 µs] | 1.12 µs [1.09 µs, 1.19 µs] | Hyper 6.15× | hyper |
| `matrix4.subtract` | matched | 1.82 µs [1.81 µs, 1.84 µs] | 725.93 ns [701.28 ns, 741.10 ns] | Hyper 2.51× | hyper |
| `matrix4.transpose` | matched | 404.03 ns [403.46 ns, 404.64 ns] | 329.41 ns [329.16 ns, 329.76 ns] | Hyper 1.23× | hyper |
| `mesh.plane_point_classification` | matched | 542.04 ns [540.96 ns, 545.43 ns] | 91.62 ns [91.25 ns, 91.96 ns] | Hyper 5.92× | hyper |
| `mesh.triangle_boundary_point` | divergent-family | 5.15 µs [5.14 µs, 5.20 µs] | 641.81 ns [639.65 ns, 646.82 ns] | Hyper 8.02× | hyper |
| `mesh.triangle_contains_point` | divergent-family | 6.03 µs [6.00 µs, 6.15 µs] | 350.74 ns [349.80 ns, 356.07 ns] | Hyper 17.18× | hyper |
| `mesh.triangle_contains_point_strictly` | divergent-family | 6.02 µs [6.00 µs, 6.06 µs] | 349.83 ns [349.57 ns, 350.70 ns] | Hyper 17.22× | hyper |
| `mesh.triangle_triangle_intersection` | divergent-family | 24.68 µs [24.63 µs, 24.76 µs] | 2.27 µs [2.27 µs, 2.29 µs] | Hyper 10.87× | hyper |
| `path.circle_circle_relation` | matched | 287.07 ns [286.40 ns, 288.15 ns] | 1.15 µs [1.14 µs, 1.15 µs] | exactCore 4.00× | exactCore |
| `path.circle_point_membership` | matched | 127.32 ns [127.15 ns, 127.72 ns] | 101.56 ns [101.45 ns, 101.83 ns] | Hyper 1.25× | hyper |
| `path.circle_segment_intersection` | matched | 2.02 µs [2.02 µs, 2.03 µs] | 1.39 µs [1.38 µs, 1.40 µs] | Hyper 1.46× | hyper |
| `path.line_axis_classification` | matched | 185.11 ns [178.66 ns, 202.84 ns] | 593.25 ns [591.46 ns, 595.95 ns] | exactCore 3.20× | exactCore |
| `path.line_endpoint_equality` | matched | 815.03 ns [798.67 ns, 824.01 ns] | 23.67 ns [23.63 ns, 23.71 ns] | Hyper 34.44× | hyper |
| `path.line_length` | matched | 105.09 ns [104.92 ns, 105.31 ns] | 208.63 ns [208.17 ns, 208.81 ns] | exactCore 1.99× | exactCore |
| `path.line_parameter_order` | matched | 988.33 ns [978.25 ns, 993.98 ns] | 37.29 ns [37.09 ns, 37.48 ns] | Hyper 26.50× | hyper |
| `polynomial.binary_0` | matched | 317.46 ns [315.79 ns, 321.99 ns] | 184.28 ns [183.09 ns, 185.64 ns] | Hyper 1.72× | hyper |
| `polynomial.binary_1` | matched | 308.12 ns [307.90 ns, 308.64 ns] | 186.71 ns [186.35 ns, 187.48 ns] | Hyper 1.65× | hyper |
| `polynomial.binary_2` | matched | 930.29 ns [929.07 ns, 931.81 ns] | 189.79 ns [189.08 ns, 190.17 ns] | Hyper 4.90× | hyper |
| `polynomial.binary_3` | matched | 2.16 µs [2.14 µs, 2.17 µs] | 1.22 µs [1.22 µs, 1.22 µs] | Hyper 1.77× | hyper |
| `polynomial.binary_4` | matched | 2.67 µs [2.66 µs, 2.67 µs] | 139.32 ns [138.97 ns, 139.64 ns] | Hyper 19.16× | hyper |
| `polynomial.derivative` | matched | 426.94 ns [426.00 ns, 427.83 ns] | 51.84 ns [51.51 ns, 51.98 ns] | Hyper 8.24× | hyper |
| `polynomial.discriminant` | divergent-family | 5.27 µs [5.26 µs, 5.28 µs] | 4.11 µs [4.09 µs, 4.12 µs] | Hyper 1.28× | hyper |
| `polynomial.evaluate` | matched | 62.42 ns [62.40 ns, 62.59 ns] | 98.85 ns [98.62 ns, 98.91 ns] | exactCore 1.58× | exactCore |
| `polynomial.gcd` | matched | 2.59 µs [2.59 µs, 2.60 µs] | 1.18 µs [1.18 µs, 1.18 µs] | Hyper 2.19× | hyper |
| `polynomial.resultant` | adapted | 2.42 µs [2.40 µs, 2.43 µs] | 3.88 µs [3.80 µs, 3.99 µs] | exactCore 1.61× | exactCore |
| `polynomial.root_count` | divergent-family | 18.66 µs [18.63 µs, 18.69 µs] | 17.76 µs [17.71 µs, 18.95 µs] | Hyper 1.05× | overlap |
| `polynomial.root_count_interval` | matched | 16.27 µs [16.25 µs, 16.29 µs] | 34.37 µs [33.22 µs, 34.76 µs] | exactCore 2.11× | exactCore |
| `polynomial.root_isolation` | matched | 63.02 µs [62.90 µs, 63.12 µs] | 36.38 µs [36.02 µs, 36.51 µs] | Hyper 1.73× | hyper |
| `polynomial.square_free` | matched | 7.88 µs [7.87 µs, 7.89 µs] | 1.80 µs [1.70 µs, 1.95 µs] | Hyper 4.37× | hyper |
| `rational.absolute` | matched | 28.22 ns [27.66 ns, 29.61 ns] | 3.66 ns [3.64 ns, 3.67 ns] | Hyper 7.71× | hyper |
| `rational.add` | matched | 67.54 ns [67.11 ns, 68.01 ns] | 7.27 ns [7.26 ns, 7.28 ns] | Hyper 9.29× | hyper |
| `rational.compare` | matched | 3.37 ns [3.37 ns, 3.39 ns] | 3.61 ns [3.59 ns, 3.65 ns] | exactCore 1.07× | exactCore |
| `rational.divide` | matched | 108.71 ns [104.70 ns, 112.68 ns] | 119.23 ns [119.08 ns, 119.66 ns] | exactCore 1.10× | exactCore |
| `rational.multiply` | matched | 108.29 ns [103.10 ns, 112.61 ns] | 14.32 ns [14.24 ns, 14.36 ns] | Hyper 7.56× | hyper |
| `rational.negate` | matched | 28.06 ns [28.01 ns, 28.12 ns] | 10.45 ns [10.41 ns, 10.52 ns] | Hyper 2.68× | hyper |
| `rational.parts` | matched | 35.03 ns [35.00 ns, 35.10 ns] | 4.20 ns [4.20 ns, 4.20 ns] | Hyper 8.34× | hyper |
| `rational.reciprocal` | matched | 30.76 ns [30.74 ns, 30.86 ns] | 7.65 ns [7.64 ns, 7.67 ns] | Hyper 4.02× | hyper |
| `rational.subtract` | matched | 67.25 ns [67.16 ns, 67.50 ns] | 8.52 ns [8.51 ns, 8.55 ns] | Hyper 7.89× | hyper |
| `real.absolute` | matched | 1.26 ns [1.26 ns, 1.26 ns] | 39.04 ns [37.92 ns, 40.15 ns] | exactCore 30.93× | exactCore |
| `real.acos` | matched | 528.98 ns [528.15 ns, 529.47 ns] | 148.99 ns [148.39 ns, 149.42 ns] | Hyper 3.55× | hyper |
| `real.add` | matched | 15.37 ns [15.33 ns, 15.44 ns] | 33.16 ns [33.02 ns, 33.26 ns] | exactCore 2.16× | exactCore |
| `real.asin` | matched | 531.73 ns [531.37 ns, 532.46 ns] | 155.60 ns [155.33 ns, 155.99 ns] | Hyper 3.42× | hyper |
| `real.atan` | matched | 136.47 ns [136.34 ns, 136.65 ns] | 166.33 ns [161.64 ns, 174.57 ns] | exactCore 1.22× | exactCore |
| `real.cbrt` | matched | 30.74 ns [30.70 ns, 30.77 ns] | 650.05 ns [646.15 ns, 653.73 ns] | exactCore 21.15× | exactCore |
| `real.ceil` | matched | 811.68 ns [810.57 ns, 814.25 ns] | 95.06 ns [94.80 ns, 95.19 ns] | Hyper 8.54× | hyper |
| `real.cos` | matched | 11.53 ns [11.52 ns, 11.56 ns] | 234.12 ns [233.50 ns, 235.16 ns] | exactCore 20.31× | exactCore |
| `real.cot` | matched | 1.56 µs [1.55 µs, 1.56 µs] | 648.47 ns [637.43 ns, 682.76 ns] | Hyper 2.40× | hyper |
| `real.divide` | matched | 17.25 ns [17.24 ns, 17.27 ns] | 97.82 ns [96.66 ns, 100.91 ns] | exactCore 5.67× | exactCore |
| `real.e` | matched | 14.33 ns [14.30 ns, 14.35 ns] | 51.07 ns [50.58 ns, 51.34 ns] | exactCore 3.56× | exactCore |
| `real.exp` | matched | 17.96 ns [17.57 ns, 18.13 ns] | 115.50 ns [115.05 ns, 115.71 ns] | exactCore 6.43× | exactCore |
| `real.exp10` | matched | 16.43 ns [16.43 ns, 16.45 ns] | 4.79 µs [4.79 µs, 4.81 µs] | exactCore 291.68× | exactCore |
| `real.exp2` | matched | 16.89 ns [16.88 ns, 16.94 ns] | 5.84 µs [5.77 µs, 5.88 µs] | exactCore 346.02× | exactCore |
| `real.floor` | matched | 624.76 ns [622.94 ns, 626.56 ns] | 50.71 ns [50.67 ns, 50.83 ns] | Hyper 12.32× | hyper |
| `real.ln` | matched | 16.68 ns [16.67 ns, 16.70 ns] | 113.26 ns [112.98 ns, 113.43 ns] | exactCore 6.79× | exactCore |
| `real.log10` | matched | 16.52 ns [16.46 ns, 16.96 ns] | 164.84 ns [160.72 ns, 167.91 ns] | exactCore 9.98× | exactCore |
| `real.log2` | matched | 16.19 ns [16.18 ns, 16.21 ns] | 167.87 ns [167.34 ns, 168.48 ns] | exactCore 10.37× | exactCore |
| `real.multiply` | matched | 15.54 ns [15.52 ns, 15.57 ns] | 35.70 ns [35.66 ns, 35.77 ns] | exactCore 2.30× | exactCore |
| `real.negate` | matched | 16.45 ns [16.42 ns, 16.49 ns] | 47.62 ns [46.40 ns, 56.35 ns] | exactCore 2.89× | exactCore |
| `real.pi` | matched | 14.14 ns [14.07 ns, 21.28 ns] | 36.82 ns [36.79 ns, 36.88 ns] | exactCore 2.60× | exactCore |
| `real.powi` | matched | 129.23 ns [128.24 ns, 130.79 ns] | 60.42 ns [60.32 ns, 60.60 ns] | Hyper 2.14× | hyper |
| `real.root_n` | matched | 495.94 ns [492.63 ns, 498.19 ns] | 661.76 ns [660.05 ns, 663.00 ns] | exactCore 1.33× | exactCore |
| `real.sign` | matched | 1.51 µs [1.51 µs, 1.52 µs] | 262.72 ns [262.41 ns, 263.56 ns] | Hyper 5.76× | hyper |
| `real.sin` | matched | 11.67 ns [11.64 ns, 11.69 ns] | 241.36 ns [235.42 ns, 251.90 ns] | exactCore 20.69× | exactCore |
| `real.sqrt` | matched | 17.85 ns [17.84 ns, 17.89 ns] | 142.61 ns [142.03 ns, 143.15 ns] | exactCore 7.99× | exactCore |
| `real.square` | matched | 15.15 ns [15.04 ns, 15.42 ns] | 47.25 ns [47.16 ns, 47.38 ns] | exactCore 3.12× | exactCore |
| `real.subtract` | matched | 16.72 ns [16.62 ns, 16.75 ns] | 32.72 ns [32.63 ns, 32.79 ns] | exactCore 1.96× | exactCore |
| `real.tan` | matched | 1.20 µs [1.19 µs, 1.29 µs] | 75.56 ns [74.17 ns, 76.36 ns] | Hyper 15.91× | hyper |
| `triangulation.delaunay_complex` | adapted | 150.64 µs [150.06 µs, 157.31 µs] | 99.34 µs [77.76 µs, 154.51 µs] | Hyper 1.52× | overlap |
| `vector2.add` | matched | 191.92 ns [191.69 ns, 192.38 ns] | 51.39 ns [51.20 ns, 51.52 ns] | Hyper 3.73× | hyper |
| `vector2.dot` | matched | 254.05 ns [253.86 ns, 254.61 ns] | 56.96 ns [56.78 ns, 57.34 ns] | Hyper 4.46× | hyper |
| `vector2.norm` | matched | 176.46 ns [176.30 ns, 176.84 ns] | 88.66 ns [88.45 ns, 88.84 ns] | Hyper 1.99× | hyper |
| `vector2.subtract` | matched | 194.46 ns [193.79 ns, 194.96 ns] | 53.35 ns [53.29 ns, 53.56 ns] | Hyper 3.64× | hyper |
| `vector2.wedge` | matched | 228.21 ns [227.04 ns, 244.37 ns] | 57.61 ns [57.36 ns, 57.68 ns] | Hyper 3.96× | hyper |
| `vector3.add` | matched | 282.23 ns [281.63 ns, 283.02 ns] | 80.27 ns [80.03 ns, 80.39 ns] | Hyper 3.52× | hyper |
| `vector3.cross` | matched | 907.58 ns [904.42 ns, 911.66 ns] | 189.10 ns [188.66 ns, 189.34 ns] | Hyper 4.80× | hyper |
| `vector3.dot` | matched | 363.67 ns [362.81 ns, 364.21 ns] | 82.61 ns [82.37 ns, 83.01 ns] | Hyper 4.40× | hyper |
| `vector3.norm` | matched | 206.92 ns [206.67 ns, 207.36 ns] | 177.73 ns [177.28 ns, 178.70 ns] | Hyper 1.16× | hyper |
| `vector3.subtract` | matched | 310.06 ns [289.68 ns, 318.32 ns] | 82.31 ns [81.95 ns, 82.78 ns] | Hyper 3.77× | hyper |
| `vector4.add` | matched | 391.11 ns [390.52 ns, 391.94 ns] | 102.22 ns [102.05 ns, 102.39 ns] | Hyper 3.83× | hyper |
| `vector4.dot` | matched | 474.93 ns [474.15 ns, 475.54 ns] | 99.30 ns [99.16 ns, 99.47 ns] | Hyper 4.78× | hyper |
| `vector4.norm` | matched | 256.36 ns [255.77 ns, 257.02 ns] | 244.17 ns [242.74 ns, 245.75 ns] | Hyper 1.05× | hyper |
| `vector4.subtract` | matched | 390.21 ns [389.38 ns, 393.85 ns] | 104.59 ns [104.32 ns, 104.90 ns] | Hyper 3.73× | hyper |
