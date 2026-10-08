# exactCorelib / Hyper comprehensive memory comparison

Run date: 2026-08-21 (America/Detroit)

This report records the full allocator-instrumented, process-isolated memory
campaign: **392 worker measurements / 196 exactCorelib–Hyper pairs**. That is 72
workers for the retained geometry size sweep and 320 workers covering all 160
concrete comparable operations. Every fixture was traversed twice. The latency
campaign remains separate in
[RETAINED_BENCHMARK_REPORT.md](RETAINED_BENCHMARK_REPORT.md), because allocation
interposition would perturb its hot paths.

The complete generated tables are in
[MEMORY_SWEEP_RESULTS.md](MEMORY_SWEEP_RESULTS.md), including one paired row for
every comparable operation and every geometry configuration. The recorded raw
source is [`results/memory-sweep-2026-08-21.tsv`](results/memory-sweep-2026-08-21.tsv).

## Executive result

The operation manifest is fully covered: all **160 of 160** concrete operation
IDs produced complete exactCorelib/Hyper pairs. Hyper used less retained fixture
payload for 108 operations, exactCorelib for 51, with one tie. Hyper generated
less operation-phase allocation traffic for 119 operations, exactCorelib for 36,
with five ties. These are unweighted fixture counts, not a composite performance
score; the operations and native result contracts vary substantially.

In the geometry scaling tier, Hyper retained the same exact integer geometry in
**3.03–3.95× less logical heap memory** at the largest tested sizes. Its
streaming line classifier performed no heap allocation. At the largest triangle
sizes, exactCore generated 23.29× the allocation traffic per point query and
45.73× the traffic per triangle-pair query.

The most important finding is not a ratio: exactCore's
`Triangle3d::intersects(const Triangle3d&)` path retains approximately **60 KiB
per processed pair after the fixture and MPFR cache are destroyed**. With 1,024
retained pairs traversed twice, 120.00 MiB remained logically live and the
process stayed roughly 210 MiB above its initial RSS. Hyper left 43.72 KiB
logically live in the corresponding worker.

This is consistent with—and source inspection directly explains—a native leak.
`Triangle3d::intersects` obtains a heap-returned `GeomObj*` from `intersection`,
reads `dim()`, and never deletes it. The nested triangle intersection path also
abandons intermediate heap objects. The instrumented result should therefore be
read as the memory behavior of the public exactCore operation, not as an
intrinsic cost of exact arithmetic.

| Largest streaming workload | Cases | Retained exact / Hyper | Allocation traffic per item exact / Hyper | Post-operation growth exact / Hyper | Residual after teardown exact / Hyper |
|---|---:|---:|---:|---:|---:|
| Line / point side | 8,192 | 21.06 MiB / 6.96 MiB | 1.53 KiB / 0 B | 648 B / 0 B | 0 B / 1.93 KiB |
| Triangle / point location | 4,096 | 18.91 MiB / 5.29 MiB | 14.99 KiB / 659 B | 648 B / 5.15 MiB | 0 B / 15.15 KiB |
| Triangle / triangle relation | 1,024 | 7.09 MiB / 1.80 MiB | 164.34 KiB / 3.59 KiB | 113.11 MiB / 2.71 MiB | 120.00 MiB / 43.72 KiB |

`Post-operation growth` is storage still live after both traversals while the
fixture remains alive. `Residual` is storage still live after fixture teardown
and `mpfr_free_cache()`. It is intentionally distinct from allocator-held RSS.

## Comparable-operation sweep

The fixed tier uses the same concrete IDs as the Criterion comparison suite and
retains each library's native fixture outside the measured operation phase. Its
traffic number is the mean requested allocation/reallocation payload over the
two executions. Fixture payload and traffic answer different questions, so the
report does not combine them into one score.

| Metric across 160 operations | exactCorelib lower | Hyper lower | Tie |
|---|---:|---:|---:|
| Retained fixture payload | 51 | 108 | 1 |
| Operation traffic per iteration | 36 | 119 | 5 |

Hyper performed zero operation-phase allocation in 34 fixtures and exactCorelib
in six; five of those were zero for both. A zero here means that the retained
fixture already contains everything needed by the measured public call.

The category breakdown makes the split clearer. `E/H/T` means exactCorelib
lower, Hyper lower, or tied within that category.

| Category | Operations | Retained E/H/T | Traffic E/H/T |
|---|---:|---:|---:|
| Bivariate polynomial | 2 | 2 / 0 / 0 | 1 / 1 / 0 |
| Complex | 5 | 4 / 0 / 1 | 3 / 2 / 0 |
| Curve | 9 | 1 / 8 / 0 | 1 / 8 / 0 |
| 2D geometry | 23 | 0 / 23 / 0 | 3 / 17 / 3 |
| 3D geometry | 30 | 0 / 30 / 0 | 2 / 28 / 0 |
| Matrix 3×3 | 6 | 6 / 0 / 0 | 0 / 6 / 0 |
| Matrix 4×4 | 6 | 6 / 0 / 0 | 0 / 6 / 0 |
| Mesh | 5 | 1 / 4 / 0 | 0 / 5 / 0 |
| Path | 7 | 1 / 6 / 0 | 0 / 7 / 0 |
| Univariate polynomial | 14 | 14 / 0 / 0 | 4 / 10 / 0 |
| Rational | 9 | 5 / 4 / 0 | 5 / 3 / 1 |
| Real | 29 | 0 / 29 / 0 | 17 / 11 / 1 |
| Triangulation | 1 | 0 / 1 / 0 | 0 / 1 / 0 |
| Vector 2D | 5 | 4 / 1 / 0 | 0 / 5 / 0 |
| Vector 3D | 5 | 4 / 1 / 0 | 0 / 5 / 0 |
| Vector 4D | 4 | 3 / 1 / 0 | 0 / 4 / 0 |

Representative traffic extremes show why counts alone are insufficient:

| Operation | exactCorelib / iteration | Hyper / iteration | Lower-traffic implementation |
|---|---:|---:|---:|
| `geometry3.triangle_relation_1` | 13,736 B | 52 B | Hyper, 264.15× |
| `real.exp10` | 104 B | 6,180 B | exactCorelib, 59.42× |
| `real.exp2` | 104 B | 4,984 B | exactCorelib, 47.92× |
| `mesh.triangle_contains_point` | 13,736 B | 352 B | Hyper, 39.02× |
| `matrix4.inverse` | 38,224 B | 1,664 B | Hyper, 22.97× |
| `polynomial.evaluate` | 48 B | 384 B | exactCorelib, 8.00× |
| `polynomial.root_isolation` | 70,696 B | 14,196 B | Hyper, 4.98× |
| `triangulation.delaunay_complex` | 152,220 B | 82,516 B | Hyper, 1.84× |

The largest one-fixture residuals were 38,728 B for exactCorelib's
`mesh.triangle_triangle_intersection` and 59,360 B for Hyper's
`triangulation.delaunay_complex`. Eleven exactCorelib workers and 153 Hyper
workers had a nonzero logical residual. In a cold, single-fixture worker that
establishes process-lifetime retained state, but does not by itself distinguish
a bounded lazy cache from per-call growth. The geometry size sweep below makes
that distinction where multiple sizes are available.

Every operation and all five primary allocator metrics are listed in the
[complete results appendix](MEMORY_SWEEP_RESULTS.md#comparable-operation-coverage).

## Retained-input scaling

Least-squares slopes across all six sizes quantify the logical payload retained
for native inputs. Tracker headers, malloc metadata, fragmentation, code, stacks,
and the packed source-coordinate array are excluded; RSS/PSS capture their
process-level effects.

| Workload | exactCore slope | Hyper slope | Largest-size E/H ratio |
|---|---:|---:|---:|
| Line / point | 2,696.0 B/case | 895.1 B/case | 3.03× |
| Triangle / point | 4,840.0 B/case | 1,358.5 B/case | 3.58× |
| Triangle / triangle | 7,264.0 B/case | 1,840.7 B/case | 3.95× |

The exactCore slopes are constant because its retained points follow one stable
`Expr` representation. Hyper's fitted slopes reflect the integer range in the
deterministic grid as well as the geometry carrier; larger coordinates cross
internal integer-storage thresholds. The largest-size ratios are consequently
more useful for capacity planning than extrapolating the fitted intercept to a
different coordinate distribution.

## Execution behavior

Allocation traffic is total requested bytes from allocation plus the new
requested size of every reallocation. It measures allocator work, not
simultaneously live memory.

| Largest streaming workload | exactCore alloc / realloc calls | Hyper alloc / realloc calls | exactCore traffic/item | Hyper traffic/item |
|---|---:|---:|---:|---:|
| Line / point, 16,384 evaluations | 532,568 / 49,407 | 0 / 0 | 1,563 B | 0 B |
| Triangle / point, 8,192 evaluations | 1,999,898 / 44,907 | 30,684 / 0 | 15,345 B | 659 B |
| Triangle / triangle, 2,048 evaluations | 6,088,565 / 373,448 | 54,667 / 0 | 168,283 B | 3,680 B |

Hyper's triangle-point operation grows the retained object graph by 5.15 MiB at
4,096 cases. Almost all of it is released with the fixture: only 15.15 KiB
remains afterward. This is consistent with exact state or memoized expression
data becoming attached to retained inputs, rather than per-call process-lifetime
loss. exactCore's triangle-point growth is a constant 648 B and is fully
released.

The triangle-pair picture differs. exactCore's fitted residual slope is
122,879.7 B per fixture case with two traversals, or 61,439.8 B per processed
pair. Hyper's residual slope is 19.9 B per fixture case across the same six-size
fit.

## Streaming versus materialized output

Streaming mode observes and immediately drops each native result. Materialized
mode reserves one output array, retains all results until the traversal ends,
then drops it. Subtracting streaming from materialized ephemeral peak recovers
the native carrier size exactly:

| Workload result | exactCore | Hyper |
|---|---:|---:|
| Line side | 4 B/result | 3 B/result |
| Triangle point location | 4 B/result | 3 B/result |
| Triangle-pair relation/report | 4 B/result | 56 B/result |

The 56-byte Hyper triangle-pair result retains predicate evidence; exactCore
returns only an integer intersection dimension. Materialized-output memory is
therefore a contract/output-richness comparison, not a like-for-like allocator
efficiency result. Streaming mode is the cleaner kernel comparison.

## Process working set

Linux rollup RSS and PSS were sampled before fixture construction, after
construction, after execution, and after teardown. `VmHWM` supplies the kernel's
automatic high-water mark. The table reports execution snapshot and high-water
deltas from the pre-fixture baseline at the largest streaming sizes.

| Workload | RSS delta exact / Hyper | PSS delta exact / Hyper | HWM delta exact / Hyper |
|---|---:|---:|---:|
| Line / point | 38.66 MiB / 10.39 MiB | 38.65 MiB / 10.39 MiB | 38.56 MiB / 10.15 MiB |
| Triangle / point | 35.52 MiB / 14.53 MiB | 35.48 MiB / 14.50 MiB | 35.46 MiB / 14.38 MiB |
| Triangle / triangle | 210.08 MiB / 7.42 MiB | 210.07 MiB / 7.42 MiB | 209.93 MiB / 7.31 MiB |

Logical teardown and RSS answer different questions. For example, exactCore's
line and triangle-point workers have zero tracked residual, but glibc need not
return freed arenas to the kernel before process exit. Conversely, the
triangle-pair logical residual and RSS both grow strongly, corroborating the
leak rather than merely allocator caching.

RSS deltas below roughly one MiB are dominated by process startup, code-page
faults, proc-file sampling, and allocator granularity. The size sweep and
allocator counters—not a single small-worker RSS subtraction—are the primary
evidence.

## Complete geometry primary-metric table

Each row combines the paired streaming and materialized workers for one input
size. `S traffic` and `M traffic` are total operation-phase requested bytes for
streaming and materialized modes. Retained, post-operation, and residual values
are identical across modes, as expected.

| Workload | Cases | Retained exact / Hyper | S traffic exact / Hyper | M traffic exact / Hyper | Post-op growth exact / Hyper | Residual exact / Hyper |
|---|---:|---:|---:|---:|---:|---:|
| Line / point | 8 | 21.09 KiB / 5.09 KiB | 25.48 KiB / 0 B | 25.55 KiB / 48 B | 448 B / 0 B | 0 B / 1.02 KiB |
| Line / point | 32 | 84.28 KiB / 19.16 KiB | 95.73 KiB / 0 B | 95.98 KiB / 192 B | 448 B / 0 B | 0 B / 1.02 KiB |
| Line / point | 128 | 337.03 KiB / 75.41 KiB | 376.73 KiB / 0 B | 377.73 KiB / 768 B | 448 B / 0 B | 0 B / 1.02 KiB |
| Line / point | 512 | 1.32 MiB / 300.51 KiB | 1.50 MiB / 0 B | 1.50 MiB / 3.00 KiB | 648 B / 0 B | 0 B / 1.73 KiB |
| Line / point | 2,048 | 5.27 MiB / 1.63 MiB | 6.08 MiB / 0 B | 6.09 MiB / 12.00 KiB | 648 B / 0 B | 0 B / 1.93 KiB |
| Line / point | 8,192 | 21.06 MiB / 6.96 MiB | 24.42 MiB / 0 B | 24.48 MiB / 48.00 KiB | 648 B / 0 B | 0 B / 1.93 KiB |
| Triangle / point | 4 | 18.94 KiB / 4.38 KiB | 116.42 KiB / 9.19 KiB | 116.45 KiB / 9.21 KiB | 448 B / 8.78 KiB | 0 B / 8.73 KiB |
| Triangle / point | 16 | 75.66 KiB / 17.02 KiB | 474.80 KiB / 19.16 KiB | 474.92 KiB / 19.25 KiB | 448 B / 18.75 KiB | 0 B / 11.46 KiB |
| Triangle / point | 64 | 302.53 KiB / 63.63 KiB | 1.86 MiB / 52.33 KiB | 1.86 MiB / 52.70 KiB | 448 B / 51.92 KiB | 0 B / 11.73 KiB |
| Triangle / point | 256 | 1.18 MiB / 249.63 KiB | 7.46 MiB / 184.33 KiB | 7.46 MiB / 185.83 KiB | 648 B / 183.92 KiB | 0 B / 11.73 KiB |
| Triangle / point | 1,024 | 4.73 MiB / 1.17 MiB | 29.95 MiB / 1.04 MiB | 29.95 MiB / 1.04 MiB | 648 B / 1.04 MiB | 0 B / 15.15 KiB |
| Triangle / point | 4,096 | 18.91 MiB / 5.29 MiB | 119.88 MiB / 5.15 MiB | 119.92 MiB / 5.17 MiB | 648 B / 5.15 MiB | 0 B / 15.15 KiB |
| Triangle / pair | 1 | 7.13 KiB / 1.55 KiB | 339.59 KiB / 14.33 KiB | 339.60 KiB / 14.44 KiB | 114.47 KiB / 13.41 KiB | 120.92 KiB / 14.13 KiB |
| Triangle / pair | 4 | 28.41 KiB / 6.42 KiB | 1.20 MiB / 27.73 KiB | 1.20 MiB / 28.16 KiB | 452.30 KiB / 22.27 KiB | 479.42 KiB / 19.96 KiB |
| Triangle / pair | 16 | 113.53 KiB / 24.98 KiB | 4.81 MiB / 94.46 KiB | 4.81 MiB / 96.21 KiB | 1.77 MiB / 51.12 KiB | 1.87 MiB / 30.20 KiB |
| Triangle / pair | 64 | 454.03 KiB / 94.73 KiB | 19.29 MiB / 323.67 KiB | 19.29 MiB / 330.67 KiB | 7.06 MiB / 141.87 KiB | 7.49 MiB / 37.51 KiB |
| Triangle / pair | 256 | 1.77 MiB / 373.73 KiB | 80.39 MiB / 1.19 MiB | 80.39 MiB / 1.22 MiB | 28.26 MiB / 476.95 KiB | 29.98 MiB / 37.71 KiB |
| Triangle / pair | 1,024 | 7.09 MiB / 1.80 MiB | 328.68 MiB / 7.19 MiB | 328.69 MiB / 7.30 MiB | 113.11 MiB / 2.71 MiB | 120.00 MiB / 43.72 KiB |

## Instrumentation design

The memory campaign is a feature-gated executable rather than Criterion
instrumentation:

- `memory-profile` conditionally compiles `cpp/exactcore_memory.cpp`. Ordinary
  tests and latency benchmarks do not contain its allocation overrides.
- The exactCore worker interposes global C++ `new`/`delete` and installs GMP's
  allocation, reallocation, and free hooks before native static objects. MPFR
  allocations use the GMP hooks. Every allocation carries its requested size
  and whether it belongs to a measured phase.
- The Hyper worker uses an equivalent Rust global allocator with an aligned
  prefix header, allowing allocations made during a phase to be distinguished
  from process-startup allocations even when freed later.
- Counters track allocation, reallocation, and deallocation calls; requested
  bytes; current logical live bytes; and peak logical live bytes.
- Fixture construction, execution, and teardown are individually reset and
  sampled. Resetting counters preserves the current live-byte baseline.
- Each library/workload/mode/size or library/operation tuple runs in a fresh
  child process. This isolates caches, allocator arenas, and `VmHWM`, and
  serializes exactCore.
- Fixed-operation workers use the same 160 IDs and native retained fixtures as
  the Criterion campaign, paired with retained Hyper fixtures that execute the
  corresponding public APIs.
- Identical packed `i64` coordinate records are generated once before tracking
  and supplied to each native fixture. No parsing or input construction occurs
  during execution.
- Linux `smaps_rollup` supplies RSS, PSS, and private mappings;
  `/proc/self/status` supplies `VmHWM`.

The workloads use mixed line sides, interior/edge/exterior coplanar triangle
points, and alternating intersecting/disjoint noncoplanar triangle pairs. The
triangle pairs deliberately avoid exactCore's unsafe coplanar clipping path.

## Interpretation limits

- Requested payload bytes exclude the benchmark's allocation headers, malloc
  metadata, alignment padding, fragmentation, code, stacks, and file mappings.
  RSS/PSS include those effects but are page-granular and noisier.
- Direct C allocation outside C++ `new` and GMP/MPFR hooks would not appear in
  logical counters. The audited exactCore geometry path uses C++ objects and
  GMP/MPFR storage; RSS remains an independent backstop.
- This campaign measures cold isolated workers. Persistent caches are charged
  to the first phase that creates them. A long-running application may amortize
  fixed caches but cannot amortize per-operation leaks.
- Input coordinates are exact integers of modest magnitude. Rational
  denominators, algebraic expressions, degeneracies, and increasing bit length
  require separate sweeps.
- `Line2d::orientation`, `Triangle3d::contains`, and triangle intersection have
  known exactCore semantic defects documented in
  [COMPARISON_REPORT.md](COMPARISON_REPORT.md). The workloads measure their
  public implementation memory; the divergent results are not correctness
  endorsements.
- Hyper triangle-pair output is a retained evidence report while exactCore
  returns an integer. Use streaming numbers for kernel comparison and the
  materialized result sizes for capacity planning under each native contract.
- Allocation tracking is deliberately not a timing tool. Continue to use the
  uninstrumented retained Criterion tier for latency.

## Reproduce and analyze

```console
cargo build --locked --release --features memory-profile --bin memory-sweep
target/release/memory-sweep --repetitions 2 \
  --output target/memory-sweep/results.tsv
node scripts/analyze-memory.mjs target/memory-sweep/results.tsv summary
node scripts/analyze-memory.mjs target/memory-sweep/results.tsv markdown
node scripts/analyze-memory.mjs target/memory-sweep/results.tsv json
```

The default command runs both tiers. Use `--operations-only` or `--scaling-only`
to isolate one, and add `--quick` for its smoke subset. `--operations`,
`--sizes`, `--workloads`, `--modes`, and `--repetitions` select custom inputs;
`--list-operations` prints the full catalog. The committed raw TSV has 393 lines
and SHA-256
`ac62c41a0d60d6a670e7c4535b56303f73b870f8d7992eee39da50058c5599ac`.

| Item | Run value |
|---|---|
| Host | Fedora 43, Linux 7.1.8, x86-64 |
| CPU | AMD Ryzen 7 5800X3D, 8 cores / 16 threads, 96 MiB L3 |
| Rust | rustc/cargo 1.97.0; release profile |
| Native compiler | GCC 15.3.1; C++11, `-O3 -DNDEBUG`, `CORE_LEVEL=3` |
| Native libraries | GMP 6.3.0, MPFR 4.2.2 |
| Sweep | 392 workers; 196 pairs; two traversals per fixture |

## Hyper source identities

The release executable linked all eight local path dependencies below. Seven
were clean at the recorded commit. `hypercurve` had tracked working-tree changes
at build time; no source file changed after the executable timestamp, and the
binary patch digest records that exact deviation from its base commit.

| Crate | Base commit | Recorded state |
|---|---|---|
| `hyperreal` | `276f08c53b66a2b43941320edc9a054082faaf57` | clean |
| `hyperlattice` | `ac2aaebeaf676f837b28ed103a4391f473bd7786` | clean |
| `hyperlimit` | `b0418bddff50183fa782e5caa6da6974a2b969a1` | tracked files clean |
| `hypertri` | `1e21c5788b19ef43cbc34fd9010d6da6ea3815dc` | clean |
| `hypersolve` | `5279e41e3394602aebc815bfa133007270bd2e8c` | clean |
| `hypercurve` | `e2ffe6943b5a97c044192a6692350ade22d40391` | tracked modifications; `git diff --binary` SHA-256 `53948ef95a719ff18b817ca04c4ffd8e7468fc80015ea08d42477be1b316fa48` |
| `hypermesh` | `397f6fe01052d75f7944e34f1822557d086259a5` | clean |
| `hyperpath` | `d792aa8dc843218b26fc0d1730033e5cd06bdf2f` | clean |

`hyperlimit` retained pre-existing untracked fuzz data and a standalone binary;
none are Cargo inputs. The all-feature test suite and clippy had passed before
this release build, and the 392-worker campaign itself completed without a
failed or incomplete child.
