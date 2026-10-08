# CSGRS / CGAL / OpenCascade Baseline — 2026-07-17

This baseline compares the portable solid-kernel workload shared by CSGRS,
CGAL EPECK, and OpenCascade. Ratios above `1x` mean the comparison engine is
slower than CSGRS. It is a performance baseline, not evidence that dissimilar
representations produce identical facet streams.

## Environment

- Fedora Linux 43, kernel `7.0.4-100.fc43.x86_64`, x86-64
- Rust `1.97.0 (2d8144b78 2026-07-07)`
- GCC `15.2.1 20260123`
- CGAL `6.0.3`, `Exact_predicates_exact_constructions_kernel`
- OpenCascade `7.9.3`, tight-double runner with zero added Boolean fuzzy tolerance

CGAL documents EPECK as providing exact geometric predicates and exact
geometric constructions:
<https://doc.cgal.org/latest/Kernel_23/classCGAL_1_1Exact__predicates__exact__constructions__kernel.html>.
OpenCascade documents `Precision::Confusion()` as `1e-7`, so its runner is not
labeled exact:
<https://dev.opencascade.org/doc/refman/html/class_precision.html>.

## Corpus parity check

```sh
benchmarks/run.sh --quick --output /tmp/csgrs-competitor-quick-20260717
```

Result: `47 workloads x 1 temperature x 3 engines`, with engine parity,
semantic assertions, output traversal, and stable per-engine checksums accepted.
The one-sample scan identified six possible comparison-engine wins; five were
CGAL rows and one was OpenCascade point containment.

## Focused cold/warm confirmation

```sh
CSGRS_BENCH_FILTER='construct_sphere/medium,construct_sphere/large,rotate_xyz,distribute_grid,distribute_linear,contains_point' \
CSGRS_BENCH_SAMPLES=10 \
CSGRS_BENCH_WARMUP=2 \
CSGRS_BENCH_COLD_SAMPLES=5 \
benchmarks/run.sh --cold-warm --output /tmp/csgrs-competitor-losses-20260717
```

| Workload | CSGRS cold ns | CSGRS warm ns | CGAL / CSGRS cold | CGAL / CSGRS warm | OCCT / CSGRS cold | OCCT / CSGRS warm |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| sphere construction, large | 802,945 | 280,176 | 2.791x | 6.091x | 52.502x | 141.687x |
| sphere construction, medium | 279,231 | 66,069 | 2.326x | 6.300x | 31.024x | 109.673x |
| point containment, two queries | 747,729 | 3,400 | 1.089x | 208.351x | **0.252x** | 11.272x |
| grid distribution, 4x4 | 58,496 | 6,380 | 1.153x | 11.012x | 126.451x | 998.570x |
| linear distribution, 8 | 35,217 | 3,040 | 1.020x | 11.632x | 120.520x | 1,199.467x |
| XYZ rotation, medium sphere | 231,704 | 105,885 | 1.213x | 2.510x | 32.120x | 68.398x |

All focused warm paths and every focused CGAL cold path favored CSGRS. At this
baseline stage, the remaining confirmed gap was first-call point containment
against OpenCascade: CSGRS was about `3.97x` slower cold, then `11.27x` faster
warm. The retained-bounds result below closes that gap.

## Profile and retain/reject record

`perf stat` on the cold CSGRS containment row measured approximately 48.7M
instructions and 19.9M cycles for the runner process. Sampling localized the
first-call cost to certified polygon-bound construction, exact triangle-query
preparation, dyadic/rational conversion, and allocation. The warm result is
served by retained ray-query state.

Two exactness-preserving local experiments were measured and fully removed:

1. Prefer `Real::to_f64_exact_dyadic()` before certified interval construction.
   This increased the observed cold median to roughly 855 microseconds.
2. Remove the discarded exact-X-axis prepared-query warm-up from
   `contains_vertex`. The observed cold median remained roughly 757 microseconds,
   within baseline noise and still slower than OpenCascade.

An eager prototype that computed and retained a separately allocated bounds
record for every sphere triangle reduced cold containment but moved too much
work into construction: medium-sphere warm construction rose from roughly 66
to 90 microseconds. It was replaced by a shared per-position table in the lazy
sphere vertex pool, with each triangle's inline cache populated only on query.

An analytic sphere test was deliberately rejected: containment in the analytic
sphere is not interchangeable with containment in the sampled polyhedron.

## Retained certified sphere bounds

The retained table is built only when the radius has an exact binary64 value.
Sphere sine/cosine samples are the same binary64 values promoted to exact
dyadic `Real`s for the polygon vertices. Each multiplication is enclosed by
outward binary64 rounding, and triangle bounds are unions of those certified
point intervals. If the radius is not exactly representable or outward rounding
would reach infinity, the fast path is absent and the pre-existing exact
certification path remains authoritative.

`sphere_retained_binary_bounds_contain_exact_vertices` compares every retained
lower and upper bound with every exact `Real` coordinate on a 960-triangle
sphere. `sphere_binary_bounds_fall_back_before_outward_rounding_reaches_infinity`
checks the extreme-radius fallback.

Fresh three-engine confirmation:

```sh
CXX=/usr/bin/g++ \
CSGRS_BENCH_FILTER='construct_sphere/medium,contains_point' \
CSGRS_BENCH_SAMPLES=10 \
CSGRS_BENCH_WARMUP=2 \
CSGRS_BENCH_COLD_SAMPLES=5 \
benchmarks/run.sh --cold-warm \
  --output /tmp/csgrs-competitor-retained-bounds-final-20260717
```

Result: `2 workloads x 2 temperatures x 3 engines`, with parity and checksum
assertions accepted.

| Workload | CSGRS cold ns | CSGRS warm ns | CGAL / CSGRS cold | CGAL / CSGRS warm | OCCT / CSGRS cold | OCCT / CSGRS warm |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| sphere construction, medium | 257,403 | 71,421 | 2.956x | 5.849x | 32.931x | 100.739x |
| point containment, two queries | 89,014 | 3,188 | 11.232x | 221.209x | 1.877x | 11.857x |

Cold point containment improved `8.40x` against the original CSGRS baseline
and changed the OpenCascade comparison from a `0.252x` ratio (OCCT faster) to a
`1.877x` ratio (CSGRS faster). Medium-sphere construction remains faster than
both comparison engines in both temperatures.

Validation after the retained implementation:

- `cargo test --all-targets --all-features`
- `cargo clippy --all-targets --all-features -- -D warnings`
- `RUSTDOCFLAGS='-D warnings' cargo doc --no-deps --all-features`
- `cargo fmt --all -- --check`
- `git diff --check`
