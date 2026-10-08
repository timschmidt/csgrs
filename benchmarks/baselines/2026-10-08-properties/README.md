# Policy contract performance evidence

## Grammar cache policy

This directory records the focused constructor probe for the explicit
`solid::cache` retention policy. The baseline is revision
`0c5508b89`; the candidate extracts the five grammar caches into typed storage
with caller-visible entry limits, usage, clearing, and scoped policies.

The measured workload calls `sphere(10, 24, 12)` repeatedly with the default
policy. Each process includes one cold miss and otherwise exercises warm
primitive retention. It is not a cold-construction, allocation, Boolean, or
competitive-library benchmark. The probe checks the expected 528 triangles
per call. Exact coordinate/index equality, copy-on-write isolation, context
and certainty preservation, eviction, disabled retention, nested scopes, and
thread isolation are checked separately in
[`tests/solid_cache.rs`](../../../tests/solid_cache.rs).

| Run | Baseline median | Candidate median | Interpretation |
| --- | ---: | ---: | --- |
| `cache-constructor` | 36.202 ns/call | 36.293 ns/call | Final inline helper, same CPU; 0.25% difference, no speedup claim |
| `cache-constructor-factored` | 36.546 ns/call | 38.585 ns/call | Earlier helper without explicit inlining, unpinned exploratory run; not the retained variant |

Each CSV retains 15 alternating baseline/candidate pairs after two warmup
pairs, with one million calls per process. Its matching JSON records settings,
toolchain, host, executable size and SHA-256, and medians. The final run pins
both children to CPU 0. The exploratory run used a prior script without
affinity pinning; its difference cannot be attributed solely to inlining.
Executable sizes are whole benchmark artifacts, not library-size measurements.

Build and save the baseline benchmark executable before changing sources:

```sh
cargo bench --locked --no-default-features --bench mesh_pipeline --no-run
# Copy the executable printed by Cargo to a separate baseline path.
```

Build the candidate with the same command, save its executable separately,
then run from the repository root:

```sh
python3 scripts/compare-cache-probe.py /path/to/baseline /path/to/candidate \
  benchmarks/baselines/2026-10-08-properties/cache-constructor.csv
cargo test --locked --all-features --test solid_cache
cargo test --locked --no-default-features --test solid_cache
```

The retained binary was built after the cache changes and before the later
Hypercurve preview-policy correction. That correction does not participate in
this no-default-features sphere-constructor path. No broader performance or
memory conclusion follows from this probe.

## Curve work-budget accounting

[`curve-budget/manifest.json`](curve-budget/manifest.json) retains 11 alternating
baseline/candidate process pairs after two warmup pairs, pinned to CPU 0.
The baseline executable was built at `e225e6f69`, before budget instrumentation,
with `cargo bench --locked -p hypercurve --bench bezier_algebraic_parameter
--no-run`. The candidate uses the same build command and includes the shared
work counters with inlined accounting accessors. Exact executable hashes,
whole-benchmark sizes, raw stdout, per-sample timings, and every reported
checksum/count are retained. Both binaries report identical evidence on all
11 workloads, including the original quintic and deep boundary-root fixtures.

| Workload | Baseline median | Candidate median | Change |
| --- | ---: | ---: | ---: |
| Simple-root refined ordering | 16.212 µs | 16.390 µs | +1.10% |
| Repeated-root refined ordering | 64.632 µs | 66.092 µs | +2.26% |
| Five-root quintic isolation | 34.398 µs | 34.980 µs | +1.69% |
| Deep boundary-root isolation | 47.423 µs | 48.805 µs | +2.91% |

The final medians range from -3.74% to +2.91% across these workloads. This is
evidence of the cost of adding the explicit contract, not a speedup claim or
a statistical guarantee of a regression ceiling. The retained variant adds
7,048 bytes (0.060%) to this benchmark executable; that is not a measurement
of library binary size or allocations. Active-budget throughput and the full
competitive suite are not measured here. Exact curve results and retry
behavior are checked separately in
[`work_budget.rs`](../../../crates/hypercurve/tests/work_budget.rs).

The preceding variant without the inline attributes is retained in
[`curve-budget-initial/manifest.json`](curve-budget-initial/manifest.json).
Its deep boundary-root median was 5.28% slower than its paired baseline.
The inlined variant was retained to reduce the observed overhead. Both runs
use the same baseline binary and verify the same evidence; run-to-run timing
variation means the difference cannot be attributed wholly to inlining.

To reproduce, save each revision's executable separately and run:

```sh
python3 scripts/compare-curve-budget-probe.py /path/to/baseline /path/to/candidate \
  /path/to/new-evidence-directory
```

## Shared ordered-field and selected-fiber isolators

The subsequent extension charges the shared Bernstein isolator, selected-fiber
Sturm isolation, reusable fiber-root refinement, and local-field source-root
refinement. The first version borrowed the frame stack even when no budget was
active. The retained version checks a thread-local activity flag first, with
scope restoration covered by nested, unwind, and thread tests.

[`fiber-budget/manifest.json`](fiber-budget/manifest.json) records five
alternating process pairs pinned to CPU 0. Each of the three existing
`certification` Criterion workloads uses 30 samples, 0.2 seconds of warmup, and
0.5 seconds of measurement per process. The table compares the median of the
five process mean estimates, without claiming a statistical regression bound.

| Workload | Baseline median | Candidate median | Change |
| --- | ---: | ---: | ---: |
| Bernstein rational deflation | 20.044 µs | 19.186 µs | -4.28% |
| Repeated irrational root with Sturm fallback | 33.266 µs | 33.667 µs | +1.20% |
| Partitioned rational roots with Sturm fallback | 410.539 µs | 425.266 µs | +3.59% |

The benchmark's existing setup assertions verify isolation and expected
fallback use; it does not checksum every timed result. Exact interval equality
after exhausted attempts and larger-budget retries is checked separately for
simple and repeated fibers, both parameter axes, and both predicate policies.
All 583 Hypersolve library tests pass on the candidate.

The initial extension is retained in
[`fiber-budget-initial/manifest.json`](fiber-budget-initial/manifest.json).
Its repeated-root row was +7.33% against its paired baseline. The activity-flag
variant was retained to reduce that observed overhead; the other rows vary
across runs, so there is no general speedup claim. Its whole benchmark binary
is 2,592 bytes larger than the baseline, not a library-size or allocation result.

Both binaries use the same isolated four-crate workspace, compiler, profiles,
and pruned lockfile. The baseline sources are `546cd35fa`; only the listed
solver source files change for the candidate. The lockfile, temporary workspace
manifest, source fingerprints, and exact build procedure are retained in
[`benchmark-provenance.json`](../../../docs/evidence/2026-10-08-curve-budget-isolators/benchmark-provenance.json).
All package identities and registry checksums match the root lockfile; unused
workspace dependency edges are absent, so these are explicitly isolated builds.

```sh
python3 scripts/compare-criterion-probe.py /path/to/baseline /path/to/candidate \
  isolate_bivariate_fiber_roots/ /path/to/new-evidence-directory
```

The final extension also reran the unchanged curve-parameter probe against
the original pre-budget executable. In
[`curve-budget-fast/manifest.json`](curve-budget-fast/manifest.json), all
reported counts and trace evidence match across 11 alternating process pairs
after two warmup pairs. Median changes across its 11 workloads range from
-4.18% to +3.95%; deep boundary-root isolation changes from 49.633 to
51.034 µs (+2.82%). This later candidate includes both the shared-isolator
hooks and the activity flag. Its benchmark executable is 11,072 bytes larger
than the original baseline. These measurements have the same limited scope
as the earlier curve probe and do not establish an operation-time bound.

## Represented-root image promotions

The next extension charges construction of new polynomial, rational, and
binary algebraic image eliminants. It carries typed exhaustion through
represented-root arithmetic and curve coordinate images. Rational maps may
read a previously cached eliminant inside a budget, but leave new elimination
attempts uncached. Native regressions exercise both policies, four binary
operations, nested shared limits, denominator refinement, exact retries,
different selected source roots, and zero-cost constant/affine paths. The
curve regression also checks that a denied lazy point stays retryable and a
completed point needs no new budget.

[`image-budget/manifest.json`](image-budget/manifest.json) retains five
alternating process pairs on CPU 0, with 30 Criterion samples, 0.2 seconds of
warmup, and 0.5 seconds of measurement per selected workload. No other
validation commands from this task ran during this final probe. As with the
fiber probe, these are medians of process mean estimates, without a statistical
regression bound or per-iteration result checksums.

| Workload | Baseline median | Candidate median | Change |
| --- | ---: | ---: | ---: |
| Polynomial image | 3.560 µs | 3.593 µs | +0.91% |
| Constant polynomial image | 120.830 ns | 117.420 ns | -2.82% |
| Polynomial image with foreign-root refinement | 28.326 µs | 29.424 µs | +3.88% |
| Repeated degree-six polynomial carrier | 52.323 µs | 52.063 µs | -0.50% |
| Rational image with denominator refinement | 10.605 µs | 10.809 µs | +1.93% |
| Independent binary image | 19.007 µs | 19.349 µs | +1.80% |
| Independent binary quotient | 17.310 µs | 17.641 µs | +1.92% |

Across all 11 workloads, final median changes range from -3.11% to +3.88%.
The [initial idle probe](image-budget-initial/manifest.json) showed the constant
image increasing from 123.039 to 132.618 ns (+7.79%). The retained change
avoids a second exhaustion lookup after constructing a constant witness:
this path has already checked exhaustion and performs no charged work before
returning. A regression verifies that an already-exhausted scope still gets a
typed failure for the same constant image. The timing difference between runs
cannot be attributed wholly to this change, and no general speedup is claimed.
An earlier [concurrent diagnostic](image-budget-concurrent/manifest.json) is
also retained and labeled with its background validation load.

The baseline source is `fd3579925`, using the same isolated workspace and
pruned lockfile as the fiber gate. All 243 Rust source files were compared
against that revision before copying the candidate changes. The final whole
benchmark executable grows by 22,520 bytes; this is not a library-size or
allocation measurement. The
[provenance record](../../../docs/evidence/2026-10-08-curve-budget-images/benchmark-provenance.json)
retains source fingerprints, the initial constant-image implementation, and
build details. Exact-output tests and their limits are recorded separately in
the [validation manifest](../../../docs/evidence/2026-10-08-curve-budget-images/manifest.json).

## Selected-fiber projection promotions

Selected-fiber local/global image norms and bivariate quotient norms now
charge promotion attempts. Tagged tensor-fiber projection and its final
quotient norm share the enclosing budget and together require two units.
Native tests exercise denied attempts, exact full-report retries, repeated
carriers, both retained axes, and both predicate policies. Curve tests compare
the retained saturated component and global schedule after an exact retry.
A denied projection exposes no partial coefficients or retained relation.

[`projection-budget/manifest.json`](projection-budget/manifest.json) retains
five alternating process pairs on CPU 0 across ten existing Criterion
workloads, with 30 samples, 0.2 seconds of warmup, and 0.5 seconds of
measurement per workload. No other task validation or build commands ran
during the final probe. The table reports medians of process mean estimates.

| Workload | Baseline median | Candidate median | Change |
| --- | ---: | ---: | ---: |
| Polynomial fiber image | 6.175 µs | 6.422 µs | +4.01% |
| Cubic source, quadratic image | 20.198 µs | 20.361 µs | +0.81% |
| Saturated conjugate factor | 87.686 µs | 87.794 µs | +0.12% |
| Saturated fourfold component | 2.281 ms | 2.252 ms | -1.26% |
| Quotient norm, first retained axis | 2.175 µs | 2.050 µs | -5.71% |
| Quotient norm, second retained axis | 1.916 µs | 1.888 µs | -1.45% |
| Tagged opposite-conjugate cubic | 210.615 µs | 205.090 µs | -2.62% |
| Tagged repeated degree-six carriers | 219.618 µs | 226.170 µs | +2.98% |

Changes across all ten rows range from -5.71% to +4.01%. These measurements
record the observed cost of a broader resource contract; they establish
neither a general speedup nor a statistical regression ceiling. Existing
benchmark setup assertions are retained, and exact-output tests supply
separate evidence. The whole benchmark executable changes by -24,232 bytes;
this is not an allocation or library-size measurement.

The baseline is `12176ca1a`. Both executables use the same isolated four-crate
workspace and pruned lockfile described in
[`benchmark-provenance.json`](../../../docs/evidence/2026-10-08-curve-budget-projections/benchmark-provenance.json).
The first probe overlapped the end of a release integration-test build and is
preserved as a diagnostic in
[`projection-budget-concurrent/manifest.json`](projection-budget-concurrent/manifest.json).
No source optimization separates the diagnostic and final runs.

```sh
python3 scripts/compare-criterion-probe.py /path/to/baseline /path/to/candidate \
  '^(project_bivariate_fiber/|project_algebraic_fiber_polynomial_image($|/)|project_selected_tensor_fiber_via_tagged_norm/)' \
  /path/to/new-evidence-directory
```

## Shared bivariate and tensor elimination

Bivariate system resultants, their polynomial/rational curve wrappers, and
constrained tensor-axis resultants now participate in the shared promotion
allowance. Bivariate degree admission itself needs no promotion; the complete
continuation charges when it starts elimination. Nested tensor images and
tagged projections count their required constrained-axis eliminations too.
This extends the preceding accounting contract: the two-source tagged fixture
now admits four units (entry, two axes, final quotient norm). Exact removal of
an independent source axis still needs no resultant unit.

Solver tests compare complete reports after denial and retry, including all
three tensor axes, both bivariate orientations, continuation past a degree
limit, and correlated-product multiplicity. Curve tests distinguish a sum
needing two eliminations from a product needing only one after exact quotient
reduction. The public intersection regression reports a typed pair blocker
under denial, then returns all three certified contacts with a larger budget
and matches their evidence against an unbudgeted query.

[`elimination-budget/manifest.json`](elimination-budget/manifest.json) retains
five alternating process pairs across thirteen workloads, pinned to CPU 0.
Each uses 30 Criterion samples, 0.2 seconds of warmup, and 0.5 seconds of
measurement. No other task build or validation ran during measurement.
The table reports medians of process mean estimates.

| Workload | Baseline median | Candidate median | Change |
| --- | ---: | ---: | ---: |
| Polynomial-curve resultant | 1.187 µs | 1.130 µs | -4.78% |
| Rational-curve resultant | 1.435 µs | 1.385 µs | -3.51% |
| Tensor resultant | 1.059 µs | 1.088 µs | +2.79% |
| Compact 2 × 2 retained grid | 6.502 µs | 6.506 µs | +0.07% |
| Padded 64 × 64 retained grid | 73.295 µs | 78.994 µs | +7.77% |
| Padded closed-form norm | 7.789 µs | 7.137 µs | -8.37% |
| Strict exact-normal constraint | 10.989 µs | 10.589 µs | -3.64% |
| Tagged repeated degree-six carriers | 213.916 µs | 213.957 µs | +0.02% |

The padded-grid increase prompted a separate nine-pair follow-up with the
same binaries and settings, retaining the compact grid as a control. In
[`elimination-budget-padding/manifest.json`](elimination-budget-padding/manifest.json),
the padded row changes from 77.401 to 75.035 µs (-3.06%), and the compact row
from 6.410 to 6.393 µs (-0.26%). No source change separates these runs. Both
observations remain part of the evidence; they do not establish a stable
speedup or a statistical regression ceiling.

The baseline is `3464aa428`. The isolated workspace, pruned lockfile, compiler,
and profiles are unchanged from the prior gate; all source fingerprints and
build details are retained in
[`benchmark-provenance.json`](../../../docs/evidence/2026-10-08-curve-budget-elimination/benchmark-provenance.json).
The whole benchmark executable grows by 2,832 bytes. This measures neither
library size nor allocations. Existing fixture setup assertions and separate
exact-output tests remain the correctness evidence; no per-iteration checksum
was added to Criterion.

```sh
python3 scripts/compare-criterion-probe.py /path/to/baseline /path/to/candidate \
  '^(resultant_tensor_polynomial_univariate_constraint($|/)|resultant_(rational_)?parametric_curve_intersection$|project_selected_tensor_fiber_via_tagged_norm/)' \
  /path/to/new-evidence-directory
```

## Fixed-rank norms and subresultants

The next extension admits three- and four-variable constrained norms and
subresultants against the shared promotion allowance. Degree/order admission
spends no unit; each interpolation attempt spends one. Terminal-order reuse
also spends no unit. Exact-report regressions cover every trivariate axis,
the constrained fourth axis, nested scopes, and exhausted-scope reuse. Curve
regressions preserve selected degree drops, even contacts, typed exhaustion,
and certified retries under both policies.

[`fixed-elimination-budget/manifest.json`](fixed-elimination-budget/manifest.json)
retains five alternating process pairs, pinned to CPU 0, with 30 Criterion
samples, 0.2 seconds of warmup, and 0.5 seconds of measurement per workload.
All task builds and tests had finished before measurement. These are medians
of process mean estimates with no active budget.

| Workload | Baseline median | Candidate median | Change |
| --- | ---: | ---: | ---: |
| Constrained trivariate norm | 23.929 µs | 23.781 µs | -0.62% |
| Strict exact-normal constraint | 13.405 µs | 13.358 µs | -0.35% |
| Polynomial-curve resultant control | 1.185 µs | 1.161 µs | -2.03% |
| Tensor resultant control | 1.042 µs | 1.039 µs | -0.24% |

The baseline is `c74e952b7`; compiler, isolated workspace, profiles, and pruned
lockfile are unchanged. The benchmark executable grows by 456 bytes.
[Build and source provenance](../../../docs/evidence/2026-10-08-curve-budget-fixed-elimination/benchmark-provenance.json)
records those identities. These selected observations establish neither a
general speedup nor a statistical regression ceiling. Subresultants and the
four-variable paths have correctness coverage here, without new timing or
allocation measurements. Whole-executable size is not library size. Exact
reports and retained curve parameters are compared in separate tests; no
per-iteration checksum was added to Criterion.

```sh
python3 scripts/compare-criterion-probe.py /path/to/baseline /path/to/candidate \
  '^(resultant_trivariate_polynomial_univariate_constraint($|/)|resultant_tensor_polynomial_univariate_constraint$|resultant_parametric_curve_intersection$)' \
  /path/to/new-evidence-directory
```
