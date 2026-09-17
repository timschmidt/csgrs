# Pre-Verus acceptance baseline

The reference for the entire Verus implementation is
`a2da8e2b5de9a1a4653d3662f2f05cb6533fd7ef`, **Document shared integer magnitude
GCD for algebraic consumers** (2026-09-15). It is the parent of `c5f7868`, the
first Verus commit. The reference has no `verification/` or `src/verified/`
tree. Compare the final implementation with this fixed reference, including
all intervening verification changes, rather than moving the reference to
the preceding proof milestone.

The acceptance priorities, in order, are:

1. Exactness.
2. Completeness.
3. Performance.
4. Memory use.
5. Binary size.
6. Code size.

Preserve the reference's supported behavior, exactness, representations,
fallbacks, feature combinations and API. Proving a simplified or restricted
substitute does not complete the objective. Test coverage is supplementary
evidence; neither passing tests nor a count of verified functions establishes
a proof of all behavior. The full implementation and dependency refinement
obligations in [scope.json](scope.json) still apply.

Do not accept a runtime regression merely to simplify a proof. Investigate
individual representative regressions rather than hiding them in an average.
Any proposed tradeoff must be supported by measurements and assessed in the
priority order above. In particular, smaller code, binaries or memory cannot
justify a performance regression. The repository's
[equality evaluation](../PERFORMANCE.md#rational-equality-unification-evaluation-2026-09-06)
and its [checkpoint](../benchmarks/checkpoints/2026-09-06-rational-equality.json)
provide the established comparison precedent.

## Required qualification evidence

Use the existing infrastructure on the reference and candidate. Record each
revision, source and workload hashes, compiler, lockfile, features, target,
profile flags, command, exit status and raw output. Build both revisions in
the same checkout path, preserve their executables, and time them serially
in alternating order after all compilation has finished. Report host load,
CPU affinity, uncontrolled frequency/thermal effects and statistical
uncertainty. Short runs screen for regressions; they do not establish parity.

| Priority / surface | Existing infrastructure and required comparison |
| --- | --- |
| Exactness and completeness | All locally executable CI commands; default and all-feature tests, benchmark fixture assertions, examples, documentation tests, strict Clippy and build checks. Preserve the baseline's test inventory and add independent arithmetic oracles for changed behavior. |
| Feature and representation coverage | `scripts/representation_coverage.sh` and the full seven-configuration `scripts/coverage.sh`; inspect uncovered production branches as proof obligations, not as permission to exclude them. |
| Fuzzing | Build and replay all six `fuzz/` targets, including the saved corpora, with AddressSanitizer where supported. Preserve any newly discovered regression input. |
| Performance | All nine Criterion binaries: `scalar_micro`, `numerical_micro`, `borrowed_ops`, `float_convert`, `library_perf` (requires `simple`), `real_representations`, `gmp_api`, `adversarial_transcendentals`, `adversarial_library`. Compare identical benchmark inventories and fixtures, with repeated paired runs and focused confirmation of changes. |
| Dispatch and end-to-end performance | Run the separate `dispatch_trace` diagnostic and the available downstream guards documented in `PERFORMANCE.md`: Hyperlattice mathbench, Hyperlimit predicates, and Hypercurve/Hypermesh exact geometry fixtures. Include relevant downstream test, lint, fuzz, UI and WASM infrastructure; pin every consumer revision and dependency substitution. |
| Memory use | `scripts/memory_profile.sh 64` (all 20 representations), allocation counts/bytes/peak/retained state; existing downstream allocation, heaptrack and Callgrind workloads. Keep profiling separate from native timing. |
| Binary size | Matched-feature release executables and examples, loadable/text sections and stripped size; available downstream native fixtures and WASM builds. An `.rlib` archive alone is not a final binary comparison. |
| Code size | Report production source, proof annotations/models, tests and qualification infrastructure separately as well as together. Do not describe ghost erasure as a reduction in source size. |

Set `HYPERREAL_SKIP_BENCHMARK_REPORTS=1` for comparison and coverage runs so
existing generated ledgers are not overwritten with mixed-revision data.
Preserve Criterion's raw samples and estimates for each revision and pair.
Do not run timing concurrently with builds, tests, profilers or other timing.
Document unavailable infrastructure or a failed command explicitly; neither
counts as passing. Changed candidates invalidate the affected qualification
results and require new comparisons with the fixed reference.

## Current status

Qualification is **pending**, and the full Verus proof is **incomplete**.
The [initial comparison checkpoint](../benchmarks/checkpoints/2026-09-17-verus-baseline.json)
records the native-dyadic candidate `598f059`, three alternating pilot pairs,
the complete local candidate validation matrix, allocation/heap/stack profiles,
dispatch traces, native/WASM sizes and source counts. The complete nine-suite
screen measured 1,449 matching cases; longer alternating repeats include
four pairs with identical executable and output paths. The
[compressed raw pilot samples](../benchmarks/checkpoints/2026-09-17-verus-baseline-pilot.json.gz)
and the [complete screen](../benchmarks/checkpoints/2026-09-17-verus-screen.json.gz),
[longer repeats](../benchmarks/checkpoints/2026-09-17-verus-confirmations.json.gz)
and [matched-path repeats](../benchmarks/checkpoints/2026-09-17-verus-matched-confirmations.json.gz)
are retained with the checkpoint. Short timing runs identify follow-up cases;
they do not establish parity. A persistent `pi_pow` cloning slowdown, unstable
cached conversion measurements and other screen flags remain unresolved.
The checkpoint explicitly retains the baseline API-inventory failure,
supplementary baseline validation with only that test classification repaired,
and the inherited fuzz precision-budget defect corrected in `add6365`.
All six saved fuzz corpora pass with the corrected harness on both revisions;
the frozen inputs are retained. Downstream qualification and final repeated
performance comparisons remain open.

The subsequent [cache investigation](../benchmarks/checkpoints/2026-09-17-verus-cache-diagnostics.json)
compares the fraction-reduction runtime `8c74849` with the same fixed baseline.
Six balanced original Criterion rounds retain a median 4.19% `pi_pow` clone
slowdown. A native cached-conversion probe also consistently regresses.
Inlining the cache accessor improves cached conversion, but does not resolve
the clone finding; that overlay remains experimental. Raw counters, samples,
assembly and the completed pinned Hypercurve follow-up failures are retained
with the checkpoint. No live Hypercurve files were changed.

The [shared-clone checkpoint](../benchmarks/checkpoints/2026-09-17-verus-shared-clone.json)
records the subsequently adopted runtime `561e36a`. Cloning symbolic reals
reuses an existing immutable computable graph, and the primitive cache getter
is inlined. Six alternating original Criterion pairs show median time changes
of -75.17% for `pi_pow` cloning, -54.64% for `pi` cloning and -43.05% for
cached tangent conversion against the fixed pre-Verus baseline. Every pair
improves in these three cases; raw samples, external comparator rows and paired
bootstrap intervals are retained. These focused results resolve those specific
timing findings for this runtime, without establishing performance parity
across the repository. The full benchmark screen, resource comparison and
downstream qualification remain separate requirements.

Default and all-feature all-target checks, the representation matrix, strict
Clippy and all 6,492 frozen AddressSanitizer corpus inputs pass for this
runtime. Its proof report verifies 363 units, including 52 production contracts
and 65 model theorems, while all 13 full-crate obligation groups remain open.
The prior square-root metadata checkpoint retains the rejected clone variants
and the failures that led to the metadata and fixed-decimal precision repairs.

The [complete shared-clone screen](../benchmarks/checkpoints/2026-09-17-verus-shared-clone-screen.json)
retains 1,449 matching cases across all nine suites. Its 236 positive
nonoverlapping within-run interval flags, including external comparator rows,
were followed up in [four alternating pairs](../benchmarks/checkpoints/2026-09-17-verus-shared-clone-confirmations.json).
All 64 processes succeeded, with identical selected cases and frozen binaries.
Forty-seven cases were slower in all four pairs, including external comparator
rows. Persistent Hyperreal signals include the square-root MSD query (+40.90%
median paired mean change), symbolic pi inversion (+11.69%) and construction
from a small signed integer (+9.94%). Every selected case, paired estimate,
raw sample and comparator movement is retained. Other flags did not persist
across all pairs; that alone does not establish parity. The remaining
regressions require investigation, and this follow-up is not acceptance.
The [resource checkpoint](../benchmarks/checkpoints/2026-09-17-verus-shared-clone-resources.json)
records the completed release, lint, documentation, fuzz-build, WASM and
seven-configuration coverage checks. Production executable-line coverage is
91.10%, which is separate from formal proof coverage. Allocation counts fall
in 12 of 20 representation workloads, per-case peak allocations do not increase
and retained bytes are unchanged. Matched exact-peak Massif runs nevertheless
show a 176-byte increase in whole-process peak memory. Native and WASM binary
size increases are also retained for assessment. The interrupted first coverage
attempt and truncated diagnostic profile are explicitly excluded from passing
results; the complete rerun and compact matched profiles succeeded.

The subsequent [cancellation induction proof](../benchmarks/checkpoints/2026-09-17-verus-cancel-pair-proof.json)
verifies 364 units and 66 model theorems. Normal expanded Rust is byte-identical
to `561e36a`; the production cross-cancellation traversal and its caller
preconditions still require refinement proofs. All 13 obligation groups and
final baseline acceptance remain open.

The [production cancellation kernel](../benchmarks/checkpoints/2026-09-17-verus-cross-cancel-kernel.json)
in `da3a6fd` now proves the complete native pair traversal and ordered product
checks, including the existing fallback when overflow precedes a later zero.
The official proof passes 374 units, 54 production contracts and 66 model
theorems. Default/all-feature tests and fixture assertions, the representation
matrix, 73 mutation rejections and all 6,492 frozen ASan inputs pass. Strict
Clippy passes after a scoped annotation preserves an explicit branch needed
by the proof; the initial lint failure and the annotation-only correction are
retained. New native binaries are built, but this runtime's timing, resource,
size, coverage and downstream qualification remain pending. Importing native
parts from `BigUint` and proving the callers' invariants also remain open.

The [unit-scale shortcut trial](../benchmarks/checkpoints/2026-09-17-verus-unit-scale-trial.json)
remains unadopted. Six balanced orders improve the square-root structural
query by 30.30% against the baseline, but tau and a non-unit dense expression
are slower in every pair (median +3.97% and +5.76%). Its passing correctness
checks and all raw timing data are retained; the faster unit cases do not
dismiss those regressions.

The [magnitude-range repair](../benchmarks/checkpoints/2026-09-17-verus-magnitude-range.json)
in `ee773bb` fixes observed baseline errors. With pinned dependencies, a debug
query of `2^2147483647` panics despite its representable exponent. In release,
`2^2147483648` receives a false exact exponent of `-2147483648`. The repaired
queries retain wide lengths and shifts, reuse their comparison, and apply the
computable offset before narrowing. Actual large-value checks cover both
signed limits, out-of-range results, and an outer exponent brought back into
range by its offset. All six debug checks and three fixed release checks pass.

The official proof verifies 375 units, 55 production contracts and 66 model
theorems. Default/all-feature tests (790/893 executions), their 1,271/1,447
benchmark fixtures, the representation matrix, strict Clippy, all 77 mutation
rejections and 6,492 frozen ASan inputs pass. The initial probe's differing
dependency resolution and the mutation driver's corrected textual anchor are
retained explicitly. This is an exactness repair; the full source/caller/
dependency proof remains incomplete.

The [magnitude resource checkpoint](../benchmarks/checkpoints/2026-09-17-verus-magnitude-resources.json)
records all twelve remaining build/check steps for `ee773bb`, including strict
documentation, both WASM builds, seven coverage configurations and benchmark
fixtures. Production executable-line coverage is 26,018 of 28,555 lines
(91.12%); this is not formal proof coverage. All allocation, Callgrind, Massif,
Memcheck, Heaptrack, dispatch and section-size commands pass. Twelve of twenty
allocation workloads improve, retained bytes are unchanged, and Callgrind
instructions decrease from 29,845,483 to 28,776,316. Memcheck reports no errors
or definitely, indirectly or possibly lost bytes.

The identical-path AB/BA Massif comparison still increases the exact whole-
process peak from 24,896 to 25,072 bytes in both pairs. Native stripped binaries
grow by up to 16,384 bytes, and WASM grows from 1,646,116 to 1,660,615 bytes.
Expanded ordinary Rust has 36,082 lines versus 35,597 at the baseline. These
differences remain assessment items; the twenty-form workload does not prove
full resource parity. Current native timings and downstream qualification
remain pending.

The [magnitude model checkpoint](../benchmarks/checkpoints/2026-09-17-verus-magnitude-model.json)
in `f7e256c` connects the executable exponent specification to a unique rational
binary interval and proves signed binary scaling. The official proof passes
395 units, 55 production contracts and 71 model theorems. All 77 executable
mutations, two magnitude-model mutations, the assumption-bypass guard and seven
acceptance guards pass. Normal expanded Rust is byte-identical to `ee773bb`.
The first model-guard invocation rejected the deliberately overlapping
intervals through failed proof preconditions; its driver did not recognize that
diagnostic. That invocation and the successful corrected suite are retained.
BigUint bit-length/comparison refinement and all thirteen broader obligation
groups remain open.

The [fixed consumer supplement](../benchmarks/checkpoints/2026-09-17-verus-consumer-fixed-inputs.json)
builds four benchmark-only overlays on both the baseline and `ee773bb`.
Every variant replays the same 400 stored inputs in their original order.
All eight builds and all eight 101-row fixture replays pass; each consumer's
aggregate row repeats its same 100 inputs. This removes timing-based promotion
from the supplemental comparison while preserving the original adaptive
harness. Native timings remain pending.

The [current consumer build checkpoint](../benchmarks/checkpoints/2026-09-17-verus-consumer-magnitude-builds.json)
contains all **41** non-tracing benchmark targets in the pinned baseline
manifests; the earlier working count of 42 was incorrect. Five additional
feature-gated tracing targets belong to the separate trace checks. Current
release suites pass for Hyperlattice (204 tests), Hyperlimit (361), Hypertri
(191), Hypersolve (860) and Hypermesh (222, with seven ignored). Hypercurve's
complete release suite and long cases, the remaining consumer validation
matrices and native comparisons remain pending. All temporary sources were
restored, and live Hypercurve was untouched.

The [bounded peak attribution](../benchmarks/checkpoints/2026-09-17-verus-magnitude-peak.json)
reproduces the 176-byte whole-process increase with deeper allocation stacks.
Its 160 heap bytes comprise 112 bytes allocated through cache publication and
48 through approximation evaluation; allocator bookkeeping adds 16 bytes and
stack usage is unchanged. This identifies allocation paths, not the source
change responsible for their lifetimes. Per-case allocation windows subtract
warmed starting live bytes, whereas the whole-process peak includes warmup
and process caches. The resource assessment remains open.

The [magnitude runtime screen](../benchmarks/checkpoints/2026-09-17-verus-magnitude-screen.json)
compares the fixed baseline with `ee773bbf` across all nine ordinary benchmark
suites: 18 successful processes and 1,449 matching cases. All workload files
match both revisions. The short screen flags 281 positive, nonoverlapping
within-run median intervals, including external comparators. Every flag remains
open pending longer alternating repeats; suite averages cannot dismiss them.
The archive preserves all cases and raw Criterion output. It establishes
neither performance parity nor acceptance of the runtime.

The [limb bit-length model](../benchmarks/checkpoints/2026-09-17-verus-bit-length-model.json)
passes 402 official verification units at `31f6977`, with 55 production
contracts and 75 model theorems. Its four additional theorems connect normalized
mathematical limbs to exact bit lengths, shifted widths and width-based ordering.
All source hashes match that commit. Runtime sources and Cargo inputs are
unchanged from the preceding magnitude proof; actual `BigUint` import and caller
refinements remain open. Seven acceptance guard tests also pass.

The [cold precision probes](../benchmarks/checkpoints/2026-09-17-verus-precision-counterexamples.json)
expose the same two debug failures in the fixed baseline and `ee773bbf`:
`pi().approx(i32::MAX)` overflows a related-cache precision and
`zero().approx(i32::MIN)` overflows precision negation. Four debug/release builds
and all 20 process outcomes are retained, including four panics. The release
cases tested here return permitted values; these observations do not qualify
precision arithmetic generally. The counterexamples require repair under the
exactness-first priority.

Completion requires both the full source/caller/dependency proof audit and
source-bound qualification evidence against this baseline. The
`verify --require-complete` gate requires both the proof obligation audit and
an accepted [assessment record](acceptance.json). It rejects pending or
unresolved assessments, a changed reference/order, stale source/workload hashes
and missing or changed evidence artifacts. All six priority assessments require
a rationale and evidence references. These checks enforce the record's identity
and completeness; interpreting measurements and resolving regressions still
requires the audits above. Proof coverage itself is not an improvement in the
baseline's supported behavior and cannot justify a runtime regression.
