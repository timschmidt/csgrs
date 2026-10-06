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
precision arithmetic generally. These counterexamples motivated the repair below.

The [precision repair](../benchmarks/checkpoints/2026-09-17-verus-precision-repair.json)
at `7b95cac` preserves the full integer-leaf shift range and checks the Pi/Tau
related-cache precision before lookup. Extended probes also expose a release
error in the fixed baseline: both `1` and `-1` approximate to zero at
`i32::MIN`. The repair returns the exact signed values with 2,147,483,649 bits.
All seven cold-process cases pass in both debug and release. This improves
integer-leaf behavior without narrowing its precision domain; other kernels'
precision arithmetic remains a separate open obligation.

That commit passes 410 official verification units, with 56 production contracts
and 78 model theorems; all 115 proof-input hashes match the commit. The executable
shift plan and signed rounding/cache-coarsening models are proved. The complete
`BigInt`, caller, constant-identity and concurrent-cache refinements remain open.
All 83 arithmetic mutations are rejected, as is the assumption bypass, and seven
acceptance guard tests pass. Default/all-features checks pass 795/898 test
executions (including the fresh-process children) and 1,271/1,447 benchmark
fixture replays. Representation, strict Clippy and doc-test checks pass too.
The cold constant regression accepts both integers allowed by the public error
contract and separately passes all four default/all-features debug/release
combinations. Earlier timing flags remain unresolved; this correctness milestone
does not establish baseline acceptance.

The [precision runtime resource checkpoint](../benchmarks/checkpoints/2026-09-17-verus-precision-resources.json)
completes the remaining 13 build/check steps, the coverage matrix and all 6,492
frozen ASan inputs across six fuzz targets for `7b95cac`. Coverage reports 26,068
of 28,605 executable lines hit (91.13%); this is testing coverage, not a formal
proof percentage. All allocation, Callgrind, Massif, Memcheck and Heaptrack
commands pass. Twelve of 20 representation workloads allocate less, per-case
measured peaks do not increase and retained bytes are unchanged. Two matched
AB/BA Massif pairs still show the whole-process peak increasing from 24,896 to
25,072 bytes. That 176-byte difference remains unresolved. Callgrind instructions
decrease from 29,845,483 to 28,676,945. WASM grows from 1,646,116 to 1,661,702 bytes,
and stripped native artifacts grow by up to 17,376 bytes. Expanded ordinary Rust
has 36,105 lines versus 35,597 in the baseline. All rows and raw profiles are
retained. Current native timing, pinned-consumer qualification and the overall
resource assessment remain open.

The [precision consumer builds](../benchmarks/checkpoints/2026-09-17-verus-consumer-precision-builds.json)
cover all 41 non-tracing targets at the pinned consumer revisions. Completed
release suites report Hyperlattice 204, Hyperlimit 361, Hypertri 191, Hypersolve
860 and Hypermesh 222 passed, with seven Hypermesh tests ignored. Hypercurve's
build uses only pinned `8228394`; its full release and long-running tests remain
pending. The [fixed-input supplement](../benchmarks/checkpoints/2026-09-17-verus-consumer-precision-fixed-inputs.json)
also builds and replays all 101 rows in each of the four prepared consumer
harnesses: 400 distinct stored inputs plus the aggregate scores reusing them.
The unchanged baseline fixed-input builds are retained from the earlier
checkpoint. The temporary consumer trees are restored clean after these runs.
Neither build/replay checkpoint establishes native timing parity.

The [real approximation model](../benchmarks/checkpoints/2026-09-17-verus-real-approximation-model.json)
at `49f2b8f` passes 431 official verification units, with 56 production contracts
and 87 model theorems. All 116 proof-input hashes match that commit. Nine added
theorems connect the integer plan to exact real denotations and establish
compositional error bounds for signed binary scaling, negation, guarded
addition and cache coarsening, together with adjacent-integer and sign
certificates. All 85 mutations, the assumption bypass and seven acceptance
guards pass their rejection checks. Runtime sources, Cargo inputs and tests
are unchanged from `7b95cac`. The production graph, dependency operations,
precision arithmetic, transcendental bounds, domain validity, abort behavior,
concurrency and convergence still require refinement proofs. All 13 complete
obligation groups remain open.

The [precision consumer core checks](../benchmarks/checkpoints/2026-09-17-verus-consumer-precision-core.json)
retain all 51 commands for `7b95cac`: 41 pass and ten fail. Every failing command
also failed in the historical baseline run, whose original environment and
outcomes are retained. These failures include a Symbolica instance conflict,
formatting, strict lint/documentation checks and two Hypercurve UI tests.
All three UI WASM/Trunk builds and four allocation profiles pass in the current
run. The historical baseline's different Trunk environment and allocation
failure prevent treating those outcomes as a matched runtime improvement.
Live Hypercurve was untouched; failures remain unresolved.

The [computable metadata repair](../benchmarks/checkpoints/2026-09-17-verus-computable-metadata.json)
at `cdc6d3a` removes wrapped integer exponents and preserves nonzero bounds when
the exact exponent cannot fit `i32`. Both the baseline and previous runtime
panic in debug on integer `2^i32::MAX` metadata and falsely certify an exact
`i32::MIN` exponent for `2^(i32::MAX+1)`. A direct private-constructor probe also
finds that the previous runtime classified a nonzero rational as zero when its
exponent was unavailable; public direct-rational and rational-times-pi zero
queries did not reproduce that private defect. The repaired constructor retains
the sign and leaves the magnitude unresolved. All seven public boundary cases
pass in debug and release. The private diagnostic and focused checks used the
same code with two earlier comments; both source versions are preserved.

The exact committed repair passes 431 existing Verus units, 798/901 default and
all-feature test executions, 1,271/1,447 benchmark fixtures, representation
checks, strict Clippy and doctests. All 117 proof-input hashes match the commit.
The checked exponent helper is verified; the bound operations and their callers
still need refinement proofs. Other approximation kernels' precision limits
remain open. Build, coverage, fuzz, resource, native timing and pinned-consumer
qualification for this runtime are in progress; this is not baseline acceptance.

The [metadata resource checkpoint](../benchmarks/checkpoints/2026-09-17-verus-metadata-resources.json)
completes the remaining nine build/check steps and all 6,492 frozen ASan inputs
for `cdc6d3a`. Checks already covered by its exact-source root matrix were not
repeated. Coverage is 26,140 of 28,672 executable lines (91.17%). All resource
profilers pass; twelve representation workloads allocate less than the fixed
baseline, but both matched Massif pairs retain the 176-byte whole-process peak
increase. Callgrind reports 28,722,126 instructions versus 29,845,483. WASM is
1,662,080 bytes versus 1,646,116, and stripped native growth reaches 17,984 bytes.
Ordinary expanded Rust contains 36,100 lines versus 35,597. These differences
remain assessment items; profiler durations do not establish native performance.

The [metadata consumer builds](../benchmarks/checkpoints/2026-09-17-verus-consumer-metadata-builds.json)
pass all 41 non-tracing targets and the same five complete release suites
(204/361/191/860/222 passed, with seven Hypermesh tests ignored).
The [fixed-input replay](../benchmarks/checkpoints/2026-09-17-verus-consumer-metadata-fixed-inputs.json)
passes four builds and four 101-row fixture runs on the unchanged 400 stored
inputs. Hypercurve remains pinned to `8228394`; its full release and long cases,
the remaining consumer checks and native comparisons are still pending.

The [typed bound refinement](../benchmarks/checkpoints/2026-09-17-verus-bound-refinement.json)
at `1514ad4` includes nine production bound functions directly in the proof
target. Their contracts preserve complete typed results, checked inverse
exponents, floor-halved square-root exponents, zero sentinels and available
magnitude answers. The official run passes 443 units, 65 production contracts
and 87 model theorems, with all 124 repository input hashes bound to that
commit. A transparent import exposes the actual pinned dependency's sign enum;
it supplies no equality or arithmetic assumptions. The evidence also binds
169 source files across eight dependency packages and their compiled artifacts.

All thirteen new bound mutations are rejected, including false zero certificates
and dropped valid answers. Ten acceptance/dependency identity tests pass. The
unchanged kernel/model sources retain their prior 85-mutation and assumption
bypass evidence; no fresh combined 98-mutation run is claimed. Focused debug and
release tests and strict all-feature Clippy pass. Initial and final annotations
both erase to byte-identical ordinary Rust relative to `cdc6d3a`. Incoming bound
denotations, rational import, mapping, addition, square, multiplication, public
magnitude conversion, constants and concurrency remain outside the added proof
boundary. All thirteen whole-implementation obligation groups remain open.

The [metadata runtime screen](../benchmarks/checkpoints/2026-09-17-verus-metadata-screen.json)
compares `cdc6d3a` with the fixed pre-Verus baseline across all nine ordinary
benchmark suites. All eighteen processes pass, with 1,449 matching cases and
unchanged workload hashes. The short screen flags 157 positive, nonoverlapping
within-run intervals, including external comparator rows. Every case and raw
sample is retained. These flags require longer alternating repeats; the screen
does not establish performance parity or final acceptance.

The [bound denotation checkpoint](../benchmarks/checkpoints/2026-09-17-verus-bound-denotation.json)
at `3030b12` connects eight real-number lemmas to the checked production bound
contracts. Valid input certificates remain sound through construction,
negation, inversion and square root. The square-root binade proof covers
negative odd exponents; separated approximations certify sign and nonzero
status, and magnitude/sign queries return sound certificates. These results
remain conditional on valid incoming metadata and approximation error bounds.

The official run passes 460 units, 65 production contracts and 95 model
theorems. All 125 repository input hashes match the proof commit; dependency
source and artifact identities are checked. Thirteen production bound mutations,
six denotation mutations and ten acceptance/dependency tests pass their guards.
The unchanged kernel/model sources retain prior 85-mutation and assumption
bypass evidence; no fresh combined 104-mutation run is claimed. Runtime sources,
tests and Cargo inputs are unchanged from `1514ad4`, preserving its ordinary
runtime erasure comparison with `cdc6d3a`. Expression-graph inputs, rational
import, inexact planning bounds, cache validity and production callers remain
open, along with all thirteen whole-implementation obligation groups.

The [magnitude mapping checkpoint](../benchmarks/checkpoints/2026-09-17-verus-bound-mapping.json)
at `2d8cb1a` verifies the unchanged production `BoundInfo::map_msd` body and
adds two binary-scaling refinements. Mapping preserves signs, zero/unknown
states, exactness flags and every callback result. A checked exponent offset
preserves valid real certificates; overflow leaves the magnitude unavailable
while retaining sign and nonzero facts. Production callback closures and
incoming expression-graph certificates still need caller proofs.

The official run passes 468 units, 66 production contracts and 97 model
theorems, with all 125 repository hashes matching the commit. Sixteen production
bound mutations, nine denotation mutations and ten acceptance/dependency tests
pass. The prior 85 unchanged kernel/model mutations and assumption-bypass check
retain their existing evidence; no fresh combined 110-mutation run is claimed.
Strict all-feature all-target Clippy passes, and the annotations erase to
byte-identical ordinary Rust relative to `cdc6d3a`. All thirteen broader proof
groups and baseline acceptance remain incomplete.

The [fresh consumer core comparison](../benchmarks/checkpoints/2026-09-17-verus-consumer-metadata-core.json)
records all 51 commands on both `a2da8e2` and `cdc6d3a`, with identical commands,
controlled environments, working directories and pinned consumer sources.
The baseline passes 40 and fails eleven; the candidate passes 41 and fails ten.
Every candidate failure also occurs in the fresh control: the Symbolica instance
conflict, five formatting checks, Hypersolve lint/docs, Hypercurve lint and
Hypercurve UI tests. Both revisions fail the same two UI cases. All three UI
WASM/Trunk builds and Hypercurve UI Clippy pass on both revisions.

The candidate passes all four allocation profiles. Baseline Hypersolve fails
the Pow10 dense-Bareiss counting-epoch assertion with 432 retained bytes; the
candidate's corresponding row reports zero and its complete profile passes.
This bounded improvement does not resolve the separately measured whole-process
peak increase. The remaining feature/coverage, trace, long-case and native
consumer comparisons were not complete at this checkpoint. Live Hypercurve
remains untouched, and all failures are retained as unresolved outcomes.

The [cache codec checkpoint](../benchmarks/checkpoints/2026-09-17-verus-cache-codecs.json)
at `9418f85` verifies three pure methods from the actual production representation
file. Bound encoding preserves all typed states, signs, optional signed exponents
and exactness flags; a separate decoder specification proves lossless packing.
Exact-sign encoding and decoding preserve all five cache states, with valid tags
and unused bits zero. Decoding requires a valid input tag. Atomic operations,
publication, cache validity, memory ownership and callers remain unverified.

The official run passes 496 units, 69 production contracts and 99 model theorems,
with all 128 repository hashes matching the commit. Thirteen new cache mutation
guards and ten acceptance/dependency tests pass. The unchanged kernel/model and
bound inputs retain their earlier 85 and 25 mutation results; no fresh combined
123-mutation run is claimed. Strict all-feature all-target Clippy passes, and
ordinary expanded Rust remains byte-identical to `cdc6d3a`. All thirteen broader
proof groups and baseline acceptance remain incomplete.

The [public carrier checkpoint](../benchmarks/checkpoints/2026-09-17-verus-public-carriers.json)
at `4a162ed` verifies six certificate accessors, five dependency-mask operations
and seven mask constants from the actual public production file. Every known
answer and unknown state is preserved; masks support every `u16`, including bits
outside the named families. Private field visibility and all runtime bodies
remain unchanged. Certificate producers, dependency classification, sign
conversion adapters and generated trait implementations remain separate proof
obligations.

The official run passes 540 units, 87 production contracts (79 functions and
eight constants) and 99 model theorems. All 129 repository input hashes match
the proof commit. Twenty-one new mutation guards, ten acceptance/dependency
tests, formatting and strict all-feature all-target Clippy pass. Earlier
unchanged kernel/model, bound and cache inputs retain their 85, 25 and 13
mutation results; no fresh combined 144-mutation run is claimed. Ordinary
expanded Rust remains byte-identical to `cdc6d3a`. All thirteen broader proof
groups and baseline acceptance remain incomplete.

The [public bound adapter checkpoint](../benchmarks/checkpoints/2026-09-17-verus-public-bounds.json)
at `3d17187` verifies the unchanged public/private sign conversion bodies and
`BoundInfo::magnitude_bits`. Negative, zero and positive signs convert without
loss, and magnitude export preserves both the exponent and its exactness flag.
Three denotation lemmas connect these contracts to valid incoming real-number
certificates. Inexact magnitudes acquire no new accuracy claim.

The official run passes 546 units, 90 production contracts and 102 model
theorems, with all 129 repository input hashes matching the commit. Thirty-five
bound mutations, thirteen cache mutations and ten acceptance/dependency tests
pass. The unchanged kernel/model and public-carrier inputs retain their earlier
85 and 21 mutation results; no fresh combined 154-mutation run is claimed.
Formatting and strict all-feature all-target Clippy pass, and ordinary expanded
Rust remains byte-identical to `cdc6d3a`. The failed package-style test invocation
and successful documented script invocation are both retained. Callers,
certificate producers, all thirteen broader proof groups and baseline
acceptance remain incomplete.

The [deep consumer comparison](../benchmarks/checkpoints/2026-09-17-verus-consumer-metadata-deep.json)
records all 256 Hypertri feature checks and all 256 feature test configurations
passing on both revisions. The initial baseline test run exhausted the per-user
temporary-filesystem quota at configuration 125; its partial output and isolated
checkout recovery are retained separately from the complete rerun.

Hyperlimit production coverage is 10,797/10,849 baseline and 10,798/10,849
candidate. Hyperlattice coverage fails on the same Symbolica instance conflict
on both revisions. Hypertri baseline coverage passes at 6,271/6,600; candidate
coverage fails the unchanged 95% gate at 6,268/6,600. The three missing lines
count unknown edge zero-status results. An isolated replay of all 22 frozen
representation fixtures identifies `ConstProductSqrt`: the baseline reports
four unknown edges, while the candidate correctly recognizes all four as
axis-aligned. All other summaries agree, and an independent sine triangle
still exercises unknown outcomes. This explains the coverage change without
relabeling the failed gate as a pass.

Both Hypersolve coverage runs stop at the same stale representation assertion.
Paired isolated checks return `(2 - alpha)/7` with denominator one for
`1/(4 + 2 alpha)`, where `alpha^2 = 1/2`. Independent exact rational arithmetic
confirms equivalence with the fixture's `(1/2)/(2 + alpha)`. The original
failures remain recorded; supplemental fixture qualification is separate.
All ten dispatch-trace commands exit successfully, with 47 paired stdout
summaries. Hyperlattice executes its four trace assertions but suppresses the
report, leaving a separate count capture outstanding. Live Hypercurve remains
untouched; its long-case partition, native comparisons and final acceptance
are still pending at this checkpoint.

The [isolated consumer supplements](../benchmarks/checkpoints/2026-09-17-verus-consumer-metadata-supplements.json)
retain the subsequent fixture checks for `cdc6d3a`. Adding the sine triangle
restores Hypertri coverage to 6,271/6,600 on both revisions without changing
production code or the 95% gate. Correcting Hypersolve's expected canonical
representation lets both coverage runs pass the unchanged 85% gate, with
34,192/39,571 baseline and 34,194/39,571 candidate lines covered. The first
supplement attempt omitted tracked fuzz manifests needed by benchmark
fixtures; its four compilation failures and the complete repaired rerun are
both retained. All four Hyperlattice trace summaries were subsequently
captured and agree between revisions. Corrected certification benchmarks were
built on both revisions but have not been timed. These results concern the
intermediate `cdc6d3a` runtime; the later certified-precision repair requires
fresh affected qualification. No live consumer source was changed.

The [certified-precision checkpoint](../benchmarks/checkpoints/2026-09-17-verus-certified-planning.json)
records two numerical defects found while auditing those metadata results.
Both the baseline and `cdc6d3a` return 5,102 for `(16 + 512*pi)*pi` at precision
zero, outside the required one-unit error from approximately 5,103.50294.
For `1 / ((2 - sqrt(4 - 2^-16))*pi)`, the baseline's 83,443 satisfies the
contract, while `cdc6d3a` returns 98,304. Directed MPFR intervals independently
establish these counterexamples. Commit `89c89e2` makes error-sensitive
precision kernels use exact magnitude certificates or approximation refinement.
Its ordinary test matrices, all 6,492 frozen ASan inputs, release/WASM builds
and coverage run pass. A subsequent comparison audit finds that both the
baseline and this revision incorrectly collapse a perturbation exceeding 1,552
to `Equal` at tolerance 32. Commit `b3a23a4` applies the same certificate
requirement to this shortcut; known signs still supply exact ordering.

The [bound composition proof](../benchmarks/checkpoints/2026-09-17-verus-bound-composition.json)
then checks the actual square, multiply and add propagation operations. The
complete target verifies 563 units, 94 production contracts and 107 model
theorems. All 54 bound mutation guards and ten acceptance tests pass. Default
and all-feature runs pass 802 and 905 test executions, plus 1,271 and 1,447
benchmark fixtures. All typed outcomes are specified; real denotation remains
conditional on valid incoming bounds. Inexact hints acquire no magnitude
accuracy claim. Producer, graph, cache and caller obligations remain open,
and current resource, timing and consumer qualification is separate.

The [coverage accounting correction](../benchmarks/checkpoints/2026-09-17-verus-coverage-accounting.json)
excludes dedicated test files and named trailing test modules that the old
script counted as production. Reclassifying the preserved completed reports
gives 24,871/27,357 baseline production lines and 25,202/27,734 for `89c89e2`,
or 90.91% and 90.87%. The baseline report uses the previously preserved
classification-only API-inventory repair; its original failed command remains
a failure. Original reports are retained. These corrected counts replace the
production-only interpretation of the two listed legacy reports; neither
coverage measure is a formal proof percentage.

The [pinned Hypercurve partition](../benchmarks/checkpoints/2026-09-17-verus-hypercurve-metadata-partition.json)
completes the earlier `cdc6d3a` comparison with identical unsuccessful outcomes
on both revisions: the remaining suite times out after 900 seconds, five
isolated cases time out after 120 seconds each, and the retained mixed fillet
fails on the same Boolean predicate blocker. These are not passing suites or
native timing measurements. The temporary source is restored and live
Hypercurve remains untouched.

The [current resource checkpoint](../benchmarks/checkpoints/2026-09-17-verus-bound-composition-resources.json)
qualifies runtime `c61d6d8` with all nine remaining build/check steps and all
6,492 frozen ASan inputs passing. Corrected production coverage is
25,219/27,737 (90.92%). Every allocation row is retained: the Log10 and Log2
workloads allocate more than the baseline, and matched whole-process Massif
peaks rise from 24,896 to 25,224 bytes in both pairs. Callgrind instruction
counts are 29,845,483 baseline and 29,787,464 candidate for the representation
workload; this aggregate is not native timing or per-case performance parity.
WASM grows from 1,646,116 to 1,662,191 bytes, and the largest stripped native
increase is 17,760 bytes. These costs remain assessment items. Separate cache
query and multiplication scheduling experiments are unadopted and do not
replace this checkpoint's measured candidate.

The [cache field milestone](../benchmarks/checkpoints/2026-09-17-verus-cache-fields.json)
checks the construction and update word expressions using the production
masks and verified encoder contracts. It verifies 583 units, 94 production
contracts and 112 model theorems; all 19 cache mutation checks and ten
acceptance tests pass. Production, manifest and lockfile bytes are unchanged
from `c61d6d8`. These word-level results do not verify the atomic constructor,
compare-exchange loops, memory ownership or publication invariants. All
thirteen full-implementation groups remain open.

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

The [asymmetric product proof](../benchmarks/checkpoints/2026-09-17-verus-asymmetric-product.json)
is bound to `6124684`: 604 verification units, 94 production contracts and
117 model theorems pass with `--no-cheating`. Five new conditional real-error
lemmas cover signed scaling, cached bit-length upper bounds, asymmetric
multiplication with three guard bits and its zero shortcut. Seven new mutation
checks and the assumption-bypass control reject faults; ten acceptance and
dependency-identity tests pass. The failed guard preparations are retained,
including resource limits that were not accepted as mathematical rejections.
All production, manifest and lockfile bytes still match `c61d6d8`. These
models do not verify the graph, BigInt operations, machine precision arithmetic
or all comparison separation conditions; thirteen obligation groups remain open.

The [current consumer matrix](../benchmarks/checkpoints/2026-09-17-verus-consumer-bound-matrix.json)
records all 41 ordinary benchmark builds, five release suites, 400 fixed-input
fixtures plus their four aggregate rows, 51 core commands, both 256-configuration
Hypertri powersets and five dispatch runs for `c61d6d8`. Core results are
41 passes and ten failures, with original failure outcomes preserved against
the fixed baseline. Original Hypertri coverage is 94.97%, below its 95% gate;
Hyperlattice and Hypersolve coverage fail for the previously recorded license
and fixture reasons. The
[isolated supplemental controls](../benchmarks/checkpoints/2026-09-17-verus-consumer-bound-supplements.json)
pass both corrected fixture coverages and capture identical four-workload
Hyperlattice trace summaries. They do not relabel the original failures.
The pinned Hypercurve long-case partition and native timings remain pending;
live Hypercurve has not been changed.

The [allocation experiments](../benchmarks/checkpoints/2026-09-17-verus-anchor-experiments.json)
remain isolated overlays. The second certified-anchor version preserves shared
constant cache routing, avoids copying cached integers for magnitude queries,
and schedules an unknown multiplication operand directly from one certified
anchor. It passes 802 default and 905 all-feature test executions, 1,271 and
1,447 benchmark fixtures, strict Clippy, representation checks, doctests and
all 6,492 frozen ASan inputs. Its twenty measured allocation/byte/peak/retained
rows are each at or below baseline. Matched whole-process combined peak is
still 72 bytes higher in both pairs (24,896 to 24,968), with increased heap and
allocator-overhead maxima recorded separately. Aggregate Callgrind counts
fall from 29,847,905 to 28,039,395; Memcheck reports zero errors. These are
resource measurements, not native speed or acceptance evidence. The earlier
experiments, complete patches and unresolved costs are retained. No experiment
has been adopted; native comparisons remain pending.

The [strict-error follow-up](../benchmarks/checkpoints/2026-09-17-verus-strict-product.json)
binds 610 passing verification units, 94 production contracts and 118 model
theorems to `0d056af`. It strengthens the product result to a strict one-unit
bound under an open sampled-magnitude bound, and proves that a two-integer
approximation gap certifies strict order when both input errors are strict.
The inclusive error model alone cannot justify that ordering rule. All ten
targeted product/order mutations and the assumption-bypass control are
rejected; ten acceptance/dependency tests pass. Production callers still need
to establish the premises. No runtime byte changed and all thirteen obligation
groups remain open. The
[detailed experimental peak profiles](../benchmarks/checkpoints/2026-09-17-verus-anchor-peak-diagnostics.json)
retain allocation trees at each variant's whole-process peak; different peak
locations and compiler inlining prevent attributing their differences to one
source change. The measured 72-byte increase remains unresolved.

The [cache-refinement sign theorem](../benchmarks/checkpoints/2026-09-17-verus-cache-refinement.json)
raises the checked target to 613 verification units, 94 production contracts
and 119 model theorems. If two cache entries validly approximate the same real
and the later entry has a strictly smaller unit, it retains the nonzero sign
certified by an earlier integer whose absolute value exceeds one. This supports
the two cache reads in magnitude queries, conditional on the unverified cache
refinement invariant. Both new mutations and the assumption-bypass control are
rejected, and ten acceptance/dependency tests pass. Runtime bytes remain those
of `c61d6d8`; concurrency, ownership and all thirteen obligation groups remain
open.

The [completed pinned Hypercurve partition](../benchmarks/checkpoints/2026-09-17-verus-hypercurve-bound-partition.json)
records the final `c61d6d8` outcomes against the unchanged baseline controls:
the remaining suite times out at 900 seconds, five isolated cases time out at
120 seconds each, and the retained rational-arc/analytic-parallel fillet case
fails. All seven exit outcomes match the fixed baseline. None is counted as a
completed passing command. The temporary dependency checkout is restored and
the pinned Hypercurve inputs remain byte-identical; live Hypercurve is untouched.

The [three-way native screen](../benchmarks/checkpoints/2026-09-17-verus-anchor-screen.json)
completes all 27 serial runs across nine suites, retaining 1,449 matching cases
per variant and every comparator. Against the fixed baseline, 321 committed
runtime cases and 322 experimental cases have positive, nonoverlapping
within-run median intervals; 327 experimental cases are flagged against the
committed runtime. These are short-run follow-up signals, not confirmed
regressions or acceptance. All raw samples, commands, input bindings and logs
are archived. The optimization remains unadopted, and offset-rewrite and
precision-boundary correctness probes are separate outstanding work.
