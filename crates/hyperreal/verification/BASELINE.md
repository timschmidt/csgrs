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
