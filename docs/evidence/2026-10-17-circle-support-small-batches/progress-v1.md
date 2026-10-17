# Guarded circular-support cache: rejected on small cold batches

**Reject the E42 guarded proposal as written.** Production remains unchanged.
The earlier 35-file native bundle and the original unguarded index remain
rejected; independently admitted repairs remain intact. This is a measured
performance rejection, not a correctness failure or a claim that the remaining
native source-fidelity gaps have been repaired.

The [predeclared contract](contract-v1.md) supplements E42's single-batch timing
with repeated complete operations. Each phase performs at least 65,536 support
queries. Cold means a fresh operation-local cache for every batch, including
cache construction and destruction. Warm uses one prepared cache. Both phases
include independent exact validation, complete ordered-output materialization,
comparison and output destruction. Input construction, reference preparation,
printing and process startup remain outside the phase timer and inside the
whole-process resource observations. Scalar/process state is reused within a
run; this is cold batch-cache behavior, not a claim to reset every scalar cache.

Both diagnostic binaries use the same new test module, dependencies, toolchain,
optimization flags and no-feature configuration. Their production source bytes
are unchanged from the E42 baseline and guarded candidate. The eight focused
cache contracts pass in each binary; strict direct Clippy passes for both test
harnesses. The twelve-process preflight passed before the matched experiment.
These are not additional full-library, Cargo-matrix or downstream runs.

All **84 matched processes / 168 timed phases** completed with strict certified
results. Every query checks the independently derived 3-4-5 secant coordinates
and both circle equations. Complete ordered rows agree across repetitions,
cold/warm phases and variants. The six sizes run seven times per variant in
deterministically shuffled case order with alternating variant order. See
[raw samples](run/cost-matched-v1/samples.json),
[all commands](run/cost-matched-v1/commands.json),
[exact rows](run/cost-matched-v1/witnesses.json) and
[the complete summary](run/cost-matched-v1/summary.json).

Median seconds for the whole repeated phase:

| Pairs per batch | Baseline cold | Guarded cold | Baseline warm | Guarded warm |
| ---: | ---: | ---: | ---: | ---: |
| 1 | 0.104170548 | 0.111512783 | 0.051677678 | 0.048588529 |
| 4 | 0.119518253 | 0.139083558 | 0.066655698 | 0.063670333 |
| 8 | 0.136049177 | 0.162542595 | 0.079588628 | 0.076071060 |
| 9 | 0.138268984 | 0.165127948 | 0.080282720 | 0.081004281 |
| 16 | 0.174278428 | 0.177717082 | 0.101114358 | 0.087969062 |
| 128 | 0.436716406 | 0.171168452 | 0.356841387 | 0.092444354 |

Cold batches of 4, 8 and 9 are respectively **16.37%, 19.47% and 19.43%**
slower. Each has seven positive paired differences and its entire candidate
sample range lies above the baseline range, satisfying the rejection criterion
set before measurement. The one-pair cold median is also 7.05% worse; cold 16
and warm 9 have smaller, overlapping differences. All favorable warm and
128-pair results are retained, including the substantial control-case gains.
Those gains do not cancel the stable regressions in supported small workloads.

The source still creates and populates an exact-key index on every eligible
cold miss, including small batches that never use it for a hit. That is an
implementation observation and a candidate explanation, not a measured
allocation or instruction attribution from this experiment. E42's allocation
and binary-size evidence remains available; this follow-up does not claim new
heap, package-size, end-to-end or cross-library measurements.

No production edit or public contract migration is applied. Remaining
integration, full Cargo Clippy, documentation, example, benchmark, native
workload/report and real-binding gates are not run for this rejected version.
Its E42 source patch and all prior favorable and unfavorable evidence are
preserved. A successor must avoid the small-batch regression while preserving
ordered matching priority, exact keys, symbolic/large-coefficient fallback,
certainty and all original full-scale workload gates. Lazy index construction
is a hypothesis to test, not an admitted change.

Only `current_work` changes in the execution ledger; all bytes and decoded
top-level keys outside it remain bound to `9025de62`. Every new physical path
is hashed separately. The inherited 27,093 missing artifact paths remain
disclosed; this is not a successful whole-history preservation audit. No closed
target was reopened, altered or removed. Full properties-plan, release,
coverage, campaign and proof work remains open.
