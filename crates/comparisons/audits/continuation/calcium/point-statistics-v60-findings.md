# Checkpoint 60 — correction of audit confidence intervals

The audit's historical bootstrap sampler has a concrete correctness defect.
Four point-family campaigns are now recomputed from all 151,296 measured rows,
preserving every original paired point estimate and marginal median. The new
analysis covers 3,920 interval comparisons. It does not change Hyper arithmetic,
raw timings, source snapshots, correctness tests, allocation counts or retained
code. Historical confidence claims using the defective sampler must not be
relied on until their own correction is recorded.

## Defect and independent tests

The old helper updates a 32-bit state by
`s = (1664525*s + 1013904223) mod 2^32`, then chooses index `s % n`.
Modulo four, the multiplier is one and the increment is three. Consequently
the state residues cycle 0,3,2,1. When `n=12`, index reduction preserves those
residues: every twelve-draw bootstrap sample has exactly three indices in each
of four residue classes. It is a constrained, dependent resampling scheme,
not the stated ordinary IID bootstrap with replacement. Recomputing it exactly
in a verifier establishes reproducibility, not statistical validity.

The failure is demonstrated without performance data. Exhaustively enumerate
all 256 four-draw resamples from `[0,0,0,1]`: 189 have median zero, 54 have median
one-half, and 13 have median one. The central percentile interval is `[0,1]`.
The old generator with `n=4` always takes every index once and returns median
zero, producing `[0,0]` for all 5,000 resamples. The corrected helper reproduces
`[0,1]` in its 50,000-replicate control. This is an audit-tooling bug, not a donor
or Hyper exact-real bug.

The replacement uses an AES-256-CTR deterministic pseudorandom word stream,
with a SHA-256-derived domain/campaign/reference/group key. Integer sampling
rejects the incomplete final residue range before reducing modulo `n`, so
uniform words give unbiased indices. The published deterministic keys make
reanalysis reproducible; this is numerical simulation, not security key use.
Tests cover rejection boundaries, stream refill/reproducibility, medians,
non-forced residue counts and the exhaustive finite example above.

Each corrected interval uses 20,000 replicates instead of the former 5,000.
Thus individual endpoint differences also include changed Monte Carlo sample
size/stream, not just the sampler defect. No raw timing observation is deleted.

A separate sampler-free order-statistic interval is computed for each twelve-
block median. Its endpoints are the third and tenth sorted block ratios;
coverage is `1 - 2*(1+12+66)/4096 = 3938/4096`, approximately 96.1426%, under
independent observations from a common distribution. Ties are conservative.
This is not a proof that benchmark blocks meet those assumptions. Neither
method establishes independence, fixed frequency, causal attribution or a
universal speedup; individual intervals remain unadjusted for multiple tests.

## What was recomputed

The reanalysis reads all original measured rows and checks group identities,
sample order, variant/block counts, iteration counts and stable full reports.
It recomputes each per-block median ratio, overall paired median and marginal
query-time median, asserting exact agreement with the historical point
estimates. Where historical block-ratio arrays exist, they must also agree.
Then it records old and corrected intervals side by side.

| Campaign | Measured rows | Interval comparisons |
| --- | ---: | ---: |
| Point-image 53 | 3,840 | 80 |
| Native history 56 | 36,864 | 768 |
| Native demand 57 | 55,296 | 1,536 |
| WASM 59 | 55,296 | 1,536 |
| Total | 151,296 | 3,920 |

All paired point estimates and marginal medians are unchanged. Of the 3,920
comparisons, 3,648 bootstrap endpoint pairs change and 269 formerly directional
claims now include ratio one. No formerly inconclusive comparison becomes
directional in this particular reanalysis. These are interval comparisons,
including repeated workloads and two references in the demand campaigns, not
3,920 independent experiments or 269 distinct algorithm regressions.

Equal-full-result groups are summarized below. Entries are counts of intervals
wholly below / wholly above one; lower favors the candidate/demand variant.

| Comparison | Groups | Historical bootstrap | Corrected bootstrap | Order-statistic interval |
| --- | ---: | ---: | ---: | ---: |
| 53 candidate / baseline | 70 | 17 / 5 | 11 / 2 | 9 / 1 |
| 56 eager / baseline | 512 | 14 / 192 | 9 / 151 | 5 / 114 |
| 57 demand / baseline | 512 | 8 / 134 | 4 / 96 | 2 / 73 |
| 57 demand / eager | 768 | 119 / 184 | 95 / 141 | 79 / 111 |
| 59 demand / baseline | 512 | 165 / 34 | 140 / 19 | 113 / 6 |
| 59 demand / eager | 768 | 159 / 33 | 134 / 24 | 118 / 23 |

Newly successful baseline comparisons remain separate and do additional work.
Their full counts and every corrected interval are in
`point-statistics-v60-analysis.json`; do not relabel them equal-work speedups.

## The apparent WASM discrepancy

For case 17 / STRICT / deep / fresh, demand/baseline keeps paired ratio
**1.217574** and marginal medians **121.079 / 119.464 microseconds**. The old
interval `[1.107369,1.333005]` excluded one. The corrected bootstrap is
**[0.973856,1.360412]** and the order-statistic interval
**[0.965811,1.402592]**. Both now include no change. The original confidence
claim was unsupported; this does not make the raw point-estimator discrepancy
disappear or identify its runtime cause.

For case 44 / approximate / round-trip / retained, the early InvalidEvidence
control keeps ratio 1.335991. Corrected bootstrap `[0.981360,1.671217]` and
order-statistic `[0.968471,1.697000]` remain inconclusive.

The useful Unknown-endpoint result remains directional: case 37 / approximate /
constructed / retained has demand/eager ratio **0.767287**, corrected bootstrap
**[0.733356,0.782960]**, order-statistic **[0.729751,0.790640]**, and unchanged
eager/demand marginal medians **32.331 / 24.882 microseconds**.

The retry cost also remains directional: case 3 / approximate / round-trip /
retained has demand/eager ratio **1.081198**, corrected bootstrap
**[1.025073,1.211209]**, order-statistic **[1.017033,1.226945]**, and unchanged
medians **15.388 / 15.883 microseconds**. Retain both favorable and unfavorable
evidence. These individual estimates do not establish an overall improvement.

## Historical impact and limits

The source scan records **45 matching top-level continuation scripts**, with
hashes and matching line numbers. It includes native/WASM `e` planner work,
derivatives, monic/resultant decisions, exponential proof/cache experiments,
sign filters, complex products and rank searches, plus their checkers and
frozen checker copies. A match is a potential-impact entry, not proof that
every script uses twelve samples or that all its reported quantities are bad.
The scan is not a file-by-file closure of their statistical semantics and does
not cover every other directory in the ecosystem audit.

Only the four campaigns above are corrected here. Other historical intervals
and any inference depending on them require their own source/raw-data review.
The five retained continuation changes are not silently reverted: their raw
costs, mathematical tests and allocation results are unchanged. Their prior
confidence-based justifications are qualified by this notice until reanalysis
establishes what remains supported. Do not claim retention impact closure from
the point-family subset, or confuse source/evidence integrity with valid inference.

The planned runtime replay was not launched after this more fundamental defect
was found. Local Node 22.22.2 exposes process and thread CPU counters, and V8
exposes a single-threaded-GC diagnostic flag; those are available for a later
controlled replay, not new measurements. Node documents the CPU counters as
microsecond user/system usage and distinguishes process from thread scope.
[Node process documentation](https://nodejs.org/api/process.html#processthreadcpuusagepreviousvalue).

## Resource and verification boundary

No new temporary files, compiled modules, source-tree copies or benchmark
processes are needed. The existing raw evidence and frozen binaries remain
unchanged; generated reanalysis and captures live in the workspace. No live
production/donor edit, cleanup/deletion, commit, push or external report occurs.
Donor coverage remains 1,415 complete files, 20 partial and 183,653 unique lines
for this continuation only. All 956 live identities and 175 candidate identities
remain unchanged.

The reanalysis capture completes 2026-09-11T00:16:48.384Z, exit 0; the independent
self-test capture passes 00:19:47.747Z. The separate checker passes
00:21:38.479Z, exit 0 / empty stderr, recomputing the entire analysis instead of
merely hashing its output. The generated analysis occupies 7,317,447 workspace
bytes, excluding capture logs. The checkpoint manifest binds
the old/raw data, matching source inventory, new helper/tests and corrected
results. Its verifier extends 48–59 while preserving their original, now-qualified
statistical claims; it does not rerun the full historical 47-chain.

Next finish historical statistical-impact review and corrected intervals,
then the disclosed wall/process/thread-CPU diagnostic controls, final selected-
version consumers/representative sizes and retention decision. The separate
power-sum candidate, remaining source/support reads, all original references
and complete inventory reconciliation remain open. This is progress, not a
whole-audit completion claim.
