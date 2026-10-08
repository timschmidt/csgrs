# Checkpoint 62 — proof/reuse, polynomial-facts and monic statistics

Seven historically accepted CPU campaigns are reanalysed from all 24,048 rows /
675 comparisons. Thirty-nine formerly directional comparisons now include one;
none changes from inconclusive to directional or reverses direction. All point
estimates and marginal measurements are preserved. Large selected reuse/facts
benefits, substantial unresolved-query costs and several monic slowdowns survive
both corrected methods. The completeness-first retention decisions are not
reversed, but no universal performance or complete historical-impact claim follows.

One earlier campaign, 1,728 rows / 48 comparisons, remains rejected because it
overlapped Memcheck. Recomputing its estimator does not make its timing evidence
eligible. It is kept as explicitly ineligible historical analysis, never pooled
with the accepted campaigns. Total CPU reconstruction is 25,776 rows / 723
comparisons. Separate allocation reconstruction covers 5,040 measured rows.

## File-by-file tooling audit

All lines of twelve affected runners/verifiers were read, together with the
77-line bounded allocation runner. Hashes and exact ranges are in the analysis.
These are audit-tooling reads, not new donor source credit.

| File | Read range | Findings |
| --- | --- | --- |
| `run-root-exp-proof-bench.mjs` | 1–68 | Query/numeric CPU use 12 paired blocks; allocation uses three. Pilots are run but not preserved in summaries, so their calibration cannot be reconstructed. Additional certified identities differ from baseline Unknown. |
| `verify-root-exp-proof-checkpoint.mjs` | 1–147 | Reproduces old sampler in both modes; independently checks recorded query outcomes, test/oracle/state reports, failed build attempts and source deltas. These roles are distinct. |
| `run-root-exp-reuse-bench.mjs` | 1–66 | Three rotated palindromic variants; original CPU run overlapped a memory check and remains rejected. No pilot records in summary. |
| `run-root-exp-reuse-confirm-bench.mjs` | 1–66 | Same cases/order after overlap ended; uses the defective sampler too. Confirmation is kept separate from the rejected run. |
| `run-root-exp-reuse-alloc-bench.mjs` | 1–66 | Allocation-only, three blocks, with ten fresh or 100 warm queries. Instrumented clock intervals are not CPU performance evidence. |
| `verify-root-exp-reuse-checkpoint.mjs` | 1–103 | Explicitly checks overlap and rejection, weak-reuse source/tests and separate allocation records; its estimator reproduction is archival only. |
| `run-reuse-cost-bench.mjs` | 1–100 | Numeric and fresh-TLS first-touch campaigns; first/warm/lifecycle clocks are distinct. Warm metrics divide by 16. Pilots are preserved. Allocation clocks are explicitly excluded from CPU evidence. |
| `verify-reuse-cost-checkpoint.mjs` | 1–127 | Checks timing boundaries, query outcomes, pairing/normalization, source/binary identities, allocation equality versus v2 and preserved donor vector failures. |
| `run-polynomial-facts-cpu.mjs` | 1–88 | 36 groups, three variants, two comparisons each. Newly known results differ from baseline early Unknown; comparing both as equal work would be wrong. |
| `verify-polynomial-facts.mjs` | 1–171 | Reproduces old intervals and checks public/state/native/test records, bounded address-dependent live-byte ranges, preserved failed allocation attempt and representative sizes. |
| `run-monic-costs.mjs` | 1–84 | 162 groups with CPU/allocation split; same known/degree does not establish equal representation or downstream proof availability. |
| `verify-monic-costs.mjs` | 1–136 | Reconstructs CPU/allocation/churn, known/degree recipe outcomes, native composition/fraction-free records and retained unfavorable controls. |
| `run-polynomial-facts-allocation-bounded.mjs` | 1–77 | No confidence estimator; fixed 100-query batches and longer churn. Address-dependent retained-byte ranges are preserved rather than forced deterministic. |

Affected scripts total 1,222 lines; supporting allocation runner adds 77.
Current 956 live identities and 175 isolated point-candidate files recheck
unchanged. Retained source snapshots of 180, 953 and 954 files are checked in
their historical versions, not mistaken for the current workspace version.
All source/evidence entries in eight relevant historical/retained manifests
also recheck. No production source is edited.

## Reconstruction, validity and controls

The corrected reader enforces prescribed group descriptors, full row membership,
variant/block order, iteration/worker counts, positive integer clocks, expected
known/Unknown/query outcomes, monic degrees, instrumentation flags and stored
pilots where available. It reconstructs all paired ratios and marginal metrics
exactly. Fresh-TLS warm clocks and allocations retain the original per-worker /
sixteen-warm-query normalization; the lifecycle clock is not a per-query clock.
Empty-clock samples are preserved without subtracting them.

Early proof/reuse pilot observations and per-observation timestamps were not
preserved. This correction does not invent them or claim to reconstruct their
calibration; it validates the recorded iteration count and declared limits.
Timing rows do not generally contain full numerical outputs. They are marked
as separate numeric-contract qualification, matching recorded outcomes, or
additional answers. Matching known/degree alone is not a full-result equality
certificate. The accepted comparisons comprise 216 separate numeric-contract,
378 matching-recorded-outcome and 81 additional-answer comparisons.

Checkpoint 60's tested rejection-sampled deterministic bootstrap and independent
order-statistic interval are reused. Twenty thousand replicates replace five
thousand, so endpoint changes include stream/replicate-count effects as well as
removal of the known sampling constraint. Both methods remain conditional on
suitable independent/common-distribution blocks; neither establishes those
assumptions, adjusts for multiple comparisons, nor attributes host/runtime effects
to the algorithm. All original intervals are retained as withdrawn archival data.

Twenty deliberate corruptions are rejected, including invented baseline equality,
wrong outcome/degree, raw pairing/count corruption, missing preserved pilots,
nonzero CPU allocation instrumentation, impossible lifecycle clocks and incorrect
warm normalization. An independent eight-case binary-sign enumeration verifies
that a three-observation min/max median interval covers only six cases (75%)
under continuous IID sampling. No finite interval using those observed extremes
can claim 95% coverage. The 441 historical allocation-instrumented timing
intervals remain withdrawn and are not replaced with CPU confidence claims;
their raw medians and allocation counts remain available.

## Corrected counts

Each triplet is below one / above one / including one. The ratio numerator is
the candidate; three-variant rows include both controls and, for first-touch,
all three clocks. These are comparison counts, not independent bugs or a pooled
overall effect. Additional-answer strata remain separately available in JSON.

| CPU campaign | Comparisons / rows | Old (withdrawn) | Corrected bootstrap | Order statistic |
| --- | --- | --- | --- | --- |
| Proof queries | 33 / 1,584 | 15 / 6 / 12 | 15 / 6 / 12 | 15 / 5 / 13 |
| Proof numeric | 72 / 3,456 | 20 / 8 / 44 | 17 / 6 / 49 | 14 / 3 / 55 |
| Reuse confirmation | 48 / 1,728 | 20 / 14 / 14 | 18 / 13 / 17 | 18 / 12 / 18 |
| Reuse numeric | 144 / 5,184 | 40 / 11 / 93 | 33 / 9 / 102 | 28 / 6 / 110 |
| Reuse first-touch | 144 / 1,728 | 47 / 58 / 39 | 44 / 57 / 43 | 43 / 53 / 48 |
| Polynomial facts | 72 / 2,592 | 30 / 8 / 34 | 28 / 8 / 36 | 28 / 8 / 36 |
| Monic normalization | 162 / 7,776 | 60 / 10 / 92 | 48 / 6 / 108 | 38 / 5 / 119 |

626 endpoint pairs change in those 675 accepted comparisons. The 39 lost
directional claims are distributed 0/5/3/9/4/2/16 across the table. The already
rejected overlap has 41 changed endpoint pairs and two lost directional
classifications, but remains excluded regardless of those numbers.

## Retention implications and unfavorable controls

| Workload / comparison | Paired ratio | Corrected bootstrap | Order statistic |
| --- | --- | --- | --- |
| Deep unresolved first-touch / baseline Unknown | 3.772397 | [3.682088, 3.864299] | [3.651077, 3.910268] |
| Same worker's warm unresolved queries / v2 | 0.284191 | [0.279352, 0.288462] | [0.279255, 0.289540] |
| Same warm unresolved queries / baseline | 1.216615 | [1.173297, 1.229263] | [1.160238, 1.238184] |
| Retained degree-16 log-self / v1 | 0.069457 | [0.065662, 0.071966] | [0.065264, 0.072624] |
| Same log-self / baseline Unknown (additional work) | 1.609456 | [1.598695, 1.627763] | [1.594682, 1.628797] |
| Monic kind 2 / code 12 / fresh | 1.066729 | [1.025035, 1.148326] | [1.022852, 1.178293] |

The first-touch unresolved control is independently represented depth 128.
Marginal first-query times remain 4.186 to 15.785 microseconds versus baseline;
warm queries remain 9.055 to 2.590 microseconds versus v2, while baseline warm
queries take 2.140 microseconds. All return Unknown in this control. Additional
certified identities are separately recorded and must not obscure this cost.

Fact-first degree-16 retained log-self remains 67.286 to 4.549 microseconds
versus v1, with 1,117 to 13 requested allocation calls per query and zero live
delta in the measured batch. Baseline returns Unknown in 2.861 microseconds,
not the same answer. Monic requested allocations/bytes remain lower in 56 of
162 groups and never higher in this corpus; incremental peaks are lower in
44 groups and never higher, with equal live deltas. Its six corrected above-one
intervals (five under both methods) are not erased by the memory improvement.

These observations do not indicate reversing the three retained decisions:
their mathematical/completeness rationale is unchanged, intended-path reuse/facts
gains persist, and monic allocation savings persist with documented timing costs.
This is not a claim that all five transfers' historical statistics are closed.
Earlier prototypes/sign heuristics and other experiments still require review;
the correction does not reopen rejected transformations or silently add new ones.

## Verification, storage and open work

Historical eighteen-checkpoint verification passed 2026-09-11T01:19:25.861Z,
with 18 output records / 19,692 bytes and empty stderr. It reproduces old
intervals as evidence integrity only, while rechecking recorded public/state/
test, memory, allocation, size and donor-failure results. It does not rerun Rust
tests or numerical/benchmark executables or create a new independent numerical
oracle. Known failures, address-dependent memory occupancy, nonzero reachable
memory, the separately recorded monic thread-memory qualification, and nine
ignored downstream tests are not relabelled clean.

Reanalysis passed 01:25:53.698Z. Full corrected recomputation plus twenty
mutation controls passed 01:28:30.694Z, preserving the eighteen historical
output records verbatim before the new result. All three commands exited 0 /
null signal. The analysis adds 2,254,179 workspace bytes, no new /tmp files.
No new source copy, binary, benchmark, production/donor edit, retained change,
cleanup/deletion, commit or push. Final verification is recorded in the mutable
README/root ledger once terminal. Donor coverage remains 1,415 complete files /
20 partial files / 183,653 unique read lines.

Of the 45 known top-level sampler matches, 21 are outside the campaign scopes
addressed by checkpoints 60–62: log/erf, early root-exp/sign/opaque variants,
initial polynomial-decision, rank, sign-filter/mask and complex-product variants
including their archived checker copies. That is not an exhaustive workspace
statistical inventory. The original ecosystem references and supporting source
reads, candidate runtime attribution/consumer/size gates and separate power-sum
work also remain open. Full audit completion is not claimed.
