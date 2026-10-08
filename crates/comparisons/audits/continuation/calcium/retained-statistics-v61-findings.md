# Checkpoint 61 — retained planner and derivative statistical correction

Five additional campaigns are corrected from all 10,672 measured rows and all
199 paired comparisons. Seven formerly directional bootstrap comparisons now
include one; no inconclusive comparison becomes directional and none reverses
direction. All paired point estimates and marginal medians are unchanged.
The large selected planner/derivative benefits and disclosed planner costs
survive both corrected methods. This does not close the other historical
statistics, original ecosystem inventory, or unselected transfer experiments.

## Source audit and defect applicability

Every line of these nine audit scripts was read; exact whole-file ranges and
hashes are recorded in `retained-statistics-v61-analysis.json`. These are tooling
reads, not new Calcium/FLINT source credit. All nine occur in checkpoint 60's
45-file potential-impact inventory; that inventory is preserved unchanged.

| File | Read range | Result of comparison |
| --- | --- | --- |
| `run-e-plan-costs.mjs` | 1–74 | All 71 CPU groups use twelve paired ABBA/BAAB blocks and the defective sampler; separate allocation mode has no bootstrap inference. |
| `check-e-plan.mjs` | 1–125 | Reproduces the same sampler; also independently checks factorial thresholds, directed exact enclosures, raw pairing and allocations. Preserve these distinct roles. |
| `run-e-qualified-controls.mjs` | 1–42 | All ten follow-up groups use forty blocks; the sampler remains defective. The original unfavorable controls are not replaced or pooled away. |
| `run-e-qualified-wasm-costs.mjs` | 1–56 | All 22 groups use twelve blocks; explicit GC/setup and fresh instance boundaries are unchanged by this offline correction. |
| `check-e-qualified.mjs` | 1–122 | Reproduces both affected campaigns and separately checks archived test membership, native/WASM exact outputs, source-related records and application sizes. |
| `run-derivative-costs.mjs` | 1–69 | All 80 CPU groups use twelve blocks; allocation mode is separate. Rational/pi-scaled, high/low order and retained/fresh workloads remain included. |
| `check-derivative-costs.mjs` | 1–78 | Reproduces the same sampler; raw ordering, calibration, fixed allocation counts and shared nonzero live demand remain independently checked. |
| `run-derivative-endpoint.mjs` | 1–68 | All sixteen complete tangent-order traversal groups use twelve blocks. They are not an isolated endpoint-helper benchmark. |
| `check-derivative-qualification-costs.mjs` | 1–90 | Reproduces affected endpoint intervals; independently rechecks unchanged endpoint allocation counts and the separate 320-row retention staircase. |

Total: 724 read lines. Supporting source-binding helpers and the corrected
statistical helper were also inspected. Revalidation checks the unchanged 956
live files, 175 isolated point-candidate files, planner snapshots (955/956),
both derivative snapshots (955 each), and all entries in the four historical
manifests (94/211/70/150 entries; not disjoint file counts).

For twelve draws, the old modulo index generator fixes three indices in each
class modulo four. Forty draws do not fix this: modulo eight the original
multiplier is five and the increment seven; its state visits every residue in
an eight-step cycle. Reducing the state modulo forty preserves modulo eight,
so each forty-draw resample contains exactly five indices of each residue.
This is not ordinary independent resampling with replacement. More historical
blocks alone did not validate the estimator.

## Reconstruction and independent controls

The new reader enforces every prescribed group descriptor, row/variant/block
order, calibration, pilot and query count, positive integer clock, timestamp,
fingerprint where recorded, zero CPU allocation instrumentation, and WASM
memory-field contract. It reconstructs both paired and marginal point estimates
exactly before calculating new intervals. No observation is discarded, winsorized,
reordered, or pooled with another campaign. Original intervals remain beside
the corrections as withdrawn archival evidence.

Checkpoint 60's deterministic rejection-sampled AES-CTR bootstrap is reused
with campaign/group-separated keys and 20,000 replicates. The historical 5,000
replicates and stream both change, so individual endpoint differences must not
be attributed exclusively to the index defect. The second interval uses order
statistics without a random sampler. A separate BigInt Bernoulli convolution
verifies ranks/coverage for 6, 12 and 40 observations. For forty blocks it gives
ranks 14 and 27 with coverage 1057205379912/1099511627776, approximately 96.1523%,
under the stated sampling assumptions. Fourteen deliberate corruptions of raw
pairing, counts, clocks, descriptors, outputs and summary values are rejected.

Both interval methods require suitable independent, common-distribution timing
blocks. This audit does not establish those assumptions, rule out runtime/order
effects, or adjust these individual intervals for multiple comparisons. A
directional interval is not an algorithm-level causal or universal speed claim.

## Corrected counts

Each triplet below means interval below one / above one / including one;
candidate/baseline is the ratio, so below one favors the candidate.

| Campaign | Groups / rows | Old (withdrawn) | Corrected bootstrap | Order statistic |
| --- | --- | --- | --- | --- |
| Planner original | 71 / 3,408 | 37 / 3 / 31 | 36 / 3 / 32 | 35 / 2 / 34 |
| Planner native follow-up | 10 / 1,600 | 6 / 1 / 3 | 5 / 1 / 4 | 4 / 1 / 5 |
| Planner WASM | 22 / 1,056 | 12 / 1 / 9 | 11 / 1 / 10 | 11 / 1 / 10 |
| Derivative main | 80 / 3,840 | 55 / 1 / 24 | 52 / 0 / 28 | 48 / 0 / 32 |
| Derivative endpoint consumer | 16 / 768 | 5 / 0 / 11 | 5 / 0 / 11 | 3 / 0 / 13 |

187 of 199 bootstrap endpoint pairs change. The seven lost directional
comparisons are: original planner pi/coarsen at 65,536 bits; follow-up pi/fresh
at 65,536; WASM public/fresh at 4,096; and four pi-scaled retained-curve derivative
groups (degree/order 1/0, 3/3, 24/0, 24/1). The first derivative case was the sole
historical above-one interval; the other six comparisons were below one. These
are comparison classifications, not seven independent numerical bugs.

## Retention implications

| Selected workload | Paired ratio | Corrected bootstrap | Order statistic |
| --- | --- | --- | --- |
| Native follow-up fresh e, 65,536 bits | 0.397506 | [0.389814, 0.417342] | [0.389504, 0.419939] |
| Native follow-up fresh e, 262,144 bits | 0.241813 | [0.238711, 0.244162] | [0.238567, 0.244464] |
| WASM fresh e, 65,536 bits | 0.499180 | [0.460491, 0.543962] | [0.450091, 0.560547] |
| WASM fresh e, 262,144 bits | 0.380059 | [0.373113, 0.386156] | [0.368923, 0.387325] |
| Rational derivative, degree 8 / order 128 / retained curve | 0.146542 | [0.141653, 0.148452] | [0.138498, 0.148981] |
| Native follow-up coarsen e, 262,144 bits | 1.027059 | [1.023323, 1.029615] | [1.023185, 1.029709] |
| WASM warm e, 4,096 bits | 1.053223 | [1.024556, 1.072865] | [1.018178, 1.075404] |

The native follow-up marginal medians remain 3.650 to 1.575 ms at 65,536 bits
and 38.035 to 9.073 ms at 262,144; WASM remains 8.085 to 3.845 and 82.280 to
31.120 ms. Ratios of these marginal medians are not the paired point estimates.
The selected derivative medians remain 155.317 to 22.623 microseconds. The
planner's independently counted 65,536-bit fresh requested bytes remain
32,693,112 to 1,628,872, with identical 8,256 live and 103,248 peak deltas.
Derivative requests remain 1,733 to 5 per query in the selected eight-query batch,
with identical zero live delta / 30,960 peak delta. These are requested allocation
metrics, not RSS, allocator retention or arbitrary history bounds.

The 4,096-bit fresh-WASM comparison retains point ratio 0.689714, but the corrected
bootstrap [0.668933, 1.256309] and order interval [0.656821, 1.662340] include one.
The pi-scaled degree-1/order-0 derivative retains point ratio 1.042047; corrected
[0.976620, 1.222695] and order [0.936624, 1.232058] also include one. Preserve these
point estimates without claiming either a demonstrated gain or no possible cost.
The original warm-pi and tiny coarsening controls remain recorded separately;
their later follow-up does not erase an unfavorable original campaign.

No reversal of these two retained decisions is indicated by this reanalysis:
the substantial intended-path benefits and allocation savings persist while
known small planner costs remain disclosed. That conclusion is scoped to these
recorded workloads, source versions and existing mathematical qualification.
It does not newly qualify every timing, feature, consumer, target, or all five
retained transfers. No production code was edited or reverted.

## Evidence, resource use and remaining work

Reanalysis passed 2026-09-11T00:59:50.331Z. Full recomputation, historical-output
rechecking and independent controls passed 01:01:56.864Z, both exit 0 / null
signal with empty stderr. The old checkers are run to reproduce and bind the
historical records, not to endorse their defective confidence calculations.
They also repeat 8,302 native planner and 600 exact-enclosure checks, 8,302 WASM
planner and 600 exact-enclosure checks, test-membership/size parsing, 426 planner
allocation rows, 480 derivative allocation rows, 96 endpoint allocation rows and
320 retention rows. Archived numerical, allocation and test evidence agrees.
These are offline checks of existing outputs, not new Rust suite, memory-test,
application or benchmark executions. Known nonzero reachable/live memory and
nine ignored curve tests remain disclosed.

The analysis is 353,998 workspace bytes. No new /tmp file, source-tree copy,
binary, benchmark, production/donor change, cleanup/deletion, commit or push.
The existing 1,415 complete / 20 partial donor files and 183,653 uniquely read
lines are unchanged. The final source/evidence capture is reported in the mutable
README and root ledger after it actually completes.

Next review the remaining retained exponential-proof/reuse, polynomial-facts
and monic campaigns, then the other inventoried historical experiments. The
isolated point candidate's runtime-attribution and final consumer/size gates,
separate power-sum work, supporting donor reads and the entire original exact-
real ecosystem inventory remain open. Fixing nine campaigns across checkpoints
60 and 61 is not a complete statistical-impact or ecosystem audit.
