# Checkpoint 59 — matched extended WASM costs

The complete matched WASM campaign and offline recomputation pass. Full final
results remain unchanged from qualification. Performance is mixed and some
paired estimates are volatile; do not treat this as a uniform speedup, retention
decision or whole-audit completion. No algorithm or live production/donor source
has changed.

## What is compared

Retained baseline (`e-plan-qualified-candidate`), eager strict point-witness
recovery (`point-image-qualified-candidate`) and the unchanged demand-gated
solver (`point-demand-candidate/hypersolve`, using retained dependencies).
The 175-file demand binding remains
`d5472beb99abd3f0526d978f4e27fb0446a1ac425dfd52fe3265bdccca4ba80f`.
All 956 retained live identities and prior checkpoint 58 artifacts were
revalidated before this work. Donor coverage remains unchanged.

`point-wasm-work.rs` is exactly the `HistoryWork` struct and implementation from
the native checkpoint 56/57 workload, mechanically extracted without changing
its operations. Source verification compares the complete extracted prefix.
The WASM wrapper provides separate prepare/batch/finish exports, with no imports.

## Workload and measurement boundary

All 768 groups comprise 48 authored cases, two policies, four initial histories
and retained/fresh input lifecycles. These are the native cost groups, not the
extra repeated-serialization lifecycle introduced only for cold semantics in 58.
History names retain their existing meaning: constructed, -64 refinement,
-4096 refinement, or JSON-round-tripped input values.

Each observation uses a new WASM instance. Setup constructs reference roots and
performs nine preconditioning queries outside the clock. Retained mode uses
those inputs during the batch; fresh mode constructs and applies the selected
history on new roots for each query while keeping the initial reference roots
alive, exactly as the native workload does. This is not the fresh-input-only
memory-lifetime protocol from checkpoint 58. Constants may be reused within an
instance; this campaign does not claim absolute cold caches.

The host takes `process.hrtime.bigint()` immediately around one `history_batch`
call. The call executes the same Rust batch as native, including drops of all
but the final report. The final report stays alive after timing, and full JSON
serialization, reference comparison and input-record checks happen afterward.
The host call boundary and uncontended wrapper state bookkeeping are timed;
module compilation, instance construction, setup, final-result drop, JSON,
validation and explicitly requested host GC are not. Intermediate queries
carry a polynomial-length checksum; every final record is fully preserved.

Runtime is Node v22.22.2 / V8 12.4.254.21-node.39 with
`--expose-gc --no-liftoff --no-wasm-tier-up --no-wasm-lazy-compilation`.
These available local-runtime flags select the optimizing compiler and complete
module compilation before collection, avoiding a changing tier within samples.
This does not measure default-tier startup, arbitrary browsers or whole apps.
The flags control WASM compilation, not all host JavaScript optimization or
runtime scheduling; those remain part of the environment and boundary noise.
The entire timing runner is pinned to core 6; frequency and external load are
not controlled. Environment snapshots disclose governor and SMT sibling state.

One compiled module per variant is reused, but every observation gets fresh
linear memory. Explicit host GC follows every eight completed observations;
old instances are not retained by the driver. Recorded linear-memory sizes
are coarse instance/harness samples, not per-query requested bytes, peak live
allocation, RSS or lifetime-bound evidence. There is no allocation counter in
this WASM timing wrapper.

## Qualification before timing

All 768 groups × three variants × batch sizes one/eight produce 4,608 complete
qualification observations. They pass **129,856 independent exact checks** and
match the qualified native records, including full input and result values.
This uses the exact field/identity interpreter from 55, not Hyper comparisons,
donor arithmetic or floating overlap. Unsupported nodes fail closed. Native
and WASM repeated histories remain explicit; qualification timing fields are
not substituted for the matched campaign.

Qualification totals across variants: 3,280 Transformed, 512 baseline-only
InvalidTransformedEvidence, 288 denominator guards, 72 Undecided, 72 nonisolating,
and 384 invalid-input-evidence records. These count repeated batch sizes,
policies and histories, not independent new mathematical problems or defects.

Build completes 2026-09-10T23:31:33.615Z; all three WASM release and Clippy gates
pass warning-free. Qualification completes 23:38:14.937Z, exit 0. Its separate
recomputed offline check passes 23:48:08.455Z before any campaign measurements.

## Matched statistical protocol

Each group gets three 16-iteration pilots. The slowest observed per-query pilot
sets one common iteration count, targeting a 4 ms batch and clamped to 4–4096.
Then twelve six-observation palindromic blocks cover every permutation and its
reverse twice: **55,296 CPU observations and 2,304 pilots** across all groups.
Within each block, compare the median of the two demand observations with the
median of the two reference observations. Report the median of twelve paired
ratios and deterministic 5,000-resample bootstrap intervals. These intervals
are individual and not adjusted for 768 comparisons; no outlier is discarded.
Paired median ratios need not equal ratios of marginal query-time medians.

Demand/baseline has 512 equal-full-result groups and 256 groups that return
additional certified answers. Demand/eager has 768 equal-full-result groups.
Keep these strata separate; newly successful queries are different work, not
equal-work speedups. Do not divide native and WASM marginal times from different
campaigns to make a cross-platform performance claim.

## Results and remaining decision

The campaign completes 2026-09-10T23:57:48.384Z, exit 0. The inner timing runner
starts 23:48:08.761Z and finishes before the 23:57:47.943Z environment snapshot.
The offline checker passes 23:59:16.257Z, exit 0 / empty stderr. All 55,296
observations and 2,304 pilots match their full reference records. Common
calibration, exact group/sample order, time boundaries, checksums and every
statistical summary are recomputed; no sample is removed. Formatting also passes.

| Demand compared with | Full-result groups | Paired median ratio range | Individual CI below / above 1 |
| --- | ---: | --- | ---: |
| Baseline, equal result | 512 | 0.801026–1.335991 | 165 / 34 |
| Baseline, newly successful | 256 | 0.912081–3.960350 | 8 / 163 |
| Eager, equal result | 768 | 0.752653–1.234868 | 159 / 33 |

Ratios below one favor demand; these are not multiplicity-adjusted or proof
that every group has a stable algorithm-level change. Relevant examples:

- Retained case 37 / approximate / constructed (Unknown-width control): eager
  marginal median **32.331 microseconds**, demand **24.882 microseconds**,
  baseline 25.104 microseconds. Demand/eager paired median ratio **0.767287**,
  individual 95% CI **[0.736961, 0.782960]**. This supports a workload-specific
  benefit from avoiding eager strict witness work, consistent with native 57.
- The numerically fastest demand/eager group, case 37 / approximate / round-trip
  retained, has ratio 0.752653 but CI [0.685107, 1.046657]. Do not replace that
  inconclusive interval with the stronger constructed-control result above.
- Retained case 3 / approximate / round-trip (a recovered point): eager
  **15.388 microseconds**, demand **15.883 microseconds**; paired ratio
  **1.081198**, CI **[1.027356, 1.195473]**. This is an equal-answer retry cost
  relative to eager; baseline fails to return the answer and is different work.
- Fresh case 17 / STRICT / deep refinement has a demand/baseline paired ratio
  **1.217574**, CI **[1.107369, 1.333005]**, but marginal medians go in the opposite
  direction: **121.079 microseconds baseline, 119.464 microseconds demand**.
  Twelve paired block ratios range 0.672431–1.487246. This discrepancy is in the
  preserved raw evidence, not a rounded-number typo or equivalent estimator.
  It warrants a diagnostic replay before attributing a 22% regression to code.
- The largest equal-result baseline ratio is case 44 / approximate / round-trip
  retained: **1.335991**, CI **[0.994250, 1.671217]**. Its marginal medians are
  10.113 and 10.137 microseconds, and every variant returns InvalidEvidence
  before point-image recovery. The wide interval and estimator disagreement
  prevent interpreting this as a proved 34% witness-recovery overhead.
- Largest newly successful baseline ratio: retained case 5 / approximate /
  round-trip **3.960350**, CI [3.716204, 5.437036]. Marginal medians are 12.922
  microseconds for baseline's failure and 47.853 for demand's certified answer.
  This records additional validation work, not an equal-work regression.

The Ryzen 7 5800X3D host uses the powersave governor, with SMT sibling 14 for
core 6. Load-average snapshots change from [0.90, 0.79, 0.97] to
[3.12, 3.04, 2.15]; these snapshots cannot explain individual interruptions or
establish causality. The paired bootstrap summary is faithfully recomputed but
does not remove environment/runtime variation or establish sample independence.
Do not turn the range extrema or individual interval counts into a blanket
performance-equivalence or speedup claim.

Three frozen modules in `/tmp/calcium-point-wasm.jPk2WY` occupy 4,843,385 bytes.
Existing source snapshots and Cargo cache are reused; no new source-tree copy,
cleanup or deletion. Raw qualification and cost records are stored in the
workspace: **222,399,157 bytes** (16,405,792 qualification, 197,783,696 measured,
8,209,669 pilot, zero-byte failure logs), excluding build/check summaries.
Capacity capture records **16,237,924,352 available /tmp bytes**. The three
dependency locks agree exactly after normalizing only the harness package name.
Wrapper/module sizes are not representative final application sizes.

Next make a bounded diagnostic replay of the volatile controls, including both
unfavorable and favorable cases, with wall/CPU-time and ordering/GC context.
Preserve this original campaign unchanged; do not cherry-pick a replacement
summary, silently discard its slower samples, or revise the algorithm on this
evidence alone. This is a timing-attribution question, not a value failure.

Final selected-version consumer/representative-size gates and an explicit
retention decision remain open, as do the separate power-sum optimization,
remaining source/support reads, all original references and full inventory
reconciliation. Five retained continuation transfers remain unchanged. This
checkpoint is not a whole-ecosystem completion claim.

`point-wasm-manifest.json` binds the selected sources, modules, full raw records,
qualification and cost evidence. Run `node verify-point-wasm.mjs`. This extends
the 48–58 evidence check and preserves earlier failures; it does not rerun the
full historical 47-chain, whose latest complete capture remains
2026-09-10T04:56:34.285Z. No new solver/consumer suite, allocation instrumentation,
Memcheck, representative-size or production-retention result is claimed here.
