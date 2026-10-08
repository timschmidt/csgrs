# Checkpoint 65 — residual estimator review and focused WASM diagnostic

Status: progress, not completion of the original ecosystem audit. Five retained
Calcium/FLINT transfers remain unchanged; the point-witness repair remains isolated.

## Residual statistical matches

All twenty additional matches in the frozen checkpoint-63 top-level inventory
were read completely: 1,371 lines. None defines another estimator. The per-file
SHA-256, full read range, matched line numbers and semantic finding are in
[statistical-matches-v65.json](statistical-matches-v65.json). They are wrappers
around already corrected helpers, classifiers of existing intervals, deterministic
correctness fixture loops, prose, or the scanner itself. The WASM wrapper's
summarizer is a re-export of the demand-cost helper corrected in checkpoint 60.
The complex-v2 snapshot is checked against its original with only its recorded
imports/source-read paths rebound.

Together with the 45 known matches mapped in 60–64, this resolves the 65 matched
files in that frozen 370-file inventory. It does not semantically clear the 305
nonmatches, subdirectories, unrelated tools, or the workspace. Historical bound
scripts were not edited or rerun to overwrite old results; correction overlays
remain authoritative only for their stated datasets. No donor-read credit.

## Predeclared diagnostic and preserved initial failure

Reuse the three frozen checkpoint-59 import-free WASM modules (4,843,385 bytes
total); no Rust compilation, source-tree copy or new /tmp artifact. Eight
previously observed groups were selected before replay: noisy unchanged-path
controls, the known Unknown-result benefit and three recovered-point retry cases.
They are diagnostic selections, not a representative or independent holdout.

Four fresh Node 22.22.2 processes use default/single-threaded/single-threaded/
default GC in that order, CPU 6, the same optimizing-WASM flags, 36 balanced
six-observation blocks and unchanged checkpoint-59 batch lengths. Each observation
uses a fresh instance, nine preconditioning queries, the original history_batch,
and a held final result. The final full report is checked against the frozen
native record; intermediate iterations only have the original checksum.

The first attempt stopped on its first qualification row, before any measured
batch: a 71,495 ns query returned zero thread CPU time. Requiring strictly
positive CPU counters was invalid. Original scripts, plan, one full row, 128
empty controls, failure record and exit-one capture remain preserved.
The amended plan was recorded at 2026-09-11T02:47:32.433Z, before replay. Zero
CPU readings are legal and never dropped. A zero paired block median makes the
entire group/reference/CPU-metric inference unavailable, rather than dividing by
zero or excluding a slow/short observation. There were no zero measured CPU
counters in the completed passes; 164 of 192 short qualification rows returned
zero thread CPU time.

All four completed captures pass between 02:48:03.978 and 02:49:12.089 UTC:
6,912 measured observations, 192 qualification observations, 1,024 empty-envelope
controls, zero failure-log bytes, and every full final record matching its native
reference. Eager and demand agree in all groups. Five groups match baseline;
three recover an answer baseline cannot certify, so those baseline comparisons
are different work. No fresh independent polynomial oracle was executed here.

## Timing results and limits

Each pass is analysed separately with 20,000 corrected bootstrap replicates and
an independently checked binomial median interval. For 36 blocks, the latter
uses ranks 12 and 25, coverage 66,739,206,840 / 68,719,476,736. There are 192
group/reference/channel comparisons, not 192 independent experiments. No
multiplicity adjustment, outlier removal, overhead subtraction, or pass pooling.

Selected demand/eager paired wall-time ratios (below one is faster):

| Case/policy/history/lifecycle | Default 0 | Single 1 | Single 2 | Default 3 |
| --- | ---: | ---: | ---: | ---: |
| 17/0/2/fresh, unchanged result | 0.9900 | 1.0057 | 0.9937 | 1.0039 |
| 37/1/0/retained, Unknown control | 0.7456 | 0.7466 | 0.7599 | 0.7553 |
| 3/1/3/retained, recovered point | 1.0191 | 0.9945 | 1.0082 | 1.0067 |
| 4/0/2/retained, recovered point | 0.9994 | 0.9997 | 1.0147 | 0.9895 |
| 42/1/2/retained, recovered point | 1.0280 | 1.0188 | 1.0115 | 1.0110 |

The Unknown-control benefit survives both interval methods in every pass: about
24–25% faster than eager. Its four bootstrap intervals are [0.739809,0.750694],
[0.743154,0.757023], [0.754719,0.765281], [0.743143,0.763300]. The previously noisy
case 17 baseline slowdown is not reproduced: all four new wall intervals include
one. This is not an equivalence proof or a revision to old point estimates.

Retry costs are not uniformly absent. Case 42 has four above-one bootstrap
intervals, [1.025208,1.037293], [1.012079,1.028940], [1.004879,1.017288],
[1.001484,1.014813]; the last order-statistic interval includes one. Cases 3 and
4 are above one under both methods only in the second single-threaded pass.
Case 4 does about 3.70–3.75 times baseline wall work, but baseline produces an
invalid-evidence result instead of the certified answer: not a like-for-like
regression. Every result, including inconsistent small effects, remains available.

The CPU counters are *envelopes*, not batch-only component timings. Thread CPU
is read outside the inner wall clock; process CPU encloses both and includes
helper threads; resource counters enclose all three. Empty-envelope wall medians
are 110/130/120/115 ns; process medians are 3,000 ns in every pass, with maxima
115–181 microseconds. Thread CPU is zero for 1,023 of 1,024 empty controls.
Thus thread CPU is not a fine-resolution oracle for short queries. No empty
control is subtracted, and a process-minus-thread difference is not labelled
as exact GC time. Measured batch minima were about 1.03–1.06 ms.

With default GC, median wall/thread ratios immediately after the explicit GC
boundary are 2.23 and 2.36; with single-threaded GC they are 1.19 and 1.18.
Voluntary context-switch totals over measured envelopes are 804/13/14/818;
involuntary totals are 434/64/73/471. The boundary-associated tail is much smaller
in the single-threaded passes. This is consistent with runtime scheduling/GC
contributing to the earlier noise, but fixed process order, timer granularity,
other runtime work, workload positions and uncontrolled host state preclude a
clean causal attribution or a transferable GC-setting recommendation.

## Verification and next work

Analysis passes at 02:50:29.677 UTC; a separate full recomputation passes at
02:52:18.380 UTC. Twenty-seven mutation controls reject corrupted membership,
order, metadata, clocks, counters, checksums, memory bounds and full values.
Legal-zero preservation, unavailable zero-denominator inference, a constant
ratio and independent Pascal-convolution coverage checks pass. The corrected
checkpoint-60 sampler's independent controls are rerun as well.

The four completed raw datasets occupy 30,365,957 workspace bytes; analysis is
1,603,578 bytes. Controls, captures and preserved failure files are additional.
No production/donor edit, retention, allocation measurement, consumer test,
application-size measurement, cleanup, deletion, commit or push. Donor coverage
remains 1,415 complete files / 20 partial / 183,653 unique read lines.

Proceed to final selected-demand consumer/size qualification and the retention
decision, keeping retry costs visible. Do not extend this diagnostic indefinitely
in search of a universal timing verdict. Power sums, remaining donor/reference
reads and full inventory reconciliation still belong to the original objective.
