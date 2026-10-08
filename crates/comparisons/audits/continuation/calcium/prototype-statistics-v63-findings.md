# Checkpoint 63 — complex products, rank and sign-filter statistics

The five prototypes remain isolated and unselected. Correcting the audit's
resampling defect does not justify retaining them. This is progress in the
original ecosystem audit, not completion of it or of historical statistics.

## Scope and method

All five campaigns were reconstructed from 25,728 measured CPU rows and their
recorded pilots: 536 groups, twelve alternating ABBA/BAAB blocks per group.
Every descriptor, iteration calibration, variant/block order, timestamp sequence,
recorded outcome, paired point estimate and marginal median is checked. The
3,216 separate allocation rows are reconstructed as deterministic request,
requested-byte, live-delta and peak-delta measurements, not CPU inference.

The checkpoint-60 tested rejection-sampled pseudorandom bootstrap uses 20,000
replicates. Independent order-statistic intervals use ranks 3 and 10 of twelve
block ratios, with conditional coverage 3,938/4,096. Both methods assume suitable
independent/common-distribution blocks; neither adjusts for multiple comparisons
or establishes algorithm-level attribution. Old intervals are preserved solely
as withdrawn historical evidence. Endpoint differences also reflect the change
from 5,000 replicates and the changed stream, not only removal of index bias.

| Campaign | Groups / CPU rows | Old below / above / includes one | Corrected bootstrap | Order statistic |
| --- | ---: | ---: | ---: | ---: |
| Complex product v1 | 192 / 9,216 | 64 / 43 / 85 | 61 / 35 / 96 | 56 / 28 / 108 |
| Complex product v2 | 192 / 9,216 | 66 / 17 / 109 | 60 / 14 / 118 | 55 / 8 / 129 |
| Rank witness | 56 / 2,688 | 0 / 43 / 13 | 0 / 40 / 16 | 0 / 40 / 16 |
| Sign summary | 48 / 2,304 | 5 / 11 / 32 | 1 / 7 / 40 | 1 / 4 / 43 |
| Sign mask | 48 / 2,304 | 2 / 7 / 39 | 1 / 5 / 42 | 1 / 5 / 42 |

All point estimates survive exactly. Of 536 interval endpoint pairs, 486 change;
34 formerly directional bootstrap comparisons now include one. None changes
from inconclusive to directional or reverses direction. Including one is not
proof of equivalence or absence of an effect.

## Retention implications, in the requested priority order

Complex products add no exactness or completeness capability. Their independent
GMP qualification remains 6,912 fixtures / 82,944 components / 27,648 input
checks per variant; 5,832 trace records per variant recheck. Timing rows retain
their exact num-rational preflight flag, not full result values. The separate
numeric evidence must not be confused with full value equality in each timing
record. Historical source bindings use the archived pre-e-planner snapshot;
the archived checkers differ only in that binding import, verified bytewise.

V2's historical candidate-reaching classification contains 60 groups: 55 have
corrected bootstrap intervals below one, 52 under the order method, none above.
But its 132 bypass/reused controls still contain 14 above-one bootstrap intervals,
eight under both methods. Requested bytes fall in 60 groups, yet peak demand
rises in 33. V1 has 66 higher-peak groups. Linked benchmark-size costs and
unqualified broader consumers remain as previously recorded. The wide-input
three-product idea is still potentially useful for a demonstrated workload;
these global dispatch implementations remain unjustified.

Rank's sixteen newly answered groups are different work from baseline Unknown.
The other forty groups match recorded Known/Unknown status, not necessarily full
result representation or downstream proof availability. Twenty-four of those
forty still have above-one intervals under both methods. The unresolved width-32
retained control has paired ratio 48.006656, corrected bootstrap
[41.739881,51.477548], order interval [38.567035,52.934646]. Marginal query medians
are 23.261 versus 989.488 microseconds; their ratio is not the paired estimator.
Requested bytes per query rise from 47,320 to 1,141,240. Across the allocation
corpus, 32 groups have higher requests/bytes and 24 have higher peak demand.
The witness idea remains open for better scheduling/reuse, not retained as v1.

Both sign filters preserve the complete recorded PredicateOutcome, including
sign, certainty and stage. Re-executing the exact BigInt shoelace/Machin oracle
reproduces all 24 original signs byte-for-byte. Historical debug/release tests
remain 364 per variant/profile; exhaustive/long/trace evidence and 45 public
trace records recheck. These are not newly executed Rust regressions.

Each sign variant saves one allocation and twice the vertex count in requested
bytes per query in the 24 refinement-reaching groups. Their allocation summaries
match each other exactly; peak demand is lower in twelve groups and live deltas
are zero throughout. Nevertheless, the summary retains seven above-one bootstrap
intervals (four under both methods), the mask five (all under both). For example,
the retained four-vertex rational mask control is 1.101125 times baseline, with
bootstrap [1.086904,1.127928] and order [1.085429,1.128336]. Its smaller benchmark
executables (CPU -1,176 bytes, allocation -1,264) are not representative
application-size results and do not outrank runtime under the requested priorities.
The summary benchmark executables grow 2,536/2,552 bytes. Neither implementation
adds completeness or exactness to justify these measured costs.

## Verification, provenance and storage

Twelve statistical runners/checkers and archived copies were read completely
(1,034 lines), plus seven source-binding/oracle helpers (228 lines). These are
audit-tooling reads, not new donor coverage. Five historical manifests recheck
71/68/41/67/41 bound artifacts. Source identity checks include all 956 current
live files, 175 point-candidate files, both 955-file complex source pairs, both
956-file sign source pairs, and the 774 recorded rank-candidate source entries.

The historical 23-checkpoint rank chain passes at 2026-09-11T01:46:49.369Z.
Corrected reanalysis passes at 01:59:05.450Z. Full corrected recomputation, four
archived complex/sign checkers and thirty deliberate corruption controls pass
at 02:02:00.860Z. All three exit 0 with null signal and empty stderr. The check
preserves all 23 historical output records before its new result. Historical
bootstrap reproduction checks artifact integrity, not estimator validity.

The broader frozen top-level .mjs inventory covers 370 files, with 65 text
matches: all 45 known LCG matches and twenty other potential matches. It is not
semantic classification or a scan of subdirectories/the whole workspace. The
initial inventory incorrectly excluded fourteen version-suffixed legacy scripts,
including two known statistical checkers. Its snapshot remains preserved as
incomplete. A first correction failed an overly narrow added-membership assertion;
the failed source copy and capture remain. Two sandbox captures exited 0 but
produced no stdout and are not qualification. Separately captured read-only
inventory validation and oracle rerun have the required complete output.
Six qualified gates and four incomplete/failed/output-missing captures are
explicitly distinguished, not relabelled as an all-green history.

The new analysis occupies 1,638,145 workspace bytes. No new /tmp file, binary,
build target, source-tree copy, production/donor edit, retained transfer, cleanup,
deletion, commit or push was needed. Capacity records 16,237,924,352 bytes
available on /tmp. Current donor coverage is unchanged: 1,415 complete files,
20 partial files, 183,653 uniquely counted read lines in the Calcium/FLINT
continuation. No new backend, Memcheck, consumer or size measurements were run.

Reproduce the bound evidence and corrected analysis with:

```sh
node verify-prototype-statistics-v63.mjs
```

Nine known sampler matches remain outside corrected campaign scopes: log, erf,
early root-exp/opaque/sign, polynomial-decision and their relevant checkers.
The twenty other potential text matches also require semantic classification.
The isolated point repair still needs runtime attribution and final consumer/
size/retention work; power sums, supporting donor files and the entire original
reference inventory remain open. No complete historical or ecosystem audit
claim follows from this checkpoint.
