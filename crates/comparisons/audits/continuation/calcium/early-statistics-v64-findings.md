# Checkpoint 64 — early scalar, proof and polynomial statistics

The early log, erf, eager root/exponential, sign-proof and polynomial-decision
versions remain unselected. Their later cache/fact-aware retained replacements
are unchanged. This checkpoint corrects historical statistics; it does not
complete the ecosystem audit or supply missing binary/source provenance.

## Reconstructed evidence

Nine CPU datasets contain 15,984 measured rows / 333 comparisons. All paired
point estimates and marginal medians are preserved. Corrected intervals use the
checkpoint-60 tested rejection-sampled pseudorandom bootstrap (20,000 replicates)
and independent order-statistic median interval (ranks 3/10 of twelve blocks).
Both require suitable independent/common-distribution blocks, without multiple-
comparison adjustment. Old intervals are withdrawn, even though their exact
reproduction from the archived runner constants succeeds. Endpoint differences
also include the changed stream and increase from 5,000 replicates.

| Campaign | Comparisons | Old below / above / includes one | Corrected bootstrap | Order statistic |
| --- | ---: | ---: | ---: | ---: |
| Initial log, source-limited | 16 | 3 / 6 / 7 | 3 / 6 / 7 | 3 / 6 / 7 |
| Early log v2, source-limited | 18 | 6 / 5 / 7 | 6 / 4 / 8 | 6 / 4 / 8 |
| Expanded log v2 | 22 | 9 / 8 / 5 | 8 / 7 / 7 | 8 / 7 / 7 |
| Erf | 48 | 9 / 20 / 19 | 9 / 20 / 19 | 9 / 17 / 22 |
| Eager root/exponential | 72 | 24 / 25 / 23 | 24 / 23 / 25 | 24 / 22 / 26 |
| Sign-proof queries | 33 | 17 / 3 / 13 | 17 / 2 / 14 | 17 / 2 / 14 |
| Sign-proof numeric | 72 | 18 / 8 / 46 | 16 / 5 / 51 | 14 / 3 / 55 |
| Opaque sign-proof work | 16 | 7 / 8 / 1 | 7 / 8 / 1 | 7 / 8 / 1 |
| Polynomial, including excluded first group | 36 | 2 / 21 / 13 | 1 / 19 / 16 | 1 / 19 / 16 |

The 298 conditionally usable historical comparisons contain 284 changed interval
endpoint pairs and thirteen formerly directional claims now including one. Their
corrected below/above/includes counts are 82/84/132; order counts 80/78/140.
Thirty-four earlier log comparisons remain separately archival/source-limited:
thirty endpoint pairs change, one directional claim becomes inconclusive.
The first polynomial group (48 rows) remains rejected for formatting overlap;
its inclusion-of-one result does not rehabilitate the observation. No comparison
gains or reverses direction. Including one is not proof of equivalence.

Among the 298 historical comparisons, 192 use separately checked numerical
contracts, 59 match recorded decision outcomes, and 47 add certified answers
where baseline reports Unknown. Matching Known/Unknown or degree alone does not
prove full representation equality. Additional-answer timings are different
work, not equivalent successful-work speedups.

Six allocation campaigns add 3,156 rechecked rows / 263 groups. Their requests
and cumulative requested bytes per query are preserved; these are not peak
memory, RSS or whole-process retention. All 263 three-block instrumented timing
intervals remain withdrawn, with no replacement CPU inference. There was no
polynomial allocation campaign and none is invented.

## What survives the correction

- Expanded log's warm unresolved transcendental control remains expensive:
  paired ratio 5.057751, bootstrap [4.548175,5.211756], order
  [4.530316,5.267919]. Marginal medians are 150.917→739.663 ns/query;
  requests 1→16 and requested bytes 768→2,936. Repeated failed proof work still
  needs scheduling/reuse; generic log v2 remains isolated.
- Erf's fresh one-third case is 1.083210 times baseline, bootstrap
  [1.045807,1.112785], order [1.016698,1.114268]; requested bytes rise
  28,544→30,584. More importantly, the preserved four serialized-sign-fact
  failures remain a completeness failure. Timing correction cannot repair that.
- Eager sqrt(exp(sqrt(2))) improves fresh evaluation to paired ratio 0.428908,
  bootstrap [0.422639,0.439432], but loses hot-operand cache reuse: ratio
  4.545358, bootstrap [4.419047,4.742386], order [4.413943,4.769956].
  Hot-operand bytes rise 1,016→4,104/query. This still favors separating
  semantic relation proofs from the numeric evaluation graph, as in the later
  retained cache-preserving implementation, not retaining this eager rewrite.
- The early sign proof's depth-128 unresolved warm-pair control remains
  4.327541 times baseline, bootstrap [4.220225,4.687004], order
  [4.200565,4.852844]. Requested bytes rise 1,768→11,220/query. This is the
  unresolved-work problem addressed by the later bounded reuse experiment,
  not evidence that a collector's visit cap bounds every opaque descendant.
- Polynomial v1's degree-16 retained log-self result adds an answer but costs
  paired ratio 23.072422, bootstrap [22.529624,23.515604]. Baseline Unknown
  takes 2.918 microseconds versus 67.849 for a certified answer. This is not an
  equal-work regression; the later fact-first version avoids much repeated work
  while preserving the added answer and remains the retained implementation.

Marginal-time ratios and median paired-block ratios are different estimators;
both are preserved and must not be interchanged. None of these results establishes
a universal workload effect, whole-program resource bound, or new retention.

## Provenance, controls and open work

The nine remaining known sampler scripts were read completely (880 lines),
plus four supporting evidence verifiers (462 lines). Five source/evidence
manifests recheck 5/123/126/40/74 entries, the frozen 176-file initial Hyperreal
baseline and the two polynomial CPU executables recheck, and the previous
checkpoint's 95 artifacts / 956 current live identities remain unchanged.
These tooling reads add no donor coverage.

Most early runs did not preserve pilot observations; all lack per-observation
timestamps. Calibration is independently reconstructed only for the polynomial
campaign, which preserved its two pilots. Other runs check recorded iteration
bounds and fixed allocation counts, not fabricated pilot evidence. The
polynomial first-group exclusion is reproduced using the formatting completion
time and a conservative elapsed-time lower bound before all subsequent groups.

The first two log datasets contain eight/nine cases rather than the surviving
expanded runner's eleven. Their older source versions are not reconstructed
from today's file. A captured point-in-time check of eighteen recorded CPU
executable paths finds sixteen no longer match their old hashes; only the two
separately frozen polynomial binaries match. Reused build paths were overwritten
by subsequent work. This is not a search for every possible alternate copy,
nor a claim that today's executable qualifies an old timing. The root-eager CPU
summary/raw rows survive, but no separate root-exp-paired-cpu command capture was
found in this checkpoint's results directory. Existing bound metadata is not
silently strengthened into that missing evidence.

Historical fourteen-checkpoint verification passes 2026-09-11T02:12:49.192Z.
Reanalysis passes 02:17:18.331Z. Complete recomputation and 27 rejection controls
pass 02:20:00.240Z. All exit 0 / null signal with empty stderr; the fourteen
historical records exactly match the corresponding prefix of checkpoint 63's
rank chain, then precede the new checked result. Tests reject invented outcomes,
pilots and timestamps, bad pairing/counts/calibration, and allocation-clock
inference. The pathological four-block sample reproduces the defective [0,0]
interval while the corrected sampler gives [0,1]. Historical reproduction is
integrity evidence, not validation of the withdrawn estimator.

These are offline checks, not fresh Rust, native numerical-backend, Memcheck,
consumer, size or benchmark runs. The historical verifier rewrites an identical
existing qualification-summary.json; its contents remain unchanged. Preserved
donor numerical failures and qualified thread-memory results remain separate
from successful artifact verification. No all-green history is claimed.

The new analysis is 990,238 workspace bytes. No new /tmp file, build, binary,
source-tree copy, production/donor edit, retained transfer, deletion, cleanup,
commit or push was needed. Capacity records 16,237,924,352 available /tmp bytes.
Donor coverage remains 1,415 complete files, 20 partial files and 183,653 uniquely
counted read lines in the Calcium/FLINT continuation.

Reproduce the bound evidence with:

```sh
node verify-early-statistics-v64.mjs
```

All 45 original known-sampler file matches now map to explicitly addressed
dataset scopes across checkpoints 60–64. This is not exhaustive statistical
closure: twenty additional top-level text matches need semantic classification,
and the signature/inventory never covered the whole workspace or ecosystem.
Point-candidate runtime/consumer/size work, power sums, supporting donor files,
remaining references and full inventory reconciliation remain open.
