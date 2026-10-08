# Matched zero-factor WASM costs — checkpoint 73

Outcome: the isolated zero-factor candidate's intended unchanged-result path
is materially faster in both tested WASM instance lifetimes. All 196 bounded-
deflation groups per mode have both corrected intervals below baseline. Small
bypass costs and the extra work of newly available answers remain disclosed.
The candidate is still isolated: consumer/size gates precede any retention.
The full ecosystem audit and its remaining reference inventory are not complete.

## Protocol and complete collection

The predeclared protocol reuses the exact checkpoint-72 rational WASM modules,
without rebuilding, source copying or live edits. The 114 cases span the existing
40 cost controls, all 66 degree/sign/shared-carrier controls, and eight wider
coefficient cases. Both policies and source lifecycles yield 456 groups. Each is
measured in a persistent-instance worker and a fresh-instance-per-group worker.
Fresh instances receive preparation and warmup; this is not cold-start latency.

All 104 workers terminate successfully between 2026-09-11T21:14:13.958Z and
21:21:26.803Z. Preserve every one of the 3,648 pilot and 43,776 measured rows.
Pilot batches one/32 determine common baseline/candidate counts, targeting 3 ms
from the slower 32-query pilot. Counts remain frozen: persistent 1–13,715, fresh
1–12,940. All 3,604 measured sub-millisecond batches remain included, as do slow
observations. No post-measurement tuning or trimming.

Each instance mode has 24 paired blocks, twelve AB and twelve BA, with identical
shuffled group order within a pair. Which mode's pair executes first is also
balanced. The corrected rejection-sampled generator fixes every schedule. The
host times one batch export; all reports are destroyed inside the batch. Input
parsing, preparation/eight warmups, GC, output handling and full native-report
checks are outside timing. Fresh sources are reconstructed inside each query.
Full pre/post reports, signed coefficients, source wires and checksums remain
checked. A checksum does not independently validate each discarded batch report.

Node v22.22.2 / V8 12.4.254.21-node.39 uses the frozen no-Liftoff, no-tier-up,
no-lazy-compilation flags and explicit GC after each eighth observation. Workers
are on CPU 2, orchestration on CPU 0. Host: Ryzen 7 5800X3D, SMT sibling CPU 10,
powersave governor, sampled frequency about 3.46–3.50 GHz; sampled load rises
from 1.28 to 3.49. Snapshots do not establish an idle/reserved CPU or independent
samples. No measurement overlaps this checkpoint's statistical/control replay.

## Paired results

Ratios below are candidate/baseline paired medians. Direction requires both the
20,000-resample corrected bootstrap and the independent binomial order-statistic
median interval to lie on the same side of one. Every individual interval, all
24 paired ratios and marginal ranges are saved. These are conditional intervals,
not multiplicity-adjusted guarantees. Inconclusive does not mean equivalent.

| Instance mode / unchanged-result category | Groups | Below / above / inconclusive | Paired-median range |
| --- | ---: | ---: | ---: |
| Persistent / bounded deflation | 196 | 196 / 0 / 0 | 0.220–0.788 |
| Fresh / bounded deflation | 196 | 196 / 0 / 0 | 0.216–0.790 |
| Persistent / nondivision bypass | 136 | 0 / 0 / 136 | 0.965–1.037 |
| Fresh / nondivision bypass | 136 | 3 / 1 / 132 | 0.975–1.026 |
| Persistent / nonzero-constant bypass | 40 | 0 / 4 / 36 | 0.981–1.032 |
| Fresh / nonzero-constant bypass | 40 | 1 / 1 / 38 | 0.980–1.020 |
| Persistent / zero-divisor guard | 20 | 0 / 0 / 20 | 0.967–1.004 |
| Fresh / zero-divisor guard | 20 | 2 / 0 / 18 | 0.972–1.016 |
| Persistent / oversized, still rejected | 16 | 0 / 0 / 16 | 0.999–1.019 |
| Fresh / oversized, still rejected | 16 | 0 / 0 / 16 | 0.992–1.022 |

There are 408 unchanged-result and 48 new-answer groups per mode. For the selected
existing quotient (case 23, STRICT, retained sources), persistent marginal medians
fall from 13.209 to 7.407 microseconds; paired ratio 0.565713, bootstrap
[0.535267,0.584448], order interval [0.534107,0.584676]. Fresh-instance medians
fall from 12.630 to 7.245 microseconds; paired ratio 0.578540, intervals
[0.573204,0.585507] / [0.570533,0.589745]. A paired median is not necessarily the
ratio of the two marginal medians.

Six unchanged-result bypass combinations have both intervals above one. The
largest paired ratio is persistent case 15 / STRICT / retained sources:
101.934 to 103.828 microseconds, paired ratio 1.032490, intervals
[1.008850,1.043301] / [1.008040,1.045049]. Another nondivision control is about
2.56% slower. These observations remain in the evidence; no unmeasured code-layout
or cache explanation, universal absence of regressions, or causal attribution
is asserted. The selected early zero guard is inconclusive in both modes.

New-answer groups compare a complete result with a previous unsuccessful attempt,
not equal work. All eight bounded groups per mode have lower elapsed cost:
ratios 0.453–0.659 persistent, 0.463–0.720 fresh. Among forty oversized groups,
persistent has 19 below / 16 above / five inconclusive, fresh 16 / 16 / eight;
ratios span 0.929–4.430 and 0.943–4.385 respectively. A degree-nine answer
(case 41, STRICT, retained sources) costs 75.004 versus 17.024 microseconds
persistent, or 71.211 versus 16.465 microseconds fresh. That extra work provides
the formerly rejected answer. The shared-carrier and wide-coefficient examples
can instead finish more quickly than the old unsuccessful construction.

## Controls, memory and storage

The independent checker validates every worker command, module/input/plan hash,
complete report, timestamp window, iteration formula, balanced order and terminal
record before inference. Its initial full recomputation passes at
2026-09-11T21:22:24.510Z; a second independent recomputation and evidence assembly
passes at 21:23:36.794Z, both code 0 / null signal with empty stderr.
Thirty-six deliberately corrupted records and fourteen
ordering/terminal corruptions are rejected; four positive controls pass. The
stream controls use small authored two-row excerpts with explicit adjusted
terminal counts, not fictitious separately executed benchmark campaigns.

Memory samples include all pilots/measurements. Persistent linear-memory
capacity spans 1,703,936–1,835,008 bytes in both variants; fresh instances stay at
1,703,936 bytes. These are capacities, not allocation counts, peak/live heap or
RSS, and do not prove equal memory costs. Native checkpoint-71 allocator evidence
remains separate; there is no new WASM allocation claim.

No new binaries or copied source trees. Raw worker records occupy 49,537,622
workspace bytes; plans, summaries and compact corruption fixtures are additional.
The existing modules remain in /tmp. Recorded initial /tmp availability is
15,003,901,952 bytes, post-campaign 15,003,926,528; other temporary activity is
not attributed to this audit. No deletion, commit or push.

## Disposition

The measured intended-path benefit and previously qualified completeness gains
justify continuing to the consumer and representative stripped-size gates.
Keep the six existing retained transfers unchanged until those gates pass and
the exact candidate is bound to a live integration. The separate power-sum
candidate, remaining donor/reference reads and full inventory reconciliation stay
open. Re-reading the candidate diff/seven tests and the earlier consumer harness
adds no new donor coverage credit.

Run `node verify-zero-factor-wasm-cost-v73.mjs` to freshly recompute every paired
statistic and replay the saved controls/source bindings. The --record path only
binds the already-captured evidence and explicitly awaits this full replay.
