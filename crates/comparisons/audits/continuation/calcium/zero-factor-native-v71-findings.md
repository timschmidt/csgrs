# Zero-factor native cost qualification — checkpoint 71

Status: isolated native cost qualification, **not retention**. Live Hyper and
the checkpoint-70 candidate are unchanged; six continuation transfers remain
retained. No new donor-line credit or full ecosystem completion is claimed.

## Scope and execution

The predeclared corpus combines forty earlier cost cases, all sixty-six degree/
sign/shared controls, and eight wide cases (65/257 construction heights, degree
3×3, distinct/unused-zero carriers, Add/Divide). Both policies and retained/fresh
source lifecycles give 114 cases / 456 groups. Fresh reconstructs rational source
objects from pre-parsed JSON inside each iteration; it is not a cold process.

Four identically configured release collectors share the frozen scalar/solver
trees and cache. Full dependency metadata matches after the two intended paths:
33 packages/nodes and identical lockfiles. Clippy and final harness formatting
pass. CPU binaries contain no allocation-instrumentation symbols. All 1,824
preflight full-report checks match checkpoint 70's independently checked records;
that mathematical checker is also rerun. Sources are checked unchanged after
measurement and full reports agree before/after every measured group.

The complete serial campaign runs 2026-09-11T19:30:58.448Z–19:32:34.424Z:

- 1,824 pilot observations, at one and 32 iterations, all retained.
- 21,888 CPU observations: 24 balanced process pairs, 456 groups each variant.
- 7,296 allocation observations: four separate balanced pairs, each group at
  one and sixteen iterations. Instrumented durations are not CPU evidence.

Common iteration counts, derived from the slower 32-iteration pilot, range from
3 to 20,000. Target duration is three milliseconds. No samples are discarded;
1,636 CPU observations are below one millisecond, including fast reduced paths.
Group order is randomized identically within each pair with deterministic
AES-CTR/rejection-sampled shuffles. Variant order is exactly balanced. All full
results, checksums, source identities, wall bounds and process gates validate.

Orchestration is pinned to CPU 0, measured processes to CPU 2 (SMT sibling 10),
on an AMD Ryzen 7 5800X3D. Host snapshots show the powersave governor, roughly
3.47 GHz sampled frequency and changing background load. These are not a
reserved CPU, fixed clock, continuous contention trace or proof of statistical
independence. No audit build or Memcheck overlaps the campaign.

## Results — candidate / baseline

Both corrected confidence methods use the paired observations: a tested
20,000-replicate bootstrap median interval and a separate binomial order-statistic
median interval. They remain conditional on sampling assumptions and are not
multiplicity-adjusted. Counts below require both intervals to be on the same
side of one; they are not independent discoveries across repeated policies,
lifecycles and related carriers.

| Workload | Groups | Paired median ratio range | Below / above one under both methods |
|---|---:|---:|---:|
| Same result, bounded deflation | 196 | 0.210–0.809 | 196 / 0 |
| Same result, nondivision bypass | 136 | 0.974–1.038 | 4 / 7 |
| Same result, nonzero constant bypass | 40 | 0.986–1.017 | 0 / 1 |
| Same result, zero-divisor guard | 20 | 0.996–1.074 | 0 / 11 |
| Same result, still over the degree cap | 16 | 1.004–1.027 | 0 / 11 |
| New answer, oversized source | 40 | 0.917–6.992 | 24 / 16 |
| New answer, bounded wide source | 8 | 0.512–0.645 | 8 / 0 |

The 48 new-answer groups are **not equal-work speedups or regressions**. They
compare producing certified answers with the old failure/rejection paths.

Representative STRICT/reused-source marginal medians and separate one-query
allocation medians:

| Case | Native time, baseline → candidate | Requests | Requested bytes | Peak above starting live bytes |
|---|---:|---:|---:|---:|
| Existing sqrt(2) / selected 1 of x(x−1) | 8.944 → 4.957 µs | 55 → 43 | 9,000 → 5,240 | 2,584 → 1,816 |
| Early literal-zero divisor guard | 100.2 → 107.7 ns | unchanged | unchanged | unchanged |
| Newly admitted degree-nine result | 6.574 → 44.508 µs | 26 → 153 | 2,128 → 30,249 | 1,096 → 6,664 |
| Shared x(x−1)^8 / itself, negative scale | 25.072 → 23.001 µs | 138 → 136 | 15,903 → 13,145 | 3,688 → 3,688 |
| Wide 257-height common-zero carrier | 288.411 → 147.232 µs | 1,893.5 → 1,384 | 475,687 → 286,553 | 11,528 → 12,072 |

Marginal medians need not divide to the paired median ratio. Fractional request
counts are medians across integer observations, not fractional allocator calls.
The bounded existing sqrt(2) example has paired ratio 0.5591, bootstrap interval
[0.5456, 0.5649], order-statistic interval [0.5431, 0.5668]. The early guard's
ratio is 1.0741 with [1.0611, 1.0791] / [1.0603, 1.0818]. Its roughly 7.5 ns
cost is disclosed, not assigned to an unmeasured code-layout mechanism.

## Allocation attribution and limits

All 196 same-result bounded-deflation groups reduce requests, requested bytes
and peak above starting live bytes at both batch sizes. Other unchanged groups
preserve peak/live deltas, but 28 have small positive mixed-workload request/
byte median differences: 0.5–1.5 requests and 52–192 bytes per query. Allocation
counts vary with preceding work; the mixed-workload observations stay intact.

A separate, post-campaign diagnostic runs each of the seven affected cases in
its own fresh process, four balanced pairs per case, both policies/lifecycles,
one/sixteen iterations. All 448 records pass full-report checks. All 224 paired
request/byte/peak/live comparisons agree exactly. This supports dependence on
preceding work/state rather than an unconditional per-call allocation charge;
it does not identify a particular cache mechanism or erase the mixed-workload
costs. The diagnostic finishes at 19:37:47.135Z and supplies no CPU claims.

The wide new-answer groups reduce cumulative requests/bytes but increase peak
by up to 544 bytes; newly admitted degree-nine groups add up to 5,568 peak bytes.
Live deltas are unchanged across all measured comparisons. Batch peak/live
figures are above a nonzero starting footprint, not total peak, RSS, retained
cache boundedness or a whole-application memory guarantee.

One valid and nine deliberately corrupted controls check rejection of changed
polynomials/endpoints, checksum/iteration/policy errors, zero duration, backward
clocks, false final equality and allocation instrumentation in CPU records.
All controls behave as intended. No failed execution capture occurs in this
checkpoint; earlier failures remain preserved in their original evidence.

## Evidence and next decision

Fresh full statistical/source/evidence recomputation passes at
2026-09-11T19:47:55.425Z. The build, CPU/allocation campaign, case-isolated probe,
controls and formatting are terminal. Four new executables total 11,304,616
bytes under /tmp/calcium-zero-cost.cEHSth. No new solver/scalar copy; shared-cache
reuse keeps the recorded /tmp availability at 15,031,971,840 bytes after the
campaign. No deletion, commit, push or live edit.

`node verify-zero-factor-cost-v71.mjs` verifies this checkpoint, including fresh
statistical recomputation. [Evidence assembly](results/zero-factor-cost-evidence-v71.json),
[final verification](results/zero-factor-cost-verify-v71.json).

The native benefits support continuing qualification, not immediate retention.
Next: execute the same mathematical operations in WASM with explicit host-boundary
and state semantics, then representative consumer regressions and stripped-size
gates. The separate power-sum trial, remaining donor sources and original full
reference inventory remain open. No universal throughput/memory improvement or
representative binary-size claim is made from these collectors.
