# Zero-factor native cost protocol — checkpoint 71

Freeze this protocol and all harness/input sources before pilots. No production
edit, retention, new solver copy or deletion is planned in this campaign.

Inputs: forty existing public cost cases; all sixty-six authored degree/sign/
shared cases; eight wide cases selected before costs (height 65/257, degree 3×3,
distinct/unused-zero families, Add/Divide). These are 114 cases, each under
STRICT and APPROXIMATE_512 and two lifecycles: 456 groups. Retained reuses source
objects. Fresh constructs rational source objects from pre-parsed JSON in each
iteration; input-file parsing is outside timing. Both run in warmed processes;
neither means a cold global cache. Constructor cost is part of fresh cost.

Build identical CPU and allocation collectors against the unchanged retained
baseline and the checkpoint-70 final isolated candidate. Share frozen scalar
dependencies and cache; copy only four executables to an exclusive /tmp folder.
Use release compilation with two jobs and no incremental build. CPU collectors
must not contain the instrumented allocator. The allocation collector reuses the
existing qualified allocator hook without editing it. Preflight both collectors
and independently validate full numerical reports/source identities against the
input cases and signed-polynomial/interval oracle. Builds, lint and preflight
must be terminal before costs. No Memcheck or compilation overlaps measurement.

Pin all native measured processes to logical CPU 2 (allowed CPUs 0–15 at plan
time). Record host process summaries without command arguments, CPU model,
affinity, SMT sibling, governor/current frequency, system load and timestamps
before/after campaign. Do not alter frequency/governor or claim an idle/reserved
CPU. Background work and scheduling remain limitations.

Pilots: each CPU collector runs the full group matrix at 1 and 32 iterations,
eight untimed warmups per group. Preserve every raw pilot. For each group choose
one common iteration count for both variants:
clamp(ceil(3,000,000 ns / max(baseline32/32, candidate32/32)), 1, 20,000).
No ratio-dependent selection, adaptive stopping or dropping short samples.

CPU: twenty-four independent process pairs, exactly twelve baseline-first and
twelve candidate-first in predeclared shuffled order. A deterministic
AES-CTR/rejection-sampled Fisher–Yates permutation randomizes group order per
pair; both variants use the identical order/counts. Every group gets eight
untimed warmups and an untimed full report check, followed by its timed loop.
Each timed loop consumes the entire report through black_box; a polynomial-
length checksum checks iteration completion but is not the correctness oracle.
Record full untimed report, group, iterations, checksum, monotonic duration and
wall-clock bounds. Source objects must remain unchanged after the loop. Each
process launch/termination has its own captured timestamps and terminal status.

Allocation: after CPU collection is terminal, four balanced process pairs,
two baseline-first and two candidate-first. Measure each group at one and
sixteen iterations, separately from the CPU executable. Record requests,
cumulative requested bytes, live delta and peak above the starting live bytes.
Preserve instrumented elapsed values but never use them as CPU evidence.
Do not interpret cumulative allocation as peak/RSS or cache boundedness.

Analysis: all samples remain, with per-group paired medians and marginal
medians, corrected 20,000-replicate bootstrap median intervals and independent
binomial order-statistic median intervals. Run the sampler's self-tests. Both
methods are conditional on independent/common-distribution observations;
balanced collection does not prove these assumptions. Intervals are not
multiplicity-adjusted. Report both unchanged-full-result and new-answer groups,
but never describe new-answer ratios as equal-work speedups. Disclose short
samples, noisy controls and absolute costs, not only favorable ratios.

Recheck solver, harness, input, executable and previous-evidence identities
before/after collection. Failure or an externally changed source/environment
stays visible; do not relabel a partial run as complete. Further WASM execution,
representative consumer tests and stripped-size gates remain before retention.
