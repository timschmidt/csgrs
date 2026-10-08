# Ruffini-inspired cache-coarsening pilot — 2026-09-07

Status: isolated prototype qualified; **no production change retained yet**.
The public cache precision-subtraction overflow is confirmed in debug. Public
candidate integration, representative caller timing, concurrent/cache-history
regressions, full suites and binary/code-size gates remain OPEN. This is not a
blanket Hyperreal precision-range qualification or a completed ecosystem audit.

## Provenance and current architecture

Ruffini's precision cache prompted the existing borrowed-coarsening candidate
in RUFFINI_FILE_NOTES.md. No donor code is copied. Hyper's cache keeps a single
finest approximation behind a lazily allocated RwLock; a coarser request
currently clones the complete BigInt before scaling it while the read guard is
held. Immutable graph ownership, monotone cache replacement and lock structure
are unchanged by this private experiment. The cache comment saying it only
clones under the lock is stale: existing coarsening also occurs under that lock.

Current production representation.rs SHA256:
d2eb416cc9595697584f9a1d714a41b6a8f7c4ddb8867b220b7e73883416c2ab.
The earlier square-root pilot's 397-file Hyper snapshot matches before/after
this experiment. Hyperreal remains HEAD da26e961bf76adf13829d8a26ced6cdaacb6b564
with the pre-existing audit edits preserved. No Hypercurve current-source test
claim is added. The numeric dependencies match current Hyperreal's lockfile:
num0.4.3, num-bigint0.4.6, num-integer0.1.46, num-iter0.1.45,
num-complex0.4.6, num-rational0.4.2, num-traits0.2.19, autocfg1.5.0.

The installed num-bigint0.4.6 signed/unsigned shift implementations were read
fully for this path, not credited as a whole-dependency audit. Borrowed ordinary
right-shift copies the surviving limb slice. Its all-limbs-discarded branch
still obtains an owned copy before zeroing, motivating a separate zero guard.

## Contract and candidates

For cached precision q and requested p>q, gap d=p-q is in1..=u32::MAX even
though the signed i32 subtraction q-p can overflow. Forming d in i64 then
converting to u32 is exact. The three candidates are:

- wide: same owned-clone rounding, with the precision gap widened first;
- borrowed: shift the borrowed cached integer before allocating the result;
- guarded: borrowed, plus return zero when d>abs(n).bits().

All preserve Hyper's nearest-integer/ties-toward-positive-infinity rule:

    m = floor((floor(n/2^(d-1)) + 1)/2)
      = floor(n/2^d + 1/2).

This is deliberately not Ruffini's floor-only rounding. If the cached scaled
true value y obeys |n-y|<=1, then |m-y/2^d|<=1/2+1/2^d<=1. Equal precision
clones unchanged; finer requests remain cache misses. The zero guard is strict:
d>bits(abs(n)) implies abs(n)<2^(d-1). Equality must not take this shortcut,
especially at signed halfway cases. Zero is included in the small exhaustive
grid. No cache mutation, aliasing, synchronization or publication rule changes.

## Exact checks and confirmed public failure

Each debug, release and allocation-instrumented release executable passes:

- 149,504 original-helper checks and149,784 checks per candidate. The small
  grid covers every integer from-4096 through4096, cache miss/equal and gaps1–16.
  Another2,310 cases cover both signs, powers-of-two/carry/halfway boundaries,
  patterned values through65,536bits and precision extremes.
- Independent BigInt Euclidean quotient/remainder rounding and exact enclosure
  endpoints, rather than the implementation's nested-shift formula. Large gaps
  use a checked magnitude proof instead of allocating multi-gigabit integers.
- 280 cases per build skip the original overflowing subtraction explicitly.
  They pass all three candidates. A release baseline with an overflowing gap
  is intentionally not executed: wrapping can request a huge left shift.

The unchanged public debug call constructs sqrt(17), warms approx(-128), then
requests approx(i32::MAX). It reproducibly panics at production
node/representation.rs:197 with `attempt to subtract with overflow`. The
subprocess catches that expected panic, and the runner verifies its exact
source location and diagnostic. It is a confirmed cache-hit completeness
defect, not evidence that arbitrary fresh computations at every i32 precision
are already supported or that all other precision arithmetic is safe.

## Timing, allocation and limitations

evidence-v2 contains1,210 fresh-process CPU6-pinned timing observations,
1,100 after discarding the first warmup round, covering22 signed workloads.
Five rotated/reversed variant orders include an identical-original-code control;
each workload/variant has10 measured samples.170,170,000 timed calls have
checked deterministic checksums, with exact result assertions before/after each
timed loop. An independent JavaScript integer reconstruction verifies all
recorded checksums. Twenty-thousand-resample paired median95% intervals are
descriptive one-host uncertainty estimates, not cross-machine guarantees.

Selected after/original CPU ratios:

| Workload | Borrowed | Guarded | Important qualification |
| --- | ---: | ---: | --- |
| positive64bits, gap1 |1.1223 [1.0583,1.1903]|1.1698 [1.1051,1.2388]|small hot-path regression|
| negative64bits, gap1 |1.1067 [.9982,1.2179]|1.2112 [1.1442,1.2582]|guarded regression|
| positive4096bits, gap2048 |.7988 [.7133,.8523]|.6967 [.6713,.8492]|negative counterparts' intervals include1|
| positive65536bits, gap32768 |.8544 [.8336,.9660]|.8199 [.8099,.8588]|negative also improves|
| positive65536bits, gap65537 |.9754 [.9474,.9845]|.04525 [.04143,.04760]|zero guard avoids full clone|
| positive1048576bits, gap1048448 |.02604 [.02060,.02875]|.02115 [.02039,.03008]|negative ratios.02684/.02705|

All cases, signs, individual samples and identical-code-control intervals are
retained in analysis.json, including noisy/regressing controls. No speedup is
claimed for a real application or the public synchronized cache yet.

Separate allocation-instrumented executables produce264 observations: three
identical repeats for22 workloads/four methods,100 calls per observation.
All retained-live deltas are zero. Reported peak is additional *requested live
allocation*, not RSS or allocator-internal/transient realloc storage.
For the positive million-bit-to128bit case, per-call cumulative requested bytes
fall131096→24, calls2→1, additional peak131072→24. Positive half-width4096bit
coarsening falls512→264requested bytes, with one allocation either way. The
guarded all-discarded cases allocate zero. Equal and one-bit paths do not save
allocations. Allocation-instrumented timings are never used for speed claims;
one100-call run reports a zero process-CPU delta, which is irrelevant to its
independently repeated allocation counts.

## Preserved setup failures and evidence

Initial unqualified build resolution selected newer numeric dependencies and
failed on a missing local type annotation. Dependencies were pinned to the
current production lockfile and the annotation fixed before any qualified run.
v1 passed the correctness/public-boundary gates but its first timing round
stopped at the million-bit benchmark: the oracle's one-million-bit guard was
too low for that deliberately larger input. v1 binaries, exact pilot sources,
partial raw records and failing assertion remain frozen. Its incomplete
timings are not combined with v2. v2 raises only the oracle allocation threshold
to two million bits; numerical candidates and benchmark inputs are unchanged.
The first v2 analyzer rejected a zero CPU delta in an allocation-only record;
its gate now explicitly excludes all instrumented timings from speed analysis.
Raw observations were not changed or discarded.

| Final v2 artifact | SHA256 |
| --- | --- |
| prepared.json |34c7aadf91034ecbc72dc6d9fa3e7a261555a7b232475e567c66b7a7d30041b8|
| measurements.json |89c6c3cfc16b13c17ac775311dc39d3ea626eb9105b70a902c21016e9dbdc1ff|
| analysis.json |e43bfdb3b2ffdee92a4dcc462f4e3aa60f8e25f3e78328e6b6576764ada30c53|

prepared.json pins source and binary hashes; analysis.json pins all1,474 raw
measurement records and its analyzer. The standalone binary includes all
methods, so its size is not a candidate-versus-production size comparison.

Next: preserve the original gap-one direct-clone fast path in a separately
qualified revision; integrate only into a private current-source Hyperreal
mirror, then test real cache histories/public workflows and measure ordinary
as well as large-gap costs. No retention decision is warranted from the large
microbenchmark win alone. Full goal ACTIVE/OPEN.
