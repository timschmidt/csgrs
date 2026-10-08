# Retained cache-coarsening candidate — 2026-09-07

Status: the small cache repair and five regression tests are qualified and
retained in the Hyperreal working tree, uncommitted. All retained-source and
downstream gates pass. This closes the scoped cache transfer, not the full
ecosystem audit. Previous repairs are preserved.

## Change and proof

The earlier isolated pilot proved the signed precision subtraction can overflow
and identified excessive cloning for large precision gaps, but showed a17–21%
small gap-one regression. This revision keeps scale's gap-one direct-clone path.
For requested p>cached q, p.abs_diff(q) computes the exact positive gap in u32,
including gaps larger than i32::MAX. Equal precision still clones; finer
requests still miss. No API, representation field, dependency, lock, fact,
cache-publication rule or mathematical rounding contract changes.

For d=p-q>0 the result remains floor(n/2^d+1/2), i.e. nearest with ties toward
positive infinity. d=1 uses the old direct clone before adding one and shifting
once. Other ordinary gaps shift the borrowed cached integer first, copying only
surviving limbs. If d>bits(abs(n)), return zero directly: abs(n)<2^(d-1), so the
same rounding is zero. The strict comparison handles signed halfway values
correctly. A cached integer enclosure error<=1 becomes at most1/2+1/2^d<=1
after coarsening. This is an approximation, not new exact-zero knowledge.

The production runtime file is byte-identical to the qualified private file:
src/computable/node/representation.rs SHA256
6901ebf2795947942eea4356b0031fa2a5794187f79e30c076e5c6a01351d940.
The five tests are copied into a separate included module and formatted;
retained-gates.mjs verifies equality after canonical rustfmt, not loose textual
token removal. The node facade differs only by its new include. The misleading
cache comment is corrected: existing coarsening already happened under the read
lock; only the owned result leaves that guard.

## Real-cache correctness and static gates

A source-only175-file/5,280,467-byte mirror preserves the current Hyperreal
working tree.125 registry packages match production Cargo.lock exactly. Two
ordinary real-cache tests pass before/after. Three others fail before in debug
on the signed subtraction and pass afterward in debug and release:

- exhaustive small signed rounding/cache misses, limb/carry/halfway boundaries;
- all i32 gap endpoints, including zero and both integer signs;
- public warmed sqrt(17), negative sqrt(17),1/3, pi and tau at i32::MAX,
  followed by finer refinement;
- eight concurrent writers,32precisions each, and repeated coarse reads,
  retaining the finest valid approximation despite out-of-order completion.

The overflowing release baseline is not executed. Independent exact public
history checks cover160ordinary signed radical/rational requests before/after,
plus180requests including extreme precisions after. Checks use integer squared
enclosures or rational inequalities, not floating-point agreement. Both the
ordinary release and separately instrumented allocation binary pass.

Private full default/debug gates pass756tests including19doctests; full
release/all-feature gates pass863including24doctests. Clippyallfeatures/alltargets
with-Dwarnings, package fmtcheck and WASMlibcheck pass. These exact totals are
parsed from current raw logs, not reused from earlier repair checkpoints.
Retained-source reruns currently pass the same756/863tests, Clippy, WASM,
production cargo fmt and explicit formatting of the new included test module.

Memcheck passes all five regression tests and180public history requests with
zero errors. Leak checking is disabled; this is not a zero-leak or race-detector
claim. The first attempt stopped before tests because Valgrind's debugger pipe
could not be written under quota-exhausted/tmp. Frozen memcheck-v1 records that
setup failure; v2 disables the unused debugger interface with--vgdb=no.

## Public timing and allocation

The before and after executables use the same harness, package name, path,
dependencies and release flags. Each process independently constructs its
objects. Warm queries include equal precision, gap one, half-width and nearly
all-discarded caches; cold controls reconstruct the scalar on every call.
Both signs of sqrt(17) and1/3 are tested. CPU6 pinning, rotated/reversed order,
and identical-before-code controls are used; other host work is not excluded.

paired-v1:2,028fresh-process timing observations,1,872postwarmup,
79,872,000timed checksum-checked calls across52workloads. Twelve measured pairs
per workload; exact outputs match before/after byte-for-byte. Independent
JavaScript integer enclosures verify every recorded output. Separate memory
binaries produce312observations, three identical repeats of100calls for all
52workloads/two builds. All retained-live allocation deltas are zero.

Selected after/before CPU ratios (descriptive paired median95% intervals):

| Public workload | Ratio |
| --- | ---: |
| positive sqrt(17),64bit cache, gap1 |.9348 [.9251,.9873]|
| negative sqrt(17),64bit cache, gap1 |.9169 [.9002,.9419]|
| positive1/3,64bit cache, gap1 |.9461 [.9065,.9612]|
| negative1/3,64bit cache, gap1 |.9433 [.9169,.9957]|
| positive sqrt(17),65536→128bits |.3325 [.3255,.3454]|
| negative sqrt(17),65536→128bits |.3553 [.3444,.3784]|
| positive1/3,65536→128bits |.3612 [.3159,.4073]|
| negative1/3,65536→128bits |.3422 [.3024,.3601]|

The last four cases roughly improve2.8–3.0x. Per-call requested cumulative
allocation falls8224→32bytes for the radical and8208→24for the rational;
allocation calls2→1; additional peak requested live8200→32and8192→24.
Equal/gap-one and cold controls do not reduce allocations. Requested live
allocation is not RSS or allocator-internal/transient realloc storage.

Two apparent small slowdowns triggered a longer independent repeat rather
than selective reporting. focused-v1 adds800observations,768postwarmup and
440,000,000timed calls. Eight suspect/neighbor/cold workloads,24measured rounds,
and identical-code controls for BOTH binaries. The warmed negative64bit rational
equal-precision ratio is1.0017 [.9912,1.0328], but pooling the duplicate controls
gives1.0126 [1.0039,1.0204]. Positive/negative65536bit rational gap-one ratios are
1.0236 [1.0166,1.0305] and1.0294 [1.0114,1.0440]; control-pooled ratios1.0267 and
1.0287 also exclude parity. Positive equal/cold rational controls show smaller
roughly1–2% pooled costs. All raw/control distributions remain recorded.
Thus there are small workload-specific timing regressions, not a universal
speedup or no-regression result. They are materially smaller than the original
17–21% prototype regression and are accepted for the user's higher-priority
completeness repair and substantial large-gap memory/performance improvement.

Standalone identical-harness size: file1,552,016→1,553,216bytes(+1200);
text1,075,065→1,075,989(+924), data217,688unchanged, bss1536→608(−928), loaded
sections1,294,289→1,294,285(−4). This is a driver/layout measurement, not a claim
that every consumer binary shrinks. The runtime body grows by11source lines
plus a corrected comment; no scalar/cache storage increase.

## Retention and remaining work

Decision: KEEP the small production repair after the completed gates below.
Before applying it, prepare-retention.mjs froze947files/45,192,934bytes across
all six named crates under before-source, with heads/statuses and every hash.
The baseline includes current Hypercurve HEAD
a3c1bb4582efbe337f37ef3e2c878787edf861d4, clean; old Hypercurve qualification on
different user snapshots is not substituted. retained-gates.mjs checks every
baseline copy, every current source file and file inventory before/after each
command, allowing only the two intended Hyperreal edits and new test file.

Final retained validation: retained-gates-DKveZ9 completed all11commands, exit0.
Hyperreal756/863tests and all six scalar gates pass. Full default downstream
tests pass:Hyperlattice202,Hyperlimit348,Hypertri7,Hypersolve796. Hypercurve's
full library suite passes910tests,0failures,1ignored in755.61s on the frozen
current source. Session52381 is terminal; do not restart it. All947baseline
files plus the new test file reconcile, and all3140raw timing/allocation
records match their frozen analyses. verify-retained.mjs checks these results,
the exact allowed source changes, binaries and Memcheck records; it also runs
git diff--check in all six crates. No commits, pushes, deployments or deletions.

retained-gates-DKveZ9/complete.json SHA256
75900b1a1dc76ba4c79f323179326fa2c1a86b4abf8a0f518abd9ca221a3696a;
retained-checkpoint.json SHA256
2dd4692766559e9891f8158cbc81fee28c0dae803a56d671e6d4cf03ac576e2e.

Original private verifiers which insist that live Hyperreal matches the old
397-file snapshot now correctly reject the intentional retained change. Their
frozen results are historical evidence, not stale tests silently relabeled as
current. The new six-crate retained verifier recognizes the precise allowed
changes and keeps pre-change sources recoverable. Whole-i32 precision arithmetic
outside a valid cache hit, including shared-constant cross-cache p±1 on cache
misses, remains a separate audit question; this repair does not claim to solve it.

| Artifact | SHA256 |
| --- | --- |
| paired-v1/analysis.json |fb79e0de09a4637789c09a32a58f67ce7a10b133434e36f0484e14a24ae35b37|
| focused-v1/analysis.json |81a4b8df836517dcf2ba3faf6c0d0ac2e77f204b7bcf42975268d8ce0f06fbe4|
| gates-ddSPOg/complete.json |59dc6dedfc9d82984a4621d3601981f5ca43f5a48ff3ee041b7926ce94d5067f|
| memcheck-v2/complete.json |1ddf7d52f0237584942604b087c639ec60b5d82ae014cb47e202282f97c33425|
| retention-baseline.json |bcfd719d324cf2570e63a0cdaaf12d9c9e224417ee5ef700e21165d576e60ca4|

Unqualified initial setup attempts are preserved: missing parent-workspace
lockfile, then an incorrect test include path. No failed build was counted as
a numerical regression. The initially index-resolved lock is archived as
setup-generated-Cargo.lock; all qualified runs use the production-pinned lock.
