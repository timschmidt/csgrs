# Cancellation publication within the finish borrow

Candidate `1abfbc3fc80c52d7d57be26ac031ef824ca92d9d` moves the final cancellation
decision into the existing scope-finish borrow. It remains isolated pending
admission. This follows the [allowance/runtime review](cancellation-allowance-runtime.md)
under Hypersolve work-budget ownership, P5/P6/P20/P36 and plan 14.4. Earlier
allocation-count charges and adverse runtime observations remain unchanged.

The subsequent [complete resource review](cancellation-finish-resources.md)
records fresh lifetimes, stack and final native images. Fifteen accounts remain
outside the existing byte proposal after historical charges are retained; the
candidate remains isolated.

## Contract and implementation

The original publication candidate queried exhaustion after the callback and
then separately borrowed the thread-local frame stack to remove the scope.
The revised candidate shares the existing decision through a private owner
helper and invokes it inside the finish borrow, before removing the frame.
The public exhaustion query uses the same helper. Ordinary const-false scopes
still skip the final cancellation poll.

The innermost first-failure lookup still precedes the existing cancellation
scan. That scan, its outermost-token selection and failure propagation are
byte-identical. A live scope establishes the active-stack invariant during
finish. The returned value is destroyed after the borrow ends and the child
frame is removed. No callback preemption, rollback, time/heap bound or stronger
post-check cancellation guarantee is introduced. Removing the separate source
query/borrow does not itself establish a runtime improvement.

The missing acceptance witness checks destructor reentry. An unpublished
payload's destructor must see the restored, unexhausted parent, consume one
unit from that parent's budget and permit fresh-token retry. This detects a
retained frame borrow, a leaked cancelled child or destruction after the parent
has already disappeared. Existing publication tests and adapter assertions
remain unchanged.

## Functional qualification

Current primary `a58c2771849e57b961983f83efeb8a34cf7ca267` reproduces six owner
publication failures, including the new witness, and the adapter failure. Three
owner controls pass. Its production/build sources match the earlier primary
baseline. The original publication implementation passes all nine owner
publication tests with the new witness added.

The revised candidate passes 1,303 affected test invocations: 633 no-default
and 636 all-feature owner tests, 26 ordered-field integrations and eight
downstream budget/cancellation tests. Two strict Clippy configurations, strict
owner/adapter docs, owner examples/benches and formatting pass. Independent
audit verifies the source transitions, unchanged tests, cancellation scan,
command outputs and expected primary failures.

The functional producer uses 42 aggregate children, including six preparatory
Git reads, and about 685 seconds within its 160-child / 3,600-second limits.
Its audit uses nine read-only Git commands and about 0.1 seconds within its
separate 300-second limit. Reference `c799e8acb4a4c0bde767e818c9bd7989a6d2087c`
retains the original implementation; only the owner implementation file changes
from that reference to the revised candidate.

## Targeted runtime contract

The screen reuses the two verified trace images from primary `e63cbf68` and
original candidate `8a454c860`, then builds revised trace and trace/retry images
against the exact original consumer path, fixtures, observers and locked
dependencies. All 128 functional controls, two strict Clippy configurations and
formatting pass. Common full outputs and traces match across all three sources.
The build producer uses 19 commands / about 81 seconds within 80 commands /
1,800 seconds; independent verification passes within its 300-second bound.

Timing covers eight reused-input cases: one and four variables, both policies,
and passive/active tracing. These include the previous two cases that were
slower in all four comparisons. Six balanced permutations of the three images
give six comparisons per case against each reference. An added retry invocation
records absolute costs without a successful-old denominator.

Each iteration preserves operation, output/trace materialization, original
validation and cleanup. Reused-input setup/teardown and debug serialization
remain outside timing; caches may be warm. The screen uses CPU 0, disabled ASLR,
a shared executable path and environment, 30 samples, 0.5-second warmup and
one-second requested measurement. It is a targeted screen, not full owning,
complete-consumer, memory, distribution or platform qualification.

## Results and disposition

All 19 invocations complete: 152 case invocations, 4,560 samples and 608 public
full control records. Independent audit verifies sample arithmetic, all image
and raw-log hashes, launch controls and 39 host boundary records. No competing
compiler/routing workload is observed at those boundaries; that does not prove
continuous host exclusivity.

| Comparison of revised candidate | Range of case median changes | Faster / slower pairs |
| --- | --- | --- |
| Against primary | -1.81% to +0.94% | 30 / 18 |
| Against original publication candidate | -2.73% to -0.43% | 38 / 10 |

All eight medians are lower than the original publication candidate. Against
primary, seven are lower and one is higher. The one-variable APPROXIMATE_512,
active-trace case has a +0.94% median, about 973 ns, with four slower and two
faster pairs ranging from -1.31% to +1.85%. One pair has nonoverlapping
within-run confidence intervals in the adverse direction. Those intervals
are not independent cross-process proof. All favorable and adverse observations
remain; no universal speedup, slowdown or causal mechanism is established.

The previous two consistently adverse passive-recording domains have lower
medians against primary in this screen: one-variable APPROXIMATE_512 is -0.58%,
and four-variable STRICT is -1.81%. The latter is faster in all six comparisons.
This supports further investigation of the revised candidate, not a claim that
the full performance gate passes. No unchanged benchmark rerun replaces earlier
observations.

Timing uses 59 commands / about 327 seconds, within 100 commands / 1,800 seconds.
The audit passes in about 1.3 seconds of its separate 300-second bound. All
producers are terminal and source control is restored clean. The workflow queue
remains held and its original watcher remains terminal.

The candidate remains isolated. Full owning and complete-consumer runtime,
lifetime memory, final-artifact and platform qualification remain open.
The previous byte allowances and resource verdicts do not automatically transfer
to changed source. Historical count charges, all earlier failed obligations and
other gates remain intact. Primary production code and accepted totals of
57 size / zero memory exceptions are unchanged; inventory remains 1,644 rows.
No push; the full fifteen-phase goal remains active.

Frozen evidence: `docs/evidence/2026-11-17-interval-cancellation-finish`,
archive commit `579335fba1e0ff7a71f159390b67cdfb95df4a0d`; 3 files / 9,784,408 bytes,
SHA-256 `8f3669c71dad32650f9a5fdc4be768c35af7c2c1ea551686009d33b57e65ca01`.
The lossless payload contains 1,434 members / 33,176,529 raw bytes and
four validated images. Original, decoded and committed hashes are verified;
earlier archive paths are unchanged.
