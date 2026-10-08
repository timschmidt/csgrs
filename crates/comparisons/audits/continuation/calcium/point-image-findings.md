# Checkpoint 53 — certified point-image witnesses, isolated qualification

The guarded candidate recovers all 825 valid point images rejected by the
retained baseline. It is **not retained in production yet**. Exactness and
bounded completeness checks pass; nonrational/Unknown endpoint cost, guarded
consumer qualification, other platforms and representative application sizes
remain open. All five retained continuation transfers are unchanged.

## Proof boundary and candidate history

The binary-image helper used to discard every exact witness. Refinement
correctly refuses a collapsed interval without one. The candidate preserves a
witness only after STRICT endpoint equality; containment, polynomial vanishing
and uniqueness are still replayed. Approximate equality is never sufficient.

For multiply/divide under an approximation-permitting caller policy, the final
guard additionally replays the whole image construction under STRICT before
attaching a witness. This replay is one-level bounded. Equality of extrema
chosen using approximate ordering is not by itself a proof of a point image.
Addition/subtraction construct endpoints directly with exact arithmetic.

The initial candidate has six additional algorithm lines and six regression
tests. V2 removes one redundant u64-to-u64 cast in a test; the failed Clippy
capture and exact v1 source remain preserved. This changes no algorithm or
recorded numerical result. A small solver-only copy then adds two ordering
controls and the whole-image guard. The final source has 20 net additional
algorithm lines and 303 net test lines versus the retained baseline, with no
public type, degree-cap, polynomial-construction or cache-layout change.

The first ordering probe failed its own precondition: selected endpoints were
Unknown under STRICT, not Equal. It did not demonstrate a candidate defect.
Its source and code-101 capture remain preserved. Both corrected controls
withhold witnesses in the unstrengthened candidate. No approximate-extrema
witness defect was reproduced; the final guard closes a source-level proof
obligation rather than claiming such a reproduction.

## Mathematical and regression results

The unchanged checkpoint-52 BigInt polynomial/Sturm oracle independently checks
all 6,441 candidate records. Exactly 825 InvalidTransformedEvidence results
become Transformed; all other 5,616 complete records, including the terminal
record, are unchanged. All 5,126 returned roots and 1,794 exact witnesses pass.
All 241 divisor guards, 968 degree guards, 65 nonisolating controls and 40
zero-resultant Undecided controls remain intact. The zero-resultant cases are
a separate unresolved completeness issue.

There are 48,059 checks, including 6,441 self-paired record checks when reusing
the unchanged oracle. Actual baseline/candidate agreement and improvements are
checked separately by full-record comparison. The original failed checkpoint-52
gate is not changed or relabeled. Signed full resultant construction remains
the baseline algorithm; the independent kernel evidence is preserved in 52.
The rational public oracle uses closed intervals. Dedicated tests separately
check partition-owned half-open refinement behavior; neither is a universal
root-isolation proof.

| Qualification | Observed result |
| --- | --- |
| Matched retained baseline, all-feature solver debug/release | 803 tests pass per profile |
| Initial/v2 witness candidate, all-feature solver debug/release | 809 tests pass per profile |
| Final guarded solver, all-feature debug/release | 811 tests pass per profile |
| Final guarded focused binary-image tests | 24 pass |
| V2 Hypercurve consumer, all-feature debug | 1,764 pass; existing nine ignored |
| V2 and guarded Clippy/all-targets; changed-file rustfmt | Pass after test-only cast cleanup |
| Documentation test command | Succeeds with zero doctests; no coverage claim |

The consumer run took about 26 minutes; it is a regression result, not a
benchmark. It used v2, not the subsequently guarded solver. Further consumer
qualification must use the exact final source before retention. There is no
new full-CI, guarded consumer, WASM or other-platform qualification claim.

## Native cost and memory results

The final guarded variant was measured after all regression/memory work had
finished. The Linux/Ryzen 5800X3D host snapshot records a powersave governor,
not fixed frequency or guaranteed absence of external load. Forty authored
public queries have retained/fresh input lifecycles. Each process preconditions
eight queries. CPU runs are pinned to core 6, with twelve alternating ABBA/BAAB
blocks, 3,840 observations and 160 calibration records. A separate allocation
binary supplies 480 observations. Every observation rechecks its expected full
report and checksum; raw order and statistics are replayable.

Seventy groups return identical results. Their paired median timing ratios
range from 0.9302 to 1.0804; five per-group bootstrap intervals lie wholly above
one and seventeen below. These intervals are not multiplicity-adjusted, and
no blanket speedup or causal attribution is claimed. All seventy groups have
identical allocation-request, requested-byte, live-delta and peak-delta counts.

Ten groups gain a certified answer. Their paired ratios range from 0.9866 to
1.1673; these are additional-work costs, not equal-work speedups. For example,
the retained rational sum has marginal medians 2.411 versus 2.707 microseconds,
while saving one allocation and 57 requested bytes per query. Six changed
groups save those requests/bytes; four request more bytes (119 or 223 per query),
and one of those adds one request. All eighty groups have unchanged measured
live/peak deltas. Marginal medians and medians of paired ratios are different
statistics. Nonrational/Unknown endpoints and cold-process caches were not timed.

Focused Memcheck reports zero errors and no definite/indirect/possible lost
blocks in baseline, v1 and guarded runs. Each output matches its own native
collection. Baseline retains 1,839,816 bytes in 15,311 reachable blocks; both
candidate versions retain 1,840,544 bytes in 15,318 blocks: +728 bytes/seven
blocks. These are not zero-live results. Cumulative collector requests, including
setup and larger output, are 213,126,963 baseline bytes, 215,408,740 initial
candidate bytes and 215,408,746 guarded-candidate bytes. The six-byte difference
is consistent with the longer executable path, not a measured kernel cost.
These are not peak/RSS measurements. Ownership and
boundedness of every reachable block are not newly established.

## Evidence, resources and disposition

Thirty-three terminal build/test/check/memory/environment/cost/binding captures include
two preserved code-101 failures: the redundant test cast and the failed probe
precondition. An additional code-1 binding failure incorrectly required the two
candidate collectors' cumulative requested bytes to match exactly; its original
sources and output are preserved, and both measured totals are now recorded.
No mathematical oracle or measured output changed. Current mathematical/statistical
checks pass. The public output
has 4,428,994 bytes per candidate collection; v1, guarded and their Memcheck
outputs match exactly. Earlier baseline and power-sum evidence is unchanged.

Six dedicated executables occupy 56,943,680 bytes in the bounded
`/tmp/calcium-point-image.4f6mja` directory. One isolated 956-file source copy and
one 175-file solver-only copy reuse the existing baseline, scalar sources and
shared Cargo cache. About 17 GB remained free on `/tmp` after cache growth.
No cleanup/deletion, donor or production edit, commit, push or external report.
Unstripped driver bytes are not representative application-size evidence.

Next: qualify the guarded source in consumers and representative native/WASM
applications, and measure nonrational/Unknown endpoint and state-history costs.
Keep or refine it only after those checks justify retention. Resume the separate
power-sum optimization afterward. No new donor read credit is added here:
Calcium/FLINT remains 1,415 complete files, 20 partial files and 183,653 unique
read lines. The full original ecosystem inventory remains incomplete.
