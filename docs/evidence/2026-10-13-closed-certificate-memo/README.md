This experiment asks whether successful publication of a closed Laurent
certificate makes the same node's query-memo entry redundant. It starts from
the uninstrumented endpoint/P4 parent, with Vec storage and its original
dependency features. Neither rejected inline-storage implementation is used.

The retained recognizer checks the persistent cache before the query memo.
Currently, a shared result is cloned into the memo before attempting persistent
publication. When that publication succeeds, later retained visits use the
persistent certificate. Closed coefficients contain only rational data, so
omitting their memo copies should not change Computable node ownership or the
original sharing decisions. This is a source-level hypothesis, not measured
coverage or a performance claim.

The candidate makes publication report success and skips the memo insertion
only on that success. Failed publication, opaque results, ineligible nodes and
the non-retaining recognizer keep their memo behavior. The persistent cache
still publishes under its existing lock; no guard escapes. The original
failed-projection retry still releases its first memo and uses the original
512-node/depth and eight-term/eight-radical limits. Arithmetic, stored forms,
fact retention and both original native timing assertions are unchanged.

Two fresh sources receive the same eight existing storage controls and three
new publication/memo controls. The new controls check exact closed results and
zero-work warm reuse, opaque ownership/reclamation, and zero-work reuse through
the original query memo. Their observed memo inventories distinguish the two
sources without making their mathematical assertions differ. Qualification
must verify those inventories and unchanged layouts, plus the inherited P4,
cache-frontier, default/all-feature library, doctest, Clippy and docs checks.

Preparation completed and froze two 10,133-file sources. The baseline differs
from the parent only by the two shared test modules and their declarations.
The candidate differs from that baseline only in algebra.rs and
representation.rs. Exact recovery checks preserve every parent token outside
the declared edits. All manifests, lockfiles, 35 pending native entries and
the main runtime remain unchanged. All 22 qualification commands passed;
see [preparation-v1/protocol.json](preparation-v1/protocol.json) and
[correctness-v1.json](correctness-v1.json). Each source passes 885 all-feature
and 798 default scalar library tests, 28 doctests, the focused controls, strict
default/all-feature/all-target Clippy, formatting and warning-denied docs.
These configuration and focused counts overlap.

The retained closed-result control records one query-memo entry in baseline
and zero in the candidate. Both sources retain one entry for the original
recognizer and two for the opaque expression and its shared atom. Exact results,
zero-work reuse and final owner release pass in both. All seven measured private
layouts match: Rational 8 bytes, coefficient 56, polynomial 24, closed
coefficient 32, closed polynomial 24, Computable 16 and Node 56.

The fixed scalar comparison completed 12 preflights and 60 measurement processes, in five
predeclared alternating pairs. Each timing phase uses 4,096 inputs; allocation
phases use 1,000. Separate executables and fresh targets keep the allocation
wrapper out of timing. All builds preceded every probe. Every mathematical
witness and representation inventory matches. No sample was added to either
completed storage experiment. See [cost-assessment-v1.json](cost-assessment-v1.json)
for all samples and [scalar-decision-v1.json](scalar-decision-v1.json) for the
provisional decision to investigate native impact with scalar concerns open.

Of 68 timing phases, seven are faster in all five blocks and 28 in at least four;
three are slower in all five and seven in at least four. Allocation calls and
allocated bytes decrease in 21 phases; peak requested bytes decrease in 17.
No memory counter increases, and retained requested-byte deltas match in every
pair. Every memory delta repeats across the five blocks. The largest reductions
per 1,000 inputs are 3,000 calls, 352,000 allocated bytes and 352 peak bytes.
All six probe executables shrink by 208–216 bytes.

| Adverse phase | Slower pairs | Baseline median ns/input | Candidate median ns/input | Ratio of medians |
| --- | ---: | ---: | ---: | ---: |
| First affine-surd query | 5 / 5 | 459.658 | 508.937 | 1.107208 |
| Warm affine-surd query | 5 / 5 | 378.702 | 412.631 | 1.089591 |
| Opaque-decline construction | 5 / 5 | 796.164 | 816.658 | 1.025741 |
| Radical-Laurent construction | 4 / 5 | 728.993 | 840.682 | 1.153210 |

The first three adverse phases have unchanged memory counters. Savings elsewhere
do not explain or excuse their timing costs. Some apparent large integer gains
also depend on variable baseline samples: signed-word construction has a ratio
of medians of 0.625348, but a median paired ratio of 1.041420 and is slower in
three pairs. These descriptive statistics establish neither statistical
significance nor an application speedup. All adverse rows remain in the evidence.

The candidate remains unadmitted. All nine native validation phases completed
serially. Strict four-crate Clippy and warning-denied docs pass. The full suites
pass 974 solver results, 2,494 curve results with 33 existing ignores, and 1,644
DRC results excluding the two separately executed timing gates. The five focused
endpoint controls also pass and overlap the full curve count.

The original board assertion fails at **132.328922579 seconds of worker CPU
against 120**. Both complete boards emit reports with 12 and 197 findings.
The original copper-neck assertion passes its 60-second check-body wall limit;
it does not print its passing internal duration. Its 120.636210331-second whole
process wall time includes setup and is a different metric. These are standalone
gate outcomes, not a matched application-speedup result. See
[native-assessment-v1.json](native-assessment-v1.json).

The fixed native comparison completed all 36 declared processes: 20 endpoint
timing/allocation runs, ten broad-operation runs and six final-coordinate abort
runs, after four preflights. Every endpoint identity, all 231 broad witnesses
per process and all 210 abort receipts per process agree across sources. The
broad witnesses also match the historical corpus. Both sources and their
artifacts pass the independent checks in
[native-controls-assessment-v1.json](native-controls-assessment-v1.json).

| Native timing group | Phases | Slower in all five pairs | Slower in at least four | Faster in all five | Faster in at least four |
| --- | ---: | ---: | ---: | ---: | ---: |
| Endpoint | 48 | 5 | 12 | 8 | 18 |
| Broad operations | 231 | 18 | 58 | 9 | 49 |

Near-endpoint off-circle warm and cloned queries are slower in every pair,
with ratios of medians of 1.064517 and 1.146813. Their median paired ratios are
1.076641 and 1.152364. Their requested-memory counters are unchanged. Native
samples include substantial within-source variation; all samples, ranges and
adverse rows remain recorded. These are descriptive results, not a claim of
statistical significance or a matched application speedup.

Six endpoint phases reduce allocation calls and allocated bytes, and five reduce
peak requested bytes. No counter increases; retained requested-byte deltas match
in all 48 phases. Every memory delta repeats across the five pairs. Per 1,000
inputs, the largest reductions are 13,857 calls, 1,718,784 allocated bytes and
440 peak bytes. Four native probe binaries shrink by 232–248 bytes; production
CLI sizes were not measured.

[decision-v1.json](decision-v1.json) rejects this version. Removing redundant
memo copies preserves the tested facts and reduces allocations, but the
unresolved scalar and native slowdowns prevent retention under the
performance-before-memory ladder. The original board assertion also remains
failed. Neither fixed sample set was extended. The complete-report replay and
production-size runners remain prepared and unexecuted after this rejection;
complete report-byte identity is not established for this candidate.

The subsequent [shared-radical-storage experiment](../2026-10-13-shared-radical-storage/README.md)
addressed reconstruction of rational/radical arrays on recursive persistent-cache
reads. It qualified the alias and ownership controls but was also rejected after
fixed scalar costs. It starts from the frozen parent without this memo change;
its decision records the next review of owned scratch arrays versus immutable
retained sharing. The main runtime and all 35 pending native entries remain
unchanged. The parent still fails the board assertion at 133.31248412 seconds.
All 39 properties, 14 targets and crate-specific future work remain in scope.
