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

The prepared fixed scalar comparison covers timing, requested allocations and
probe sizes through 12 preflights and 60 measurement processes, in five
predeclared alternating pairs. Each timing phase uses 4,096 inputs; allocation
phases use 1,000. Separate executables and fresh targets keep the allocation
wrapper out of timing. All builds precede every probe. No sample is added to
either completed storage experiment. Native qualification, fixed endpoint/broad/final-abort
controls, complete report bytes, production CLI size and the original full-board
120-second worker-CPU and 60-second neck check-body limits remain required.
The parent still fails the board assertion at 133.31248412 seconds. No candidate
is admitted, and all 39 properties, 14 targets and crate-specific future work
remain in scope.
