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

If correctness passes, a new fixed scalar comparison must cover timing,
requested allocations and probe sizes. No sample is added to either completed
storage experiment. Native qualification, fixed endpoint/broad/final-abort
controls, complete report bytes, production CLI size and the original full-board
120-second worker-CPU and 60-second neck check-body limits remain required.
The parent still fails the board assertion at 133.31248412 seconds. No candidate
is admitted, and all 39 properties, 14 targets and crate-specific future work
remain in scope.
