# Selected polynomial enclosure ownership review

At `ee4932b4c`, all 182 lines and nine declarations in
`crates/hypersolve/src/selected_polynomial_field/bounds.rs` were reviewed with
their immediate policy, work-accounting and Hypercurve consumers. The inventory
updates two lexical entries and adds seven missing declarations; its other
1,576 rows remain unchanged. The resulting 1,585-row inventory is still partial.

This is an ownership audit for plan phases 0/7, not an arithmetic migration.
Relevant properties are P1, P4, P5, P8, P10, P19 and P20. The missing inventory
witness was a complete declaration-to-owner mapping for this module. Existing
geometric witnesses remain separate from that static completeness check.

`SelectedPolynomialEnclosure` certifies closed `Real` bounds around an immutable
`SelectedPolynomialValue`. Its private fields retain the selected field graph;
the bounds are not independently selected coordinates. The carrier and its
accessors belong to plan category 5, certificate/provenance. Construction,
validation and refinement belong to category 3, exact proof adapters. Moving
the carrier to generic scalar interval arithmetic would require preserving its
selected-value authority and cannot be justified by its two endpoints alone.

Bracketing preserves the sign/comparison sequence and reverses negative bounds
exactly. Authored bounds are checked for order and containment in a strict
predicate pass. Refinement narrows a clone; failure does not change the original
certificate. The context must honor its strict-pass contract. Accessors borrow
the retained value or checked bounds, and diagnostics omit the selected value
graph. No allocation or cost improvement is claimed by this source review.

Each new bound comparison charges a shared root-refinement unit before exact
field subtraction and sign evaluation. That counter does not bound coefficient
growth, total algebraic work, memory, or cancellation latency inside a decision.
Structural zero, existing singleton and zero-step reuse have distinct paths.
There is no direct enclosure serializer: selected-definition replay reconstructs
the value, and checked bounds are established again. Arbitrary symbolic inputs
do not acquire a universal decision or completion guarantee from this adapter.

The immediate consumers retain these distinctions: Hypercurve's field helper
extracts checked bounds, selected scalar queries retain both source and value,
and algebraic chord displacement uses an enclosure to construct an open
selector while retaining its original quadratic relation. None authorizes
projecting the exact value into an unrelated endpoint scalar.

Existing full-owner qualification at composed revision `f65589b44` passed the
five selected-polynomial-bound tests, the original second-chamfer enclosure
test including its child-process replay, and the selected-polynomial replay
suite. The audited module and eleven related source/test files are byte-identical
between that revision and `ee4932b4c`. These are reused receipts, not new test
executions on the primary revision. Their assertions cover independent rational
bounds, dependent fields, negative/rational images, close distinct values,
invalid authored bounds, zero budgets, retry and absence of approximation.

Only inventory and documentation change. Static declaration coverage, preservation
of unrelated inventory rows, source identity, spelling and diff checks are run
for this review. Focused cost, whole-resource bounds, broader downstream closure,
and the queued runtime campaigns remain separate obligations. No production
change, resource allowance or phase completion is admitted.

Frozen review record: `docs/evidence/2026-11-17-interval-selected-enclosure-review`, local evidence commit
`3382575d943f9d7801fe56d185827120576a2420`; 15 files / 1,672,600 bytes, SHA-256
`b05e1ff27456794dcbcb0432cbd77401e6cb7fbb45a8aab53ad30f413adb88c7`. The source manifest registers this record.
No repository has been pushed for this review.
