The complete boards execute 6,379,817 single-radical products with matching
radicands: 29.06% of all coefficient multiplications. Diagnostic qualification
passed. The [reviewed decision](decision-v2.json) calls for a distinct direct
kernel experiment; no runtime optimization is prepared or admitted.

The [cache-local product trial](../2026-10-13-cache-local-polynomial-products/README.md)
failed the original board assertion with scalar costs still unresolved. This
diagnostic returns to the unchanged endpoint/P4 parent. Earlier operand-shape
counts include 8,276,574 radical-by-radical calls, but do not establish how many
have one matching radicand. Recursive certificate-copy counts do not answer
that question either.

The [source review](scope-v1.json) follows the original coefficient multiplier.
It inserts left-linear and then right-linear terms before the nested radical
products. For two single, equal, non-unit radicands and no opaque atoms, at most
one radical remains; conjugates may cancel it. A future direct kernel would
need to preserve arithmetic order, zero handling, bounds, private noncanonical
inputs and ownership. An algebraic identity alone does not establish those
contracts.

Instrumentation reads only vector lengths and observes the existing radicand
comparison branches. It counts the output vector length just before successful
return. No extra scalar comparison, arithmetic, owner, mathematical recognizer
or cache write is introduced. Reversing explicit edits must recover every
original byte. Fixed atomic counters and diagnostic branches change execution
cost, so coverage does not measure time share, allocations or expected savings.

Two helper calibrations cover bins, disabled/reset/unwind behavior and joined
threads. Three site controls cover affine products, exact conjugates, pure-root
products, different-radical insertion order, other shapes and opaque decline
with owner release. Qualification also includes eleven inherited P4 tests, six
cache controls, the all-feature scalar library, 28 doctests, five endpoint
regressions, formatting, strict four-crate all-feature/all-target release Clippy
and warning-denied scalar docs. Counts overlap; this is not the full workflow
matrix.

The original complete boards ran once from a bound executable, retaining
the 120-second worker-CPU assertion. The report printed after the failed
assertion during unwinding. Finding counts are not complete report-byte comparisons.
The 60-second copper-neck assertion remains unchanged but is not exercised by
this coverage run. The entire properties, target and future-work plan remains
open.

The uncompiled mutable draft exposed two macro line-wrap differences between
standalone and crate-level rustfmt settings. The [draft review](draft-format-review-v1.json)
records the correction before frozen preparation. The prepared input files now
match the crate formatting; frozen preparation checks formatting without
editing its generated source.

Version 1 preparation stopped before formatting or compilation because a
derive-line patch anchor appeared five times. The [failure](preparation-failure-v1.json)
preserves all 10,133 copied files, original inputs and the partial source delta.
Version 2 uses the unique full declaration prefix. Forward application now
recovers the already formatted draft bytes exactly; every other edit and both
test/helper inputs remain unchanged. The old source and unexecuted version 1
diagnostic runners remain preserved.

Version 2 freezes 10,133 source files, with five changed or added paths relative
to the 10,131-file parent. Its complete bytes match the formatted mutable draft.
Reversing the explicit edits recovers the original runtime files exactly; the
parent, main runtime and all 35 pending native paths remain unchanged. The
source maps, delta archive, patch and formatting check are committed before
qualification.

The [independent assessment](assessment-v2.json) verifies all twelve commands,
logs, source bindings, the executable and every counter row. Eleven
qualification commands pass: two calibrations, three site tests, eleven P4
controls, six cache controls, 879 all-feature scalar library results, 28
doctests and five endpoint regressions, plus formatting, strict four-crate
Clippy and warning-denied scalar docs. Focused counts overlap the library.

| Observed work | Calls |
| --- | ---: |
| All coefficient multiplications | 21,954,159 |
| Both operands have one radical and no opaque atoms | 6,986,607 |
| Of those, radicands compare equal | 6,379,817 |
| Equal-radicand outputs with no radical term | 3,778,231 |
| Equal-radicand outputs with one radical term | 2,601,586 |
| Single-radical inputs with unequal radicands | 606,790 |

The original instrumented board assertion fails at 139.129132266 worker-CPU
seconds against 120. Both complete reports are emitted with 12 and 197
findings. Instrumentation changes execution cost; this duration is not a
matched comparison or an estimate of an optimization's benefit. The counters
do not independently check non-unit or square-reduced radicands.

The [kernel review](next-kernel-review-v1.json) identifies a narrow next trial
from the unchanged endpoint/P4 parent. Preserve the original arithmetic order,
zero-skipping behavior and selected radicand payload: a zero left linear
contribution means ordinary insertion retains the right payload when its
contribution survives. Unit radicands and all nonmatching shapes retain the
original path. Reusing a consumed vector needs ownership, cancellation,
frontier, original-retry and P4 controls before fresh fixed scalar costs. Native
correctness, original gates, matched costs, complete reports and production
size remain further retention requirements.

A wording review clarifies that the entire plan remains incomplete and in
scope; already verified individual target statuses are unchanged. The initial
decision remains preserved, with every measurement, summary and next-step
decision identical in the reviewed version. The main runtime and all 35 pending
native paths remain unchanged.
