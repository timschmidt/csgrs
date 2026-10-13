This diagnostic will count multiplication of two single-radical coefficients
with the same radicand. Inputs are prepared; qualification and full-board
coverage remain pending. No runtime optimization is prepared or admitted.

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

The original complete boards will run once from a bound executable, retaining
the 120-second worker-CPU assertion. The report prints after the assertion,
including unwinding. Finding counts are not complete report-byte comparisons.
The 60-second copper-neck assertion remains unchanged but is not exercised by
this coverage run. The entire properties, target and future-work plan remains
open.

The uncompiled mutable draft exposed two macro line-wrap differences between
standalone and crate-level rustfmt settings. The [draft review](draft-format-review-v1.json)
records the correction before frozen preparation. The prepared input files now
match the crate formatting; frozen preparation checks formatting without
editing its generated source.
