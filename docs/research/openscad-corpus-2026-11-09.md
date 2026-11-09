# OpenSCAD corpus comparison admission

The D03 comparison now has a maintained collector in
[`scripts/openscad-corpus`](../../scripts/openscad-corpus/README.md). The original
prototype collected all 524 pinned cases, with no skiplist. Collection is
complete; D03 functional conformance remains open.

## Contract and ownership

The comparison adopts the exact emitted decimal OBJ triangle model or the
closed SVG path boundary model. OBJ decimals enter through exact scalar parsing.
SVG paths use NonZero winding and exact reflection of the producer's Y axis;
viewport and unfilled presentation strokes do not supply geometry. The adapter
accepts the pinned producer's boundary convention, rejects unsupported XML,
painted paths, transforms, invalid paths and custom declarations, and delegates
region semantics to `CurveRegion2`. Hypermesh owns 3D admission and Boolean
decisions. There is no second arrangement or tolerance-based equality test.

SynapsCAD evaluates the original source with `preview=false` and its original
`fn/fa/fs` settings. Native regions and solids feed exact XOR comparisons.
Each successful comparison also consumes a union-with-empty result in a further
XOR identity. Construction flags, compiler diagnostics, failed parts and
operation-local certainty remain explicit. Neither successful reimport nor an
empty XOR establishes preservation of the original SCAD source or OpenSCAD's
internal floating geometry. No bound between those objects and the adopted
exports has been established.

An explicit producer-empty marker is distinct from missing, malformed, or
pathless geometry. Native process failures and wall timeouts override partial
receipts. Reference empty/dimension markers require a completed ordinary exit
of 0 or 1; other exits are `reference_process_failed`, even after a marker.
Missing certainty or continuation evidence cannot produce
a successful comparison. The owning contracts are P6, P8–P9, P17–P19, P25–P30
and P39. The original prototype's negative protocol witnesses preceded the
maintained implementation; strengthened tests reproduce missing-receipt,
malformed-receipt, crash-after-receipt, timeout, source mismatch, dimension and
certainty failures. Two new memory-limit witnesses failed before containment
was added and pass with it.

## Completed baseline

The original worker used CSGRS `29d8b8ce69`, SynapsCAD `1f1ec3e66d`,
openscad-rs `c0d92e6ea4`, and all original files at OpenSCAD
`8e8df3ddae295e8667cbc9f6c4fbb7c88b0ec45b`. The official portable OpenSCAD
2026.11.01 distribution was checksum-verified during preflight. Raw source,
driver and executable hashes and the terminal completion audit are retained.

Reference collection produced 365 emitted models (109 planar and 256 solid),
154 reported-empty outcomes and five parse failures. Sixty-four emitted
models carry producer diagnostics. Native observations are:

| Outcome | Cases |
| --- | ---: |
| Represented sets equal | 198 |
| Represented sets different | 188 |
| Native typed failure | 58 |
| Native process failure | 10 |
| Mixed dimensions | 6 |
| Comparison failure | 15 |
| Native external wall containment | 30 |
| Native evaluation or import failure | 19 |

All 524 paths were attempted exactly once and their bytes remained unchanged.
Among the equal outcomes, 28 have native warnings and 77 have reference
warnings. These observations cannot be collapsed into a source pass rate.
Every difference and noncompletion still needs causal classification.

## Resources and maintained controls

The original campaign used two workers and 300 seconds per engine per case.
It had no declared heap or total-work bound. One range fixture was killed after
using about 32 GiB RSS; its kill cause is unestablished. It is a process failure,
not evidence of typed exhaustion. The maintained collector now adds a default
4 GiB per-process virtual-address limit using an exec-side `prlimit` wrapper.
Descendants inherit it independently. Aggregate RSS, heap and total work remain
unbounded by this mechanism. Signals alone do not prove that a limit was hit.

Twenty-six Python protocol/inventory tests pass, including native and reference
workers that observe the imposed limit and fail a larger allocation. Nine Rust
worker tests, strict all-target Clippy and formatting passed for the maintained
worker. Eight geometry controls pass under the address-space limit, including
holes, empty results, different tessellation, invalid input, and an unequal set.
Paired real-engine cube, square and producer-empty controls also agree under that limit.
These controls qualify the maintained adapter; the original full-corpus counts
belong to the frozen prototype, not a fresh bounded campaign.

Before publication, a reference worker that printed the exact empty diagnostic
and then killed itself reproduced a false empty on `43b2dd6b0b` (exit 137).
Three maintained regressions failed for OBJ empty, SVG empty, and a dimension
marker before a crash. Exit validation now precedes those marker decisions.
Positive controls preserve ordinary exit 0/1 behavior in both export stages.
The partial pre-push run for `43b2dd6b0b` was stopped and retained; it does not
qualify the corrected source. The correction evidence is retained locally as
`docs/evidence/2026-11-09-reference-process-exit-correction`.

## Follow-up and limits

A two-level `children()` forwarding stack overflow and a cache that hid a later
invalid child were minimized against original cases. SynapsCAD's isolated local
repair at `c46634f4f8` passes nine acceptance witnesses, including caller lexical
scope, dynamic special variables, failed/empty children, allocation reuse and
generated-region continuation. Its configured CI still fails the unchanged
B-text deadline, so that downstream repair has not been pushed.

Original B-text phase probes complete in roughly 0.018 seconds for text,
113 seconds for round offset, 391 seconds through extrusion and 932 seconds
through scene validation and comparison. They ran amid other checks: these
are attribution observations, not controlled speedup measurements. The original
font, size, offset, subdivision and height remain unchanged. General algebraic
closure, unsupported modeling operations, recursion limits, encoding cases,
runtime asset attribution and the remaining corpus outcomes are still open.

Raw campaign receipts, original hard cases, minimized controls, memory tests,
source snapshots and the local downstream repair are retained in the local
evidence record `docs/evidence/2026-11-09-openscad-corpus-and-indexed-closure`.
The [archive index](../evidence-archive.md) records its digest and availability.
