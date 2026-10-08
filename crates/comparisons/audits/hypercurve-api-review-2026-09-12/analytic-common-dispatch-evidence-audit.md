# Common analytic intersection dispatch

Committed Hypercurve `6b05bf1fcebfe49442e47c5aff55facca5ba4f38`. Qualified against the documented regression baseline; the original architecture goal remains active.

## Implemented behavior

The common curve/path dispatcher now consumes the existing exact parallel/rational, parallel/chord, and parallel/parallel kernels. It transports the original selected parameter-component authority, clips both operand domains, preserves operand order and traversal signs, and replays zero-dimensional contacts when two closed overlap domains merely touch. An incomplete lower-kernel result remains incomplete with its shared `CurveIntersectionCandidates2` evidence.

Point-image parameter components are retained separately from isolated contacts and positive-length geometric overlaps. `CurveParameterSet2` describes either one parameter or one nondegenerate closed range. A component retains the full Cartesian parameter set and the shared exact point. Result clones share these components; ordinary intersection results do not allocate a component slice. Curve and path topology cut the nonconstant operand at a fixed component parameter, and retain all component boundary parameters. This distinction avoids widening each ordinary contact with higher-dimensional parameter evidence.

A free parameter range is not published across an authored rational denominator zero. Homogeneous controls can represent such an authored pole even when the image away from it is constant. The finite-range denominator test preserves the original domain; a pole-free restriction remains usable. The new result does not introduce open or punctured parameter domains.

## Consolidation and proofs

The ordinary-only analytic point evaluator was removed. The existing selected-parameter evaluator now supplies `point_evidence_on_regular_range` for every caller. Both regular-range intersection kernels accept `CurveParameterRange2` directly, eliminating region fallbacks that discarded selected range endpoints. Containment now requests exact parameter ordering, rather than demanding a scalar envelope that geometric chord/circle parameters do not need.

The source-diagonal overlap test now distinguishes a shared closed endpoint from disjoint ranges. A stationary source can have equal or opposite one-sided normal limits at the same parameter. Both endpoint images are replayed before selecting that structural component; a source parameter alone is not an incidence certificate. Source reversal uses the retained affine parameter chart.

Nonlinear component transport reuses `BezierParameterComponentOverlap2` and the shared `CurveOverlapCorrespondence2`. Swapped rational correspondence moves operand roles while preserving the kernel's original map ranges. No compatibility aliases or wrappers were added.

## Validation records

The preserved `analytic-common-dispatch-baseline.rs` and its original executable demonstrate twelve incomplete common queries, while their lower kernels complete. `analytic-common-dispatch-completed.rs` repeats those public queries with completion, contact replay, path intersection, and topology assertions. The baseline source and executable are not overwritten.

Eight focused tests cover both policies and operand orders; traversal reversal; clipped, disjoint and singleton overlaps; distinct source charts; nonlinear maps with retained selected parameters; stationary source endpoints; point-image rectangles; path cuts and clone sharing; and authored denominator poles. Attempt 5 failed because its fixture used a zero-weight-rejecting convenience constructor. Attempt 6 uses the intended homogeneous constructor and passes all eight tests in 3.92 seconds. The final frozen sources additionally test rational-quadratic denominator poles and reversed stationary source charts.

Final qualification records are in `analytic-common-dispatch-qualification.json`: 49 targets, 2,218 passes (1,986 Hypercurve and 232 HyperBREP), the same five known failures, nine ignored tests, eight unchanged expensive exclusions, and no new failures or timeouts. All-targets no-default-feature checks and locked offline fuzz/UI caller checks pass. All 396 frozen source hashes match, formatting/whitespace checks pass, and all 30 repositories are clean after the commit. The public success reproducer completes all twelve queries with 32 point replays; all five previous public regression probes/matrices pass against the recorded final normal library. No throughput, allocation, or memory improvement is claimed from this migration.

## Remaining original-goal work

This slice does not establish complete exterior/retraced-domain transport or unify all native degenerate intersection dispatch. Region report publication still discards non-native overlap correspondence. Normalized public region construction, the documented five failing promotion tests and eight expensive unqualified cases, broader algebraic authority consolidation, and computational-closure measurement remain open. The goal must not be marked complete for this slice.
