# Finish normalized public region publication

Continue after qualifying and committing the downstream callers for Hypercurve `9f7a636`.

The remaining public low-level factories are `CurveRegion2::new`, `try_new_with_loop_topology`, and the retained arrangement/linear-overlap/rational-overlap traversal factories in `src/bezier_region.rs`. Their internal raw counterparts remain useful construction authorities; public regions must not publish those intermediate states. Prefer consolidating public construction around closed `CurvePath2` values and explicit authored loop semantics, while keeping selected parameters, fragment provenance, and interior-side evidence intact. Do not lose exact procedural curves merely to remove an entry point.

External production consumers found by workspace search are HyperBREP `src/builder.rs:1972` and HyperDRC `src/lib.rs:589`. Hypercurve's integration tests, benches and fuzz target also construct retained loops directly. Any removal must migrate all of them and requalify those consumers. Historical audit snapshots are immutable evidence, not callers to update.

Raw operation producers to inspect include corner replacement (`replace_loop`), certified segmentation, exact offset band construction, affine transform certificate propagation, and XOR's union-minus-intersection fallback. A valid filled-left arrangement certificate may publish directly; an authored side hint alone cannot certify that crossing or coincident boundaries have been removed. Preserve approximate-decision dependencies when replaying or caching topology.

Do not remove public re-regularization or its cache until every public producer satisfies the invariant. The full normalization migration must also remove redundant state and traversal adapters that are no longer needed, rather than leaving deprecated wrappers.

Before the next implementation, bind another public counterexample to the committed library for retained-loop cancellation or filled seams. Keep authored raw-input tests inside the crate and strengthen public tests around exact sets, boundary ownership, and repeated operation closure. The current `src/bezier_offset.rs` working candidate still has a required compact normal-sheet inverse failure and remains separate.

## Progress after arrangement and component consolidation

`28fd4d6` replaces the three traversal entry points with normalized `try_from_arrangement_traversal`; overlap clients pass refined graph and traversal directly. The public overlap and self-crossing probe now passes. The newer component extraction slice moves HyperBREP/HyperDRC off manual raw-loop reconstruction; see `material-components-plan.md` and its qualification records when complete.

The raw-loop counterexample remains `normalized-retained-admission-public.rs`, bound to the committed `2d26b0a` normal library. All eight cases failed publication invariants and passed explicit normalization. This is still the acceptance target for removing `new` and `try_new_with_loop_topology` from public construction.

Avoid adding a new legacy-loop wrapper merely to hide those constructors. `Curve2::from_retained_fragment` is currently crate-private, and public `BezierSplitFragment2` can contain unchecked authored fields. A blanket public `From<BezierSplitFragment2>` would need a careful validity contract. Prefer authoring through generic curves/subcurves and validated support types, or consolidate region storage with the already general `CurvePath2`. Retain graph connectivity/provenance directly when the arrangement authority supplies it; reconstructing independent endpoint equalities can lose selected-field proofs.

The follow-up producer audit confirms `regularized_exact_offset_band_arrangement` already normalizes its internal band before publication. Its raw-loop builder is therefore an internal construction stage, not by itself a publication defect. `with_corner_chain_replaced` still returns raw loop topology, and `compose_xor_from_exact_regions` still concatenates union/intersection boundaries without normalization when difference traversal fails. Those remain concrete publication targets. Inspect complete offset and segmentation return paths before assuming every occurrence of the raw builder leaks a region.

## General-curve authoring now available

Hypercurve `1f77b66` converts validated generated chords, analytic parallels, selected circles, and selected fibers directly into `Curve2`. Three open-path fixtures no longer fabricate a closed region just to extract a curve. Both affected integration targets passed (63 tests); see `direct-generated-curves-qualification.json`.

Hypercurve `61a9af2` adds `Curve2::try_from_bezier_range(source, CurveParameterRange2, policy)`. This is finite source-chart authoring, distinct from restricting an already finite curve. It supports exterior intervals, ascending or descending traversal, and ordinary or selected-fiber parameters. Rational finiteness is checked on the retained interval. The endpoint evaluator and selected restriction share a prepared rational source; selected scalars are not reconstructed. Three Boolean fixtures now use general paths and normalized admission. Three new range regressions and all 37 Boolean tests pass; see `finite-bezier-curves-qualification.json`.

Hypercurve `06733b0` moves three spline tests, two Bezier-region benchmark fixtures, and the region fuzz constructor to normalized path admission. All 24 spline tests pass. All-target checking, release test/benchmark builds, and the affected fuzz target check pass. The benchmark executable was compiled but its full timing workload was not run. See `normalized-spline-region-callers-qualification.json`.

The remaining raw public call sites are recorded in `raw-region-callers-after-finite-curves.json`. No raw constructor has been hidden yet: migrate all remaining clients before making the builders crate-private. Preserve low-level provenance-validation tests in crate-private unit scope instead of deleting them. Public tests should assert the normalized set rather than authored fragment indices or clockwise area signs.

The editing benchmark includes a retraced chord paired with a noninjective collinear Bezier. Regularized regions discard those lower-dimensional traces. Check whether each such fixture is intended to measure region cancellation or open-path incidence, and keep the appropriate geometric workload during migration; do not silently substitute an empty-region fast path for an incidence benchmark.

The isolated qualification tree now matches committed Hypercurve `06733b0` except that `src/bezier_offset.rs` remains the committed baseline. Its mapped-point inverse working candidate remains unqualified and excluded. Hyperreal working changes belong to another session; the pinned `a2da8e2b5de9a1a4653d3662f2f05cb6533fd7ef` snapshot was used throughout. All process/session handles from these three slices have been reaped.

## Raw public factory migration completed

Hypercurve `e62b786` migrated the remaining benchmark fixtures and retained open-path incidence for the retraced noninjective case. `c1eb639` migrated six analytic-region fixtures (13 passing tests; two 300-second parent/candidate timeouts remain unresolved). `9aab7d2` migrated fourteen promotion fixtures (97 passing tests; five identical parent/candidate failures remain).

Hypercurve `07d6b30` makes `new` and `try_new_with_loop_topology` crate-private after migrating the last five integration call sites. Three malformed-loop validation tests moved into unit scope. General selected-fiber authoring exposed and fixed missing retained-geometry classification and exact native-line query dispatch. The native-line shortcut requires source monotonicity, a covered finite range, and represented endpoint witnesses; an irrational exact-Real endpoint test and a nonmonotone restriction counterexample qualify its guard. All existing area and native-line assertions remain.

Qualification: 1,384 unique passing tests; all eight public admission counterexamples and sixteen membership checks pass; Hypercurve and four consumers pass all-target no-default checks; the region fuzz target checks. Five promotion failures remain identical to their baseline. Four expensive library cases were held out and six library tests remain ignored. See `private-region-factories-qualification.json`. The analytic stale-build attempt was rejected and replaced with verified fresh executables; see `normalized-analytic-region-candidate2-rejection.json`.

All process handles from these slices have been reaped. The isolated qualification tree matches these committed changes, with the unqualified working `src/bezier_offset.rs` excluded and Hyperreal pinned. Continue with `normalized-operation-publication-next.md`: corner replacement and the XOR fallback are still raw publication paths, so public re-regularization and its cache remain necessary.
