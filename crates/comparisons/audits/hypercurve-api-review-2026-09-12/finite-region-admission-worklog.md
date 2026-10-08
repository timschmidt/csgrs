# Finite region admission

Parent: Hypercurve `df5e6a9a7376edb8106bde381dbbbdcdbe01d9ed`; HyperBREP `878fcb441b281ab1b44051ca4420b28092c3c2ed`.

The parent passes the finite circle/rational curve matrix but fails the same composition at public region intersection with `Blocked(Boolean, Line, Unsupported)`. Explicit unary regularization repairs the unit-chart case, then fails on the cap retained over `[1,2]` with `InvalidBezierParameter` during splitting. Both failures have preserved public probes and matching library hashes.

This step makes region intersection use the existing regularized operand preparation used by Boolean operations. Raw input winding, internal seams, and canceled loops are resolved before contacts are reported. Completed regions reuse their existing certificates and regularization cache. Reported carrier curves remain the parameter replay authority; indices refer to the operation's prepared operand. Curve trimming still retains its authored boundary provenance.

The internal Bezier splitter now receives an explicit finite ordered range. Four repeated per-family split dispatches become one call through the existing subcurve and lazy endpoint-image methods. Native represented intervals retain their compact kernels; exterior represented intervals use the existing exact affine restriction. Algebraic intervals retain their source chart and first-order endpoint images. The region splitter no longer splits the whole source and discards fragments outside its range. Controlled native callers explicitly pass their unit range; there is no compatibility wrapper.

A circular compaction guard preserves exterior retained charts. A native circular span's minor-arc certificate does not justify replacing an exterior major arc by the minor arc between the same endpoint coordinates. The regression retains algebraic endpoints at plus/minus sqrt(2), exact endpoint contact evidence, and both traversal directions.

Evidence history:

- Attempt1 compiled the changed sources but linked an older named normal-library artifact for its two probes. Those outputs are baseline rechecks, not tests of the changes. `finite-region-admission-attempt1-linking-note.md` records the mistake.
- Attempt2 explicitly linked `target/release/libhypercurve.rlib`, SHA `d3e368f9476d71b64c7dd4366cce32bb96bb681508c7e02933f20bc022f4e146`. Both original public matrices passed: each had 48 curve queries, 96 point replays, 48 topology queries, 96 splits, 192 child queries, 12 region queries, and 12 Boolean reentries. No production or probe sources changed during either run.
- The new region regressions passed before the circular guard. Their tests additionally require complete certified reports and replay every reported contact and overlap after Boolean reentry.
- The pre-guard build metadata, sources, patch, normal library, and relevant test binaries are archived in `finite-region-admission-attempt2-before-circle-guard/`.
- Final sources, including the circular guard and regression, are frozen in `finite-region-admission-source-before-tests.json`. Qualification is in progress; do not treat this note as a completed validation record.

Outstanding finite-domain audit, separate from this step:

- `CurveSupport2` bounds and injectivity methods do not accept the active range. Several region decisions therefore still consult native-unit certificates for retained exterior carriers.
- `retained_fragment_query_bounds` and selected-fiber conservative bounds also reuse native source boxes. For `P(t)=(-(t+2)^2,t+2)` on `[-2,-1]`, that box describes the wrong image. Add a public containment regression before repair.
- Unary and cross-operand Bezier pair preparation constructs `Curve2::from(source)` without the retained domain. Self-intersection caches are likewise keyed only by the native source.
- Retained ray winding computes a native control-hull shortcut and unit-root contacts before clipping to the active range.
- Reuse the existing represented pole-free finite-envelope authority for scheduling, while preserving original selected boundaries for admission. Continue the common support/range migration; do not infer finite closure from the repaired cap alone.

The original architecture goal remains active, including normalized public region construction, finite analytic pair discovery, independent circle inverse replay, five known promotion failures, eight previously unqualified expensive tests, and broader algebraic/computational consolidation. No general performance claim is made by this step.

Further semantic regression found during full qualification:

- Attempt3 had 2,261 passes, the five known failures, and one new failure: `noninjective_endpoint_preimages_complete_public_region_intersection`. Its source region includes an out-and-back quadratic spur on the x axis. The old test expected two overlaps and four contacts on that zero-area spur; normalized filled-set boundaries must omit it. Unary regularization blocked on the retracing self-component instead.
- The complete attempt3 metadata/logs, five changed source files, patch, normal library (SHA `717c6e5b55eee71e193278c95bb7ecb769c556205fec5d1b330fc6cdcd5932d2`), and failing libtest are preserved in `finite-region-admission-attempt3-retraced-spur/`.
- The repair removes a native finite Bezier traversal when exact control symmetry proves it equal to its reverse. Its oriented winding contribution is identically zero, including non-collinear retracing. Rational cases require a nonzero denominator on the complete unit range. This reduction applies only to unary filled-region arrangement; ordinary curve intersection still retains all parameter visits.
- The migrated regression requires the original two overlaps and four preimage contacts through `Curve2::intersect_curve`, then requires no contacts/overlaps between the regularized regions and an empty Boolean intersection. It also checks that the four rectangle boundary edges survive. The fixture's rectangle had been declared filled-left despite clockwise traversal; its side is corrected to filled-right. A separate regression covers polynomial, rational quadratic, and genuinely curved rational symmetric retracing, including an operand whose entire boundary cancels.
- These changes are in a new frozen qualification run. No production, test, or public-probe sources were edited while any owned build or test process was running.

Attempt4 corrected a test-scope mistake, without changing the production repair: the initial migrated curve query compared only the horizontal chord with the retraced source. That query correctly produced two overlaps and zero isolated contacts. The original region report's four contacts came from the triangle's two closing edges. The final test queries all three authored triangle curves, keeps the two-overlap/four-contact assertions, and replays every contact parameter and overlap endpoint. Attempt4 build/focused metadata, source, patch, normal library and libtest are archived in `finite-region-admission-attempt4-curve-test-scope/`. The independent symmetric-retracing, major-arc, seam-removal and fillet-composition regressions passed on attempt4.


## Finite region admission and splitting — adb4756b7e9c

Committed Hypercurve `adb4756b7e9cf4adcb81f1f3d0d50461e5f9f13b`. Region intersection shares Boolean operand regularization, so internal seams and canceled traversals do not appear as filled-boundary contacts. The private Bezier splitter accepts the actual finite range and consolidates four family dispatches. Exterior circular fragments retain their major-arc chart and endpoint evidence. Exact symmetric finite retracing is removed from unary filled arrangements while ordinary curve queries preserve all preimages. Controlled callers and provenance documentation are updated without a compatibility interface.

Final qualification has **2,263 passes** across 49 targets (2,031 Hypercurve, 232 HyperBREP), five unchanged known failures, nine ignored tests, eight existing expensive exclusions, and no new failures or timeouts. All 23 public probes/matrices, five focused tests, caller checks, formatting and 396 frozen source hashes pass. All thirty repositories are clean after commit. See [finite-region-admission-worklog.md](finite-region-admission-worklog.md), [finite-region-admission-qualification.json](finite-region-admission-qualification.json) and [finite-region-admission-post-commit.json](finite-region-admission-post-commit.json). Failed attempts are preserved separately. No general performance or memory improvement is claimed.

The original architecture goal remains active. Active finite-domain bounds, preparation and winding are next; normalized public construction, remaining analytic pair domains, independent inverse replay, known failures/exclusions and broader algebraic/computational consolidation remain unfinished.
