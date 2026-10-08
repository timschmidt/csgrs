# Finite analytic pair domains and point incidence

Status: committed Hypercurve `b518e5ff82ce2e6dd80c426aac0cb53236047d05`; qualified within the recorded scope.

Parent Hypercurve is `d74648a256a03494c247cb965158af09a49cae33`. HyperBREP is pinned to `878fcb441b281ab1b44051ca4420b28092c3c2ed`; all other dependencies reuse the committed-source manifest in `finite-parallel-pruning-isolated-sources.json`. The other session owns Hyperreal source and verification work. It is outside this session's edits, staging and commits. The working workspace is not claimed globally clean.

## Implemented scope

Ordinary retained parallel/rational and parallel/parallel intersections consume both finite parameter ranges through the existing domain selector. The same equation, root and component machinery serves finite pair queries and corner queries, with an explicit `ParameterComponentQuery2` selecting retained correspondence versus existence evidence. Native workers remain fast paths only after bounded exact proof that both ranges lie in their native domain. Optional proof failure escalates to the finite authority. Finite components keep their overlap correspondence and original charts.

Zero displacement reuses the rational source, including stationary and constant images. Rational PH images require the normal sheet selected on the actual regular range. Two rational images reuse the existing rational finite-pair authority, including complete constant-image parameter rectangles. The ordinary pair query does not acquire the off-diagonal self-query exclusion. Point publication retains the original rational support and parameter evidence rather than reconstructing a native Curve2 and rejecting an exterior parameter.

`BezierParallel2::point_incidence` and `contains_point` now require `&CurveParameterRange2`. The unit-only public signatures and private common-unit-polynomial-root helper are removed; controlled callers are migrated directly. Pole exclusion, source regularity, root clipping, constant-image branch selection and selected endpoint replay consume the actual range. Zero displacement needs no normal. Retained region membership uses this one finite incidence authority, eliminating two post-query clipping loops. The existing terminal Approximate512 decision behavior is preserved; root fallback uses strict predicate scope without replacing retained policy identity.

Source-cusp endpoints need one-sided frames; this finite pair prefix deliberately returns uncertainty there instead of silently omitting an undefined normal. The selected-circle/non-PH-parallel finite guard and the internal selected-point incidence queries are separate unfinished work. This change does not establish general operation closure or normalized public region construction.

## Independent evidence

The public pair fixture uses P(t)=(t,(t-2)^2), displacement -1/4, on [1,3], and the equivalent native chart P(2s+1). The other curve is vertical x=2, represented both as an ordinary rational span and as an offset of x=9/4. Its unique contact is independently (2,-1/4). Both charts, operand orders, reversals and policies give 128 checks, including exact contact and public location replay. The executed parent passed the 32 native/native cases and returned incomplete Unsupported for all 96 exterior cases. The frozen fixture SHA256 is `1454fbfaa47cf90e8312b20bc4f6d255d7e2145646ec96e8ccdae85071f71046`.

A separate public cap/rectangle Boolean probe checks explicit normalization, classification of interior/exterior/boundary points, exact boundary-path reentry, and normalization after reentry (20 checks). It is a successful composition regression, not a proved parent defect: no parent baseline was run for that fixture. Automatic normalization on raw construction remains unfinished, as recorded in the prior admission stage.

Three new unit tests cover transverse/tangent/disjoint contacts, non-PH component transport, and constant-image fibers/rectangles, including a rational source whose unused native interval crosses a denominator zero. A fourth integration test covers finite incidence with an excluded pole, opposite normal sheets, reversed ranges, and a stationary source outside the active range. The focused suite also retains ordinary native component tests and the Approximate512 terminal-decision regression.

## Attempts retained separately

- `finite-analytic-pair-domains-compile-attempt1/`: compile-only public fixture mistake, corrected before baseline execution.
- `finite-analytic-pair-domains-attempt1/`: first candidate built; two focus checks passed and public pair checks improved to 112/128. Sixteen exterior rational-first contacts still hit native point publication. Matching sources, manifests, artifacts and logs are retained.
- `finite-analytic-pair-domains-attempt2/`: all four builds passed; public pair 128/128 and region 20/20 passed. Five of six focused checks passed; constant-image fiber replay returned Boundary. This motivated the explicit-range incidence migration.
- `finite-analytic-pair-domains-compile-attempt3/`: all-target check caught a scripted extra range argument on Aabb2::contains_point. No release/runtime evidence was produced.
- `finite-analytic-pair-domains-attempt4/`: all four builds and both public probes passed; seven of eight focused checks passed. The new rational constant fixture used a forbidden zero constructor weight and failed before incidence execution. Its corrected version uses equal control points with nonzero weights 1,-1, with the pole excluded by the active range. The final candidate also routes rational constant rectangles through the existing rational authority.

No successful qualification writer is rerun on changed sources. Production and test sources remain frozen while an owned build/test/probe is live. Final qualification must bind the 396 working-source hashes, 1,005 isolated-source hashes, compiler artifacts, toolchain, all 49 broad targets and all 33 public matrices/probes. The five established failures, nine ignored tests and eight expensive exclusions remain explicitly reported. No general performance or memory improvement is claimed.

## Further premise audit

Before claiming all finite pair closure, execute the case where both ranges are contained in [0,1] but a source pole or source-speed zero exists elsewhere in the full native interval. `unit_domain_covers` proves query containment, not all preconditions of an optional native whole-source worker. Source inspection suggests an uncertain native worker may need finite-domain fallback. This is an unexecuted completeness hypothesis, not a reported regression or a reason to alter the current frozen qualification sources. A rational source with controls (0,0),(1,1), weights 1,-1, range [0,1/4], zero displacement, has the exact visit (-1/6,-1/6) at t=1/8; its excluded pole is t=1/2. An independent horizontal line gives a compact public baseline for the next stage.

## Final qualification

All 49 broad targets completed with **2,293 passes** (2,061 Hypercurve, 232 HyperBREP), the same five known failures, nine ignored tests, eight prior expensive exclusions, and no new failures or broad-run timeouts. All eight focused tests and all 33 public probes/matrices pass. Both repositories pass all-target no-default checks and all-feature release test builds. Formatting, git diff checking, the 396 working-source hashes and 1,005 isolated-source hashes verify. Public source, executable, library, libtest and toolchain hashes are bound by `finite-analytic-pair-domains-qualification.json`. The qualified normal library SHA256 is `d680fbce859f7d1e7f470916621fb407de5f877a836cacaf4fe3a7fa514000c0`; the libtest SHA256 is `3a7c5e04a2b0cfe6c133b4ca06723ba8fe6973d3938623704043371c37a2e334`. Matching artifacts are retained in `finite-analytic-pair-domains-libraries/`.

The architecture goal remains active. See `finite-circle-analytic-domains-next.md` and the native-worker premise audit above for the next bounded migrations.
