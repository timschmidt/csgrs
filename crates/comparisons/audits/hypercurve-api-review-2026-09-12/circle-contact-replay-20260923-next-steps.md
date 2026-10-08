# Current continuation: retained probe endpoint evidence

The full implementation goal remains active. Other-session Hyperreal work is untouched and was not a build input. All owned processes are terminal/reaped. No source freeze remains. No agents are authorized or in use. The user authorizes incremental commits, with no push.

## Committed state

Hypercurve HEAD is d00fa186b720f4224e512d9185bc859c624b4608, "Align circle parameter maps and cancel inverse transports". Only src/bezier_offset.rs was committed, SHA256 cbdb8cfc30d03362c3e3b723a57bad3bdd322ec3e7b019f959eb4866ca92e871. Its clean final qualification has 1,192 passes, six ignored cases, nine unchanged assertions and nine unchanged 75-second limits. Both all-target feature checks pass. No new or changed nonpass. Qualified executable SHA256 is 97756af7df908515bf2620c17453a45c33e68bac8490e2ec706c5c051a29715f. Session 89276 is reaped. Qualification/staged/HEAD bindings are in circle-map-direction-20260923-{qualification,staged,post-commit}; implementation.md and the proof note record the commit.

Only main src/bezier_region.rs is dirty: the unqualified mandatory-normalization candidate, SHA256 1bc26463270fbe520873708bc6909db19a7af36356367f8bbc366b92696ed720. Do not stage this until its predicate, caller and performance regressions are resolved. Main offset source exactly matches d00fa18; neither subsequent fallback trial remains there.

Hypersolve remains clean at e4f59ba19e01c224c9fdc37f44debb752b6d40ae. Continue using the pinned dependency snapshots; never build the live Hyperreal worktree. Build settings and shared target paths are unchanged from the preceding continuation.

## Exact remaining failure and rejected trials

The normalized fixture major_retained_rational_arc_and_general_chord_share_the_fillet_kernel fails for STRICT, forward, not elevated, TrimOrExtend. All 17 exterior boundary probes fail on pair (0,3): the probe Line against its own RationalBezier boundary, AlgebraicChordPair context. Carrier 2 owns materialized Bezier support; the representative is a scalar Point2. This occurs before winding accumulation. Trace 5 locates the returned blocker in algebraic_chord_rational_pair_result (PAIR_UNCERTAIN:2851).

Trace 6, session 27518, binary f21fa912f703e4a794a6e4333775309ed785ff0cc349669a50e071dcf5bc0924, finishes with the expected failure in 2.51 seconds. The actual branch is exact_line_retained_circle_intersections -> parameter_at_certified_point. Its lower order is Greater; its upper order is Uncertain(Ordering). Candidate point, probe start and probe end are all scalar Point2 variants. Retained endpoint identity cannot prove the equality. Labels are CHORD_RATIONAL:78224 and CHORD_RATIONAL:72724.

Trace 7, session 65248, binary c4975192e7ba27f50e22cf4858ef98e1261369e0a1e2dd423deccebb475aaf54, tries only certified STRICT circle-accelerator results, otherwise ordinary rational/line contacts. It fails at the same finite endpoint comparison, now through CHORD_RATIONAL:79630. Both paths reconstructed scalar points. This trial was briefly in main and then fully reverted; it is not a fix.

Trace 8, session 57246, binary 61c83a3e1f99783d281828e2597097649bb140e22d5a7ac743b57a359911a41b, treats the entire represented-line kernel as an optional STRICT accelerator and falls through to recursive_projective_rational_intersections. It too fails: CHORD_RATIONAL_RECURSIVE stage=finite-chord-parameter, with the same scalar point kinds and endpoint Ordering. It ran only in the diagnostic root. No fallback trial should be committed or rerun without a new evidence-preservation change.

All three runs are terminal/reaped, sources unchanged during each run, with 60-second bounds. Their diagnostic timings are not performance comparisons. Summaries, sources and binaries use corner-publication-decision-sites-20260923-trace{6,7,8}. The diagnostic root /tmp/hypercurve-corner-publication-decision-sites-2026-09-23 currently contains trace 8, including earlier trace instrumentation. Never copy its source files into main. circle-contact-replay-20260923-trials.json verifies main restoration and reaping.

## Next bounded implementation task

First reduce this to a small independent contact regression. A useful candidate is a retained quarter circle, a point evaluated at a known interior source parameter, and a chord from the circle center to that point. The finite radial chord has exactly one endpoint contact with known source parameter and tangent sign. Exercise rational and irrational exact parameters and translated/irrational circle frames; do not require a particular internal variant as the semantic assertion. Reproduce the failure on source-bound d00fa18 before using it to qualify a correction. A diagnostic test can print only safe result statuses, scalar parameter categories and retained evidence identity.

Then preserve the construction identity that the current probe loses. In curve_region_boolean.rs, regularized_fragment_geometric_decision (~8590) uses local_circular_curve.point_at(1/2) for a materialized circular fragment and returns parameter=None. It stores only the Point2 in boundary_probe_representative (~8821), then passes CurvePoint2::from(representative) to regularized_fragment_decision_by_boundary_probe (~8859). The original local curve/chart and its half parameter are absent from the probe endpoint. The local circular chart can differ from the boundary's original source chart; do not assume affine parameter equality or skip contact discovery. The complete finite set of contacts, including any residual circle contact, must remain certified.

Existing mechanisms worth reusing, without adding public or compatibility interfaces:

- RationalBezierOverlapParameterCorrespondence2::map_parameter_between_curves can retain the exact correspondence across rational source charts. The adjacent-boundary helper algebraic_chord_shared_image_endpoint_pair_result currently relies on authored adjacent region carriers and therefore does not directly provide probe endpoint provenance.
- source_related_intersections and source_incidence_system use a retained endpoint field and exact diagonal-factor deflation. Their current algebraic endpoint importer can erase useful source evidence: algebraic_chord_endpoint_images (~86257) normalizes an Algebraic image to scalar coordinates whenever exact_point succeeds. Any repair must retain existing successful independent-field fallbacks rather than universally forcing unrelated endpoints into one field.
- RationalBezierAlgebraicPointImage2::from_parametric_source retains the source and parameter. An exact scalar can have a linear defining polynomial, but inventing a selected-root wrapper alone is insufficient if later normalization discards it. Do not add synthetic wrappers without demonstrating reuse through the whole contact path.
- recursive_projective_polynomial_parameters_with_crossing (~64262) creates direct low-degree root evidence, then scalar.exact_real_value() can project it back to BezierParameter2::Exact (~64410). That preserves scalar meaning but can lose a useful construction certificate. Likewise rational_point_evidence_at_region_parameter materializes ordinary Exact parameters. Audit where the needed proof should remain authoritative; avoid arbitrary rational-payload admission guards or a proliferation of point variants.

The two failed fallback trials show that dispatch alone is insufficient. The next change must carry and consume actual endpoint/chart/root evidence, or otherwise prove the finite endpoint identity. Never infer equality from overlapping isolators, approximation, or a presumed geometric incidence. The earlier canonical-companion bug demonstrated why a shortcut that merely skips this proof is unsafe.

## Remaining normalization work

The direction-aware map correction resolves the smooth-seam chamfer blocker in the clean normalized focused run. The major-arc endpoint replay blocker remains. Six tests still assume one raw boundary or traversal direction; migrate them across every normalized boundary while retaining geometric, tangent and contact assertions. The selected_circle_direct_line_fillet_cut helper (~22853) uses ascending support endpoints, independent of fragment traversal, but must still select the correct source cut after normalization. Five additional 75-second limits require complete-workload performance qualification. The prior broad normalization run and exact names remain in circle-map-direction-20260923-next-steps.md and earlier publication reports.

Source cusps, duplicate/XOR publication, shared source-bound scheduling, point/location evidence consolidation and held-out analytic/PCB/DRC qualification remain part of the active larger goal. No full closure claim is warranted.
