# Current implementation continuation

**Superseded live state:** all sessions below are terminal/reaped. Read [circle-contact-replay-20260923-next-steps.md](circle-contact-replay-20260923-next-steps.md) for the current committed state, rejected fallback trials and next evidence-preservation task. Main has only the unqualified normalization change.

## Current state: map correction committed; trace 6 active

Hypercurve HEAD is d00fa186b720f4224e512d9185bc859c624b4608. Session 89276 is terminal/reaped. Its full qualification is 1,192 pass, six ignored, nine unchanged assertions and nine unchanged 75-second limits; both all-target feature checks pass. Source, staged and HEAD bytes are bound in circle-map-direction-20260923-{qualification,staged,post-commit}. Main has only the unqualified src/bezier_region.rs change (SHA256 1bc26463270fbe520873708bc6909db19a7af36356367f8bbc366b92696ed720).

Session 27518 is ACTIVE: run-corner-publication-decision-sites6-20260923.py, diagnostic root /tmp/hypercurve-corner-publication-decision-sites-2026-09-23, prefix corner-publication-decision-sites-20260923-trace6. It adds 24 narrow uncertainty labels in chord/rational replay, source-related replay, retained-circle replay and finite certified-point admission, plus recursive replay stage labels and safe point-kind/finite-order summaries. It runs only the major-arc normalized fixture with a 60-second bound. All owned production/test/probe sources are frozen until this exact session is reaped. No diagnostic code is in main.

Inspect trace 6 labels after completion, then implement the identified predicate correction. The full goal remains active. Other-session Hyperreal work remains untouched and unused.

## Updated live status after trace5

Session92134 is terminal/reaped. Trace5 binary6c8a776fc2f82a75c11ed71de8758648433de5b81129cb7c01e76864832b7853 confirms all17 failing boundary probes contain the same Ordering blocker in pair(0,3): probe Line against boundary RationalBezier, AlgebraicChordPair context. The failure is returned by algebraic_chord_rational_pair_result before contact clipping (PAIR_UNCERTAIN:2851). The next diagnostic instruments the chord/rational replay and finite support-point admission directly; its inactive preparation script is prepare-corner-publication-decision-sites6-20260923.py. Do not execute it until session89276 is reaped.

Session89276 remains active. Final production binary97756af7df908515bf2620c17453a45c33e68bac8490e2ec706c5c051a29715f built freshly; both all-target feature checks pass. The full1216-case run continues with no source edits. All owned source inputs remain frozen. Only src/bezier_offset.rs is eligible for the forthcoming direction-aware-map commit, after terminal comparison succeeds.

## Latest live status (supersedes live-handle paragraphs below)

Sessions16239 and16708 are terminal/reaped. Clean focused1 binary b0ce87c55063663f5a19c479d9485dbbaabe28b56006c553f47ee8b55bf61e5b passes all three cases: the expanded reconstruction/inverse-identity regression, the full normalized chamfer seam regression (all policies/traversals/reframings/modes,1.80s test runtime), and the existing full/partial/endpoint overlap regression. The exact source bytes remain in /tmp/hypercurve-circle-map-direction-2026-09-23 and its focused1 manifest.

Trace4 completes in2.38s and shows carrier2 (RationalBezier, materialized Bezier support, represented Point2) exhausting all17 probes at evidence.blockers(), before winding accumulation. Its only branch label is BOUNDARY_PROBE:9553. No early ExactCurveError blocker is created; the uncertainty is retained in pair-result evidence. Earlier carriers1 normalize far enough to produce probe starts without this failure.

Two NEW handles are active, and ALL owned source inputs remain frozen until BOTH are reaped:
- Session92134: run-corner-publication-decision-sites5-20260923.py. Same diagnostic root; adds concise pair indices/families/context discriminants in build_intersection_evidence and13 labeled RegionPairBlocker::Uncertain(reason) constructors. Prefix corner-publication-decision-sites-20260923-trace5; branch map corner-publication-decision-sites5-20260923-branches.json. Runs only the major-arc fixture,60-second bound.
- Session89276: qualify-circle-map-direction-20260923.py. Clean root /tmp/hypercurve-circle-map-direction-final-2026-09-23 contains905e2bc plus ONLY the current src/bezier_offset.rs, without mandatory normalization. Prefix circle-map-direction-20260923-final1. Builds fresh release libtest, checks both all-target feature configurations, and runs all1,216 cases against the source-bound905e2bc final1 parent. No new named tests; the reconstruction regression is extended. Final source hashes are in circle-map-direction-20260923-final-candidate.json.

After these handles finish, inspect the narrow pair diagnostics and the full qualification report. Stage only src/bezier_offset.rs if qualified; preserve the unqualified region file. No other-session Hyperreal edits are used. Temporary storage has about15GiB free; no additional cache cleanup was needed.

---

The goal remains active. Another session owns Hyperreal working changes: never edit, stage, commit or build that worktree. All current runners use immutable pinned dependency snapshots. No agents are authorized or in use. User authorizes incremental commits, no push.

## Committed and qualified

Hypercurve HEAD905e2bc74e2ba01a4dcf0b3e7fbee8d854383949 removes the mixed-Bernstein-weight hull restriction from positive endpoint-projective overlap replay and reuses rebuilt selected-circle overlap certificates on the same ordered charts. Two independent-value regressions pass. Both all-target feature checks pass. All1,216 library cases were attempted:1,192pass,6ignored,9unchanged assertions,9unchanged75-second limits; no new or changed nonpasses. Qualification and staged/HEAD bindings are in corner-overlap-authority-20260923-{final1,qualification,staged,post-commit}. The qualified library SHA256 is78e4fbbb4abf317b177160b116e3a4711b5bf727e5ce2527875ddbdf287cab69. All corresponding processes are terminal/reaped, including95573 and diagnostic68806.

Only the unqualified normalization file src/bezier_region.rs remained dirty after that commit, SHA2561bc26463270fbe520873708bc6909db19a7af36356367f8bbc366b92696ed720. The previous commits deeed31 (canonical companion chart) and fc0197e (all-boundary public composition caller) remain qualified. Hypersolve e4f59ba remains clean.

## Current source change and live handles

Main additionally modifies src/bezier_offset.rs. The private shares_parameter_authority helper is replaced by shares_parameter_map(source_first,other,other_source_first), and all six callers are updated. It aligns exact source and destination charts independently of pair enumeration order, retaining orientation and policy checks. Boundary labels no longer constrain map identity; endpoint-only PairOverlap evidence still compares ordered charts and matching endpoint labels. Certified inverse cancellation now precedes the full-overlap fast path, preventing repeated unit-complement wrappers. The existing reconstruction regression now tests swapped operands and eight forward/inverse full-circle transports with original Arc identity retained. Unrelated rustfmt-only hunks were removed before snapshots.

- Session16239: run-circle-map-direction-20260923.py, root /tmp/hypercurve-circle-map-direction-2026-09-23, prefix circle-map-direction-20260923-focused1. Clean source plus mandatory normalization; builds a fresh all-feature release libtest and runs the expanded reconstruction regression, selected_circle_chamfer_crosses_one_sided_smooth_run_seam, and algebraic_cusp_semicircle_pair_maps_full_partial_and_endpoint_only_overlap. Bounds60/75/75seconds.
- Session16708: run-corner-publication-decision-sites4-20260923.py, existing diagnostic root /tmp/hypercurve-corner-publication-decision-sites-2026-09-23, prefix corner-publication-decision-sites-20260923-trace4. Contains current map correction, earlier caller/backtrace instrumentation, and15 narrow last_reason labels inside regularized_fragment_decision_by_boundary_probe. Runs only major_retained_rational_arc_and_general_chord_share_the_fillet_kernel,60-second bound. Branch map:corner-publication-decision-sites4-20260923-branches.json.

ALL owned production/test/probe source inputs are frozen until BOTH exact handles are reaped, even across roots. Audit Markdown and inactive runners can be edited. Do not dump full geometry or use HYPERCURVE_DEBUG_RATIONAL_BLOCKER; its earlier224MB log measured formatting overhead and was terminated/reaped.

## What the preceding diagnostics established

The normalized major-arc fixture formerly failed at the mixed-weight guard. With905e2bc it completes contact discovery and split topology, then exhausts boundary-side probes with Ordering. Trace3 backtrace identifies regularized_fragment_decision_by_boundary_probe, called from regularized_fragment_geometric_decision and regularized_fragment_actions. Trace4 will locate the actual comparison/replay failure among those candidates.

The normalized independently reframed chamfer clears its forward case with905e2bc, then fails in reversed TrimOrExtend. Trace3 compares PairOverlapMap(true,Exact) against PairOverlapMap(false,Exact): source evidence matches; ordered chart identity(false,false); swapped chart identity(true,true); orientations(Same,Same); policies equal. The current direction-aware patch addresses exactly this missing correspondence identity.

## Next actions after reaping

Inspect focused cases and concise DECISION_SITE/BOUNDARY_PROBE_START diagnostics. If the map change passes, prepare a clean final snapshot from905e2bc with ONLY the current src/bezier_offset.rs, excluding mandatory normalization, and qualify it against corner-overlap-authority-20260923-final1 as the parent. The existing qualify-corner-overlap-authority-20260923.py runner can be adapted: new root/prefix, parent rows/summary/binary are the905e2bc final1 artifacts, and the new-name set is empty (the regression was extended, not newly named). Both all-target checks and all1,216 library cases remain required. Preserve source/staged/HEAD byte bindings; stage only src/bezier_offset.rs for that commit. Never stage the unqualified normalization file.

Continue the actual major-arc predicate correction after the exact site is known. Mandatory normalization still has six all-boundary/traversal-sensitive assertions and five additional75-second limits beyond its two predicate blockers. The direct-line selected-circle helper's end_parameter is in ascending support order, independent of is_reversed(), so its non-reversed assertion is obsolete; any migration must still identify the correct retained source cut across all boundaries. Other count/adjacency assertions must preserve exact geometric/evidentiary obligations. No full closure claim is warranted. Broader work includes source cusps, XOR publication/duplicate cancellation, shared-source-box scheduling, API/location consolidation and held-out analytic/PCB/DRC qualifications.
