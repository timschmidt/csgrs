# Normalized corner callers — active candidate

The previous goal turn made progress and committed Hypercurve 4763c7ca97567a7fdfd14fac982019c390a248ce (probe endpoint incidence). All of its qualification processes were terminal/reaped. The full goal remains active.

Current main has only src/bezier_region.rs dirty. Its production change removes the single-loop selected-circle publication bypass; that change remains unqualified. This continuation updates six test callers to inspect every normalized boundary, preserve per-loop adjacency, and accept either traversal of the same exact source range. The direct-circle cut helper now checks the untouched start and strict-interior end parameter instead of asserting a traversal direction. Counts, tangent relations, endpoint-only proofs and exact seam ownership remain required.

Baseline source before these caller edits is archived as normalization-before-callers-20260923.rs (SHA256 1bc26463270fbe520873708bc6909db19a7af36356367f8bbc366b92696ed720). Current region candidate SHA256: 7ae2bbd876ba65c84644dc7cb4ecc82674e88ba76764700a8fd6eef7f1095c53. Base HEAD: 4763c7ca97567a7fdfd14fac982019c390a248ce.

TERMINAL/REAPED owned process: session 2123, run-normalized-corner-callers-20260923.py. Prefix normalized-corner-callers-20260923-focused1. Clean snapshot /tmp/hypercurve-normalized-corner-callers-2026-09-23 contains committed 4763c7c plus the current region file; dependencies are pinned snapshots, not Hyperreal's other-session worktree. ALL owned production/test/probe sources are frozen until this exact handle is terminal and reaped. Audit Markdown and inactive runners may be edited.

Eight focused cases run at 75 seconds each: the six caller failures, selected_circle_corner_candidates_publish_normalized_single_loops, and the old independent_oblique_chord_pair_fillets_extend_on_infinite_supports failure. Inspect actual results before making further edits. The oblique test still counts exactly two chord adjacencies on boundary 0 and expects a two-loop distant union; its baseline failure already reports three adjacencies, while the earlier normalized candidate reports zero in the first loop. It requires a geometric contract migration, not a new hardcoded observed count.

Pending: semantic verification of normalized results, remaining caller migrations, five additional previously recorded 75-second normalization limits, and the original 18 baseline nonpasses. Do not claim completion or commit the normalization change merely because the focused callers pass.


Focused1 is terminal/reaped: five of six migrated caller failures pass, along with the normalized single-loop publication test. The independently reframed seam now exposes Invalid/DegenerateOverlapRange under STRICT, seam cut, forward traversal. The old oblique test still fails its boundary-0 adjacency count (0 versus 2). Binary c18ab7ab965379405484e2901f27659e56c0bdc1d60a5d1381aea832413368d4; source/case records use normalized-corner-callers-20260923-focused1.

Main additionally migrates the oblique test across all boundary loops, replacing the exact fragment-adjacency count with retained tangent identity on both incident source supports and at least two certified chord adjacencies. Its disjoint union expects all original boundary loops plus the square, and checks square-center membership. This stronger caller migration is unqualified.

TERMINAL/REAPED owned process: session 70705, run-reframed-seam-20260923.py. Root /tmp/hypercurve-reframed-seam-diagnostic-2026-09-23; prefix reframed-seam-20260923-trace1. Eight range-error expressions print only source site/caller location; selected range functions are track_caller. No geometry or recursive-field Debug is printed. The diagnostic runs the reframed seam and oblique tests. Diagnostic instrumentation is snapshot-only; main still only changes src/bezier_region.rs. All owned sources are frozen until 70705 is terminal and reaped. Do not copy diagnostic production files back to main.


Trace1 is terminal/reaped (session 70705), binary 1c42254c1e4aa64c80d51f62f349cc09a4ad921025684cd3ff981448c96ba801. The stronger oblique caller passes all combinations and Boolean replay in 0.85 seconds including startup. The reframed seam error is ordered_endpoints rejecting the second input to intersect_parameter_ranges (bezier_split.rs:1156 in the diagnostic source). No generic degeneracy acceptance has been added.

TERMINAL/REAPED owned process: session 78930, run-reframed-seam2-20260923.py, prefix reframed-seam-20260923-trace2. Same diagnostic root; only an additional track_caller attribute forwards the range helper's immediate caller. Only the reframed seam test runs. All owned sources remain frozen until this exact handle is terminal/reaped. Main is unchanged from trace1 (only the region candidate).


Trace2 is terminal/reaped (session 78930), binary b97ea95aa7dd56a5911564c79cd58afc0eaabb9a3d7e6882603d682d513107e9. The caller is CurveIntersectionOverlap2::restrict_raw's range clipping closure at curve_intersection.rs:2514. Its second argument (the active limit) is zero-width. Trace3 compile failed because the diagnostic called mem::discriminant on the CurveParameter2 wrapper struct; session 97108 is terminal/reaped and that snapshot-only diagnostic was corrected.

TERMINAL/REAPED owned process: session 43400, run-reframed-seam4-20260923.py, prefix reframed-seam-20260923-trace4. Same diagnostic root, now logs only the labels of any zero-width overlap/limit, the restrict_raw caller, and whether its endpoint parameters are circle parameters. Main remains unchanged (only region candidate); all owned source inputs are frozen until 43400 is terminal/reaped. Public CurveParameterRange2::try_new explicitly requires a nonempty oriented range, and ordered_endpoints enforces that. Do not weaken that invariant without identifying the producer of the invalid range.

Trace4 is terminal/reaped, binary 29eab4706c0981b8bb9f78898c7881024e02475c2e66863e2e44805f508f5106. The zero-width limit is the second carrier domain, with two circle parameters. Trace5 will locate zero-width circle construction before choosing its repair. Main has no new edits yet.

TERMINAL/REAPED owned process: session 35692, run-reframed-seam5-20260923.py. All owned source inputs frozen until terminal/reaped. Snapshot-only construction trace reports zero-width circle producer.

Trace5 confirms zero-circle construction at retained_corner_fragment_trim -> CurveSupport2::restrict_certified (region line 3280). All diagnostic processes reaped. Main now changes the private trim result to Option, omits exactly consumed native parameter ranges, and migrates callers directly. The injective promoted-line branch uses point identity; nonlinear supports use source parameters. New cubic regression distinguishes coincident endpoints from an empty trace. Clean focused qualification is next.

TERMINAL/REAPED owned process: session 66170, run-consumed-corner-ranges-20260923.py; prefix consumed-corner-ranges-20260923-focused1. Clean normalized-corner-callers snapshot has current two main files, no diagnostic instrumentation. All owned source inputs frozen until terminal/reaped.

Focused1 was a compile failure only: the new test incorrectly used RationalBezier2::from(CubicBezier2); corrected to existing try_from_subcurve. Session 66170 reaped before any edits. TERMINAL/REAPED owned process: session 66154, run-consumed-corner-ranges2-20260923.py; prefix consumed-corner-ranges-20260923-focused2. All owned source inputs frozen until terminal/reaped.

Focused2 is terminal/reaped. All 13 cases pass, including exact reframed seam, consumed-range/closed-trace regression, distinct selected fields, analytic/native interval reuse, circle tangency, normalized publication, all migrated callers and oblique Boolean replay. Binary df98b5a79c151c3300ef5b4e941ba115b1036f12b100c3d8d37d272ff2de2b4e. Broad qualification reuses this exact binary and source bindings.

TERMINAL/REAPED owned process: session 67216, qualify-normalized-corner-publication-20260923.py. Prefix normalized-corner-publication-20260923-broad1, same clean root and focused2 binary. All-target all-features/no-default-features followed by all 1,219 cases, 75 seconds each, two workers. All owned source inputs frozen until terminal/reaped.

Broad1 uses a uniform 75-second per-case bound, unlike the previous normalized postchart report which merged the companion-side membership fixture from focused qualification at 260.98 seconds. That fixture therefore appears as an additional timeout relative to the parent, but has prior successful normalized evidence. The long1 runner includes it plus the five previously known normalization limits, at 300 seconds each with two workers, on the same focused2 executable.

Broad1 completed: 1,219 attempted, 1,190 pass, six ignored, eight unchanged assertions and 15 75-second limits. All six new nonpasses relative to parent are timeouts; there are no changed existing nonpasses. Oblique tangent-support/Boolean replay improves from assertion failure to pass. Both all-target checks pass. All processes reaped. Long1 will run the six new timeouts at 300 seconds, two workers, on exactly the same executable and source bindings.

TERMINAL/REAPED owned process: session 3137, run-normalized-corner-publication-long-20260923.py; prefix normalized-corner-publication-20260923-long1. All owned production/test/probe source inputs frozen until terminal/reaped. Audit Markdown and inactive runners may change.

While long1 runs, read-only review found two old offset assertions require the internal Rational overlap-map variant before their exact round-trip checks. A third requires a particular dispatch path although the actual trace retains local selected-fiber pair correlation via one subresultant GCD. These are next caller-review candidates; preserve or strengthen exact round trips, signs, source ownership, expiry/reconstruction and tangency contracts before removing representation-specific assumptions. No offset source has been edited.

Long1 completed and session 3137 is terminal/reaped. Companion-side membership passes in 262.87 seconds; the other five normalization cases still exceed 300 seconds. No new assertion or geometric predicate failure appeared in the longer run. These five cases remain unverified for exact completion and unresolved for performance. All owned source inputs are now editable. The full normalization/consumed-span candidate is ready for an incremental correctness commit with the known limits explicitly documented; this is not a full closure/completeness claim.


CURRENT AUTHORITATIVE STATE: committed `e7d575be40d87445f1d936ecdef5946aa24839c5`; Hypercurve worktree clean. Both source files in HEAD match the focused2/broad1/long1 candidate exactly. All owned processes (including 3137 and 67216) are terminal/reaped, so there is no source freeze. Hypersolve was not edited this turn. Hyperreal remains owned by the other session; continue using immutable pinned dependencies.

Next bounded task: profile `selected_circle_and_promoted_line_extend_through_the_chord_support_cell` in a fresh diagnostic snapshot of committed HEAD. It has a small circle/line fixture but exceeds 300 seconds under mandatory normalization. Preserve all candidates, policies and traversals; use bounded phase/caller/degree timing only, never recursive geometry Debug. Identify the actual contact or boundary decision that repeats expensive algebra before changing machinery. The five 300-second normalization limits are nonlinear algebraic endpoint fillet, chord/selected-circle full kernel, PH closed-loop self-contact, selected-circle/promoted-line extension, and selected-parallel companion. Existing nine 75-second limits and eight assertions remain. The completed companion-side test is 262.87 seconds, not a timeout at 300 seconds.

This goal turn made and committed progress. The full goal remains active, not blocked or complete. The mandatory-normalization change is now committed with explicit unresolved performance limits; do not restore the single-loop selected-circle bypass to meet an arbitrary time gate.
