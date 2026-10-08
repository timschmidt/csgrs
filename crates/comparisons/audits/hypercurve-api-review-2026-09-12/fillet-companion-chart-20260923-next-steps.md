Current normalization work is recorded in corner-publication-postchart-20260923-next-steps.md. Chart correction deeed31 is committed and fully qualified for this increment.

# Committed chart correction

Hypercurve deeed31d92f07cc6d7ffc8b93bc89061630c094d is committed. Broad2 is terminal/reaped with1,190passes/6ignored/18unchanged nonpasses, no new or changed failure status. Focused3 and all four public2 cases pass. Qualified/staged/HEAD bytes match; main's sole remaining change is the unqualified src/bezier_region.rs publication candidate. All owned handles are terminal/reaped. Another session owns Hyperreal; its working tree remains untouched and unused. Hypersolve is clean at e4f59ba. The full goal remains active.

Preparation is complete. CURRENT LIVE session30887 runs qualify-corner-publication-postchart-20260923.py against /tmp/hypercurve-corner-publication-postchart-2026-09-23. All owned production/test/probe inputs are frozen until this exact handle and the broad qualifier below are reaped. The qualifier builds a fresh library test binary and runs five focused cases, including the independently checked non-PH memberships with a300-second total case bound for four policy/traversal combinations. The first four focused normalization cases pass. CURRENT LIVE session27122 runs qualify-corner-publication-broad-postchart-20260923.py against the same immutable binary (112f98a02511d87efb1986baf0d9ad8a70438728a46f953a2839627840b2d73d). It checks both feature sets and runs the other1,210 library cases once, then incorporates the five source-bound focused results. Parent baseline is committed deeed31, represented by fillet-companion-chart-20260923-broad2. No compatibility APIs or agents have been introduced. The shared-source-box prototype remains separate and unqualified.

---

Historical diagnosis and rejected attempts follow.

# Current qualification update

The first clean chart candidate is REJECTED for commitment: broad1 attempts all1,214 cases, with1,189passes/6ignored and the18 known nonpasses plus a new selected-circle/promoted-line Unsupported failure. The same test passes on the unchanged parent. All broad1/public1/focused2 processes are terminal/reaped.

The chord contact route uses the exact point and geometric line support, without interpreting the cut parameter in that source chart. Switching to the replacement carrier before this route hid the existing direct-line proof behind a selected-fiber fragment. The final candidate moves companion-chart binding after the chord routes and before all parameterized contact routes. It preserves point-based chord authority and corrects every parameterized point/tangent evaluation. No compatibility shim or original-parameter reinterpretation is introduced.

Final root: /tmp/hypercurve-fillet-companion-final-2026-09-23. Public root: /tmp/hypercurve-fillet-companion-public2-2026-09-23. Focused3 is terminal/reaped: both new regressions pass and selected-circle/promoted-line extension passes. CURRENT LIVE: session69905 runs qualify-fillet-companion-broad2-20260923.py; session95756 is terminal/reaped; all four final public cases pass. Only session69905 remains live. All owned source inputs are frozen until both handles and any subsequent broad run are reaped. New broad qualifier is qualify-fillet-companion-broad2-20260923.py, to run after focused3 passes. Main Hypercurve has not been changed; src/bezier_region.rs remains the separate unqualified publication candidate. Main Hypersolve is clean at e4f59ba. Hyperreal remains untouched and unused as a working-tree dependency.

After final chart qualification, commit ONLY src/curve_corner_chain.rs, preserving the region candidate bytes. The prepared next step is prepare-corner-publication-postchart-20260923.py followed by qualify-corner-publication-postchart-20260923.py. These scripts require the final chart qualification and read the main region-publication candidate. Neither has been run.

---

Historical diagnosis and first-candidate details follow; current run status above supersedes older status paragraphs.

# Companion chart correction — in progress

Full implementation goal remains active. Hypersolve e4f59ba19e01c224c9fdc37f44debb752b6d40ae is committed and clean: all503 solver tests pass; Hypercurve1188passes/6ignored/18unchanged nonpassing cases. Staged and HEAD hashes match the qualified snapshot. Another session owns Hyperreal; never edit/stage/build its working tree. All owned production/test/probe sources remain frozen while any owned build or test runs. No agents used.

The previous Boundary diagnosis was refined. boundary-probe-trace-20260923-trace1 proves the endpoint-to-circle probe has four correct contacts and returns winding [[1],[0]]. No probe rejection occurs. boundary-decision-trace-20260923-trace1 instead shows carrier3 split1 incorrectly classified [[0],[0]], discarded, then an eight-edge open graph returns Boundary at build_regularized_region line6948. side-ray-trace-20260923-trace1 shows the other Bezier crossing contributes -1 correctly, but the second circle contributes a spurious +1.

The native minor-arc winding decision is independently checked on9360 exact integer-circle cases with no disagreement (minor-arc-winding-20260923-independent.json). circle-ray-predicates-20260923-trace1 captures the actual selected endpoint error: the radius2/5 circle center has y in[1.517,1.547] but its stored terminal point has y in[0.989,0.995], below the circle. The actual endpoints are separated from the circle by certified boxes (logged coordinates are approximate displays). Public points (279/260 +/- 1/10400,5501/5200) return incorrectly Outside/Inside with Certified certainty. Diagnostic exits0 because it prints the failure, not because closure passed.

Cause: canonicalize_retained_corner_cut transports the cut parameter into a replacement finite chart, but retained_fillet_fragments continues pairing that parameter with the original companion geometry. Here the companion changes from Q(s) to Q(2u-1); the new u is evaluated as s in endpoint and tangent evidence. The frame center/anchor is correct.

Trial fix binds the companion fragment to other_cut.replacement once before all family dispatch. Curve replacements use the same Materialized unit chart as reconstruction; analytic and selected replacements preserve their own ranges and reversal. Remove the tautological family guard matching every variant. The trial fillet-companion-chart-20260923-point1 restores the terminal y to[1.131,1.133] and the public memberships to Inside/Outside. No Boolean closure claim yet.

Clean candidate /tmp/hypercurve-fillet-companion-clean-2026-09-23 contains exactly src/curve_corner_chain.rs changes against Hypercurve a34c0fe + Hypersolve e4f59ba, pinned other dependencies. It additionally allows boundary contacts on the replacement range: canonicalization makes the exact cut an endpoint of that newly retained domain, unlike the original source interior check. This final small condition is not in the earlier diagnostic.

Two new regressions cover both policies and both traversals: open-path circle endpoints obey the independently certified center-y>3/2/r=2/5 implication y>11/10; closed-region nearby-point memberships are Inside/Outside. The clean parent /tmp/hypercurve-fillet-companion-parent-2026-09-23 has identical tests and unchanged a34 production. prepare-fillet-companion-clean-20260923.py records preparation.

Focused builds and corrected replays are terminal/reaped. The focused1 suffix filter ran zero tests and is rejected as test evidence. Focused2 uses full names and verifies one executed test: both parent cases fail and both candidate cases pass. CURRENT LIVE: session7708 runs qualify-fillet-companion-broad-20260923.py; session59604 is terminal/reaped with all four public cases passing (about65 seconds each). Source inputs remain frozen until session7708 is reaped.

Public Boolean probe is terminal/reaped; all four policy/traversal combinations pass: /tmp/hypercurve-fillet-companion-public-2026-09-23, example fillet_companion_public_20260923. It uses the clean candidate file, exact e4 solver and pinned scalar snapshot. A numeric command-line argument0..3 selects policy/traversal. It removes the unjustified old union-loop-count2 assertion and prints the resulting count instead, while asserting empty intersection and independent nearby-point membership on the normalized union. Fillet extension leaves two source crossings, so normalized loops need not equal the raw publication count. Qualify broad library/all-target checks and this public composition before committing.

Shared-source-box interval prototype remains separate and unqualified. It was used only in diagnostic trees; it is not in the clean candidate. Main Hypercurve still only has the older unqualified src/bezier_region.rs removal of the selected-circle publication exception. Do not stage that file with this correction. Main Hyperreal remains untouched and unused.
